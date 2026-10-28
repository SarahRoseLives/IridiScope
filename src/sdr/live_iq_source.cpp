#include "sdr/live_iq_source.h"

#include <algorithm>
#include <cmath>

namespace iridi {

#ifndef HAS_SDRPLAY
// SDRplay support not built in: the backend factory is a no-op.
std::unique_ptr<SdrSource> make_sdrplay_source() { return nullptr; }
#endif
#ifndef HAS_HACKRF
// HackRF support not built in: the backend factory is a no-op.
std::unique_ptr<SdrSource> make_hackrf_source() { return nullptr; }
#endif
#ifndef HAS_AIRSPY
// Airspy support not built in: the backend factory is a no-op.
std::unique_ptr<SdrSource> make_airspy_source() { return nullptr; }
#endif
#ifndef HAS_RTLSDR
// RTL-SDR support not built in: the backend factory is a no-op.
std::unique_ptr<SdrSource> make_rtlsdr_source() { return nullptr; }
#endif

std::unique_ptr<SdrSource> make_sdr_source(SdrKind kind) {
    switch (kind) {
    case SdrKind::HackRF: return make_hackrf_source();
    case SdrKind::Airspy: return make_airspy_source();
    case SdrKind::RTL:    return make_rtlsdr_source();
    case SdrKind::Sdrplay:
    default:              return make_sdrplay_source();
    }
}

LiveIqSource::LiveIqSource(SdrKind kind, double center_hz, double sample_rate_hz)
    : kind_(kind), center_hz_(center_hz), sample_rate_(sample_rate_hz) {
    // The buffer is allocated when the radio starts, sized to its rate, so
    // idle backends don't hold memory.
}

// About one second of samples (at least the original 1 Mi): long enough to
// ride out the processing thread being held up briefly (other SDR programs,
// UI work), without adding latency while processing keeps up.
void LiveIqSource::size_ring(double rate_hz) {
    const size_t want = std::max<size_t>(1u << 20, (size_t)std::llround(std::max(0.0, rate_hz) * 1.0));
    std::lock_guard<std::mutex> lk(mtx_);
    if (want != cap_ || ring_.size() != want * 2) {
        cap_ = want;
        std::vector<float>().swap(ring_);
        ring_.assign(cap_ * 2, 0.0f);
    }
    head_ = 0;
    count_ = 0;
    reset_positions();
}

// Caller holds mtx_.
void LiveIqSource::reset_positions() {
    rpos_ = wpos_ = 0;
    gaps_.clear();
    last_gap_ = 0;
    drv_seen_ = sdr_ ? sdr_->driverDroppedSamples() : 0;
}

// Caller holds mtx_. Gaps are kept in position order; same position merges.
void LiveIqSource::add_gap(uint64_t pos, uint64_t n) {
    if (!n) return;
    if (!gaps_.empty() && gaps_.back().first >= pos) { gaps_.back().second += n; return; }
    gaps_.emplace_back(pos, n);
}

uint64_t LiveIqSource::take_gap() {
    std::lock_guard<std::mutex> lk(mtx_);
    const uint64_t g = last_gap_;
    last_gap_ = 0;
    return g;
}

LiveIqSource::~LiveIqSource() { close(); }

bool LiveIqSource::ensure_backend() const {
    if (!sdr_) sdr_ = make_sdr_source(kind_);
    return sdr_ != nullptr;
}

bool LiveIqSource::open(int device_index, double center_hz, double sample_rate_hz,
                        std::string &err) {
    close();
    err.clear();
    device_name_.clear();

    if (!ensure_backend()) {
        err = "SDR backend is not compiled in";
        return false;
    }

    // Cache the selected device name before start(): some backends no longer
    // enumerate the device once it is open (notably libairspy).
    {
        auto devs = sdr_->listDevices();
        if (!devs.empty()) {
            int idx = (device_index >= 0 && device_index < (int)devs.size()) ? device_index : 0;
            device_name_ = devs[idx].name;
        }
    }

    center_hz_ = center_hz;
    sample_rate_ = sample_rate_hz;
    sdr_->setCenterFreq(center_hz_);
    sdr_->setSampleRate(sample_rate_);
    sdr_->setGain(gain_);
    sdr_->setPpm(ppm_);
    sdr_->setBiasTee(bias_);
    sdr_->setAntenna(antenna_);
    sdr_->setLnaGain(lna_);
    sdr_->setVgaGain(vga_);
    sdr_->setAmp(amp_);
    sdr_->setGainMode(gain_mode_);
    sdr_->setSenseGain(sense_);
    sdr_->setLinearGain(linear_);
    sdr_->setMixerGain(mixer_);
    sdr_->setLnaAgc(lna_agc_);
    sdr_->setMixerAgc(mixer_agc_);
    sdr_->setAllowUnofficialRates(unofficial_rates_);

    size_ring(sample_rate_);
    dropped_.store(0);
    received_.store(0);
    open_.store(true);

    std::string e;
    if (!sdr_->start(device_index, [this](const float *iq, int n) { on_samples(iq, n); }, e)) {
        open_.store(false);
        cv_.notify_all();
        err = e.empty() ? "failed to start SDRplay device" : e;
        return false;
    }
    // The backend may have adjusted the rate to one the device supports
    // (e.g. Airspy Mini: 6 / 3 Msps); the DSP must use the real value.
    if (sdr_->sampleRate() > 0 && sdr_->sampleRate() != sample_rate_) {
        sample_rate_ = sdr_->sampleRate();
        size_ring(sample_rate_);
    }
    return true;
}

void LiveIqSource::close() {
    open_.store(false);
    cv_.notify_all();
    if (sdr_) sdr_->stop();
    std::lock_guard<std::mutex> lk(mtx_);
    std::vector<float>().swap(ring_); // free the buffer while stopped
    head_ = 0;
    count_ = 0;
    reset_positions();
}

void LiveIqSource::wake() {
    open_.store(false);
    cv_.notify_all();
}

void LiveIqSource::set_center_frequency(double hz) {
    center_hz_ = hz;
    if (sdr_) sdr_->setCenterFreq(hz);
}

void LiveIqSource::set_sample_rate(double hz) {
    sample_rate_ = hz;
    if (sdr_) {
        sdr_->setSampleRate(hz);
        if (sdr_->sampleRate() > 0) sample_rate_ = sdr_->sampleRate();
    }
    if (open_.load()) size_ring(sample_rate_);
}

void LiveIqSource::clear_buffer() {
    std::lock_guard<std::mutex> lk(mtx_);
    // The discarded samples are a gap in the stream: keep time continuous.
    add_gap(wpos_, count_);
    rpos_ = wpos_;
    head_ = 0;
    count_ = 0;
}

void LiveIqSource::set_gain(double db) {
    gain_ = db;
    if (sdr_) sdr_->setGain(db);
}

void LiveIqSource::set_ppm(double ppm) {
    ppm_ = ppm;
    if (sdr_) sdr_->setPpm(ppm);
}

void LiveIqSource::set_bias_tee(bool on) {
    bias_ = on;
    if (sdr_) sdr_->setBiasTee(on);
}

void LiveIqSource::set_antenna(int port) {
    antenna_ = port;
    if (sdr_) sdr_->setAntenna(port);
}

void LiveIqSource::set_lna_gain(int db) {
    lna_ = db;
    if (sdr_) sdr_->setLnaGain(db);
}

void LiveIqSource::set_vga_gain(int db) {
    vga_ = db;
    if (sdr_) sdr_->setVgaGain(db);
}

void LiveIqSource::set_amp(bool on) {
    amp_ = on;
    if (sdr_) sdr_->setAmp(on);
}

void LiveIqSource::set_gain_mode(int m) {
    gain_mode_ = m;
    if (sdr_) sdr_->setGainMode(m);
}

void LiveIqSource::set_sense_gain(int v) {
    sense_ = v;
    if (sdr_) sdr_->setSenseGain(v);
}

void LiveIqSource::set_linear_gain(int v) {
    linear_ = v;
    if (sdr_) sdr_->setLinearGain(v);
}

void LiveIqSource::set_mixer_gain(int v) {
    mixer_ = v;
    if (sdr_) sdr_->setMixerGain(v);
}

void LiveIqSource::set_lna_agc(bool on) {
    lna_agc_ = on;
    if (sdr_) sdr_->setLnaAgc(on);
}

void LiveIqSource::set_mixer_agc(bool on) {
    mixer_agc_ = on;
    if (sdr_) sdr_->setMixerAgc(on);
}

bool LiveIqSource::running() const { return sdr_ && sdr_->running(); }

std::vector<SdrDeviceInfo> LiveIqSource::list_devices() const {
    if (!ensure_backend()) return {};
    return sdr_->listDevices();
}

std::string LiveIqSource::device_name() const {
    if (!device_name_.empty()) return device_name_;
    switch (kind_) {
    case SdrKind::Airspy: return "Airspy";
    case SdrKind::HackRF: return "HackRF";
    case SdrKind::RTL:    return "RTL-SDR";
    case SdrKind::Sdrplay:
    default:              return "SDRplay";
    }
}

void LiveIqSource::on_samples(const float *iq, int n) {
    if (n <= 0) return;
    std::lock_guard<std::mutex> lk(mtx_);
    if (ring_.size() < cap_ * 2) return; // not allocated (stopped)
    size_t un = (size_t)n;
    received_.fetch_add(un);

    // Samples the driver lost before this block (Airspy reports them per
    // transfer, on this same thread, just before delivering the block).
    if (sdr_ && sdr_->reportsDriverDrops()) {
        const uint64_t d = sdr_->driverDroppedSamples();
        if (d > drv_seen_) add_gap(wpos_, d - drv_seen_);
        drv_seen_ = d;
    }

    // A block larger than the ring keeps only the newest cap_ samples.
    if (un > cap_) {
        const size_t lost = un - cap_ + count_;
        dropped_.fetch_add(lost);
        iq += (un - cap_) * 2;
        un = cap_;
        head_ = 0;
        count_ = 0;
        // Everything up to the kept part is lost: one gap just before it.
        // (Discarded samples never got a stream position.)
        uint64_t g = lost;
        for (auto &e : gaps_) g += e.second;
        gaps_.clear();
        rpos_ = wpos_;
        add_gap(rpos_, g);
    }

    // Drop the oldest samples to make room, preserving ordering.
    if (count_ + un > cap_) {
        size_t drop = count_ + un - cap_;
        dropped_.fetch_add(drop);
        head_ += drop;
        if (head_ >= cap_) head_ -= cap_;
        count_ -= drop;
        // The dropped samples become a gap at the new read position; any
        // gaps inside the dropped span merge into it.
        rpos_ += drop;
        uint64_t g = drop;
        size_t k = 0;
        while (k < gaps_.size() && gaps_[k].first <= rpos_) g += gaps_[k++].second;
        gaps_.erase(gaps_.begin(), gaps_.begin() + k);
        gaps_.insert(gaps_.begin(), {rpos_, g});
    }

    for (size_t i = 0; i < un; ++i) {
        size_t idx = head_ + count_;
        if (idx >= cap_) idx -= cap_;
        ring_[2 * idx] = iq[2 * i];
        ring_[2 * idx + 1] = iq[2 * i + 1];
        ++count_;
    }
    wpos_ += un;
    cv_.notify_all();
}

size_t LiveIqSource::read_cf32(float *out, size_t max_samples) {
    if (max_samples == 0) return 0;
    std::unique_lock<std::mutex> lk(mtx_);
    cv_.wait(lk, [this] { return count_ > 0 || !open_.load(); });
    if (count_ == 0) return 0;

    // Report the gap (if any) right before this block, and never let a block
    // run across a later gap.
    last_gap_ = 0;
    size_t gi = 0;
    while (gi < gaps_.size() && gaps_[gi].first <= rpos_) last_gap_ += gaps_[gi++].second;
    gaps_.erase(gaps_.begin(), gaps_.begin() + gi);
    size_t k = std::min(max_samples, count_);
    if (!gaps_.empty() && gaps_.front().first - rpos_ < k) k = (size_t)(gaps_.front().first - rpos_);
    for (size_t i = 0; i < k; ++i) {
        size_t idx = head_ + i;
        if (idx >= cap_) idx -= cap_;
        out[2 * i] = ring_[2 * idx];
        out[2 * i + 1] = ring_[2 * idx + 1];
    }
    head_ += k;
    if (head_ >= cap_) head_ -= cap_;
    count_ -= k;
    rpos_ += k;
    return k;
}

} // namespace iridi

namespace iridi {
void LiveIqSource::set_allow_unofficial_rates(bool on) {
    unofficial_rates_ = on;
    if (sdr_) sdr_->setAllowUnofficialRates(on);
}
} // namespace iridi
