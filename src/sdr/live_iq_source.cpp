#include "sdr/live_iq_source.h"

#include <algorithm>

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
    ring_.resize(cap_ * 2);
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

    {
        std::lock_guard<std::mutex> lk(mtx_);
        head_ = 0;
        count_ = 0;
    }
    open_.store(true);

    std::string e;
    if (!sdr_->start(device_index, [this](const float *iq, int n) { on_samples(iq, n); }, e)) {
        open_.store(false);
        cv_.notify_all();
        err = e.empty() ? "failed to start SDRplay device" : e;
        return false;
    }
    return true;
}

void LiveIqSource::close() {
    open_.store(false);
    cv_.notify_all();
    if (sdr_) sdr_->stop();
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
    if (sdr_) sdr_->setSampleRate(hz);
}

void LiveIqSource::clear_buffer() {
    std::lock_guard<std::mutex> lk(mtx_);
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
    size_t un = (size_t)n;

    // A block larger than the ring keeps only the newest cap_ samples.
    if (un > cap_) {
        iq += (un - cap_) * 2;
        un = cap_;
        head_ = 0;
        count_ = 0;
    }

    // Drop the oldest samples to make room, preserving ordering.
    if (count_ + un > cap_) {
        size_t drop = count_ + un - cap_;
        head_ += drop;
        if (head_ >= cap_) head_ -= cap_;
        count_ -= drop;
    }

    for (size_t i = 0; i < un; ++i) {
        size_t idx = head_ + count_;
        if (idx >= cap_) idx -= cap_;
        ring_[2 * idx] = iq[2 * i];
        ring_[2 * idx + 1] = iq[2 * i + 1];
        ++count_;
    }
    cv_.notify_all();
}

size_t LiveIqSource::read_cf32(float *out, size_t max_samples) {
    if (max_samples == 0) return 0;
    std::unique_lock<std::mutex> lk(mtx_);
    cv_.wait(lk, [this] { return count_ > 0 || !open_.load(); });
    if (count_ == 0) return 0;

    size_t k = std::min(max_samples, count_);
    for (size_t i = 0; i < k; ++i) {
        size_t idx = head_ + i;
        if (idx >= cap_) idx -= cap_;
        out[2 * i] = ring_[2 * idx];
        out[2 * i + 1] = ring_[2 * idx + 1];
    }
    head_ += k;
    if (head_ >= cap_) head_ -= cap_;
    count_ -= k;
    return k;
}

} // namespace iridi
