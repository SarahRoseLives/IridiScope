#include "core/engine.h"
#include "store/msg_db.h"

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include <chrono>
#include <cmath>
#include <ctime>
#include <cstdio>
#include <cstring>

namespace iridi {

namespace {
// CPU time used by the calling thread so far.
uint64_t thread_cpu_ns() {
#if defined(_WIN32)
    FILETIME c, e, k, u;
    if (!GetThreadTimes(GetCurrentThread(), &c, &e, &k, &u)) return 0;
    const uint64_t kk = ((uint64_t)k.dwHighDateTime << 32) | k.dwLowDateTime;
    const uint64_t uu = ((uint64_t)u.dwHighDateTime << 32) | u.dwLowDateTime;
    return (kk + uu) * 100;   // 100 ns units
#else
    timespec ts{};
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
#endif
}
} // namespace


extern "C" void ir_engine_frame_cb(const ir_frame_info_t *info, void *user) {
    static_cast<Engine *>(user)->on_frame(info);
}

Engine::~Engine() { stop(); }

bool Engine::start(IqSource *source, float threshold) {
    if (running_.load()) return false;
    if (!source || !source->is_open()) return false;

    source_ = source;

    double center = source->center_frequency() > 0 ? source->center_frequency() : 1622000000.0;
    int rate = source->sample_rate() > 0 ? source->sample_rate() : 10000000;

    pipe_ = ir_pipeline_create(center, rate, threshold, 0, ir_engine_frame_cb, this);
    if (!pipe_) return false;
    // A recording with a known start time (from its file name) keeps that
    // time; live sources start at the current time.
    if (start_time_ns_) ir_pipeline_set_start_time(static_cast<ir_pipeline_t *>(pipe_), start_time_ns_);
    start_time_ns_ = 0;

    total_samples_.store(source->total_samples());
    samples_.store(0);
    done_.store(false);
    raw_base_ns_.store(0);
    paused_.store(false);
    running_.store(true);
    {
        std::lock_guard<std::mutex> lk(mtx_);
        frames_.clear();
        messages_.clear();
        sats_.clear();
        beams_.clear();
        pages_.clear();
        spectrum_inst_.clear();
        spectrum_avg_.clear();
        parsed_count_.store(0);
        uw_recovered_.store(0);
        uw_tolerated_failed_.store(0);
        parse_error_count_.store(0);
    }
    voice_.flush_all(); // keep anything left from a previous run
    voice_.reset();
    voice_recovered_.store(0);
    acars_.reset();
    pager_.reset();
    spectrum_seq_.store(0);
    spectrum_size_.store(0);

    thread_ = std::thread(&Engine::thread_main, this);
    return true;
}

void Engine::stop() {
    if (!running_.load() && !thread_.joinable()) return;
    running_.store(false);
    paused_.store(false);
    if (source_) source_->wake();
    if (thread_.joinable()) thread_.join();
    if (pipe_) {
        ir_pipeline_destroy(static_cast<ir_pipeline_t *>(pipe_));
        pipe_ = nullptr;
    }
    iq_rec_.stop();
    // Save calls that were still in progress (Stop, new source, exit).
    voice_.flush_all();
}

bool Engine::start_iq_recording(const std::string &dir, std::string &err) {
    if (!running_.load() || !source_) { err = "start a radio first"; return false; }
    if (source_->total_samples() != 0) { err = "recording is for live radios (this is a file)"; return false; }
    return iq_rec_.start(dir, source_->sample_rate(), source_->center_frequency(), err);
}

void Engine::stop_iq_recording() { iq_rec_.stop(); }

void Engine::on_frame(const ir_frame_info_t *info) {
    {
        // A targeted read can occasionally decode a burst the detector also
        // caught (its detection started off-slot). Keep only the first copy
        // of any frame pair where one came from a targeted read.
        struct Seen { uint64_t ts; double f; bool probe; };
        static std::mutex seen_mtx;
        static Seen seen[2048];
        static unsigned seen_n = 0;
        const bool probe = (info->id >> 63) != 0;
        std::lock_guard<std::mutex> lk(seen_mtx);
        const unsigned n = seen_n < 2048 ? seen_n : 2048;
        for (unsigned i = 0; i < n; i++) {
            const Seen &x = seen[(seen_n - 1 - i) % 2048];
            const uint64_t dt = x.ts > info->timestamp_ns ? x.ts - info->timestamp_ns : info->timestamp_ns - x.ts;
            if ((probe || x.probe) && dt < 3000000ULL && std::fabs(x.f - info->frequency) < 20000.0) return;
        }
        seen[seen_n++ % 2048] = Seen{info->timestamp_ns, info->frequency, probe};
    }
    if (raw_dump_.load()) {
        static std::mutex dump_mtx;
        std::lock_guard<std::mutex> lk(dump_mtx);
        uint64_t base = raw_base_ns_.load();
        if (base == 0) {
            base = info->timestamp_ns;
            raw_base_ns_.store(base);
        }
        unsigned long long ms =
            info->timestamp_ns >= base ? (info->timestamp_ns - base) / 1000000ULL : 0ULL;
        // iridium-toolkit RAW format:
        // RAW: <name> <mstime> <freq> A:OK L:<id> <conf>% <level> <len> <bits>
        printf("RAW: i-0-t1 %llu %.0f A:OK L:0 %d%% %.6f %d ", ms, info->frequency,
               info->confidence, info->level, info->n_symbols);
        if (info->bits)
            for (int i = 0; i < info->n_bits; i++) putchar(info->bits[i] ? '1' : '0');
        putchar('\n');
    }

    FrameEvent ev;
    ev.frequency = info->frequency;
    ev.timestamp_ns = info->timestamp_ns;
    ev.confidence = info->confidence;
    ev.direction = info->direction;
    ev.n_bits = info->n_bits;

    irid::RawFrame rf;
    rf.frequency = info->frequency;
    rf.timestamp_ns = info->timestamp_ns;
    rf.confidence = info->confidence;
    rf.level = info->level;
    rf.noise = info->noise;
    rf.snr = info->magnitude;
    rf.n_symbols = info->n_symbols;
    if (info->bits && info->n_bits > 0) {
        rf.bitstream.resize(info->n_bits);
        for (int i = 0; i < info->n_bits; i++)
            rf.bitstream[i] = info->bits[i] ? '1' : '0';
    }

    irid::ParsedMessage pm;
    irid::parse_frame(rf, pm);
    parsed_count_.fetch_add(1);
    if (pm.error) parse_error_count_.fetch_add(1);
    if (pm.uw_errors > 0) {
        // Frame accepted only thanks to unique-word tolerance.
        if (pm.error) uw_tolerated_failed_.fetch_add(1);
        else uw_recovered_.fetch_add(1);
    }
    last_ts_.store(pm.timestamp_ns);
    if (!pm.voice_payload.empty()) {
        voice_.feed(pm);
        // Let the pipeline target this call's next bursts (weak ones the
        // detector misses). Only real speech frames, not VOD data.
        if (!pm.error && pm.voice_subtype != "VOD" && pipe_)
            ir_pipeline_voice_seen(static_cast<ir_pipeline_t *>(pipe_), pm.timestamp_ns, pm.frequency);
        if (!pm.error && (info->id >> 63)) voice_recovered_.fetch_add(1);
    }
    if (pm.has_ida) acars_.feed(pm);
    if (pm.has_msg) pager_.feed(pm);

    std::lock_guard<std::mutex> lk(mtx_);
    frames_.push_back(ev);
    if (frames_.size() > 4000) frames_.pop_front();
    messages_.push_back(pm);
    if (messages_.size() > 4000) messages_.pop_front();
    if (pm.has_ira) {
        Satellite &s = sats_[pm.ira_sat];
        s.id = pm.ira_sat; s.beam = pm.ira_beam;
        s.last_seen_ns = pm.timestamp_ns; s.count++;
        // Only satellite ring alerts (alt ~780 km) carry the satellite subpoint.
        // Beam/spot positions (alt < 700 km) must not move the satellite marker.
        if (pm.ira_alt_km >= 700 && pm.ira_alt_km <= 900) {
            s.lat = pm.ira_lat; s.lon = pm.ira_lon; s.alt_km = pm.ira_alt_km;
            s.has_pos = true;
        }
        int key = (pm.ira_sat << 8) | (pm.ira_beam & 0xff);
        GroundBeam &g = beams_[key];
        g.sat = pm.ira_sat; g.beam = pm.ira_beam;
        if (pm.ira_is_ground && pm.ira_alt_km < 100) {
            g.lat = pm.ira_lat; g.lon = pm.ira_lon; g.alt_km = pm.ira_alt_km;
            g.last_seen_ns = pm.timestamp_ns; g.count++; g.has_pos = true;
        }
        for (const auto &p : pm.ira_page_list) {
            PageEvent e;
            e.timestamp_ns = pm.timestamp_ns;
            e.sat = pm.ira_sat; e.beam = pm.ira_beam;
            e.tmsi = p.tmsi; e.msc_id = p.msc_id;
            pages_.push_back(e);
            MsgDb::instance().ring_alert(e.timestamp_ns, e.sat, e.beam, e.tmsi, e.msc_id);
            g.pages++;
            if (pages_.size() > 20000) pages_.pop_front();
        }
    } else if (pm.has_ibc) {
        // IBC carries no position; only track presence/beam.
        Satellite &s = sats_[pm.ibc_sat];
        s.id = pm.ibc_sat; s.beam = pm.ibc_beam;
        s.last_seen_ns = pm.timestamp_ns; s.count++;
    }
}

void Engine::set_center_frequency(double hz) {
    if (pipe_) {
        ir_pipeline_set_center(static_cast<ir_pipeline_t *>(pipe_), hz);
        // The spectrum under the detector changed: its learned noise floor
        // no longer applies.
        ir_pipeline_reset_noise(static_cast<ir_pipeline_t *>(pipe_));
    }
}

int Engine::worker_threads() const {
    return pipe_ ? ir_pipeline_workers(static_cast<ir_pipeline_t *>(pipe_)) : 0;
}

void Engine::set_queue_budget(double bytes) { ir_pipeline_set_queue_budget(bytes); }
void Engine::set_live_backlog(double seconds) { ir_pipeline_set_live_backlog(seconds); }

uint64_t Engine::targeted_reads() const {
    uint64_t pr = 0;
    if (pipe_) ir_pipeline_probe_stats(static_cast<ir_pipeline_t *>(pipe_), &pr, nullptr);
    return pr;
}

uint64_t Engine::skipped_bursts() const {
    return pipe_ ? ir_pipeline_skipped(static_cast<ir_pipeline_t *>(pipe_)) : 0;
}

int Engine::queue_capacity() const {
    return pipe_ ? ir_pipeline_queue_capacity(static_cast<ir_pipeline_t *>(pipe_)) : 0;
}

uint64_t Engine::queue_full_count() const {
    return pipe_ ? ir_pipeline_queue_full(static_cast<ir_pipeline_t *>(pipe_)) : 0;
}

float Engine::detector_noise_floor() const {
    return pipe_ ? ir_pipeline_noise_floor(static_cast<ir_pipeline_t *>(pipe_)) : 0.0f;
}

void Engine::reset_noise_floor() {
    if (pipe_) ir_pipeline_reset_noise(static_cast<ir_pipeline_t *>(pipe_));
}

void Engine::thread_main() {
    // The burst detector is the one thread that must keep up with the radio
    // (anything it misses is lost), so it runs at high priority. It is light
    // now that bursts aren't copied; the decoders run at normal priority and
    // skip the oldest bursts if they fall too far behind.
#if defined(_WIN32)
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);
#endif
    const size_t block = 65536; // complex samples per read
    std::vector<float> buf(block * 2);

    using clock = std::chrono::steady_clock;
    auto t0 = clock::now();
    auto last_spec = t0;

    while (running_.load()) {
        if (paused_.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            int prate = source_->sample_rate() > 0 ? source_->sample_rate() : 1;
            t0 = clock::now() - std::chrono::duration_cast<clock::duration>(
                     std::chrono::duration<double>(samples_.load() / (double)prate));
            continue;
        }

        size_t n = source_->read_cf32(buf.data(), block);
        if (n == 0) {
            if (source_->done()) { done_.store(true); break; }
            continue;
        }
        // Samples lost just before this block (driver or buffer overrun):
        // advance the detector's clock so timestamps stay on real time.
        const uint64_t gap = source_->take_gap();
        if (gap) {
            ir_pipeline_skip_samples(static_cast<ir_pipeline_t *>(pipe_), gap);
            gap_samples_.fetch_add(gap);
        }
        if (iq_rec_.active()) iq_rec_.push(buf.data(), n, gap);

        {
            // Detector CPU time: real work, as opposed to elapsed time (which
            // also counts time the thread was waiting for a CPU).
            const uint64_t cpu0 = thread_cpu_ns();
            // Detector load: time spent detecting vs the signal time it covers.
            // Waiting for burst-queue space is not detector work: count it apart.
            ir_pipeline_t *pp = static_cast<ir_pipeline_t *>(pipe_);
            const uint64_t w0 = ir_pipeline_queue_wait_ns(pp);
            const auto f0 = std::chrono::steady_clock::now();
            ir_pipeline_feed(pp, buf.data(), n);
            const uint64_t spent = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now() - f0).count();
            const uint64_t waited = ir_pipeline_queue_wait_ns(pp) - w0;
            det_busy_ns_.fetch_add(spent > waited ? spent - waited : 0);
            det_wait_ns_.fetch_add(waited);
            const uint64_t cpu1 = thread_cpu_ns();
            if (cpu1 > cpu0) det_cpu_ns_.fetch_add(cpu1 - cpu0);
            det_samples_.fetch_add(n);
        }
        samples_.fetch_add(n);
        voice_.sweep(last_ts_.load());

        auto now = clock::now();
        if (now - last_spec > std::chrono::milliseconds(33)) {
            last_spec = now;
            ir_pipeline_t *pp = static_cast<ir_pipeline_t *>(pipe_);
            int sn = ir_pipeline_spectrum_size(pp);
            const float *inst = ir_pipeline_spectrum_inst(pp);
            const float *avg = ir_pipeline_spectrum_avg(pp);
            if (sn > 0 && inst && avg) {
                std::lock_guard<std::mutex> lk(mtx_);
                spectrum_inst_.assign(inst, inst + sn);
                spectrum_avg_.assign(avg, avg + sn);
                spectrum_size_.store(sn);
                floor_db_.store(ir_pipeline_spectrum_floor(pp));
                peak_db_.store(ir_pipeline_spectrum_peak(pp));
                spectrum_seq_.fetch_add(1);
            }
        }

        if (realtime_.load()) {
            int rate = source_->sample_rate();
            if (rate > 0) {
                double target = (double)samples_.load() / rate;
                double elapsed = std::chrono::duration<double>(clock::now() - t0).count();
                if (target > elapsed)
                    std::this_thread::sleep_for(
                        std::chrono::duration<double>(target - elapsed));
            }
        }
    }

    running_.store(false);
}

EngineStats Engine::stats() const {
    EngineStats s;
    s.samples = samples_.load();
    s.total_samples = total_samples_.load();
    if (pipe_) {
        s.bursts = ir_pipeline_bursts(static_cast<ir_pipeline_t *>(pipe_));
        s.frames_ok = ir_pipeline_frames_ok(static_cast<ir_pipeline_t *>(pipe_));
        s.frames_fail = ir_pipeline_frames_fail(static_cast<ir_pipeline_t *>(pipe_));
    }
    s.running = running_.load();
    s.paused = paused_.load();
    s.done = done_.load();
    int rate = (source_ && source_->is_open()) ? source_->sample_rate() : 0;
    if (rate > 0) {
        s.elapsed_s = (double)s.samples / rate;
        s.rate_sps = 0; // set by UI from wall clock if needed
    }
    return s;
}

std::vector<float> Engine::spectrum_inst() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return spectrum_inst_;
}

std::vector<float> Engine::spectrum_avg() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return spectrum_avg_;
}

std::vector<FrameEvent> Engine::recent_frames(size_t max_count) const {
    std::lock_guard<std::mutex> lk(mtx_);
    std::vector<FrameEvent> out;
    size_t n = frames_.size();
    size_t start = n > max_count ? n - max_count : 0;
    out.reserve(n - start);
    for (size_t i = start; i < n; i++) out.push_back(frames_[i]);
    return out;
}

std::vector<irid::ParsedMessage> Engine::recent_messages(size_t max_count) const {
    std::lock_guard<std::mutex> lk(mtx_);
    std::vector<irid::ParsedMessage> out;
    size_t n = messages_.size();
    size_t start = n > max_count ? n - max_count : 0;
    out.reserve(n - start);
    for (size_t i = start; i < n; i++) out.push_back(messages_[i]);
    return out;
}

std::vector<Satellite> Engine::satellites() const {
    std::lock_guard<std::mutex> lk(mtx_);
    std::vector<Satellite> out;
    out.reserve(sats_.size());
    for (auto &kv : sats_) out.push_back(kv.second);
    return out;
}

std::vector<GroundBeam> Engine::ground_beams() const {
    std::lock_guard<std::mutex> lk(mtx_);
    std::vector<GroundBeam> out;
    out.reserve(beams_.size());
    for (const auto &kv : beams_)
        if (kv.second.has_pos) out.push_back(kv.second);
    return out;
}

std::vector<PageEvent> Engine::paging_events(size_t max_count) const {
    std::lock_guard<std::mutex> lk(mtx_);
    std::vector<PageEvent> out;
    size_t n = pages_.size();
    size_t start = n > max_count ? n - max_count : 0;
    out.reserve(n - start);
    for (size_t i = start; i < n; i++) out.push_back(pages_[i]);
    return out;
}

} // namespace iridi
