#include "core/engine.h"

#include <chrono>
#include <cstdio>
#include <cstring>

namespace iridi {

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
        spectrum_inst_.clear();
        spectrum_avg_.clear();
        parsed_count_.store(0);
        parse_error_count_.store(0);
    }
    voice_.reset();
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
}

void Engine::on_frame(const ir_frame_info_t *info) {
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
    last_ts_.store(pm.timestamp_ns);
    if (!pm.voice_payload.empty()) voice_.feed(pm);
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
    } else if (pm.has_ibc) {
        // IBC carries no position; only track presence/beam.
        Satellite &s = sats_[pm.ibc_sat];
        s.id = pm.ibc_sat; s.beam = pm.ibc_beam;
        s.last_seen_ns = pm.timestamp_ns; s.count++;
    }
}

void Engine::thread_main() {
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

        ir_pipeline_feed(static_cast<ir_pipeline_t *>(pipe_), buf.data(), n);
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

} // namespace iridi
