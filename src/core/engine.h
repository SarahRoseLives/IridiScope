// Real-time processing engine: drives the vendored DSP pipeline over an IQ
// file on a background thread and exposes live statistics, a spectrum, and
// recent demodulated frames to the UI.
#ifndef IRIDISCOPE_ENGINE_H
#define IRIDISCOPE_ENGINE_H

#include <atomic>
#include <cstdint>
#include <deque>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

#include "core/file_source.h"
#include "iridium/pipeline_c.h"
#include "messages/acars.h"
#include "parser/iridium_parser.h"
#include "voice/voice_manager.h"

namespace iridi {

struct FrameEvent {
    double frequency = 0;
    uint64_t timestamp_ns = 0;
    int confidence = 0;
    int direction = 0;
    int n_bits = 0;
};

struct Satellite {
    int id = 0;
    double lat = 0, lon = 0;
    int alt_km = 0;
    int beam = 0;
    uint64_t last_seen_ns = 0;
    int count = 0;
    bool has_pos = false;   // true once a satellite ring-alert supplied a position
};

struct EngineStats {
    uint64_t samples = 0;
    uint64_t total_samples = 0;
    uint64_t bursts = 0;
    uint64_t frames_ok = 0;
    uint64_t frames_fail = 0;
    double elapsed_s = 0;
    double rate_sps = 0;
    bool running = false;
    bool paused = false;
    bool done = false;
};

class Engine {
public:
    Engine() = default;
    ~Engine();

    bool start(IqFileSource *source, float threshold = 16.0f);
    void stop();

    bool running() const { return running_.load(); }
    bool done() const { return done_.load(); }

    void set_paused(bool p) { paused_.store(p); }
    bool paused() const { return paused_.load(); }
    void set_realtime(bool r) { realtime_.store(r); }
    bool realtime() const { return realtime_.load(); }

    EngineStats stats() const;

    // Spectra (magnitude dB, fftshifted). Empty until the first block.
    std::vector<float> spectrum_inst() const;
    std::vector<float> spectrum_avg() const;
    int spectrum_size() const { return spectrum_size_.load(); }
    uint64_t spectrum_seq() const { return spectrum_seq_.load(); }
    float spectrum_floor() const { return floor_db_.load(); }
    float spectrum_peak() const { return peak_db_.load(); }

    // Spectrum configuration (forwarded to the pipeline).
    void set_fft_size(int n) { if (pipe_) ir_pipeline_set_fft_size((ir_pipeline_t *)pipe_, n); }
    int  fft_size() const { return pipe_ ? ir_pipeline_get_fft_size((ir_pipeline_t *)pipe_) : 0; }
    void set_avg_alpha(float a) { if (pipe_) ir_pipeline_set_avg_alpha((ir_pipeline_t *)pipe_, a); }
    void set_dc_block(bool on) { if (pipe_) ir_pipeline_set_dc_block((ir_pipeline_t *)pipe_, on ? 1 : 0); }

    std::vector<FrameEvent> recent_frames(size_t max_count) const;

    // Parsed messages and satellites (from the native parser).
    std::vector<irid::ParsedMessage> recent_messages(size_t max_count) const;
    std::vector<Satellite> satellites() const;
    uint64_t parsed_count() const { return parsed_count_.load(); }
    uint64_t parse_error_count() const { return parse_error_count_.load(); }

    const VoiceManager &voices() const { return voice_; }
    const AcarsAssembler &acars() const { return acars_; }

    // Callback target (public so a C trampoline can invoke it).
    void on_frame(const ir_frame_info_t *info);

private:
    void thread_main();

    IqFileSource *source_ = nullptr;
    void *pipe_ = nullptr; // ir_pipeline_t*
    std::thread thread_;

    std::atomic<bool> running_{false};
    std::atomic<bool> paused_{false};
    std::atomic<bool> realtime_{false};
    std::atomic<bool> done_{false};

    std::atomic<uint64_t> samples_{0};
    std::atomic<uint64_t> total_samples_{0};
    std::atomic<uint64_t> spectrum_seq_{0};
    std::atomic<int> spectrum_size_{0};
    std::atomic<float> floor_db_{-120.0f};
    std::atomic<float> peak_db_{-120.0f};

    std::atomic<uint64_t> parsed_count_{0};
    std::atomic<uint64_t> parse_error_count_{0};
    std::atomic<uint64_t> last_ts_{0};

    VoiceManager voice_;
    AcarsAssembler acars_;

    mutable std::mutex mtx_;
    std::vector<float> spectrum_inst_;
    std::vector<float> spectrum_avg_;
    std::deque<FrameEvent> frames_;
    std::deque<irid::ParsedMessage> messages_;
    std::unordered_map<int, Satellite> sats_;
};

} // namespace iridi

#endif // IRIDISCOPE_ENGINE_H
