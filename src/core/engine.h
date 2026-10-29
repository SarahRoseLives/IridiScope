// Real-time processing engine: drives the vendored DSP pipeline over an IQ
// file on a background thread and exposes live statistics, a spectrum, and
// recent demodulated frames to the UI.
#ifndef IRIDISCOPE_ENGINE_H
#define IRIDISCOPE_ENGINE_H

#include <atomic>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "core/file_source.h"
#include "core/iq_recorder.h"
#include "iridium/pipeline_c.h"
#include "messages/acars.h"
#include "messages/pager.h"
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

// Ground spot-beam position from a ring alert (Iridium reports the beam center
// with a low altitude, alternating with the satellite subpoint).
struct GroundBeam {
    int sat = 0, beam = 0;
    double lat = 0, lon = 0;
    int alt_km = 0;
    uint64_t last_seen_ns = 0;
    int count = 0;
    int pages = 0;          // TMSI pages seen in this beam
    bool has_pos = false;
};

// One TMSI page from a ring alert.
struct PageEvent {
    uint64_t timestamp_ns = 0;
    int sat = 0, beam = 0;
    uint32_t tmsi = 0;
    int msc_id = 0;
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

    bool start(IqSource *source, float threshold = 16.0f);
    void stop();

    bool running() const { return running_.load(); }
    bool done() const { return done_.load(); }

    void set_paused(bool p) { paused_.store(p); }
    bool paused() const { return paused_.load(); }
    void set_realtime(bool r) { realtime_.store(r); }
    bool realtime() const { return realtime_.load(); }

    // Diagnostics: print iridium-toolkit RAW lines for each demodulated frame.
    void set_raw_dump(bool on) { raw_dump_.store(on); }

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
    void set_center_frequency(double hz);
    void reset_noise_floor(); // after gain changes: re-learn detector baseline
    float detector_noise_floor() const; // dBFS/Hz, 0 if not running
    int worker_threads() const;        // downmix/demod workers in the pipeline
    int queue_capacity() const;        // burst queue size (memory-budgeted)
    static void set_queue_budget(double bytes); // applies to the next start()
    // Live radio: seconds the decoders may fall behind before the oldest
    // bursts are skipped (0 = playback, nothing skipped). Next start().
    static void set_live_backlog(double seconds);
    uint64_t skipped_bursts() const;   // bursts skipped (decoders too far behind)
    uint64_t targeted_reads() const;   // expected voice bursts read without a detection
    uint64_t voice_recovered() const { return voice_recovered_.load(); } // voice frames those recovered
    // Detector load accounting (cumulative): busy time and samples processed.
    uint64_t detector_busy_ns() const { return det_busy_ns_.load(); }
    uint64_t detector_samples() const { return det_samples_.load(); }
    // CPU time the detector thread actually used (busy time minus time it
    // spent waiting for a CPU).
    uint64_t detector_cpu_ns() const { return det_cpu_ns_.load(); }
    uint64_t detector_wait_ns() const { return det_wait_ns_.load(); }   // waiting for queue space
    uint64_t gap_samples() const { return gap_samples_.load(); }        // lost samples skipped in time
    // Wall-clock time of the first sample for the next start() (0 = now).
    void set_start_time_ns(uint64_t ns) { start_time_ns_ = ns; }
    // Raw IQ recording of the live radio (see core/iq_recorder.h).
    bool start_iq_recording(const std::string &dir, std::string &err);
    void stop_iq_recording();
    const IqRecorder &iq_recorder() const { return iq_rec_; }
    uint64_t uw_recovered() const { return uw_recovered_.load(); }
    uint64_t uw_tolerated_failed() const { return uw_tolerated_failed_.load(); }
    uint64_t queue_full_count() const; // bursts that waited for a free worker

    std::vector<FrameEvent> recent_frames(size_t max_count) const;

    // Parsed messages and satellites (from the native parser).
    std::vector<irid::ParsedMessage> recent_messages(size_t max_count) const;
    std::vector<Satellite> satellites() const;
    std::vector<GroundBeam> ground_beams() const;
    std::vector<PageEvent> paging_events(size_t max_count) const;
    uint64_t parsed_count() const { return parsed_count_.load(); }
    uint64_t last_timestamp() const { return last_ts_.load(); } // newest frame (ns)
    uint64_t parse_error_count() const { return parse_error_count_.load(); }

    const VoiceManager &voices() const { return voice_; }
    VoiceManager &voices() { return voice_; }
    const AcarsAssembler &acars() const { return acars_; }
    const PagerAssembler &pager() const { return pager_; }

    // Callback target (public so a C trampoline can invoke it).
    void on_frame(const ir_frame_info_t *info);

private:
    void thread_main();

    IqSource *source_ = nullptr;
    void *pipe_ = nullptr; // ir_pipeline_t*
    std::thread thread_;

    std::atomic<bool> running_{false};
    std::atomic<bool> paused_{false};
    std::atomic<bool> realtime_{false};
    std::atomic<bool> done_{false};
    std::atomic<bool> raw_dump_{false};
    std::atomic<uint64_t> raw_base_ns_{0};

    std::atomic<uint64_t> samples_{0};
    std::atomic<uint64_t> total_samples_{0};
    std::atomic<uint64_t> spectrum_seq_{0};
    std::atomic<int> spectrum_size_{0};
    std::atomic<float> floor_db_{-120.0f};
    std::atomic<float> peak_db_{-120.0f};

    std::atomic<uint64_t> parsed_count_{0};
    std::atomic<uint64_t> parse_error_count_{0};
    std::atomic<uint64_t> last_ts_{0};
    std::atomic<uint64_t> uw_recovered_{0};        // good frames recovered by UW tolerance
    std::atomic<uint64_t> uw_tolerated_failed_{0}; // UW tolerated but frame failed later
    std::atomic<uint64_t> det_busy_ns_{0};   // time spent in the detector
    std::atomic<uint64_t> det_samples_{0};   // samples it has processed
    std::atomic<uint64_t> det_cpu_ns_{0};    // detector thread CPU time
    std::atomic<uint64_t> voice_recovered_{0}; // voice frames from targeted reads
    std::atomic<uint64_t> det_wait_ns_{0};   // time blocked on a full burst queue
    std::atomic<uint64_t> gap_samples_{0};   // dropped samples the clock skipped over
    uint64_t start_time_ns_ = 0;
    IqRecorder iq_rec_;

    VoiceManager voice_;
    AcarsAssembler acars_;
    PagerAssembler pager_;

    mutable std::mutex mtx_;
    std::vector<float> spectrum_inst_;
    std::vector<float> spectrum_avg_;
    std::deque<FrameEvent> frames_;
    std::deque<irid::ParsedMessage> messages_;
    std::unordered_map<int, Satellite> sats_;
    std::unordered_map<int, GroundBeam> beams_;
    std::deque<PageEvent> pages_;
};

} // namespace iridi

#endif // IRIDISCOPE_ENGINE_H
