// Voice-call capture: clusters VOC/VOD/VO6 frames per channel and decodes the
// AMBE superframes to 8 kHz PCM via the osmo-ir77 vocoder.
#ifndef IRIDISCOPE_VOICE_MANAGER_H
#define IRIDISCOPE_VOICE_MANAGER_H

#include <cstdint>
#include <atomic>
#include <condition_variable>
#include <thread>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "parser/iridium_parser.h"

namespace iridi {

struct VoiceCall {
    int id = 0;
    double channel_hz = 0;
    uint64_t start_ns = 0, end_ns = 0;
    int frames = 0;
    double error_ratio = 1.0;
    bool active = false;
    std::string subtype; // VOC / VOD / VOZ / ...
    std::string wav_path;
    std::string announcement; // network announcement this call matched ("" = a real call)
    bool classified = false;  // announcement check done (saved calls only)
    double duration_s = 0;
    int handoffs = 0;    // channel changes followed via LCW handoff responses
    int missing = 0;     // superframes never received (filled with silence)
    double speech_s = 0; // seconds of actual speech (silence updates excluded)
};

class VoiceManager {
public:
    VoiceManager();
    ~VoiceManager();

    void reset();
    // Finish every active call now (saved like a normal end). Called when
    // decoding stops (Stop, new source, exit) so calls in progress are kept.
    void flush_all();
    // Thread-safe: called from pipeline worker threads.
    void feed(const irid::ParsedMessage &pm);
    // Close calls idle for > threshold. Call periodically.
    void sweep(uint64_t now_ns);
    // Snapshot for the UI.
    std::vector<VoiceCall> calls() const;
    std::vector<int16_t> pcm(int id) const;
    int sample_rate() const { return 8000; }
    // Follow calls across handoffs (LCW handoff_resp gives the next channel).
    void set_follow_handoffs(bool on) { follow_handoffs_.store(on); }
    // File calls that match a known network announcement under
    // recordings/announcements/<name>/ (on by default).
    void set_sort_announcements(bool on) { sort_ann_.store(on); }
    uint64_t announcements() const { return ann_count_.load(); }
    bool follow_handoffs() const { return follow_handoffs_.load(); }
    // Handoff diagnostics: responses seen on active calls, handoffs followed,
    // and log lines comparing predicted vs actual new channels.
    uint64_t handoffs_seen() const { return ho_seen_.load(); }
    uint64_t handoffs_followed() const { return ho_followed_.load(); }
    std::vector<std::string> drain_diagnostics();
    // Session totals: calls with at least 1 s of speech, their speech time,
    // and the longest one.
    uint64_t speech_calls() const { return speech_calls_.load(); }
    double speech_total_s() const { return speech_ms_.load() / 1000.0; }
    double longest_speech_s() const { return longest_ms_.load() / 1000.0; }

private:
    struct Active;
    struct SaveJob {
        std::string path;
        std::vector<int16_t> pcm;
        std::string csv_line;
        VoiceCall call;   // for the message database
    };
    void finalize(std::unique_ptr<Active> &a, bool have_end);
    void decode_ready(Active &a, bool all);
    static void close_run(Active &a, long long gap_after);
    void writer_main();

    mutable std::mutex mtx_;
    std::vector<std::unique_ptr<Active>> active_;
    std::atomic<uint64_t> speech_calls_{0}, speech_ms_{0}, longest_ms_{0};
    std::atomic<bool> sort_ann_{true};
    std::atomic<uint64_t> ann_count_{0};
    std::deque<VoiceCall> done_;
    std::deque<std::pair<int, std::vector<int16_t>>> done_pcm_;
    int next_id_ = 0;
    std::atomic<bool> follow_handoffs_{true};
    std::atomic<uint64_t> ho_seen_{0}, ho_followed_{0};
    struct PendingHo {
        double from = 0, nominal = 0, predicted = 0;
        int sb = 0, fa = 0;
        int call_id = -1;
        uint64_t ts = 0;
        bool resolved = false;
    };
    std::deque<PendingHo> pending_ho_;
    std::deque<std::string> diag_;

    // Recordings are written by a separate thread so disk I/O never holds up
    // the decoder workers or the detector.
    std::mutex wmtx_;
    std::condition_variable wcv_;
    std::deque<SaveJob> wqueue_;
    bool wstop_ = false;
    std::thread writer_;
};

} // namespace iridi

#endif // IRIDISCOPE_VOICE_MANAGER_H
