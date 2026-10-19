// Voice-call capture: clusters VOC/VOD/VO6 frames per channel and decodes the
// AMBE superframes to 8 kHz PCM via the osmo-ir77 vocoder.
#ifndef IRIDISCOPE_VOICE_MANAGER_H
#define IRIDISCOPE_VOICE_MANAGER_H

#include <cstdint>
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
    double duration_s = 0;
};

class VoiceManager {
public:
    VoiceManager();
    ~VoiceManager();

    void reset();
    // Thread-safe: called from pipeline worker threads.
    void feed(const irid::ParsedMessage &pm);
    // Close calls idle for > threshold. Call periodically.
    void sweep(uint64_t now_ns);
    // Snapshot for the UI.
    std::vector<VoiceCall> calls() const;
    std::vector<int16_t> pcm(int id) const;
    int sample_rate() const { return 8000; }

private:
    struct Active;
    void finalize(std::unique_ptr<Active> &a, bool have_end);

    mutable std::mutex mtx_;
    std::vector<std::unique_ptr<Active>> active_;
    std::deque<VoiceCall> done_;
    std::deque<std::pair<int, std::vector<int16_t>>> done_pcm_;
    int next_id_ = 0;
};

} // namespace iridi

#endif // IRIDISCOPE_VOICE_MANAGER_H
