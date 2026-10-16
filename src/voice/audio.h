// Minimal in-app audio playback (miniaudio) for decoded voice calls.
#ifndef IRIDISCOPE_AUDIO_H
#define IRIDISCOPE_AUDIO_H

#include <atomic>
#include <cstdint>
#include <mutex>
#include <vector>

namespace iridi {

class AudioPlayer {
public:
    AudioPlayer();
    ~AudioPlayer();

    // Plays 8 kHz mono int16 PCM. Replaces any current playback.
    bool play(const std::vector<int16_t> &pcm, int sample_rate = 8000);
    void stop();
    bool playing() const { return playing_.load(); }

    struct Impl; // public so the miniaudio callback can use it

private:
    Impl *impl_ = nullptr;
    std::atomic<bool> playing_{false};
};

} // namespace iridi

#endif // IRIDISCOPE_AUDIO_H
