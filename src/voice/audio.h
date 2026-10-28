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
    // True from play() until the end or stop() - including while paused.
    bool playing() const { return playing_.load(); }

    // Pause/resume in place (position kept).
    void pause();
    void resume();
    bool paused() const { return paused_.load(); }
    // Jump to a position in the current call; plays on from there (also
    // restarts a call that has finished). Keeps the paused state.
    void seek(float sec);
    float position_sec() const;

    // Playback gain (1.0 = unity). Calls are peak-normalised before this.
    void set_volume(float v);
    float volume() const { return volume_.load(); }

    struct Impl; // public so the miniaudio callback can use it

private:
    Impl *impl_ = nullptr;
    std::atomic<bool> playing_{false};
    std::atomic<bool> paused_{false};
    std::atomic<float> volume_{1.0f};
};

} // namespace iridi

#endif // IRIDISCOPE_AUDIO_H
