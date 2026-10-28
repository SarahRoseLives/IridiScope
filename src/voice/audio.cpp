#include "voice/audio.h"

#include "miniaudio.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace iridi {

struct AudioPlayer::Impl {
    ma_device device{};
    bool inited = false;
    int rate = 8000;
    std::mutex mtx;
    std::vector<int16_t> pcm;
    std::atomic<size_t> pos{0};
    std::atomic<bool> *playing = nullptr;
    std::atomic<bool> *paused = nullptr;
};

static void data_callback(ma_device *dev, void *out, const void * /*in*/, ma_uint32 frames) {
    auto *im = (AudioPlayer::Impl *)dev->pUserData;
    int16_t *o = (int16_t *)out;
    if (im->paused && im->paused->load()) {
        std::memset(o, 0, frames * sizeof(int16_t));
        return;
    }
    size_t p = im->pos.load();
    size_t n = im->pcm.size();
    for (ma_uint32 i = 0; i < frames; i++) {
        if (p < n) o[i] = im->pcm[p++];
        else o[i] = 0;
    }
    im->pos.store(p);
    if (p >= n && im->playing) im->playing->store(false);
}

AudioPlayer::AudioPlayer() : impl_(new Impl()) {
    impl_->playing = &playing_;
    impl_->paused = &paused_;
}

AudioPlayer::~AudioPlayer() {
    if (impl_) {
        if (impl_->inited) ma_device_uninit(&impl_->device);
        delete impl_;
    }
}

bool AudioPlayer::play(const std::vector<int16_t> &pcm, int sample_rate) {
    if (!impl_ || pcm.empty()) return false;

    // Peak-normalise quiet calls (up to 6x) and then apply the user volume.
    int peak = 0;
    for (int16_t s : pcm) peak = std::max(peak, std::abs((int)s));
    float norm = 1.0f;
    if (peak > 0) {
        norm = 30000.0f / (float)peak;
        norm = std::clamp(norm, 1.0f, 6.0f);
    }
    float gain = norm * volume_.load();

    {
        std::lock_guard<std::mutex> lk(impl_->mtx);
        impl_->pcm.resize(pcm.size());
        for (size_t i = 0; i < pcm.size(); i++) {
            int v = (int)std::lround((double)pcm[i] * gain);
            impl_->pcm[i] = (int16_t)std::clamp(v, -32768, 32767);
        }
        impl_->pos.store(0);
        impl_->rate = sample_rate;
    }

    if (!impl_->inited || impl_->device.sampleRate != (ma_uint32)sample_rate) {
        if (impl_->inited) { ma_device_uninit(&impl_->device); impl_->inited = false; }
        ma_device_config cfg = ma_device_config_init(ma_device_type_playback);
        cfg.playback.format = ma_format_s16;
        cfg.playback.channels = 1;
        cfg.sampleRate = (ma_uint32)sample_rate;
        cfg.dataCallback = data_callback;
        cfg.pUserData = impl_;
        if (ma_device_init(nullptr, &cfg, &impl_->device) != MA_SUCCESS) return false;
        impl_->inited = true;
    }
    paused_.store(false);
    playing_.store(true);
    if (ma_device_start(&impl_->device) != MA_SUCCESS) { playing_.store(false); return false; }
    return true;
}

void AudioPlayer::set_volume(float v) {
    volume_.store(std::clamp(v, 0.0f, 8.0f));
}

void AudioPlayer::stop() {
    if (impl_ && impl_->inited) ma_device_stop(&impl_->device);
    playing_.store(false);
    paused_.store(false);
}

void AudioPlayer::pause()  { if (playing_.load()) paused_.store(true); }
void AudioPlayer::resume() { paused_.store(false); }

void AudioPlayer::seek(float sec) {
    if (!impl_ || !impl_->inited || impl_->pcm.empty()) return;
    const size_t n = impl_->pcm.size();
    size_t p = (size_t)std::max(0.0, (double)sec * impl_->rate);
    if (p >= n) p = n - 1;
    impl_->pos.store(p);
    if (!playing_.load()) {
        playing_.store(true);
        ma_device_start(&impl_->device);   // no-op if still running
    }
}

float AudioPlayer::position_sec() const {
    if (!impl_ || impl_->rate <= 0) return 0.0f;
    return (float)((double)impl_->pos.load() / impl_->rate);
}

} // namespace iridi
