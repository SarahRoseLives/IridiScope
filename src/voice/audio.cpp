#include "voice/audio.h"

#include "miniaudio.h"

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
};

static void data_callback(ma_device *dev, void *out, const void * /*in*/, ma_uint32 frames) {
    auto *im = (AudioPlayer::Impl *)dev->pUserData;
    int16_t *o = (int16_t *)out;
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
}

AudioPlayer::~AudioPlayer() {
    if (impl_) {
        if (impl_->inited) ma_device_uninit(&impl_->device);
        delete impl_;
    }
}

bool AudioPlayer::play(const std::vector<int16_t> &pcm, int sample_rate) {
    if (!impl_ || pcm.empty()) return false;

    {
        std::lock_guard<std::mutex> lk(impl_->mtx);
        impl_->pcm = pcm;
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
    playing_.store(true);
    if (ma_device_start(&impl_->device) != MA_SUCCESS) { playing_.store(false); return false; }
    return true;
}

void AudioPlayer::stop() {
    if (impl_ && impl_->inited) ma_device_stop(&impl_->device);
    playing_.store(false);
}

} // namespace iridi
