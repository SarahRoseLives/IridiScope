#include "voice/voice_manager.h"

#include <cmath>
#include <cstdio>
#include <cstring>

#if defined(_WIN32)
#include <direct.h>
#endif

extern "C" {
#include "ambe.h" // osmo-ir77 (C linkage)
}

namespace iridi {

struct VoiceManager::Active {
    int id = 0;
    double channel = 0;
    uint64_t start = 0, last = 0;
    struct ir77_ambe_decoder *dec = nullptr;
    std::vector<int16_t> pcm;
    int frames = 0; // subframes (2 per superframe)
    int ok = 0;
};

VoiceManager::VoiceManager() {
#if defined(_WIN32)
    _mkdir("recordings");
#else
    mkdir("recordings", 0755);
#endif
}

VoiceManager::~VoiceManager() {
    std::lock_guard<std::mutex> lk(mtx_);
    for (auto &a : active_)
        if (a->dec) ir77_ambe_decode_release(a->dec);
}

void VoiceManager::reset() {
    std::lock_guard<std::mutex> lk(mtx_);
    for (auto &a : active_)
        if (a->dec) ir77_ambe_decode_release(a->dec);
    active_.clear();
    done_.clear();
    done_pcm_.clear();
    next_id_ = 0;
}

static void write_wav(const std::string &path, const std::vector<int16_t> &pcm, int rate) {
    FILE *f = fopen(path.c_str(), "wb");
    if (!f) return;
    uint32_t data_bytes = (uint32_t)(pcm.size() * 2);
    uint32_t riff = 36 + data_bytes;
    uint16_t block_align = 2, bits = 16, fmt = 1, ch = 1;
    uint32_t byte_rate = rate * block_align;
    fwrite("RIFF", 1, 4, f); fwrite(&riff, 4, 1, f); fwrite("WAVE", 1, 4, f);
    fwrite("fmt ", 1, 4, f); uint32_t fsz = 16; fwrite(&fsz, 4, 1, f);
    fwrite(&fmt, 2, 1, f); fwrite(&ch, 2, 1, f); fwrite(&rate, 4, 1, f);
    fwrite(&byte_rate, 4, 1, f); fwrite(&block_align, 2, 1, f); fwrite(&bits, 2, 1, f);
    fwrite("data", 1, 4, f); fwrite(&data_bytes, 4, 1, f);
    fwrite(pcm.data(), 1, data_bytes, f);
    fclose(f);
}

void VoiceManager::finalize(std::unique_ptr<Active> &a, bool have_end) {
    VoiceCall c;
    c.id = a->id;
    c.channel_hz = a->channel;
    c.start_ns = a->start;
    c.end_ns = a->last;
    c.frames = a->frames;
    c.error_ratio = a->frames > 0 ? (1.0 - (double)a->ok / (double)a->frames) : 1.0;
    c.duration_s = (double)a->pcm.size() / 8000.0;
    if (!have_end && c.error_ratio > 0.6) {
        // Rejected/aborted: keep the record but flag low quality via error ratio.
    }
    if (!a->pcm.empty()) {
        char path[128];
        snprintf(path, sizeof(path), "recordings/voice-%04d.wav", a->id);
        write_wav(path, a->pcm, 8000);
        c.wav_path = path;
    }
    done_.push_back(c);
    done_pcm_.emplace_back(a->id, std::move(a->pcm));
    if (done_.size() > 500) done_.pop_front();
    if (done_pcm_.size() > 500) done_pcm_.pop_front();
    if (a->dec) ir77_ambe_decode_release(a->dec);
}

void VoiceManager::feed(const irid::ParsedMessage &pm) {
    if (pm.voice_payload.size() != 39) return;
    std::lock_guard<std::mutex> lk(mtx_);

    Active *a = nullptr;
    for (auto &c : active_) {
        if (std::fabs(c->channel - pm.frequency) < 20000.0 &&
            (pm.timestamp_ns - c->last) < 20000000000ULL) {
            a = c.get();
            break;
        }
    }
    if (!a) {
        auto up = std::make_unique<Active>();
        up->id = next_id_++;
        up->channel = pm.frequency;
        up->start = pm.timestamp_ns;
        up->last = pm.timestamp_ns;
        up->dec = ir77_ambe_decode_alloc();
        if (!up->dec) return;
        a = up.get();
        active_.push_back(std::move(up));
    }

    // The AMBE superframe is the payload with each byte's bits reversed
    // (matches iridium-toolkit's bits_to_dfs.py before ir77_ambe_decode).
    uint8_t sf[39];
    for (int i = 0; i < 39; i++) {
        uint8_t b = pm.voice_payload[i], r = 0;
        for (int k = 0; k < 8; k++) r = (uint8_t)((r << 1) | ((b >> k) & 1));
        sf[i] = r;
    }
    int16_t audio[720];
    int rv = ir77_ambe_decode_superframe(a->dec, audio, 720, sf);
    if (rv >= 0) {
        a->pcm.insert(a->pcm.end(), audio, audio + 720);
        a->frames += 2;
        a->ok += rv;
    }
    a->last = pm.timestamp_ns;
}

void VoiceManager::sweep(uint64_t now_ns) {
    std::lock_guard<std::mutex> lk(mtx_);
    for (size_t i = 0; i < active_.size();) {
        if (now_ns > active_[i]->last && (now_ns - active_[i]->last) > 3000000000ULL) {
            finalize(active_[i], true);
            active_.erase(active_.begin() + i);
        } else ++i;
    }
}

std::vector<VoiceCall> VoiceManager::calls() const {
    std::lock_guard<std::mutex> lk(mtx_);
    std::vector<VoiceCall> out(done_.begin(), done_.end());
    for (const auto &a : active_) {
        VoiceCall c;
        c.id = a->id; c.channel_hz = a->channel; c.start_ns = a->start; c.end_ns = a->last;
        c.frames = a->frames;
        c.error_ratio = a->frames > 0 ? (1.0 - (double)a->ok / (double)a->frames) : 1.0;
        c.active = true;
        c.duration_s = (double)a->pcm.size() / 8000.0;
        out.push_back(c);
    }
    return out;
}

std::vector<int16_t> VoiceManager::pcm(int id) const {
    std::lock_guard<std::mutex> lk(mtx_);
    for (const auto &kv : done_pcm_)
        if (kv.first == id) return kv.second;
    for (const auto &a : active_)
        if (a->id == id) return a->pcm;
    return {};
}

} // namespace iridi
