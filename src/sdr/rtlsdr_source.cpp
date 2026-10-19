// RTL-SDR source backend.
#include "sdr/sdr_source.h"

#ifdef HAS_RTLSDR

#include "sdr/rtlsdr_source.h"

#include <rtl-sdr.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

namespace iridi {

namespace {

void readTrampoline(unsigned char *buf, uint32_t len, void *ctx) {
    static_cast<RtlSdrSource *>(ctx)->handleRx(buf, len);
}

} // namespace

RtlSdrSource::~RtlSdrSource() { stop(); }

std::vector<SdrDeviceInfo> RtlSdrSource::listDevices() {
    std::vector<SdrDeviceInfo> out;
    int n = rtlsdr_get_device_count();
    for (int i = 0; i < n; ++i) {
        SdrDeviceInfo info;
        info.index = i;
        const char *name = rtlsdr_get_device_name((uint32_t)i);
        info.name = name ? name : "RTL-SDR";
        char manufact[256] = {0}, product[256] = {0}, serial[256] = {0};
        if (rtlsdr_get_device_usb_strings((uint32_t)i, manufact, product, serial) == 0) {
            if (serial[0]) info.serial = serial;
            if (product[0]) info.name = product;
        }
        out.push_back(info);
    }
    return out;
}

void RtlSdrSource::applyTune() {
    if (dev_)
        rtlsdr_set_center_freq(dev_, (uint32_t)std::llround(centerFreq_));
}

void RtlSdrSource::applyGain() {
    if (!dev_) return;
    if (gainDb_ < 0.0) {
        rtlsdr_set_tuner_gain_mode(dev_, 0); // AGC
        return;
    }
    rtlsdr_set_tuner_gain_mode(dev_, 1);
    int count = rtlsdr_get_tuner_gains(dev_, nullptr);
    if (count > 0) {
        std::vector<int> gains((size_t)count);
        if (rtlsdr_get_tuner_gains(dev_, gains.data()) == count) {
            int want = (int)std::lround(gainDb_ * 10.0);
            int best = gains[0];
            for (int g : gains)
                if (std::abs(g - want) < std::abs(best - want)) best = g;
            rtlsdr_set_tuner_gain(dev_, best);
            return;
        }
    }
    rtlsdr_set_tuner_gain(dev_, (int)std::lround(gainDb_ * 10.0));
}

bool RtlSdrSource::start(int deviceIndex, SdrSampleCb cb, std::string &err) {
    if (running_.load()) return true;

    if (rtlsdr_open(&dev_, (uint32_t)deviceIndex) < 0) {
        dev_ = nullptr;
        err = "RTL-SDR open failed (device busy or not found)";
        return false;
    }

    rtlsdr_set_sample_rate(dev_, (uint32_t)std::llround(sampleRate_));
    applyTune();
    rtlsdr_set_freq_correction(dev_, (int)std::llround(ppm_));
    applyGain();
    rtlsdr_set_bias_tee(dev_, biasTee_ ? 1 : 0);
    rtlsdr_reset_buffer(dev_);

    cb_ = std::move(cb);
    dcOffRe_ = dcOffIm_ = 0.0f;
    dcRate_ = (float)(50.0 / (sampleRate_ > 0 ? sampleRate_ : 1.0));
    running_.store(true);
    thread_ = std::thread([this] {
        rtlsdr_read_async(dev_, &readTrampoline, this, 0, 0);
    });
    return true;
}

void RtlSdrSource::stop() {
    if (dev_) {
        running_.store(false);
        rtlsdr_cancel_async(dev_);
        if (thread_.joinable()) thread_.join();
        rtlsdr_close(dev_);
        dev_ = nullptr;
    } else if (thread_.joinable()) {
        thread_.join();
    }
    running_.store(false);
    cb_ = nullptr;
}

void RtlSdrSource::handleRx(unsigned char *buf, uint32_t len) {
    if (!running_.load() || !cb_ || len < 2) return;
    int n = (int)(len / 2);
    if ((int)scratch_.size() < n * 2) scratch_.resize((size_t)n * 2);
    float *out = scratch_.data();

    float offRe = dcOffRe_, offIm = dcOffIm_;
    const float rate = dcRate_;
    for (int i = 0; i < n; ++i) {
        float re = ((float)buf[i * 2] - 127.5f) * (1.0f / 127.5f);
        float im = ((float)buf[i * 2 + 1] - 127.5f) * (1.0f / 127.5f);
        float ore = re - offRe;
        offRe += ore * rate;
        float oim = im - offIm;
        offIm += oim * rate;
        out[i * 2] = ore;
        out[i * 2 + 1] = oim;
    }
    dcOffRe_ = offRe;
    dcOffIm_ = offIm;

    cb_(out, n);
}

void RtlSdrSource::setCenterFreq(double hz) {
    centerFreq_ = hz;
    applyTune();
}

void RtlSdrSource::setSampleRate(double hz) {
    sampleRate_ = hz;
    dcRate_ = (float)(50.0 / (hz > 0 ? hz : 1.0));
    if (dev_) {
        rtlsdr_set_sample_rate(dev_, (uint32_t)std::llround(hz));
        rtlsdr_reset_buffer(dev_);
    }
}

void RtlSdrSource::setGain(double db) {
    gainDb_ = db;
    applyGain();
}

void RtlSdrSource::setBiasTee(bool on) {
    biasTee_ = on;
    if (dev_) rtlsdr_set_bias_tee(dev_, on ? 1 : 0);
}

void RtlSdrSource::setPpm(double ppm) {
    ppm_ = ppm;
    if (dev_) rtlsdr_set_freq_correction(dev_, (int)std::llround(ppm));
}

std::unique_ptr<SdrSource> make_rtlsdr_source() {
    return std::unique_ptr<SdrSource>(new RtlSdrSource());
}

} // namespace iridi

#endif // HAS_RTLSDR
