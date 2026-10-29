// Native Airspy source backend. Adapted from AirScope.
#include "sdr/sdr_source.h"

#ifdef HAS_AIRSPY

#include "sdr/airspy_source.h"

#include <airspy.h>
#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <string>

namespace iridi {

namespace {

void ensureAirspyInit() {
    static std::once_flag once;
    std::call_once(once, []() { airspy_init(); });
}

int rxTrampoline(airspy_transfer_t *transfer) {
#ifdef _WIN32
    // libairspy calls this from its conversion thread (USB data -> IQ, the
    // heavy filtering at 10 Msps). If that thread is starved its queue fills
    // and samples are lost "in the driver", so it gets top priority.
    static thread_local bool prio_set = false;
    if (!prio_set) { SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST); prio_set = true; }
#endif
    auto *self = static_cast<AirspySource *>(transfer->ctx);
    int n = transfer->sample_count;
    // libairspy reports samples it discarded (queue full) before this transfer.
    self->addDriverDropped(transfer->dropped_samples);
    // Airspy outputs interleaved float IQ in [-1, 1] for FLOAT32_IQ.
    return self->handleRx((float *)transfer->samples, n);
}

// Sample rates the open device reports, highest first. The Airspy R2 reports
// 10 / 2.5 MSPS, the Airspy Mini 6 / 3 MSPS (newer firmware may add more).
std::vector<uint32_t> queryRates(airspy_device *dev) {
    std::vector<uint32_t> rates;
    uint32_t count = 0;
    if (airspy_get_samplerates(dev, &count, 0) != AIRSPY_SUCCESS || count == 0 || count > 64)
        return rates;
    rates.resize(count);
    if (airspy_get_samplerates(dev, rates.data(), count) != AIRSPY_SUCCESS) return {};
    std::sort(rates.rbegin(), rates.rend());
    rates.erase(std::unique(rates.begin(), rates.end()), rates.end());
    return rates;
}

// Model from the firmware string ("AirSpy MINI ..." on the Mini), falling
// back to the rate list (only the R2 offers 10 MSPS).
std::string modelName(airspy_device *dev, const std::vector<uint32_t> &rates) {
    char ver[128] = {0};
    if (airspy_version_string_read(dev, ver, (uint8_t)(sizeof(ver) - 1)) == AIRSPY_SUCCESS) {
        std::string up(ver);
        for (auto &c : up) c = (char)std::toupper((unsigned char)c);
        if (up.find("MINI") != std::string::npos) return "Airspy Mini";
        if (up.find("NOS") != std::string::npos || up.find("R2") != std::string::npos) return "Airspy R2";
    }
    if (!rates.empty())
        return std::find(rates.begin(), rates.end(), 10000000u) != rates.end() ? "Airspy R2" : "Airspy Mini";
    return "Airspy";
}

// Requested rate if supported, else the highest supported rate at or below
// it, else the lowest supported rate.
uint32_t pickRate(const std::vector<uint32_t> &rates, double requested, bool allowUnofficial) {
    if (rates.empty()) return (uint32_t)std::llround(requested);
    const uint32_t want = (uint32_t)std::llround(requested);
    // Unadvertised 10 Msps: libairspy passes an unlisted rate straight to the
    // firmware (ADC at 20 MHz), which the Mini accepts.
    if (allowUnofficial && want == 10000000u) return want;
    for (uint32_t r : rates) if (r == want) return r;
    for (uint32_t r : rates) if (r <= want) return r; // rates are high -> low
    return rates.back();
}

} // namespace

AirspySource::AirspySource() { ensureAirspyInit(); }

AirspySource::~AirspySource() { stop(); }

std::vector<SdrDeviceInfo> AirspySource::listDevices() {
    ensureAirspyInit();
    std::vector<SdrDeviceInfo> out;
    uint64_t serials[256];
    int n = airspy_list_devices(serials, 256);
    for (int i = 0; i < n; ++i) {
        SdrDeviceInfo info;
        info.index = i;
        info.name = "Airspy";
        // Identify the model and its sample rates. A device that is already
        // streaming cannot be opened again, so keep what we know about it.
        airspy_device *probe = nullptr;
        if (dev_ && serials[i] == selectedSerial_) {
            info.name = modelName(dev_, deviceRates_);
            for (uint32_t r : deviceRates_) info.rates.push_back((double)r);
        } else if (airspy_open_sn(&probe, serials[i]) == AIRSPY_SUCCESS && probe) {
            airspy_set_sample_type(probe, AIRSPY_SAMPLE_FLOAT32_IQ);
            const auto rates = queryRates(probe);
            info.name = modelName(probe, rates);
            for (uint32_t r : rates) info.rates.push_back((double)r);
            airspy_close(probe);
        }
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%016llX", (unsigned long long)serials[i]);
        info.serial = buf;
        info.name += " (" + info.serial.substr(info.serial.size() - 8) + ")";
        out.push_back(info);
    }
    return out;
}

bool AirspySource::start(int deviceIndex, SdrSampleCb cb, std::string &err) {
    if (running_.load()) return true;
    ensureAirspyInit();

    uint64_t serials[256];
    int nDevs = airspy_list_devices(serials, 256);
    if (nDevs <= 0) {
        err = "No Airspy device found";
        return false;
    }
    if (deviceIndex < 0 || deviceIndex >= nDevs) deviceIndex = 0;
    selectedSerial_ = serials[deviceIndex];

    int r = airspy_open_sn(&dev_, selectedSerial_);
    if (r != AIRSPY_SUCCESS || !dev_) {
        err = "Airspy open failed: " + std::string(airspy_error_name((airspy_error)r));
        dev_ = nullptr;
        return false;
    }

    airspy_set_sample_type(dev_, AIRSPY_SAMPLE_FLOAT32_IQ);
    deviceRates_ = queryRates(dev_);
    sampleRate_ = (double)pickRate(deviceRates_, sampleRate_, allowUnofficial_);
    r = airspy_set_samplerate(dev_, (uint32_t)sampleRate_);
    if (r != AIRSPY_SUCCESS) {
        err = "Airspy rejected " + std::to_string(sampleRate_ / 1e6) + " Msps: " +
              std::string(airspy_error_name((airspy_error)r));
        airspy_close(dev_);
        dev_ = nullptr;
        return false;
    }
    airspy_set_freq(dev_, tunedHz());
    applyGain();
    airspy_set_rf_bias(dev_, biasTee_ ? 1 : 0);

    cb_ = std::move(cb);
    driverDropped_.store(0);
    dcOffRe_ = dcOffIm_ = 0.0f;
    dcRate_ = (float)(50.0 / sampleRate_);

    r = airspy_start_rx(dev_, &rxTrampoline, this);
    if (r != AIRSPY_SUCCESS) {
        err = "Airspy start_rx failed: " + std::string(airspy_error_name((airspy_error)r));
        airspy_close(dev_);
        dev_ = nullptr;
        cb_ = nullptr;
        return false;
    }
    running_.store(true);
    return true;
}

void AirspySource::stop() {
    if (dev_) {
        airspy_stop_rx(dev_);
        airspy_set_rf_bias(dev_, 0);
        airspy_close(dev_);
        dev_ = nullptr;
    }
    deviceRates_.clear();
    running_.store(false);
    cb_ = nullptr;
}

int AirspySource::handleRx(float *buf, int nComplex) {
    if (!running_.load() || !cb_ || nComplex <= 0) return 0;

    if ((int)scratch_.size() < nComplex * 2) scratch_.resize((size_t)nComplex * 2);
    float *out = scratch_.data();

    float offRe = dcOffRe_, offIm = dcOffIm_;
    const float rate = dcRate_;
    for (int i = 0; i < nComplex; ++i) {
        float re = buf[i * 2];
        float im = buf[i * 2 + 1];
        float ore = re - offRe;
        offRe += ore * rate;
        float oim = im - offIm;
        offIm += oim * rate;
        out[i * 2] = ore;
        out[i * 2 + 1] = oim;
    }
    dcOffRe_ = offRe;
    dcOffIm_ = offIm;

    cb_(out, nComplex);
    return 0;
}

uint32_t AirspySource::tunedHz() const {
    return (uint32_t)std::llround(centerFreq_ * (1.0 + ppm_ / 1e6));
}

void AirspySource::applyTune() {
    if (dev_) airspy_set_freq(dev_, tunedHz());
}

void AirspySource::setCenterFreq(double hz) {
    centerFreq_ = hz;
    applyTune();
}

void AirspySource::setPpm(double ppm) {
    ppm_ = ppm;
    applyTune();
}

void AirspySource::setSampleRate(double hz) {
    sampleRate_ = hz;
    applyRate();
}

void AirspySource::applyRate() {
    if (!dev_) return; // validated against the device in start()
    const uint32_t rate = pickRate(deviceRates_, sampleRate_, allowUnofficial_);
    // Change rate with streaming paused rather than mid-transfer.
    const bool streaming = airspy_is_streaming(dev_) == AIRSPY_TRUE;
    if (streaming) airspy_stop_rx(dev_);
    if (airspy_set_samplerate(dev_, rate) == AIRSPY_SUCCESS) sampleRate_ = (double)rate;
    dcRate_ = (float)(50.0 / sampleRate_);
    if (streaming) airspy_start_rx(dev_, &rxTrampoline, this);
}

void AirspySource::setGain(double db) {
    int v = (int)std::lround(db);
    if (v < 0) v = 0;
    if (v > 15) v = 15;
    setVgaGain(v);
}

void AirspySource::setBiasTee(bool on) {
    biasTee_ = on;
    if (dev_) airspy_set_rf_bias(dev_, on ? 1 : 0);
}

void AirspySource::setGainMode(int mode) {
    if (mode < 0) mode = 0;
    if (mode > 2) mode = 2;
    gainMode_ = mode;
    if (dev_) applyGain();
}

void AirspySource::setSenseGain(int val) {
    senseGain_ = std::clamp(val, 0, 21);
    if (dev_ && gainMode_ == 0) airspy_set_sensitivity_gain(dev_, (uint8_t)senseGain_);
}

void AirspySource::setLinearGain(int val) {
    linearGain_ = std::clamp(val, 0, 21);
    if (dev_ && gainMode_ == 1) airspy_set_linearity_gain(dev_, (uint8_t)linearGain_);
}

void AirspySource::setLnaGain(int val) {
    lnaGain_ = std::clamp(val, 0, 15);
    if (dev_ && gainMode_ == 2 && !lnaAgc_) airspy_set_lna_gain(dev_, (uint8_t)lnaGain_);
}

void AirspySource::setMixerGain(int val) {
    mixerGain_ = std::clamp(val, 0, 15);
    if (dev_ && gainMode_ == 2 && !mixerAgc_) airspy_set_mixer_gain(dev_, (uint8_t)mixerGain_);
}

void AirspySource::setVgaGain(int val) {
    vgaGain_ = std::clamp(val, 0, 15);
    if (dev_ && gainMode_ == 2) airspy_set_vga_gain(dev_, (uint8_t)vgaGain_);
}

void AirspySource::setLnaAgc(bool on) {
    lnaAgc_ = on;
    if (dev_ && gainMode_ == 2) {
        airspy_set_lna_agc(dev_, on ? 1 : 0);
        if (!on) airspy_set_lna_gain(dev_, (uint8_t)lnaGain_);
    }
}

void AirspySource::setMixerAgc(bool on) {
    mixerAgc_ = on;
    if (dev_ && gainMode_ == 2) {
        airspy_set_mixer_agc(dev_, on ? 1 : 0);
        if (!on) airspy_set_mixer_gain(dev_, (uint8_t)mixerGain_);
    }
}

void AirspySource::applyGain() {
    if (!dev_) return;
    switch (gainMode_) {
    case 0: // Sensitivity
        airspy_set_lna_agc(dev_, 0);
        airspy_set_mixer_agc(dev_, 0);
        airspy_set_sensitivity_gain(dev_, (uint8_t)senseGain_);
        break;
    case 1: // Linear
        airspy_set_lna_agc(dev_, 0);
        airspy_set_mixer_agc(dev_, 0);
        airspy_set_linearity_gain(dev_, (uint8_t)linearGain_);
        break;
    case 2: // Free
        if (lnaAgc_) {
            airspy_set_lna_agc(dev_, 1);
        } else {
            airspy_set_lna_agc(dev_, 0);
            airspy_set_lna_gain(dev_, (uint8_t)lnaGain_);
        }
        if (mixerAgc_) {
            airspy_set_mixer_agc(dev_, 1);
        } else {
            airspy_set_mixer_agc(dev_, 0);
            airspy_set_mixer_gain(dev_, (uint8_t)mixerGain_);
        }
        airspy_set_vga_gain(dev_, (uint8_t)vgaGain_);
        break;
    }
}

std::unique_ptr<SdrSource> make_airspy_source() {
    return std::unique_ptr<SdrSource>(new AirspySource());
}

} // namespace iridi

#endif // HAS_AIRSPY
