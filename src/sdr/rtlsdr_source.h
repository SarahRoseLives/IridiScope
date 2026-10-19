// RTL-SDR source backend using librtlsdr directly.
#ifndef IRIDISCOPE_RTLSDR_SOURCE_H
#define IRIDISCOPE_RTLSDR_SOURCE_H

#include "sdr/sdr_source.h"

#ifdef HAS_RTLSDR

#include <rtl-sdr.h>

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

namespace iridi {

class RtlSdrSource : public SdrSource {
public:
    RtlSdrSource() = default;
    ~RtlSdrSource() override;

    std::vector<SdrDeviceInfo> listDevices() override;

    void setCenterFreq(double hz) override;
    void setSampleRate(double hz) override;
    void setGain(double db) override;  // <0 => tuner AGC
    void setBiasTee(bool on) override;
    void setPpm(double ppm) override;

    double centerFreq() const override { return centerFreq_; }
    double sampleRate() const override { return sampleRate_; }

    bool start(int deviceIndex, SdrSampleCb cb, std::string &err) override;
    void stop() override;
    bool running() const override { return running_.load(); }

    // Called from the librtlsdr async thread. `len` is the byte count
    // (2 interleaved unsigned 8-bit samples per complex value).
    void handleRx(unsigned char *buf, uint32_t len);

private:
    void applyTune();
    void applyGain();

    rtlsdr_dev_t *dev_ = nullptr;
    std::atomic<bool> running_{false};
    std::thread thread_;
    SdrSampleCb cb_;

    double centerFreq_ = 1621250000.0;
    double sampleRate_ = 2.4e6;
    double ppm_ = 0.0;
    double gainDb_ = 28.0;
    bool biasTee_ = false;

    float dcOffRe_ = 0.0f, dcOffIm_ = 0.0f;
    float dcRate_ = 2.0e-5f;

    std::vector<float> scratch_;
};

} // namespace iridi

#endif // HAS_RTLSDR
#endif // IRIDISCOPE_RTLSDR_SOURCE_H
