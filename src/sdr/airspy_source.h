// Native Airspy source backend using libairspy directly.
// Adapted from AirScope's src/sdr/airspy_source.{h,cpp}.
#ifndef IRIDISCOPE_AIRSPY_SOURCE_H
#define IRIDISCOPE_AIRSPY_SOURCE_H

#include "sdr/sdr_source.h"

#ifdef HAS_AIRSPY

#include <atomic>
#include <cstdint>
#include <string>
#include <vector>

struct airspy_device;

namespace iridi {

class AirspySource : public SdrSource {
public:
    AirspySource();
    ~AirspySource() override;

    std::vector<SdrDeviceInfo> listDevices() override;

    void setCenterFreq(double hz) override;
    void setSampleRate(double hz) override;
    void setGain(double db) override;
    void setBiasTee(bool on) override;
    void setPpm(double ppm) override;

    // Airspy-specific gain controls.
    // Gain mode: 0 = Sensitivity, 1 = Linear, 2 = Free.
    void setGainMode(int mode) override;
    void setSenseGain(int val) override;  // 0-21
    void setLinearGain(int val) override; // 0-21
    void setLnaGain(int val) override;    // 0-15 (free mode)
    void setMixerGain(int val) override;  // 0-15 (free mode)
    void setVgaGain(int val) override;    // 0-15 (free mode)
    void setLnaAgc(bool on) override;
    void setMixerAgc(bool on) override;
    void setAllowUnofficialRates(bool on) override { allowUnofficial_ = on; }
    uint64_t driverDroppedSamples() const override { return driverDropped_.load(); }
    bool reportsDriverDrops() const override { return true; }
    void addDriverDropped(uint64_t n) { if (n) driverDropped_.fetch_add(n); }

    double centerFreq() const override { return centerFreq_; }
    double sampleRate() const override { return sampleRate_; }

    bool start(int deviceIndex, SdrSampleCb cb, std::string &err) override;
    void stop() override;
    bool running() const override { return running_.load(); }

    // Called by the libairspy RX trampoline.
    int handleRx(float *buf, int nComplex);

private:
    void applyGain();
    void applyTune();
    void applyRate();
    uint32_t tunedHz() const;

    std::vector<uint32_t> deviceRates_; // rates the open device supports
    bool allowUnofficial_ = false;      // Mini: permit unadvertised 10 Msps
    std::atomic<uint64_t> driverDropped_{0};

    airspy_device *dev_ = nullptr;
    uint64_t selectedSerial_ = 0;
    std::atomic<bool> running_{false};
    SdrSampleCb cb_;

    double centerFreq_ = 1621250000.0;
    double sampleRate_ = 10.0e6;
    double ppm_ = 0.0;

    int gainMode_ = 0;    // 0=Sensitivity, 1=Linear, 2=Free
    int senseGain_ = 10;  // 0-21
    int linearGain_ = 10; // 0-21
    int lnaGain_ = 8;     // 0-15
    int mixerGain_ = 8;   // 0-15
    int vgaGain_ = 4;     // 0-15
    bool lnaAgc_ = false;
    bool mixerAgc_ = false;
    bool biasTee_ = false;

    float dcOffRe_ = 0.0f, dcOffIm_ = 0.0f;
    float dcRate_ = 2.0e-6f;

    std::vector<float> scratch_;
};

} // namespace iridi

#endif // HAS_AIRSPY
#endif // IRIDISCOPE_AIRSPY_SOURCE_H
