// SDRplay source backend using the SDRplay API (v3.x).
// Supports RSP1A / RSP1B / RSP2 / RSPduo / RSPdx single-tuner receive,
// including antenna/input port selection on the multi-port models.
// Adapted from AirScope's src/sdr/sdrplay_source.{h,cpp}.
#ifndef IRIDISCOPE_SDRPLAY_SOURCE_H
#define IRIDISCOPE_SDRPLAY_SOURCE_H

#include "sdr/sdr_source.h"

#ifdef HAS_SDRPLAY

#include <sdrplay_api.h>

#include <atomic>
#include <string>
#include <vector>

namespace iridi {

class SdrplaySource : public SdrSource {
public:
    SdrplaySource();
    ~SdrplaySource() override;

    std::vector<SdrDeviceInfo> listDevices() override;

    void setCenterFreq(double hz) override;
    void setSampleRate(double hz) override;
    void setGain(double db) override; // <0 => AGC
    void setBiasTee(bool on) override;
    void setPpm(double ppm) override;
    void setAntenna(int port) override;

    double centerFreq() const override { return centerFreq_; }
    double sampleRate() const override { return sampleRate_; }

    bool start(int deviceIndex, SdrSampleCb cb, std::string &err) override;
    void stop() override;
    bool running() const override { return running_.load(); }

    // Called from the SDRplay stream callback.
    void handleRx(const short *xi, const short *xq, int nSamples);

private:
    void applyGain();
    void applyTune();
    void applyAntenna();
    void applyDeviceParams();
    // One attempt to select the device and start streaming.
    bool acquire(int deviceIndex, SdrSampleCb cb, std::string &err);
    int portCount() const;

    sdrplay_api_DeviceT *dev_ = nullptr;
    sdrplay_api_DeviceParamsT *params_ = nullptr;
    sdrplay_api_RxChannelParamsT *rx_ = nullptr;

    std::atomic<bool> running_{false};
    SdrSampleCb cb_;

    double centerFreq_ = 1623326935.0;
    double sampleRate_ = 8.0e6;
    double gainDb_ = -1.0; // <0 => AGC
    double ppm_ = 0.0;
    bool biasTee_ = false;
    int antenna_ = 0;
    sdrplay_api_TunerSelectT activeTuner_ = sdrplay_api_Tuner_A;

    float dcOffRe_ = 0.0f, dcOffIm_ = 0.0f;
    float dcRate_ = 2.0e-5f;

    std::vector<float> scratch_;
    int hwVer_ = 0;
};

} // namespace iridi

#endif // HAS_SDRPLAY
#endif // IRIDISCOPE_SDRPLAY_SOURCE_H
