// SDRplay source backend (SDRplay API v3.x). Adapted from AirScope.
#include "sdr/sdr_source.h"

#ifdef HAS_SDRPLAY

#include "sdr/sdrplay_source.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <thread>

namespace iridi {

namespace {

std::mutex g_apiMtx;
bool g_opened = false;

// sdrplay_api_Open() must be called exactly once per process; the API is a
// singleton and its calls are not thread-safe, so serialize them.
bool ensureOpen() {
    std::lock_guard<std::mutex> lk(g_apiMtx);
    if (g_opened) return true;
    if (sdrplay_api_Open() != sdrplay_api_Success) return false;
    float ver = 0.0f;
    if (sdrplay_api_ApiVersion(&ver) == sdrplay_api_Success) {
        if (std::fabs(ver - SDRPLAY_API_VERSION) > 0.001f)
            std::fprintf(stderr, "[sdrplay] API version %.2f (built against %.2f)\n",
                         (double)ver, (double)SDRPLAY_API_VERSION);
    }
    g_opened = true;
    return true;
}

const char *hwName(unsigned char hwVer) {
    switch (hwVer) {
    case SDRPLAY_RSP1_ID:    return "RSP1";
    case SDRPLAY_RSP1A_ID:   return "RSP1A";
    case SDRPLAY_RSP1B_ID:   return "RSP1B";
    case SDRPLAY_RSP2_ID:    return "RSP2";
    case SDRPLAY_RSPduo_ID:  return "RSPduo";
    case SDRPLAY_RSPdx_ID:   return "RSPdx";
    case SDRPLAY_RSPdxR2_ID: return "RSPdx R2";
    default:                 return "SDRplay";
    }
}

std::vector<std::string> portsForHw(unsigned char hwVer) {
    switch (hwVer) {
    case SDRPLAY_RSP2_ID:
        return {"Port A", "Port B", "Hi-Z"};
    case SDRPLAY_RSPduo_ID:
        return {"Tuner 1 (50 ohm)", "Tuner 1 (Hi-Z)", "Tuner 2 (50 ohm)"};
    case SDRPLAY_RSPdx_ID:
    case SDRPLAY_RSPdxR2_ID:
        return {"Port A", "Port B", "Port C"};
    default:
        return {};
    }
}

// Map our UI port index to the API selector values. RSP2 uses antenna B plus
// the AM port switch for Hi-Z; RSPduo uses tuner 1/2 and its AM port switch.
const sdrplay_api_Rsp2_AntennaSelectT kRsp2Antenna[] = {
    sdrplay_api_Rsp2_ANTENNA_A,
    sdrplay_api_Rsp2_ANTENNA_B,
    sdrplay_api_Rsp2_ANTENNA_B,
};
const sdrplay_api_RspDx_AntennaSelectT kRspDxAntenna[] = {
    sdrplay_api_RspDx_ANTENNA_A,
    sdrplay_api_RspDx_ANTENNA_B,
    sdrplay_api_RspDx_ANTENNA_C,
};

sdrplay_api_Bw_MHzT bwForRate(double fsHz) {
    if (fsHz <= 0.25e6) return sdrplay_api_BW_0_200;
    if (fsHz <= 0.30e6) return sdrplay_api_BW_0_300;
    if (fsHz <= 0.60e6) return sdrplay_api_BW_0_600;
    if (fsHz <= 2.0e6)  return sdrplay_api_BW_1_536;
    if (fsHz <= 6.0e6)  return sdrplay_api_BW_6_000;
    return sdrplay_api_BW_8_000;
}

void streamACb(short *xi, short *xq, sdrplay_api_StreamCbParamsT *, unsigned int numSamples,
               unsigned int, void *cbContext) {
    static_cast<SdrplaySource *>(cbContext)->handleRx(xi, xq, (int)numSamples);
}

void eventCb(sdrplay_api_EventT, sdrplay_api_TunerSelectT, sdrplay_api_EventParamsT *, void *) {}

} // namespace

SdrplaySource::SdrplaySource() = default;

SdrplaySource::~SdrplaySource() { stop(); }

std::vector<SdrDeviceInfo> SdrplaySource::listDevices() {
    std::vector<SdrDeviceInfo> out;
    if (!ensureOpen()) return out;

    std::lock_guard<std::mutex> lk(g_apiMtx);
    sdrplay_api_LockDeviceApi();
    sdrplay_api_DeviceT devs[SDRPLAY_MAX_DEVICES]{};
    unsigned int n = 0;
    if (sdrplay_api_GetDevices(devs, &n, SDRPLAY_MAX_DEVICES) == sdrplay_api_Success) {
        for (unsigned int i = 0; i < n; ++i) {
            if (!devs[i].valid) continue;
            SdrDeviceInfo info;
            info.index = (int)out.size();
            info.name = hwName(devs[i].hwVer);
            info.serial = devs[i].SerNo;
            info.ports = portsForHw(devs[i].hwVer);
            out.push_back(info);
        }
    }
    sdrplay_api_UnlockDeviceApi();
    return out;
}

bool SdrplaySource::start(int deviceIndex, SdrSampleCb cb, std::string &err) {
    if (running_.load()) return true;
    if (!ensureOpen()) {
        err = "SDRplay API open failed (is the sdrplay service running?)";
        return false;
    }

    // The SDRplay service can briefly report the device as busy right after a
    // previous session releases it; retry a few times before giving up.
    std::string lastErr;
    for (int attempt = 0; attempt < 8; ++attempt) {
        if (attempt > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(600));
            std::fprintf(stderr, "[sdrplay] start retry %d (%s)\n", attempt, lastErr.c_str());
        }
        std::string e;
        if (acquire(deviceIndex, cb, e)) {
            running_.store(true);
            return true;
        }
        lastErr = e;
    }
    err = lastErr.empty() ? "SDRplay start failed" : lastErr;
    return false;
}

bool SdrplaySource::acquire(int deviceIndex, SdrSampleCb cb, std::string &err) {
    std::lock_guard<std::mutex> lk(g_apiMtx);
    sdrplay_api_LockDeviceApi();
    sdrplay_api_DeviceT devs[SDRPLAY_MAX_DEVICES]{};
    unsigned int n = 0;
    sdrplay_api_ErrT r = sdrplay_api_GetDevices(devs, &n, SDRPLAY_MAX_DEVICES);
    if (r != sdrplay_api_Success) {
        sdrplay_api_UnlockDeviceApi();
        err = "SDRplay GetDevices failed: " + std::string(sdrplay_api_GetErrorString(r));
        return false;
    }
    int validCount = 0;
    sdrplay_api_DeviceT *pick = nullptr;
    for (unsigned int i = 0; i < n; ++i) {
        if (!devs[i].valid) continue;
        if (validCount == deviceIndex) { pick = &devs[i]; break; }
        ++validCount;
    }
    if (!pick) {
        sdrplay_api_UnlockDeviceApi();
        err = "No SDRplay device at index " + std::to_string(deviceIndex);
        return false;
    }

    dev_ = new sdrplay_api_DeviceT(*pick);
    hwVer_ = pick->hwVer;
    int maxPorts = portCount();
    if (maxPorts > 0) antenna_ = std::clamp(antenna_, 0, maxPorts - 1);
    else antenna_ = 0;
    dev_->tuner = sdrplay_api_Tuner_A;
    dev_->rspDuoMode = sdrplay_api_RspDuoMode_Single_Tuner;
    sdrplay_api_DisableHeartbeat();
    r = sdrplay_api_SelectDevice(dev_);
    sdrplay_api_UnlockDeviceApi();
    if (r != sdrplay_api_Success) {
        delete dev_;
        dev_ = nullptr;
        err = "SDRplay SelectDevice failed: " + std::string(sdrplay_api_GetErrorString(r)) +
              " (device busy?)";
        return false;
    }

    r = sdrplay_api_GetDeviceParams(dev_->dev, &params_);
    if (r != sdrplay_api_Success || !params_) {
        sdrplay_api_ReleaseDevice(dev_);
        delete dev_;
        dev_ = nullptr;
        err = "SDRplay GetDeviceParams failed";
        return false;
    }

    activeTuner_ = dev_->tuner;
    rx_ = (activeTuner_ == sdrplay_api_Tuner_B) ? params_->rxChannelB : params_->rxChannelA;
    if (!rx_) {
        sdrplay_api_ReleaseDevice(dev_);
        delete dev_;
        dev_ = nullptr;
        params_ = nullptr;
        err = "SDRplay selected tuner is unavailable";
        return false;
    }

    params_->devParams->fsFreq.fsHz = sampleRate_;
    params_->devParams->ppm = ppm_;
    rx_->tunerParams.rfFreq.rfHz = centerFreq_;
    rx_->tunerParams.bwType = bwForRate(sampleRate_);
    rx_->tunerParams.ifType = sdrplay_api_IF_Zero;
    rx_->ctrlParams.decimation.enable = 0;
    rx_->ctrlParams.decimation.decimationFactor = 1;
    rx_->ctrlParams.dcOffset.DCenable = 1;
    rx_->ctrlParams.dcOffset.IQenable = 1;
    applyDeviceParams();
    applyGain();

    sdrplay_api_CallbackFnsT fns{};
    fns.StreamACbFn = &streamACb;
    fns.StreamBCbFn = nullptr;
    fns.EventCbFn = &eventCb;

    dcOffRe_ = dcOffIm_ = 0.0f;
    dcRate_ = (float)(50.0 / sampleRate_);

    r = sdrplay_api_Init(dev_->dev, &fns, this);
    if (r != sdrplay_api_Success) {
        sdrplay_api_ReleaseDevice(dev_);
        delete dev_;
        dev_ = nullptr;
        params_ = nullptr;
        rx_ = nullptr;
        activeTuner_ = sdrplay_api_Tuner_A;
        err = "SDRplay Init failed: " + std::string(sdrplay_api_GetErrorString(r));
        return false;
    }
    cb_ = std::move(cb);
    applyAntenna();
    return true;
}

void SdrplaySource::stop() {
    if (dev_) {
        std::lock_guard<std::mutex> lk(g_apiMtx);
        if (running_.load()) sdrplay_api_Uninit(dev_->dev);
        sdrplay_api_ReleaseDevice(dev_);
        delete dev_;
        dev_ = nullptr;
        params_ = nullptr;
        rx_ = nullptr;
        activeTuner_ = sdrplay_api_Tuner_A;
    }
    running_.store(false);
    cb_ = nullptr;
}

void SdrplaySource::handleRx(const short *xi, const short *xq, int n) {
    if (!running_.load() || !cb_ || n <= 0) return;

    if ((int)scratch_.size() < n * 2) scratch_.resize((size_t)n * 2);
    float *out = scratch_.data();

    float offRe = dcOffRe_, offIm = dcOffIm_;
    const float rate = dcRate_;
    for (int i = 0; i < n; ++i) {
        float re = xi[i] * (1.0f / 32768.0f);
        float im = xq[i] * (1.0f / 32768.0f);
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

int SdrplaySource::portCount() const {
    return (int)portsForHw(hwVer_).size();
}

void SdrplaySource::applyDeviceParams() {
    if (!rx_ || !params_) return;
    int p = std::clamp(antenna_, 0, 2);
    switch (hwVer_) {
    case SDRPLAY_RSP1A_ID:
    case SDRPLAY_RSP1B_ID:
        rx_->rsp1aTunerParams.biasTEnable = biasTee_ ? 1 : 0;
        break;
    case SDRPLAY_RSP2_ID:
        rx_->rsp2TunerParams.biasTEnable = biasTee_ ? 1 : 0;
        rx_->rsp2TunerParams.antennaSel = kRsp2Antenna[p];
        rx_->rsp2TunerParams.amPortSel = (p == 2) ? sdrplay_api_Rsp2_AMPORT_1
                                                  : sdrplay_api_Rsp2_AMPORT_2;
        break;
    case SDRPLAY_RSPduo_ID:
        rx_->rspDuoTunerParams.biasTEnable = biasTee_ ? 1 : 0;
        rx_->rspDuoTunerParams.tuner1AmPortSel =
            (p == 0) ? sdrplay_api_RspDuo_AMPORT_2 : sdrplay_api_RspDuo_AMPORT_1;
        break;
    case SDRPLAY_RSPdx_ID:
    case SDRPLAY_RSPdxR2_ID:
        params_->devParams->rspDxParams.biasTEnable = biasTee_ ? 1 : 0;
        params_->devParams->rspDxParams.antennaSel = kRspDxAntenna[p];
        break;
    default:
        break;
    }
}

void SdrplaySource::applyAntenna() {
    if (!dev_ || !params_ || !rx_) return;
    int n = portCount();
    if (n <= 0) return;
    antenna_ = std::clamp(antenna_, 0, n - 1);

    if (hwVer_ == SDRPLAY_RSPduo_ID) {
        sdrplay_api_TunerSelectT want = (antenna_ == 2) ? sdrplay_api_Tuner_B
                                                        : sdrplay_api_Tuner_A;
        auto am = (antenna_ == 0) ? sdrplay_api_RspDuo_AMPORT_2
                                  : sdrplay_api_RspDuo_AMPORT_1;
        std::lock_guard<std::mutex> lk(g_apiMtx);
        if (want != activeTuner_) {
            sdrplay_api_TunerSelectT t = activeTuner_;
            sdrplay_api_ErrT r = sdrplay_api_SwapRspDuoActiveTuner(dev_->dev, &t, am);
            if (r != sdrplay_api_Success) {
                std::fprintf(stderr, "[sdrplay] SwapRspDuoActiveTuner failed: %s\n",
                             sdrplay_api_GetErrorString(r));
                return;
            }
            dev_->tuner = t;
            activeTuner_ = t;
            rx_ = (t == sdrplay_api_Tuner_B) ? params_->rxChannelB : params_->rxChannelA;
            if (!rx_) return;
            rx_->tunerParams.rfFreq.rfHz = centerFreq_ * (1.0 + ppm_ / 1e6);
            rx_->tunerParams.bwType = bwForRate(sampleRate_);
            rx_->tunerParams.ifType = sdrplay_api_IF_Zero;
            rx_->ctrlParams.decimation.enable = 0;
            rx_->ctrlParams.decimation.decimationFactor = 1;
            rx_->ctrlParams.dcOffset.DCenable = 1;
            rx_->ctrlParams.dcOffset.IQenable = 1;
            applyGain();
            applyDeviceParams();
            sdrplay_api_Update(dev_->dev, activeTuner_, sdrplay_api_Update_Tuner_Frf,
                               sdrplay_api_Update_Ext1_None);
            sdrplay_api_Update(dev_->dev, activeTuner_, sdrplay_api_Update_Tuner_BwType,
                               sdrplay_api_Update_Ext1_None);
            sdrplay_api_Update(dev_->dev, activeTuner_, sdrplay_api_Update_Tuner_IfType,
                               sdrplay_api_Update_Ext1_None);
            sdrplay_api_Update(dev_->dev, activeTuner_, sdrplay_api_Update_Ctrl_Decimation,
                               sdrplay_api_Update_Ext1_None);
            sdrplay_api_Update(dev_->dev, activeTuner_, sdrplay_api_Update_Ctrl_DCoffsetIQimbalance,
                               sdrplay_api_Update_Ext1_None);
            sdrplay_api_Update(dev_->dev, activeTuner_, sdrplay_api_Update_Tuner_Gr,
                               sdrplay_api_Update_Ext1_None);
            sdrplay_api_Update(dev_->dev, activeTuner_, sdrplay_api_Update_Ctrl_Agc,
                               sdrplay_api_Update_Ext1_None);
        }
        applyDeviceParams();
        sdrplay_api_Update(dev_->dev, activeTuner_, sdrplay_api_Update_RspDuo_AmPortSelect,
                           sdrplay_api_Update_Ext1_None);
        return;
    }

    applyDeviceParams();
    std::lock_guard<std::mutex> lk(g_apiMtx);
    if (hwVer_ == SDRPLAY_RSP2_ID) {
        sdrplay_api_Update(dev_->dev, activeTuner_, sdrplay_api_Update_Rsp2_AntennaControl,
                           sdrplay_api_Update_Ext1_None);
        sdrplay_api_Update(dev_->dev, activeTuner_, sdrplay_api_Update_Rsp2_AmPortSelect,
                           sdrplay_api_Update_Ext1_None);
    } else if (hwVer_ == SDRPLAY_RSPdx_ID || hwVer_ == SDRPLAY_RSPdxR2_ID) {
        sdrplay_api_Update(dev_->dev, activeTuner_, sdrplay_api_Update_None,
                           sdrplay_api_Update_RspDx_AntennaControl);
    }
}

void SdrplaySource::applyGain() {
    if (!rx_) return;
    sdrplay_api_GainT &g = rx_->tunerParams.gain;
    if (gainDb_ < 0.0) {
        rx_->ctrlParams.agc.enable = sdrplay_api_AGC_50HZ;
        rx_->ctrlParams.agc.setPoint_dBfs = -30;
    } else {
        rx_->ctrlParams.agc.enable = sdrplay_api_AGC_DISABLE;
        // More gain => lower reduction. gRdB valid range ~20..59.
        int gr = (int)std::lround(59.0 - gainDb_);
        g.gRdB = std::clamp(gr, (int)sdrplay_api_NORMAL_MIN_GR, 59);
        g.LNAstate = 0;
    }
}

void SdrplaySource::applyTune() {
    if (!rx_ || !dev_ || !params_) return;
    rx_->tunerParams.rfFreq.rfHz = centerFreq_ * (1.0 + ppm_ / 1e6);
    std::lock_guard<std::mutex> lk(g_apiMtx);
    sdrplay_api_Update(dev_->dev, activeTuner_,
                       sdrplay_api_Update_Tuner_Frf, sdrplay_api_Update_Ext1_None);
    if (ppm_ != 0.0) {
        params_->devParams->ppm = ppm_;
        sdrplay_api_Update(dev_->dev, activeTuner_,
                           sdrplay_api_Update_Dev_Ppm, sdrplay_api_Update_Ext1_None);
    }
}

void SdrplaySource::setCenterFreq(double hz) {
    centerFreq_ = hz;
    if (running_.load()) applyTune();
}

void SdrplaySource::setPpm(double ppm) {
    ppm_ = ppm;
    if (running_.load()) applyTune();
}

void SdrplaySource::setSampleRate(double hz) {
    sampleRate_ = hz;
    if (!running_.load() || !params_ || !rx_) return;
    params_->devParams->fsFreq.fsHz = hz;
    rx_->tunerParams.bwType = bwForRate(hz);
    std::lock_guard<std::mutex> lk(g_apiMtx);
    sdrplay_api_Update(dev_->dev, activeTuner_,
                       sdrplay_api_Update_Dev_Fs, sdrplay_api_Update_Ext1_None);
    sdrplay_api_Update(dev_->dev, activeTuner_,
                       sdrplay_api_Update_Tuner_BwType, sdrplay_api_Update_Ext1_None);
}

void SdrplaySource::setGain(double db) {
    gainDb_ = db;
    if (!running_.load() || !params_ || !rx_) return;
    applyGain();
    std::lock_guard<std::mutex> lk(g_apiMtx);
    sdrplay_api_Update(dev_->dev, activeTuner_,
                       sdrplay_api_Update_Tuner_Gr, sdrplay_api_Update_Ext1_None);
    sdrplay_api_Update(dev_->dev, activeTuner_,
                       sdrplay_api_Update_Ctrl_Agc, sdrplay_api_Update_Ext1_None);
}

void SdrplaySource::setBiasTee(bool on) {
    biasTee_ = on;
    if (!running_.load() || !params_ || !rx_) return;
    applyDeviceParams();
    std::lock_guard<std::mutex> lk(g_apiMtx);
    switch (hwVer_) {
    case SDRPLAY_RSP1A_ID:
    case SDRPLAY_RSP1B_ID:
        sdrplay_api_Update(dev_->dev, activeTuner_, sdrplay_api_Update_Rsp1a_BiasTControl,
                           sdrplay_api_Update_Ext1_None);
        break;
    case SDRPLAY_RSP2_ID:
        sdrplay_api_Update(dev_->dev, activeTuner_, sdrplay_api_Update_Rsp2_BiasTControl,
                           sdrplay_api_Update_Ext1_None);
        break;
    case SDRPLAY_RSPduo_ID:
        sdrplay_api_Update(dev_->dev, activeTuner_, sdrplay_api_Update_RspDuo_BiasTControl,
                           sdrplay_api_Update_Ext1_None);
        break;
    case SDRPLAY_RSPdx_ID:
    case SDRPLAY_RSPdxR2_ID:
        sdrplay_api_Update(dev_->dev, activeTuner_, sdrplay_api_Update_None,
                           sdrplay_api_Update_RspDx_BiasTControl);
        break;
    default:
        break;
    }
}

void SdrplaySource::setAntenna(int port) {
    antenna_ = port;
    if (!running_.load() || !dev_ || !params_ || !rx_) return;
    applyAntenna();
}

std::unique_ptr<SdrSource> make_sdrplay_source() {
    return std::unique_ptr<SdrSource>(new SdrplaySource());
}

} // namespace iridi

#endif // HAS_SDRPLAY
