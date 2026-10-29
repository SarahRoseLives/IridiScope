// Abstract push-based SDR backend, modelled on AirScope's src/sdr/sdr_source.h.
//
// A backend delivers interleaved complex-float IQ ([I0,Q0,I1,Q1,...]) from its
// own read thread via SdrSampleCb. LiveIqSource adapts this to the pull-based
// IqSource interface the engine consumes.
#ifndef IRIDISCOPE_SDR_SOURCE_H
#define IRIDISCOPE_SDR_SOURCE_H

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace iridi {

struct SdrDeviceInfo {
    int index = 0;
    std::string name;
    std::string serial;
    // Selectable antenna/input ports, in backend order. Empty when the device
    // has a single fixed input (RSP1/RSP1A/RSP1B, HackRF, Airspy, RTL-SDR).
    std::vector<std::string> ports;
    // Sample rates (Hz) the device reports, highest first. Empty when the
    // backend uses a fixed list (or the device could not be queried).
    std::vector<double> rates;
};

// Which SDR backend a LiveIqSource should use.
enum class SdrKind { Sdrplay, HackRF, Airspy, RTL };

// Called from the SDR read thread with interleaved float IQ, already converted
// to [-1, 1].
using SdrSampleCb = std::function<void(const float *iq, int nComplex)>;

class SdrSource {
public:
    virtual ~SdrSource() = default;

    virtual std::vector<SdrDeviceInfo> listDevices() = 0;

    // Configuration (applied on start; some apply live while running).
    virtual void setCenterFreq(double hz) = 0;
    virtual void setSampleRate(double hz) = 0;
    virtual void setGain(double db) = 0; // <0 => auto/AGC
    virtual void setBiasTee(bool on) = 0;
    virtual void setPpm(double ppm) = 0;
    // Selectable antenna/input port for multi-port devices (SDRplay RSP2,
    // RSPduo, RSPdx). No-op on backends without selectable ports.
    virtual void setAntenna(int port) { (void)port; }

    // Multi-stage gain (HackRF / Airspy). No-ops on backends without them.
    virtual void setLnaGain(int) {}
    virtual void setVgaGain(int) {}
    virtual void setAmp(bool) {}
    virtual void setMixerGain(int) {}
    virtual void setGainMode(int) {}  // Airspy: 0=Sensitivity 1=Linear 2=Free
    virtual void setSenseGain(int) {} // Airspy sensitivity knob (0-21)
    virtual void setLinearGain(int) {}// Airspy linearity knob (0-21)
    virtual void setLnaAgc(bool) {}
    virtual void setMixerAgc(bool) {}
    // Airspy Mini: allow the unofficial 10 Msps mode, which the firmware
    // supports but does not advertise.
    virtual void setAllowUnofficialRates(bool) {}
    // Samples the vendor driver discarded before they reached us (e.g.
    // libairspy's conversion thread falling behind USB). 0 if unknown.
    virtual uint64_t driverDroppedSamples() const { return 0; }
    virtual bool reportsDriverDrops() const { return false; }

    virtual double centerFreq() const = 0;
    virtual double sampleRate() const = 0;

    // Lifecycle.
    virtual bool start(int deviceIndex, SdrSampleCb cb, std::string &err) = 0;
    virtual void stop() = 0;
    virtual bool running() const = 0;
};

// Creates an SDR backend, or nullptr when that backend is not compiled in.
// Defined in each backend's .cpp (under HAS_SDRPLAY / HAS_HACKRF) with a stub
// otherwise.
std::unique_ptr<SdrSource> make_sdrplay_source();
std::unique_ptr<SdrSource> make_hackrf_source();
std::unique_ptr<SdrSource> make_airspy_source();
std::unique_ptr<SdrSource> make_rtlsdr_source();

// Dispatches to the backend factory for the given kind.
std::unique_ptr<SdrSource> make_sdr_source(SdrKind kind);

} // namespace iridi

#endif // IRIDISCOPE_SDR_SOURCE_H
