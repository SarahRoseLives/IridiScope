// Adapts a push-based SdrSource (its own delivery thread) to the pull-based
// IqSource interface the engine consumes. Incoming blocks are buffered in a
// lock-protected ring; read_cf32() blocks until samples are available.
#ifndef IRIDISCOPE_LIVE_IQ_SOURCE_H
#define IRIDISCOPE_LIVE_IQ_SOURCE_H

#include "core/iq_source.h"
#include "sdr/sdr_source.h"

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace iridi {

class LiveIqSource : public IqSource {
public:
    // center_hz / sample_rate_hz seed the backend defaults.
    LiveIqSource(SdrKind kind = SdrKind::Sdrplay, double center_hz = 1623326935.0,
                 double sample_rate_hz = 8.0e6);
    ~LiveIqSource() override;

    // Configures and starts the underlying SDR. Returns false and sets err on
    // failure. On success, list_devices()/running() are valid.
    bool open(int device_index, double center_hz, double sample_rate_hz, std::string &err);

    // --- IqSource -----------------------------------------------------------
    bool is_open() const override { return open_.load(); }
    int sample_rate() const override { return (int)sample_rate_; }
    double center_frequency() const override { return center_hz_; }
    uint64_t total_samples() const override { return 0; } // live / unknown
    size_t read_cf32(float *out, size_t max_samples) override; // blocks
    void close() override;
    bool done() const override { return !open_.load(); }
    void wake() override;

    // --- Live controls (applied immediately when running) -------------------
    void set_center_frequency(double hz);
    void set_sample_rate(double hz);
    void clear_buffer(); // drop buffered samples after a retune / rate change
    void set_gain(double db); // <0 => AGC
    double gain() const { return gain_; }
    void set_ppm(double ppm);
    void set_bias_tee(bool on); // RSP/HackRF bias tee (powers an LNA)
    bool bias_tee() const { return bias_; }

    // HackRF multi-stage gain.
    void set_lna_gain(int db);
    int lna_gain() const { return lna_; }
    void set_vga_gain(int db);
    int vga_gain() const { return vga_; }
    void set_amp(bool on);
    bool amp() const { return amp_; }

    // Airspy gain controls.
    void set_gain_mode(int m);
    int gain_mode() const { return gain_mode_; }
    void set_sense_gain(int v);
    int sense_gain() const { return sense_; }
    void set_linear_gain(int v);
    int linear_gain() const { return linear_; }
    void set_mixer_gain(int v);
    int mixer_gain() const { return mixer_; }
    void set_lna_agc(bool on);
    bool lna_agc() const { return lna_agc_; }
    void set_mixer_agc(bool on);
    bool mixer_agc() const { return mixer_agc_; }

    bool running() const;
    std::vector<SdrDeviceInfo> list_devices() const;
    std::string device_name() const;
    bool available() const { return ensure_backend(); }

private:
    // Creates the backend on first use (needed to list devices before opening).
    bool ensure_backend() const;
    void on_samples(const float *iq, int n);

    mutable     std::unique_ptr<SdrSource> sdr_;
    SdrKind kind_ = SdrKind::Sdrplay;
    std::atomic<bool> open_{false};

    double center_hz_ = 1623326935.0;
    double sample_rate_ = 8.0e6;
    double gain_ = -1.0; // <0 => AGC
    double ppm_ = 0.0;
    bool bias_ = false;
    int lna_ = 16; // HackRF IF gain (0..40, 8 dB steps)
    int vga_ = 16; // HackRF baseband gain (0..62, 2 dB steps)
    bool amp_ = false;

    // Airspy gain state.
    int gain_mode_ = 0; // 0=Sensitivity 1=Linear 2=Free
    int sense_ = 10;
    int linear_ = 10;
    int mixer_ = 8;
    bool lna_agc_ = false;
    bool mixer_agc_ = false;

    mutable std::mutex mtx_;
    std::condition_variable cv_;
    std::vector<float> ring_; // cap_*2 floats
    size_t cap_ = 1u << 20;   // complex samples buffered
    size_t head_ = 0;         // oldest complex index
    size_t count_ = 0;        // complex samples available
};

} // namespace iridi

#endif // IRIDISCOPE_LIVE_IQ_SOURCE_H
