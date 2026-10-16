// IQ file reader for IridiScope.
//
// Supports WAV (PCM int16/int8/float32, stereo I/Q) and headerless raw
// ci8 / ci16 / cf32 IQ files. Produces interleaved complex-float samples.
#ifndef IRIDISCOPE_FILE_SOURCE_H
#define IRIDISCOPE_FILE_SOURCE_H

#include <cstdint>
#include <cstdio>
#include <string>

#include "core/iq_source.h"

namespace iridi {

enum class IqFormat { CI8, CI16, CF32, UNKNOWN };

const char *iq_format_name(IqFormat f);
int iq_format_bytes_per_sample(IqFormat f); // bytes per complex sample (I+Q)

struct IqFileInfo {
    std::string path;
    IqFormat format = IqFormat::UNKNOWN;
    bool is_wav = false;
    int sample_rate = 0;          // Hz
    int channels = 2;             // interleaved channels (2 = I,Q)
    int bits_per_sample = 0;      // for WAV
    double center_frequency = 0;  // parsed from filename if present (Hz)
    uint64_t data_offset = 0;     // byte offset of the first sample
    uint64_t data_bytes = 0;      // size of sample payload
    uint64_t num_samples = 0;     // complex samples
    double duration_s = 0.0;
    uint64_t file_size = 0;
};

class IqFileSource : public IqSource {
public:
    IqFileSource() = default;
    ~IqFileSource() override;

    IqFileSource(const IqFileSource &) = delete;
    IqFileSource &operator=(const IqFileSource &) = delete;

    // Opens a file and reads its header. Returns false and sets err on failure.
    bool open(const std::string &path, std::string &err);
    void close() override;

    bool is_open() const override { return fp_ != nullptr; }
    const IqFileInfo &info() const { return info_; }

    int sample_rate() const override { return info_.sample_rate; }
    double center_frequency() const override { return info_.center_frequency; }
    uint64_t total_samples() const override { return info_.num_samples; }
    bool done() const override { return !fp_ || info_.num_samples == 0 || pos_sample_ >= info_.num_samples; }

    // Reads up to max_samples complex samples as interleaved float I,Q.
    // Returns the number of complex samples actually read.
    size_t read_cf32(float *out, size_t max_samples) override;

    // Seeks to a complex-sample index. Returns false on failure.
    bool seekable() const override { return true; }
    bool seek_sample(uint64_t sample_index) override;

    // Current complex-sample position.
    uint64_t tell_sample() const { return pos_sample_; }

private:
    FILE *fp_ = nullptr;
    IqFileInfo info_;
    uint64_t pos_sample_ = 0;

    // Conversion details for the current stream.
    IqFormat format_ = IqFormat::UNKNOWN;
    int channels_ = 2;
    int bytes_per_channel_ = 2;
    bool wav_unsigned8_ = false;
    bool pcm32_ = false;
};

// Parses a center frequency from a filename such as
// "baseband_1623326935Hz_...". Returns 0 if none is found.
double parse_center_freq_from_name(const std::string &path);

} // namespace iridi

#endif // IRIDISCOPE_FILE_SOURCE_H
