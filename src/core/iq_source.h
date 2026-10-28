// Abstract pull-based IQ source.
//
// The engine consumes interleaved complex-float samples through this
// interface, so an IQ file and a live SDR (e.g. SDRplay) are interchangeable.
#ifndef IRIDISCOPE_IQ_SOURCE_H
#define IRIDISCOPE_IQ_SOURCE_H

#include <cstddef>
#include <cstdint>

namespace iridi {

class IqSource {
public:
    virtual ~IqSource() = default;

    // True once the source is ready to produce samples.
    virtual bool is_open() const = 0;

    // Nominal sample rate in Hz (0 if unknown).
    virtual int sample_rate() const = 0;

    // Center frequency in Hz (0 if unknown).
    virtual double center_frequency() const = 0;

    // Total complex samples, or 0 when unknown / live streaming.
    virtual uint64_t total_samples() const = 0;

    // Reads up to max_samples complex samples as interleaved float I,Q.
    // Returns the number of complex samples read. For live sources this blocks
    // until data is available or the source is closed; a return of 0 means the
    // source has ended (see done()).
    virtual size_t read_cf32(float *out, size_t max_samples) = 0;

    // Stops producing samples and releases resources.
    virtual void close() = 0;

    // True when no further samples will be produced.
    virtual bool done() const { return false; }

    // Unblocks a read_cf32() call in progress without tearing the source down.
    // The engine calls this before joining its worker thread on stop.
    virtual void wake() {}

    // Complex samples lost just before the block the last read_cf32() returned
    // (driver or buffer overruns). Reading it clears it. The engine uses this
    // to keep burst timestamps on wall-clock time across drops.
    virtual uint64_t take_gap() { return 0; }

    // Seeking is only meaningful for file-backed sources.
    virtual bool seekable() const { return false; }
    virtual bool seek_sample(uint64_t) { return false; }
};

} // namespace iridi

#endif // IRIDISCOPE_IQ_SOURCE_H
