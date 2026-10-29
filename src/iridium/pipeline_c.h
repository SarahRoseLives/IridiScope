// C interface to the vendored iridium-sniffer DSP pipeline.
//
// The iridium-sniffer headers use C99 `float complex`, which does not parse in
// C++, so this thin C wrapper exposes the pipeline with plain types only. It
// performs burst detection -> downmix -> DQPSK demodulation synchronously and
// reports each demodulated frame through a callback.
#ifndef IRIDISCOPE_PIPELINE_C_H
#define IRIDISCOPE_PIPELINE_C_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ir_pipeline ir_pipeline_t;

typedef struct {
    uint64_t id;
    uint64_t timestamp_ns;
    double   frequency;
    int      direction;        /* 1 = downlink, 2 = uplink */
    float    magnitude;        /* dB */
    float    noise;            /* dBFS/Hz */
    float    level;
    int      confidence;       /* 0-100 */
    int      n_symbols;
    int      n_payload_symbols;
    int      n_bits;
    const unsigned char *bits; /* n_bits entries, 0/1; valid only during callback */
} ir_frame_info_t;

typedef void (*ir_frame_cb)(const ir_frame_info_t *info, void *user);

/* Create a pipeline. center_freq in Hz, sample_rate in Hz, threshold in dB
 * (pass 0 for the default). cb may be NULL. num_workers <= 0 selects an
 * automatic count (CPU cores - 1). */
ir_pipeline_t *ir_pipeline_create(double center_freq, int sample_rate,
                                  float threshold, int num_workers,
                                  ir_frame_cb cb, void *user);

/* Feed interleaved complex-float IQ samples. Returns the number of demodulated
 * frames produced during this call. */
uint64_t ir_pipeline_feed(ir_pipeline_t *p, const float *iq, size_t num_samples);

/* Spectrum (magnitude in dB, fftshifted), length ir_pipeline_spectrum_size.
 * "inst" is the current frame, "avg" is exponentially averaged. */
const float *ir_pipeline_spectrum_inst(ir_pipeline_t *p);
const float *ir_pipeline_spectrum_avg(ir_pipeline_t *p);
int ir_pipeline_spectrum_size(ir_pipeline_t *p);

/* Per-frame noise floor / peak (percentiles) in dB. */
float ir_pipeline_spectrum_floor(ir_pipeline_t *p);
float ir_pipeline_spectrum_peak(ir_pipeline_t *p);

/* Spectrum configuration (safe to call from the feed thread). */
void ir_pipeline_set_fft_size(ir_pipeline_t *p, int n);
int  ir_pipeline_get_fft_size(ir_pipeline_t *p);
void ir_pipeline_set_avg_alpha(ir_pipeline_t *p, float alpha);
void ir_pipeline_set_dc_block(ir_pipeline_t *p, int on);

/* Update the capture center frequency used for reported frame frequencies
 * (call when the SDR VFO is retuned). */
void ir_pipeline_set_center(ir_pipeline_t *p, double hz);
/* Ask the detector to re-learn its noise floor (after gain/tuning changes).
 * Thread-safe; applied at the start of the next feed. */
void ir_pipeline_reset_noise(ir_pipeline_t *p);
/* Detector's learned noise floor, dBFS/Hz averaged over the band. */
float ir_pipeline_noise_floor(ir_pipeline_t *p);
/* Samples lost before the next feed: keep burst times on wall-clock time. */
void ir_pipeline_skip_samples(ir_pipeline_t *p, uint64_t n);
/* Wall-clock time (Unix ns) of the first sample; call before the first feed. */
void ir_pipeline_set_start_time(ir_pipeline_t *p, uint64_t unix_ns);
/* Total time the feeding thread waited for burst-queue space. */
uint64_t ir_pipeline_queue_wait_ns(ir_pipeline_t *p);

uint64_t ir_pipeline_bursts(ir_pipeline_t *p);
uint64_t ir_pipeline_frames_ok(ir_pipeline_t *p);
uint64_t ir_pipeline_frames_fail(ir_pipeline_t *p);
/* Times a burst had to wait because every worker was busy (queue full). */
uint64_t ir_pipeline_queue_full(ir_pipeline_t *p);
/* Number of downmix/demod worker threads. */
int ir_pipeline_workers(ir_pipeline_t *p);
/* Burst queue memory budget in bytes for pipelines created afterwards
 * (default 4 GB). */
void ir_pipeline_set_queue_budget(double bytes);
/* Live radio backlog in seconds for pipelines created afterwards: the
 * decoders may fall this far behind before the oldest bursts are skipped.
 * 0 = playback mode (no skipping; the detector waits for the decoders). */
void ir_pipeline_set_live_backlog(double seconds);
/* Bursts skipped because the decoders were further behind than the backlog. */
uint64_t ir_pipeline_skipped(ir_pipeline_t *p);

/* Wide-carrier filter (global, live): detections inside a signal >=150 kHz
 * wide (wideband carriers the classic decoders can't read) are dropped by
 * the detector and counted instead of decoded. On by default. */
void ir_set_reject_wide(int on);
int ir_reject_wide(void);
uint64_t ir_wide_dropped(void);
/* Voice targeting (live): report each decoded voice frame (any thread). For
 * calls in progress the pipeline then also decodes the burst at every
 * expected 90 ms slot that the detector didn't trigger on (too weak), so
 * weak voice frames are recovered. Frames from such reads carry bit 63 in
 * their id. */
void ir_pipeline_voice_seen(ir_pipeline_t *p, uint64_t ts_ns, double freq_hz);
void ir_pipeline_probe_stats(ir_pipeline_t *p, uint64_t *probes, uint64_t *frames);
void ir_pipeline_set_voice_targeting(int on);
/* Burst buffer pool: allocations served from reused vs fresh memory, and
 * bytes in use / held idle for reuse (process-wide). */
void ir_pipeline_buffer_stats(uint64_t *reused, uint64_t *fresh, size_t *live, size_t *idle);
/* Burst queue capacity (bursts), sized from the memory budget. */
int ir_pipeline_queue_capacity(ir_pipeline_t *p);

void ir_pipeline_destroy(ir_pipeline_t *p);

#ifdef __cplusplus
}
#endif
#endif // IRIDISCOPE_PIPELINE_C_H
