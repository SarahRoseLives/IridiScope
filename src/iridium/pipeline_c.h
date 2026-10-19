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

uint64_t ir_pipeline_bursts(ir_pipeline_t *p);
uint64_t ir_pipeline_frames_ok(ir_pipeline_t *p);
uint64_t ir_pipeline_frames_fail(ir_pipeline_t *p);

void ir_pipeline_destroy(ir_pipeline_t *p);

#ifdef __cplusplus
}
#endif
#endif // IRIDISCOPE_PIPELINE_C_H
