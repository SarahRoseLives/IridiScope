/*
 * FFT burst detector - port of gr-iridium's fft_burst_tagger
 *
 * Original work Copyright 2020 Free Software Foundation, Inc.
 * Modifications Copyright 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/*
 * FFT burst detector - port of gr-iridium's fft_burst_tagger
 *
 * Detects Iridium satellite bursts using sliding FFT with adaptive
 * noise floor estimation. Accumulates IQ samples per burst and pushes
 * complete burst buffers to burst_queue when burst ends.
 */

#ifndef __BURST_DETECT_H__
#define __BURST_DETECT_H__

#include <complex.h>
#include <stdint.h>
#include <fftw3.h>

/* Forward declaration */
struct _burst_detector;
typedef struct _burst_detector burst_detector_t;

/* Detected burst metadata */
typedef struct {
    uint64_t id;
    uint64_t start;         /* absolute sample index */
    uint64_t stop;          /* absolute sample index */
    uint64_t last_active;   /* last sample index above threshold */
    int center_bin;         /* FFT bin of burst center */
    float magnitude;        /* SNR in dB */
    float noise;            /* noise floor in dBFS/Hz */
    float width_hz;         /* IridiScope: mean occupied width while active */
    float wide_frac;        /* IridiScope: share of frames >= IR_WIDE_CARRIER_HZ wide */
} burst_info_t;

/* IridiScope wide-carrier detection: width is measured at IR_WIDTH_LEVEL_DB
 * above the noise floor; a classic Iridium burst is ~35 kHz wide there. */
#ifndef IR_WIDTH_LEVEL_DB
#define IR_WIDTH_LEVEL_DB 6.0f
#endif
#ifndef IR_WIDE_CARRIER_HZ
#define IR_WIDE_CARRIER_HZ 150000.0
#endif

/* Complete burst with IQ data, ready for downstream processing */
typedef struct {
    burst_info_t info;
    double center_frequency;  /* absolute center freq of capture */
    int sample_rate;          /* capture sample rate */
    int fft_size;             /* FFT size used for detection */
    uint64_t start_time_ns;   /* wall clock ns at sample 0 (base offset) */
    size_t num_samples;       /* number of complex float samples */
    float complex *samples;   /* IQ data covering burst lifetime */
    const void *ring_owner;   /* IridiScope: detector whose ring holds samples (NULL = owned copy) */
    uint64_t ring_start;      /* absolute sample index of samples[0] */
} burst_data_t;

/* Configuration */
typedef struct {
    double center_frequency;
    int sample_rate;
    int fft_size;           /* 0 = auto-calculate (~1ms window) */
    int burst_pre_len;      /* 0 = auto (2 * fft_size) */
    int burst_post_len;     /* 0 = auto (sample_rate * 16ms) */
    int burst_width;        /* Hz, default 40000 */
    int max_bursts;         /* 0 = auto (80% of channels) */
    int max_burst_len;      /* 0 = auto (sample_rate * 90ms) */
    float threshold;        /* dB, default 16.0 */
    int history_size;       /* default 512 */
    int use_gpu;            /* 1 = use OpenCL GPU FFT, 0 = FFTW CPU */
} burst_config_t;

/* Create a burst detector with the given configuration.
 * Unset fields (0) get reasonable defaults. */
burst_detector_t *burst_detector_create(burst_config_t *config);

/* Callback for completed bursts. Receives ownership of burst_data_t
 * (caller must free both burst->samples and burst itself). */
typedef void (*burst_callback_t)(burst_data_t *burst, void *user);

/* Feed int8 IQ samples to the detector. */
void burst_detector_feed(burst_detector_t *det, const int8_t *iq,
                         size_t num_samples, burst_callback_t cb, void *user);

/* Feed float32 interleaved IQ samples (no int8 quantization loss). */
void burst_detector_feed_cf32(burst_detector_t *det, const float *iq,
                              size_t num_samples, burst_callback_t cb, void *user);

/* Get number of active bursts */
int burst_detector_active_count(burst_detector_t *det);

/* Get total detected burst count */
uint64_t burst_detector_total_count(burst_detector_t *det);

/* Get average noise floor in dBFS/Hz (for diagnostic display) */
float burst_detector_noise_floor(burst_detector_t *det);

/* Get peak signal level in dB (for diagnostic display) */
float burst_detector_peak_signal(burst_detector_t *det);

/* Destroy and free all resources */
void burst_detector_destroy(burst_detector_t *det);

/* Forget the learned noise floor (e.g. after a gain or tuning change) and
 * drop bursts in progress; detection resumes once the baseline is re-primed.
 * Not thread-safe: call from the thread that feeds the detector. */
void burst_detector_reset_noise(burst_detector_t *det);

/* IridiScope: time base control (see burst_detect.c). */
void burst_detector_skip_time(burst_detector_t *det, uint64_t ns);
void burst_detector_set_start_time(burst_detector_t *det, uint64_t ns);

/* Burst sample buffers come from a recycling pool (IridiScope): each burst
 * carries ~2 MB at 10 Msps, and a fresh allocation of that size costs page
 * faults and zeroing every time. Free with burst_buf_free(), not free(). */
void *burst_buf_alloc(size_t bytes);
void burst_buf_free(void *buf);
/* Pool statistics: allocations served from idle buffers vs fresh. */
void burst_buf_stats(uint64_t *hits, uint64_t *misses);
/* Bytes currently in use by queued/in-flight bursts, and held idle. */
void burst_buf_memory(size_t *live, size_t *idle);
/* Idle memory kept for reuse (normally the queue budget), and the minimum
 * buffer size (normally a typical full burst at this sample rate). */
void burst_buf_configure(size_t max_idle_bytes, size_t min_bytes);
/* Call regularly from the detector thread: returns idle memory to the OS
 * after it has gone unused for a minute. */
void burst_buf_maintain(void);
/* IridiScope shared ring: bursts point into the detector's ring buffer, which
 * holds `seconds` of IQ, instead of each getting a copy. Set before creating
 * the detector (0 = copy mode). Readers must check burst_samples_intact()
 * AFTER reading: 0 means the ring overwrote the burst meanwhile (the
 * decoders fell further behind than the ring holds) and the result must be
 * discarded. Release with burst_release_samples(). */
void burst_detector_set_shared_ring(double seconds);
/* Wide-carrier filter (on by default): see burst_detect.c. */
void burst_detector_set_reject_wide(int on);
int burst_detector_reject_wide(void);
uint64_t burst_detector_wide_dropped(void);
int burst_samples_intact(const burst_data_t *b);
void burst_release_samples(burst_data_t *b);
int burst_detector_ring_burst(burst_detector_t *d, uint64_t start, size_t len,
                              int center_bin, burst_data_t *bd);
uint64_t burst_detector_sample_count(const burst_detector_t *d);
uint64_t burst_detector_start_time(const burst_detector_t *d);
int burst_detector_fft_size(const burst_detector_t *d);

/* Thread function: pulls from samples_queue, pushes to burst_queue */
void *burst_detector_thread(void *arg);

#endif
