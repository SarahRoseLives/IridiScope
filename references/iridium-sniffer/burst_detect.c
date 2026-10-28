/*
 * Burst detection pipeline - port of gr-iridium's fft_burst_tagger_impl.cc
 *
 * Windowed FFT → Magnitude computation → Threshold detection → Burst aggregation
 * with hysteresis and max-hold
 *
 * Original work Copyright 2020 Free Software Foundation, Inc.
 * Modifications Copyright 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#define _GNU_SOURCE
#include <complex.h>
#include <err.h>
#include <math.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <dlfcn.h>
#include <fftw3.h>

#ifdef __APPLE__
#include <mach-o/dyld.h>
#endif

#include "config.h"
#include <pthread.h>

#include "burst_detect.h"
#include "fftw_lock.h"
#include "iridium.h"
#include "sdr.h"
#include "simd_kernels.h"
#include "window_func.h"

#include "blocking_queue.h"

/* Forward declaration for opaque GPU context (loaded via dlopen plugin) */
typedef struct gpu_burst_fft gpu_burst_fft_t;

/* ---- Internal types ---- */

typedef struct {
    uint64_t id;
    uint64_t start;
    uint64_t stop;
    uint64_t last_active;
    int center_bin;
    float magnitude;
    float noise;
    /* IridiScope: occupied width of the signal around center_bin, measured
     * every FFT frame the burst is active (wide-carrier detection). */
    int w_frames;
    int w_wide;
    double w_sum_bins;
} active_burst_t;

typedef struct {
    int bin;
    float relative_magnitude;
} peak_t;

/* ---- Burst detector state ---- */

struct _burst_detector {
    /* Configuration */
    double center_frequency;
    int sample_rate;
    int fft_size;
    int burst_pre_len;
    int burst_post_len;
    int burst_width;        /* in FFT bins */
    int max_bursts;
    int max_burst_len;
    float threshold;        /* pre-computed: pow(10, dB/10) / history_size / ENBW */
    float width_level;      /* IridiScope: kWidthLevelDb above noise, same units */
    int wide_bins;          /* IridiScope: width counted as "wide" (bins) */
    int history_size;

    /* FFT */
    fftwf_plan fft_plan;
    float complex *fft_in;
    float complex *fft_out;

    /* Window */
    float *window;

    /* Noise floor estimation */
    float *baseline_history;    /* [history_size * fft_size] circular */
    float *baseline_sum;        /* [fft_size] running sum */
    int history_index;
    int history_primed;

    /* Per-FFT frame */
    float *magnitude_shifted;   /* [fft_size] DC-shifted mag^2 */
    float *relative_magnitude;  /* [fft_size] current / baseline */
    float *burst_mask;          /* [fft_size] 0 where burst active, 1 elsewhere */
    float *ones;                /* [fft_size] all 1.0 */

    /* Burst tracking */
    active_burst_t *bursts;
    int num_bursts;
    int bursts_cap;

    active_burst_t *new_bursts;
    int num_new_bursts;
    int new_bursts_cap;

    active_burst_t *gone_bursts;
    int num_gone_bursts;
    int gone_bursts_cap;

    peak_t *peaks;
    int num_peaks;

    uint64_t burst_id;
    uint64_t n_tagged_bursts;
    uint64_t sample_count;
    uint64_t index;             /* current absolute sample position */
    int squelch_count;

    /* Diagnostic tracking */
    float peak_signal_db;       /* maximum signal seen (for diagnostic mode) */

    /* IQ ringbuffer for burst sample extraction */
    float complex *ringbuf;
    size_t ringbuf_size;        /* total capacity in samples */
    size_t ringbuf_write;       /* write position (mod ringbuf_size) */
    uint64_t ringbuf_start;     /* absolute sample index of oldest sample */
    /* IridiScope shared ring: bursts point straight into the ringbuffer
     * instead of being copied. ring_mirror samples after the end repeat the
     * start, so every burst is contiguous. ring_claim = absolute index up to
     * which the writer may have overwritten (published before writing), so a
     * reader can tell whether a burst's samples are still intact. */
    int shared_ring;
    size_t ring_mirror;
    atomic_ullong ring_claim;

    /* int8 -> float complex conversion buffer */
    float complex *convert_buf;
    size_t convert_buf_size;

    /* Timestamp */
    uint64_t start_time_ns;     /* nanosecond timestamp at sample 0 */
    uint64_t pending_hw_ts;     /* hardware timestamp for next feed (0=none) */
    uint64_t pending_skip_ns;   /* IridiScope: time skipped before start_time is known */

    /* GPU plugin (loaded via dlopen) */
    void *gpu_handle;           /* dlopen handle */
    gpu_burst_fft_t *gpu;
    int gpu_batch_size;         /* max frames per GPU dispatch */
    int gpu_batch_count;        /* frames accumulated so far */
    float *gpu_batch_input;     /* batch_size * fft_size * 2 floats (complex) */
    float *gpu_batch_output;    /* batch_size * fft_size floats (magnitude) */
    /* Function pointers from plugin */
    gpu_burst_fft_t *(*gpu_create)(int, int, const float *);
    void (*gpu_destroy)(gpu_burst_fft_t *);
    int (*gpu_process)(gpu_burst_fft_t *, const float *, float *, int);
};

/* ---- Externs for threading integration ---- */

extern Blocking_Queue samples_queue;
extern Blocking_Queue burst_queue;
extern volatile sig_atomic_t running;
extern int verbose;
extern atomic_ulong stat_n_detected;
extern atomic_ulong stat_n_dropped;

/* ---- Helper: dynamic array push ---- */

static void push_burst(active_burst_t **arr, int *count, int *cap, active_burst_t *b) {
    if (*count >= *cap) {
        *cap = *cap ? *cap * 2 : 64;
        *arr = realloc(*arr, *cap * sizeof(active_burst_t));
    }
    (*arr)[(*count)++] = *b;
}

static void remove_burst(active_burst_t *arr, int *count, int idx) {
    if (idx < *count - 1)
        memmove(&arr[idx], &arr[idx + 1], (*count - idx - 1) * sizeof(active_burst_t));
    (*count)--;
}

/* ---- Peak comparison for qsort (descending magnitude) ---- */

static int peak_cmp_desc(const void *a, const void *b) {
    float ma = ((const peak_t *)a)->relative_magnitude;
    float mb = ((const peak_t *)b)->relative_magnitude;
    if (ma > mb) return -1;
    if (ma < mb) return 1;
    return 0;
}

/* ---- GPU plugin loader ---- */

static int gpu_plugin_load(burst_detector_t *d)
{
    /* Try next to the executable first, then standard paths */
    char exe_path[4096] = {0};
    char exe_dir_lib[4096] = {0};

    #ifdef __linux__
    ssize_t len = readlink("/proc/self/exe", exe_path, sizeof(exe_path) - 1);
    if (len > 0) {
        exe_path[len] = '\0';
        char *slash = strrchr(exe_path, '/');
        if (slash) {
            *slash = '\0';
            snprintf(exe_dir_lib, sizeof(exe_dir_lib),
                     "%s/" GPU_PLUGIN_NAME, exe_path);
        }
    }
#elif defined(__APPLE__)
    uint32_t exe_path_size = sizeof(exe_path);
    if (_NSGetExecutablePath(exe_path, &exe_path_size) == 0) {
        char *slash = strrchr(exe_path, '/');
        if (slash) {
            *slash = '\0';
            snprintf(exe_dir_lib, sizeof(exe_dir_lib),
                     "%s/" GPU_PLUGIN_NAME, exe_path);
        }
    }
#endif

    const char *names[] = {
        exe_dir_lib[0] ? exe_dir_lib : NULL, /* next to executable */
        GPU_PLUGIN_NAME,                     /* standard library path */
        "./" GPU_PLUGIN_NAME,                /* current directory */
        NULL
    };

    for (int i = 0; names[i]; i++) {
        d->gpu_handle = dlopen(names[i], RTLD_NOW);
        if (d->gpu_handle) break;
    }

    if (!d->gpu_handle)
        return -1;

    d->gpu_create  = dlsym(d->gpu_handle, "gpu_burst_fft_create");
    d->gpu_destroy = dlsym(d->gpu_handle, "gpu_burst_fft_destroy");
    d->gpu_process = dlsym(d->gpu_handle, "gpu_burst_fft_process");

    if (!d->gpu_create || !d->gpu_destroy || !d->gpu_process) {
        dlclose(d->gpu_handle);
        d->gpu_handle = NULL;
        return -1;
    }

    return 0;
}

/* ---- Create burst detector ---- */

/* IridiScope: seconds of IQ the shared ring holds for the decoders (0 = the
 * original copy-per-burst mode). Applies to detectors created afterwards. */
static double g_shared_ring_seconds = 0.0;
void burst_detector_set_shared_ring(double seconds) { g_shared_ring_seconds = seconds; }

burst_detector_t *burst_detector_create(burst_config_t *config) {
    burst_detector_t *d = calloc(1, sizeof(*d));

    d->center_frequency = config->center_frequency;
    d->sample_rate = config->sample_rate;

    /* FFT size: ~1ms window, nearest power of 2 */
    if (config->fft_size > 0) {
        d->fft_size = config->fft_size;
    } else {
        int n = (int)round(log2(config->sample_rate / 1000.0));
        d->fft_size = 1 << n;
    }

    /* Burst pre/post lengths */
    d->burst_pre_len = config->burst_pre_len > 0
        ? config->burst_pre_len
        : 2 * d->fft_size;

    d->burst_post_len = config->burst_post_len > 0
        ? config->burst_post_len
        : (int)(config->sample_rate * 16e-3);

    /* Burst width in FFT bins */
    int burst_width_hz = config->burst_width > 0
        ? config->burst_width
        : IR_DEFAULT_BURST_WIDTH;
    d->burst_width = burst_width_hz / (config->sample_rate / d->fft_size);

    /* Max bursts */
    if (config->max_bursts > 0) {
        d->max_bursts = config->max_bursts;
    } else {
        d->max_bursts = (int)((config->sample_rate / (float)burst_width_hz) * 0.8f);
    }

    /* Max burst length in samples */
    d->max_burst_len = config->max_burst_len > 0
        ? config->max_burst_len
        : (int)(config->sample_rate * 0.09);

    d->history_size = config->history_size > 0
        ? config->history_size
        : IR_DEFAULT_HISTORY_SIZE;

    /* Threshold: convert from dB to linear, normalized */
    float threshold_db = config->threshold > 0
        ? config->threshold
        : IR_DEFAULT_THRESHOLD;

    /* ENBW of Blackman window */
    float window_enbw = 1.72f;
    d->threshold = powf(10.0f, threshold_db / 10.0f) / d->history_size / window_enbw;
    d->width_level = powf(10.0f, IR_WIDTH_LEVEL_DB / 10.0f) / d->history_size / window_enbw;
    d->wide_bins = (int)(IR_WIDE_CARRIER_HZ / ((double)d->sample_rate / d->fft_size));

    if (verbose) {
        fprintf(stderr, "burst_detect: fft_size=%d, threshold=%.1f dB (linear=%e), "
                "history=%d, burst_width=%d bins, max_bursts=%d, "
                "pre_len=%d, post_len=%d, max_len=%d\n",
                d->fft_size, threshold_db, d->threshold,
                d->history_size, d->burst_width, d->max_bursts,
                d->burst_pre_len, d->burst_post_len, d->max_burst_len);
    }

    /* Allocate FFT */
    d->fft_in = fftwf_alloc_complex(d->fft_size);
    d->fft_out = fftwf_alloc_complex(d->fft_size);
    fftw_lock();
    d->fft_plan = fftwf_plan_dft_1d(d->fft_size,
                                     d->fft_in, d->fft_out,
                                     FFTW_FORWARD, FFTW_MEASURE);
    fftw_unlock();
    if (!d->fft_plan)
        errx(1, "burst_detect: FFTW plan failed (size %d)", d->fft_size);

    /* Window: Blackman scaled by 1/0.42 for accurate SNR */
    d->window = aligned_alloc_32(sizeof(float) * d->fft_size);
    blackman_window(d->window, d->fft_size);
    for (int i = 0; i < d->fft_size; i++)
        d->window[i] /= 0.42f;

    /* Noise floor arrays (aligned for SIMD) */
    d->baseline_history = aligned_calloc_32(d->fft_size * d->history_size, sizeof(float));
    d->baseline_sum = aligned_calloc_32(d->fft_size, sizeof(float));
    d->magnitude_shifted = aligned_calloc_32(d->fft_size, sizeof(float));
    d->relative_magnitude = aligned_calloc_32(d->fft_size, sizeof(float));
    d->burst_mask = aligned_alloc_32(sizeof(float) * d->fft_size);
    d->ones = aligned_alloc_32(sizeof(float) * d->fft_size);

    for (int i = 0; i < d->fft_size; i++) {
        d->burst_mask[i] = 1.0f;
        d->ones[i] = 1.0f;
    }

    d->history_index = 0;
    d->history_primed = 0;

    /* Peak array */
    d->peaks = malloc(sizeof(peak_t) * d->fft_size);
    d->num_peaks = 0;

    /* Burst arrays (start small, grow as needed) */
    d->bursts_cap = 64;
    d->bursts = malloc(sizeof(active_burst_t) * d->bursts_cap);
    d->num_bursts = 0;

    d->new_bursts_cap = 64;
    d->new_bursts = malloc(sizeof(active_burst_t) * d->new_bursts_cap);
    d->num_new_bursts = 0;

    d->gone_bursts_cap = 64;
    d->gone_bursts = malloc(sizeof(active_burst_t) * d->gone_bursts_cap);
    d->num_gone_bursts = 0;

    d->burst_id = 0;
    d->n_tagged_bursts = 0;
    d->sample_count = 0;
    d->index = 0;
    d->squelch_count = 0;

    /* IQ ringbuffer: hold enough for max burst + pre + post + headroom  (+25%)*/
    size_t minimum = (size_t)d->max_burst_len + d->burst_post_len + d->burst_pre_len;
    d->ringbuf_size = minimum + minimum / 4;   /* 1.25x minimum needed */
    d->shared_ring = 0;
    d->ring_mirror = 0;
    atomic_store(&d->ring_claim, 0);
    if (g_shared_ring_seconds > 0) {
        /* IridiScope: the ring also holds the decoders' backlog. */
        size_t want = (size_t)(g_shared_ring_seconds * config->sample_rate);
        if (want > d->ringbuf_size) d->ringbuf_size = want;
        d->shared_ring = 1;
        d->ring_mirror = minimum + 8 * (size_t)d->fft_size;
    }
    d->ringbuf = malloc(sizeof(float complex) * (d->ringbuf_size + d->ring_mirror));
    d->ringbuf_write = 0;
    d->ringbuf_start = 0;

    /* Timestamp: set when first samples arrive */
    d->start_time_ns = 0;

    /* GPU acceleration (loaded via dlopen plugin) */
    d->gpu = NULL;
    d->gpu_handle = NULL;
    if (config->use_gpu) {
        if (gpu_plugin_load(d) == 0) {
            d->gpu_batch_size = 16;
            d->gpu = d->gpu_create(d->fft_size, d->gpu_batch_size, d->window);
            if (d->gpu) {
                d->gpu_batch_count = 0;
                d->gpu_batch_input = malloc(sizeof(float) * 2 * d->fft_size
                                            * d->gpu_batch_size);
                d->gpu_batch_output = malloc(sizeof(float) * d->fft_size
                                             * d->gpu_batch_size);
            } else {
                fprintf(stderr, "burst_detect: GPU init failed, falling back to CPU\n");
                dlclose(d->gpu_handle);
                d->gpu_handle = NULL;
            }
        } else {
            fprintf(stderr, "burst_detect: GPU plugin not found, using CPU\n");
        }
    }

    return d;
}

void burst_detector_destroy(burst_detector_t *d) {
    if (!d) return;
    if (d->gpu) {
        d->gpu_destroy(d->gpu);
        free(d->gpu_batch_input);
        free(d->gpu_batch_output);
    }
    if (d->gpu_handle)
        dlclose(d->gpu_handle);
    fftwf_destroy_plan(d->fft_plan);
    fftwf_free(d->fft_in);
    fftwf_free(d->fft_out);
    free(d->window);
    free(d->baseline_history);
    free(d->baseline_sum);
    free(d->magnitude_shifted);
    free(d->relative_magnitude);
    free(d->burst_mask);
    free(d->ones);
    free(d->peaks);
    free(d->bursts);
    free(d->new_bursts);
    free(d->gone_bursts);
    free(d->ringbuf);
    free(d->convert_buf);
    fprintf(stderr, "burst_detect: tagged %lu bursts total\n",
            (unsigned long)d->n_tagged_bursts);
    free(d);
}

int burst_detector_active_count(burst_detector_t *d) {
    return d->num_bursts;
}

uint64_t burst_detector_total_count(burst_detector_t *d) {
    return d->n_tagged_bursts;
}

float burst_detector_noise_floor(burst_detector_t *d) {
    if (!d || !d->baseline_sum || d->history_size == 0)
        return 0.0f;

    /* Average baseline across all FFT bins */
    double sum = 0;
    for (int i = 0; i < d->fft_size; i++)
        sum += d->baseline_sum[i];

    float avg = (float)(sum / (d->fft_size * d->history_size));

    /* Convert to dBFS/Hz: 10*log10(mag^2 / bin_width) */
    float bin_width = (float)d->sample_rate / d->fft_size;
    if (avg > 0 && bin_width > 0)
        return 10.0f * log10f(avg / bin_width);

    return -120.0f;  /* Return very low value if no data */
}

float burst_detector_peak_signal(burst_detector_t *d) {
    if (!d)
        return 0.0f;
    return d->peak_signal_db;
}

/* ---- Internal: ringbuffer operations ---- */

static void ring_mirror_copy(burst_detector_t *d, size_t pos, const float complex *src, size_t n) {
    /* Repeat writes to the first ring_mirror samples after the end. */
    if (!d->ring_mirror || pos >= d->ring_mirror) return;
    size_t k = n;
    if (pos + k > d->ring_mirror) k = d->ring_mirror - pos;
    memcpy(&d->ringbuf[d->ringbuf_size + pos], src, k * sizeof(float complex));
}

static void ringbuf_write(burst_detector_t *d, const float complex *samples, size_t n) {
    size_t pos = d->ringbuf_write;
    if (d->shared_ring) {
        /* Announce the overwrite before doing it (readers check afterwards). */
        atomic_store(&d->ring_claim, (unsigned long long)(d->sample_count + n));
        atomic_thread_fence(memory_order_seq_cst);
        if (pos + n <= d->ringbuf_size) {
            ring_mirror_copy(d, pos, samples, n);
        } else {
            size_t first = d->ringbuf_size - pos;
            ring_mirror_copy(d, 0, samples + first, n - first);
        }
    }
    if (pos + n <= d->ringbuf_size) {
        memcpy(&d->ringbuf[pos], samples, n * sizeof(float complex));
        d->ringbuf_write = pos + n;
        if (d->ringbuf_write == d->ringbuf_size)
            d->ringbuf_write = 0;
    } else {
        size_t first = d->ringbuf_size - pos;
        memcpy(&d->ringbuf[pos], samples, first * sizeof(float complex));
        memcpy(d->ringbuf, samples + first, (n - first) * sizeof(float complex));
        d->ringbuf_write = n - first;
    }
    /* Update the oldest available sample index */
    uint64_t total_written = d->sample_count;
    if (total_written > d->ringbuf_size)
        d->ringbuf_start = total_written - d->ringbuf_size;
}


/* ---- Burst buffer pool (IridiScope) ----
 * Every burst carries ~2 MB of full-rate IQ. A fresh allocation that size is
 * served by the OS (VirtualAlloc on Windows) and costs page faults plus
 * zeroing on first touch (measured ~4x the cost of the copy itself), and its
 * release unmaps memory again. If buffers go back to the OS whenever the
 * queue shrinks, every regrowth of the queue - which happens exactly when
 * decoding is already behind - makes the detector several times slower per
 * burst, and it falls further behind.
 *
 * So: buffers are kept for reuse up to the burst queue's memory budget, and
 * idle memory is only given back after it has gone unused for a whole minute
 * (RAM drops again after a busy spell, but not in the middle of one).
 * Requests below a minimum size are rounded up to it, so the usual bursts
 * (which vary in length) all share one size and any idle buffer fits.
 * Allocation happens on the detector thread, release on worker threads. */
#define BPOOL_STEP     (256u * 1024u)
#define BPOOL_CLASSES  128
#ifndef BPOOL_TRIM_NS
#define BPOOL_TRIM_NS  60000000000ULL
#endif
typedef struct { size_t cls; size_t pad[7]; } bpool_hdr_t; /* keeps data 64-byte aligned */
typedef struct { bpool_hdr_t **v; int n, cap; } bpool_stack_t;
static pthread_mutex_t g_bpool_mtx = PTHREAD_MUTEX_INITIALIZER;
static bpool_stack_t g_bpool[BPOOL_CLASSES];
static size_t g_bpool_idle;                     /* bytes held idle */
static size_t g_bpool_live;                     /* bytes handed out */
static size_t g_bpool_max_idle = (size_t)4000 * 1000 * 1000;
static size_t g_bpool_min_bytes;                /* requests are rounded up to this */
static size_t g_bpool_idle_min;                 /* lowest idle level this trim window */
static uint64_t g_bpool_window_ns;              /* start of the trim window */
static uint64_t g_bpool_hits, g_bpool_misses;

static uint64_t bpool_now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

void burst_buf_stats(uint64_t *hits, uint64_t *misses) {
    pthread_mutex_lock(&g_bpool_mtx);
    if (hits) *hits = g_bpool_hits;
    if (misses) *misses = g_bpool_misses;
    pthread_mutex_unlock(&g_bpool_mtx);
}

void burst_buf_memory(size_t *live, size_t *idle) {
    pthread_mutex_lock(&g_bpool_mtx);
    if (live) *live = g_bpool_live;
    if (idle) *idle = g_bpool_idle;
    pthread_mutex_unlock(&g_bpool_mtx);
}

void burst_buf_configure(size_t max_idle_bytes, size_t min_bytes) {
    pthread_mutex_lock(&g_bpool_mtx);
    g_bpool_max_idle = max_idle_bytes;
    g_bpool_min_bytes = min_bytes;
    pthread_mutex_unlock(&g_bpool_mtx);
}

void *burst_buf_alloc(size_t bytes) {
#ifdef BPOOL_DISABLE /* test builds only: behave like plain malloc */
    { bpool_hdr_t *h = malloc(bytes + sizeof(bpool_hdr_t)); if (!h) return NULL; h->cls = (size_t)-1; g_bpool_misses++; return h + 1; }
#endif
    pthread_mutex_lock(&g_bpool_mtx);
    if (bytes < g_bpool_min_bytes) bytes = g_bpool_min_bytes;
    const size_t need = bytes + sizeof(bpool_hdr_t);
    const size_t cls = (need + BPOOL_STEP - 1) / BPOOL_STEP - 1;
    if (cls >= BPOOL_CLASSES) {
        pthread_mutex_unlock(&g_bpool_mtx);
        bpool_hdr_t *h = malloc(need);
        if (!h) return NULL;
        h->cls = (size_t)-1;
        return h + 1;
    }
    /* Exact size first, then any larger idle buffer (better than fresh). */
    for (size_t c = cls; c < BPOOL_CLASSES; c++) {
        bpool_stack_t *st = &g_bpool[c];
        if (st->n > 0) {
            bpool_hdr_t *h = st->v[--st->n];
            const size_t sz = (c + 1) * BPOOL_STEP;
            g_bpool_idle -= sz;
            g_bpool_live += sz;
            if (g_bpool_idle < g_bpool_idle_min) g_bpool_idle_min = g_bpool_idle;
            g_bpool_hits++;
            pthread_mutex_unlock(&g_bpool_mtx);
            return h + 1;
        }
    }
    const size_t sz = (cls + 1) * BPOOL_STEP;
    g_bpool_misses++;
    g_bpool_live += sz;
    g_bpool_idle_min = 0;   /* the pool ran dry: nothing idle was spare */
    pthread_mutex_unlock(&g_bpool_mtx);
    bpool_hdr_t *h = malloc(sz);
    if (!h) {
        pthread_mutex_lock(&g_bpool_mtx);
        g_bpool_live -= sz;
        pthread_mutex_unlock(&g_bpool_mtx);
        return NULL;
    }
    h->cls = cls;
    return h + 1;
}

void burst_buf_free(void *buf) {
    if (!buf) return;
    bpool_hdr_t *h = (bpool_hdr_t *)buf - 1;
    if (h->cls == (size_t)-1) { free(h); return; }
    const size_t sz = (h->cls + 1) * BPOOL_STEP;
    pthread_mutex_lock(&g_bpool_mtx);
    g_bpool_live -= sz;
    bpool_stack_t *st = &g_bpool[h->cls];
    if (g_bpool_idle + sz <= g_bpool_max_idle) {
        if (st->n == st->cap) {
            const int nc = st->cap ? st->cap * 2 : 64;
            bpool_hdr_t **nv = realloc(st->v, sizeof(*nv) * (size_t)nc);
            if (nv) { st->v = nv; st->cap = nc; }
        }
        if (st->n < st->cap) {
            st->v[st->n++] = h;
            g_bpool_idle += sz;
            pthread_mutex_unlock(&g_bpool_mtx);
            return;
        }
    }
    pthread_mutex_unlock(&g_bpool_mtx);
    free(h);
}

/* Called regularly from the detector thread: once a minute, give back the
 * idle memory that stayed unused for that whole minute. */
void burst_buf_maintain(void) {
    const uint64_t now = bpool_now_ns();
    bpool_hdr_t *rel[32];
    int nrel = 0;
    pthread_mutex_lock(&g_bpool_mtx);
    if (g_bpool_window_ns == 0) {
        g_bpool_window_ns = now;
        g_bpool_idle_min = g_bpool_idle;
    }
    if (now - g_bpool_window_ns < BPOOL_TRIM_NS) { pthread_mutex_unlock(&g_bpool_mtx); return; }
    size_t spare = g_bpool_idle_min;
    /* Release the largest buffers first (rare long bursts), then the rest;
     * at most 32 per call so no single call stalls the detector. */
    for (int c = BPOOL_CLASSES - 1; c >= 0 && spare > 0 && nrel < 32; c--) {
        bpool_stack_t *st = &g_bpool[c];
        const size_t sz = (size_t)(c + 1) * BPOOL_STEP;
        while (st->n > 0 && spare >= sz && nrel < 32) {
            rel[nrel++] = st->v[--st->n];
            g_bpool_idle -= sz;
            spare -= sz;
        }
    }
    if (nrel < 32 || spare == 0) {   /* done: start the next window */
        g_bpool_window_ns = now;
        g_bpool_idle_min = g_bpool_idle;
    } else {
        g_bpool_idle_min = spare;   /* more to release: continue next call */
    }
    pthread_mutex_unlock(&g_bpool_mtx);
    for (int i = 0; i < nrel; i++) free(rel[i]);
}

static float complex *ringbuf_extract(burst_detector_t *d, uint64_t start,
                                       uint64_t stop, size_t *out_len) {
    /* Clamp to available range */
    if (start < d->ringbuf_start)
        start = d->ringbuf_start;
    if (stop <= start) {
        *out_len = 0;
        return NULL;
    }

    size_t len = (size_t)(stop - start);
    float complex *buf = burst_buf_alloc(sizeof(float complex) * len);
    size_t rb_pos = (size_t)(start % d->ringbuf_size);

    if (rb_pos + len <= d->ringbuf_size) {
        memcpy(buf, &d->ringbuf[rb_pos], len * sizeof(float complex));
    } else {
        size_t first = d->ringbuf_size - rb_pos;
        memcpy(buf, &d->ringbuf[rb_pos], first * sizeof(float complex));
        memcpy(buf + first, d->ringbuf, (len - first) * sizeof(float complex));
    }

    *out_len = len;
    return buf;
}

/* ---- Shared ring: burst sample lifetime (IridiScope) ---- */

int burst_samples_intact(const burst_data_t *b) {
    if (!b || !b->ring_owner) return 1;
    const burst_detector_t *d = (const burst_detector_t *)b->ring_owner;
    atomic_thread_fence(memory_order_seq_cst);
    const uint64_t claim = atomic_load(&((burst_detector_t *)d)->ring_claim);
    return claim <= b->ring_start + d->ringbuf_size;
}

/* Targeted reads (IridiScope): a view of [start, start+len) in the shared
 * ring as a burst, for decoding a burst the detector didn't trigger on.
 * Detector thread only. Returns 0 if those samples aren't in the ring. */
int burst_detector_ring_burst(burst_detector_t *d, uint64_t start, size_t len,
                              int center_bin, burst_data_t *bd) {
    if (!d || !d->shared_ring || len == 0 || len > d->ring_mirror) return 0;
    const uint64_t oldest = d->sample_count > d->ringbuf_size ? d->sample_count - d->ringbuf_size : 0;
    if (start < oldest || start + len > d->sample_count) return 0;
    memset(bd, 0, sizeof(*bd));
    bd->info.start = start;
    bd->info.stop = start + len;
    bd->info.last_active = start + len;
    bd->info.center_bin = center_bin;
    bd->center_frequency = d->center_frequency;
    bd->sample_rate = d->sample_rate;
    bd->fft_size = d->fft_size;
    bd->start_time_ns = d->start_time_ns;
    bd->num_samples = len;
    bd->samples = &d->ringbuf[start % d->ringbuf_size];
    bd->ring_owner = d;
    bd->ring_start = start;
    return 1;
}
uint64_t burst_detector_sample_count(const burst_detector_t *d) { return d ? d->sample_count : 0; }
uint64_t burst_detector_start_time(const burst_detector_t *d) { return d ? d->start_time_ns : 0; }
int burst_detector_fft_size(const burst_detector_t *d) { return d ? d->fft_size : 0; }

void burst_release_samples(burst_data_t *b) {
    if (!b) return;
    if (!b->ring_owner) burst_buf_free(b->samples);
    b->samples = NULL;
}

/* ---- Internal: update noise floor (pre) ---- */

static int update_filters_pre(burst_detector_t *d) {
    if (!d->history_primed)
        return 0;

    /* relative_magnitude = magnitude_shifted / baseline_sum (SIMD-accelerated) */
    simd_relative_mag(d->magnitude_shifted, d->baseline_sum,
                      d->relative_magnitude, d->fft_size);
    return 1;
}

/* ---- Internal: update noise floor (post) ---- */

static void update_filters_post(burst_detector_t *d, int force) {
    /* Only update average when no bursts active (or forced) */
    if (d->num_bursts == 0 || force) {
        float *hist = d->baseline_history + d->history_index * d->fft_size;

        /* Baseline update: sum = sum - old_hist + new_mag (SIMD-accelerated) */
        simd_baseline_update(d->baseline_sum, hist,
                             d->magnitude_shifted, d->fft_size);
        memcpy(hist, d->magnitude_shifted, sizeof(float) * d->fft_size);

        d->history_index++;
        if (d->history_index == d->history_size) {
            d->history_primed = 1;
            d->history_index = 0;
        }
    }
}

/* ---- Internal: update active bursts ---- */

/* IridiScope: width (bins) of the signal around bin cb, walking outwards
 * while bins are above width_level, tolerating short dips. Capped so a
 * frame costs at most ~2*cap bins per active burst. */
static int signal_width_bins(const burst_detector_t *d, int cb) {
    const float *rm = d->relative_magnitude;
    const float lvl = d->width_level;
    const int gap_max = 3, cap = d->wide_bins * 2;
    int lo = cb, hi = cb, gap = 0;
    for (int b = cb - 1; b >= 0 && cb - b <= cap; b--) {
        if (rm[b] > lvl) { lo = b; gap = 0; } else if (++gap > gap_max) break;
    }
    gap = 0;
    for (int b = cb + 1; b < d->fft_size && b - cb <= cap; b++) {
        if (rm[b] > lvl) { hi = b; gap = 0; } else if (++gap > gap_max) break;
    }
    return hi - lo + 1;
}

static void update_bursts(burst_detector_t *d) {
    for (int i = 0; i < d->num_bursts; i++) {
        active_burst_t *b = &d->bursts[i];
        int cb = b->center_bin;
        /* Check center bin and neighbors */
        if ((cb > 0 && d->relative_magnitude[cb - 1] > d->threshold) ||
            d->relative_magnitude[cb] > d->threshold ||
            (cb < d->fft_size - 1 && d->relative_magnitude[cb + 1] > d->threshold)) {
            b->last_active = d->index;
            const int w = signal_width_bins(d, cb);
            b->w_frames++;
            b->w_sum_bins += w;
            if (w >= d->wide_bins) b->w_wide++;
        }
    }
}

/* ---- Internal: mask a burst's frequency range ---- */

static void mask_burst(burst_detector_t *d, active_burst_t *b) {
    int clear_start = b->center_bin - d->burst_width / 2;
    int clear_stop = b->center_bin + d->burst_width / 2;
    if (clear_start < 0) clear_start = 0;
    if (clear_stop >= d->fft_size) clear_stop = d->fft_size - 1;
    memset(d->burst_mask + clear_start, 0,
           (clear_stop - clear_start + 1) * sizeof(float));
}

static void update_burst_mask(burst_detector_t *d) {
    memcpy(d->burst_mask, d->ones, sizeof(float) * d->fft_size);
    for (int i = 0; i < d->num_bursts; i++)
        mask_burst(d, &d->bursts[i]);
}

/* ---- Internal: delete gone bursts ---- */

static void delete_gone_bursts(burst_detector_t *d) {
    int update_noise = 0;
    int i = 0;

    while (i < d->num_bursts) {
        active_burst_t *b = &d->bursts[i];
        int long_burst = 0;

        if (d->max_burst_len > 0) {
            if (b->last_active - b->start > (uint64_t)d->max_burst_len) {
                update_noise = 1;
                long_burst = 1;
            }
        }

        if ((b->last_active + d->burst_post_len) <= d->index || long_burst) {
            b->stop = d->index;
            push_burst(&d->gone_bursts, &d->num_gone_bursts,
                       &d->gone_bursts_cap, b);
            remove_burst(d->bursts, &d->num_bursts, i);
            /* don't increment i, next element slid into position */
        } else {
            i++;
        }
    }

    if (update_noise)
        update_filters_post(d, 1);
}

/* ---- Internal: remove peaks near existing bursts ---- */

static void remove_peaks_around_bursts(burst_detector_t *d) {
    for (int i = 0; i < d->fft_size; i++)
        d->relative_magnitude[i] *= d->burst_mask[i];
}

/* ---- Internal: extract peaks above threshold ---- */

static void extract_peaks(burst_detector_t *d) {
    d->num_peaks = 0;
    int half_bw = d->burst_width / 2;

    /* DC notch: skip bins near center frequency to reject LO leakage / ADC
     * offset spikes.  Width of 3 bins (~3.7 kHz at 10 MHz / 8192-pt FFT)
     * covers typical SDR DC spikes without losing any real Iridium signal
     * (channels are 41.667 kHz wide and never centered at DC). */
    int dc_bin = d->fft_size / 2;
    int dc_notch_half = 3;  /* ±3 bins around DC */

    for (int bin = half_bw; bin < d->fft_size - half_bw; bin++) {
        if (bin >= dc_bin - dc_notch_half && bin <= dc_bin + dc_notch_half)
            continue;
        if (d->relative_magnitude[bin] > d->threshold) {
            d->peaks[d->num_peaks].bin = bin;
            d->peaks[d->num_peaks].relative_magnitude = d->relative_magnitude[bin];
            d->num_peaks++;
        }
    }

    /* Sort by magnitude descending */
    qsort(d->peaks, d->num_peaks, sizeof(peak_t), peak_cmp_desc);
}

/* ---- Internal: create new bursts from peaks ---- */

static void create_new_bursts(burst_detector_t *d) {
    d->num_new_bursts = 0;

    for (int i = 0; i < d->num_peaks; i++) {
        peak_t *p = &d->peaks[i];

        if (d->burst_mask[p->bin] == 0.0f)
            continue;

        active_burst_t b;
        memset(&b, 0, sizeof(b));
        b.id = d->burst_id;
        b.center_bin = p->bin;
        d->burst_id += 10;  /* leave room for sub-IDs downstream */

        /* Normalize relative magnitude for SNR estimate */
        b.magnitude = 10.0f * log10f(p->relative_magnitude * d->history_size * 1.72f);

        /* Track peak signal for diagnostic mode */
        if (b.magnitude > d->peak_signal_db)
            d->peak_signal_db = b.magnitude;

        /* Burst might have started one FFT frame earlier */
        b.start = d->index - d->burst_pre_len;
        b.last_active = b.start;

        /* Noise floor in dBFS/Hz */
        b.noise = 10.0f * log10f(d->baseline_sum[b.center_bin] / d->history_size
                                  / ((float)d->fft_size * d->fft_size)
                                  / 1.72f
                                  / ((float)d->sample_rate / d->fft_size));

        push_burst(&d->bursts, &d->num_bursts, &d->bursts_cap, &b);
        push_burst(&d->new_bursts, &d->num_new_bursts, &d->new_bursts_cap, &b);
        mask_burst(d, &b);
    }

    /* Squelch check */
    if (d->max_bursts > 0 && d->num_bursts > d->max_bursts) {
        if (verbose)
            fprintf(stderr, "burst_detect: squelch at %.3f s\n",
                    d->index / (float)d->sample_rate);

        d->num_new_bursts = 0;

        /* Move all non-just-created bursts to gone */
        int i = 0;
        while (i < d->num_bursts) {
            if (d->bursts[i].start != d->index - (uint64_t)d->burst_pre_len) {
                d->bursts[i].stop = d->index;
                push_burst(&d->gone_bursts, &d->num_gone_bursts,
                           &d->gone_bursts_cap, &d->bursts[i]);
                remove_burst(d->bursts, &d->num_bursts, i);
            } else {
                i++;
            }
        }
        /* Clear remaining */
        d->num_bursts = 0;
        update_burst_mask(d);

        d->squelch_count += 3;
        if (d->squelch_count >= 10) {
            if (verbose)
                fprintf(stderr, "burst_detect: resetting noise estimate\n");
            d->history_index = 0;
            d->history_primed = 0;
            memset(d->baseline_history, 0,
                   sizeof(float) * d->fft_size * d->history_size);
            memset(d->baseline_sum, 0, sizeof(float) * d->fft_size);
            d->squelch_count = 0;
        }
    } else {
        if (d->squelch_count > 0)
            d->squelch_count--;
    }
}

/* ---- Internal: run CPU state machine on pre-computed magnitude ---- */

static void process_magnitude_frame(burst_detector_t *d, const float *magnitude) {
    /* Copy GPU-computed magnitudes into detector state */
    memcpy(d->magnitude_shifted, magnitude, sizeof(float) * d->fft_size);

    /* Update filters and detect (same logic as CPU path) */
    if (update_filters_pre(d)) {
        update_bursts(d);
        remove_peaks_around_bursts(d);
        extract_peaks(d);
        delete_gone_bursts(d);
        update_burst_mask(d);
        create_new_bursts(d);
    }
    update_filters_post(d, 0);
}

/* ---- Internal: flush GPU batch and process results ---- */

static void gpu_flush_batch(burst_detector_t *d) {
    if (d->gpu_batch_count == 0)
        return;

    if (d->gpu_process(d->gpu, d->gpu_batch_input,
                        d->gpu_batch_output,
                        d->gpu_batch_count) != 0) {
        fprintf(stderr, "burst_detect: GPU processing failed\n");
        d->gpu_batch_count = 0;
        return;
    }

    /* Process each frame's magnitude through CPU state machine */
    for (int i = 0; i < d->gpu_batch_count; i++) {
        process_magnitude_frame(d, d->gpu_batch_output + i * d->fft_size);
        d->index += d->fft_size;
    }

    d->gpu_batch_count = 0;
}

/* ---- Internal: process one FFT frame ---- */

static void process_fft_frame(burst_detector_t *d, const float complex *samples) {
    /* Apply window and copy to FFT input (SIMD-accelerated) */
    simd_window_cf(samples, d->window, d->fft_in, d->fft_size);

    /* Execute FFT */
    fftwf_execute(d->fft_plan);

    /* DC shift (fftshift) + magnitude-squared (SIMD-accelerated) */
    simd_fftshift_mag(d->fft_out, d->magnitude_shifted, d->fft_size);

    /* Update filters and detect */
    if (update_filters_pre(d)) {
        update_bursts(d);
        remove_peaks_around_bursts(d);
        extract_peaks(d);
        delete_gone_bursts(d);
        update_burst_mask(d);
        create_new_bursts(d);
    }
    update_filters_post(d, 0);
}

/* ---- Internal: emit completed bursts ---- */

/* IridiScope wide-carrier filter: a detection that sat inside a signal at
 * least IR_WIDE_CARRIER_HZ wide for at least half of its frames is part of
 * a wideband (non-classic) carrier the decoders can't read; drop it here
 * instead of handing every slice of it to a decoder. Toggled live. */
static atomic_int g_reject_wide = 1;
static atomic_ullong g_wide_dropped = 0;
void burst_detector_set_reject_wide(int on) { atomic_store(&g_reject_wide, on ? 1 : 0); }
int burst_detector_reject_wide(void) { return atomic_load(&g_reject_wide); }
uint64_t burst_detector_wide_dropped(void) { return (uint64_t)atomic_load(&g_wide_dropped); }

static void emit_gone_bursts(burst_detector_t *d, burst_callback_t cb, void *user) {
    const int reject_wide = atomic_load(&g_reject_wide);
    for (int i = 0; i < d->num_gone_bursts; i++) {
        active_burst_t *ab = &d->gone_bursts[i];
        if (reject_wide && ab->w_frames > 0 && 2 * ab->w_wide >= ab->w_frames) {
            atomic_fetch_add(&g_wide_dropped, 1);
            continue;
        }

        /* Extract IQ samples from ringbuffer */
        uint64_t extract_start = ab->start;
        uint64_t extract_stop = ab->stop + d->burst_pre_len;
        size_t num_samples;
        float complex *samples;
        if (d->shared_ring) {
            /* Point into the ring (no copy). */
            const uint64_t oldest = d->sample_count > d->ringbuf_size ? d->sample_count - d->ringbuf_size : 0;
            if (extract_start < oldest) extract_start = oldest;
            if (extract_stop > d->sample_count) extract_stop = d->sample_count;
            num_samples = extract_stop > extract_start ? (size_t)(extract_stop - extract_start) : 0;
            if (num_samples > d->ring_mirror) num_samples = d->ring_mirror;
            samples = num_samples ? &d->ringbuf[extract_start % d->ringbuf_size] : NULL;
        } else {
            samples = ringbuf_extract(d, extract_start, extract_stop, &num_samples);
        }

        if (num_samples == 0) {
            if (!d->shared_ring) burst_buf_free(samples);
            continue;
        }

        /* Build burst data */
        burst_data_t *bd = malloc(sizeof(*bd));
        bd->info = (burst_info_t){
            .id = ab->id,
            .start = ab->start,
            .stop = ab->stop,
            .last_active = ab->last_active,
            .center_bin = ab->center_bin,
            .magnitude = ab->magnitude,
            .noise = ab->noise,
            .width_hz = ab->w_frames ? (float)(ab->w_sum_bins / ab->w_frames * d->sample_rate / d->fft_size) : 0.0f,
            .wide_frac = ab->w_frames ? (float)ab->w_wide / ab->w_frames : 0.0f,
        };
        bd->center_frequency = d->center_frequency;
        bd->sample_rate = d->sample_rate;
        bd->fft_size = d->fft_size;
        bd->start_time_ns = d->start_time_ns;
        bd->num_samples = num_samples;
        bd->samples = samples;
        bd->ring_owner = d->shared_ring ? d : NULL;
        bd->ring_start = extract_start;

        cb(bd, user);
        d->n_tagged_bursts++;
        atomic_fetch_add(&stat_n_detected, 1);
    }
    d->num_gone_bursts = 0;
}

/* ---- Public: feed samples ---- */

void burst_detector_feed(burst_detector_t *d, const int8_t *iq,
                         size_t num_samples, burst_callback_t cb, void *user) {
    /* Convert int8 IQ pairs to float complex and store in ringbuffer.
     * Process FFT frames as they become available.
     *
     * We maintain a residual buffer for partial FFT frames.
     */

    /* Track timestamp: prefer hardware timestamp from SDR if available */
    if (d->start_time_ns == 0) {
        if (d->pending_hw_ts != 0) {
            d->start_time_ns = d->pending_hw_ts;
        } else {
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            d->start_time_ns = ts.tv_sec * 1000000000ULL + ts.tv_nsec;
        }
        d->start_time_ns += d->pending_skip_ns;
        d->pending_skip_ns = 0;
    }
    d->pending_hw_ts = 0;

    /* Convert int8 to float complex and write to ringbuffer */

    if (num_samples > d->convert_buf_size) {
        free(d->convert_buf);
        d->convert_buf_size = num_samples;
        d->convert_buf = aligned_alloc_32(sizeof(float complex) * d->convert_buf_size);
    }

    /* int8 IQ -> float complex (SIMD-accelerated) */
    simd_convert_i8_cf(iq, d->convert_buf, num_samples);

    /* Write to ringbuffer */
    ringbuf_write(d, d->convert_buf, num_samples);
    d->sample_count += num_samples;

    /* Process complete FFT frames */
    if (d->gpu) {
        /* GPU path: accumulate frames into batch, flush when full.
         * Use a local read cursor (read_idx) to track accumulation
         * position. d->index is only advanced by gpu_flush_batch
         * when the state machine actually processes each frame. */
        uint64_t read_idx = d->index;

        while (read_idx + d->fft_size <= d->sample_count) {
            size_t rb_pos = (size_t)(read_idx % d->ringbuf_size);
            float *dst = d->gpu_batch_input
                         + d->gpu_batch_count * d->fft_size * 2;

            /* Copy complex samples as interleaved float pairs */
            if (rb_pos + d->fft_size <= d->ringbuf_size) {
                memcpy(dst, &d->ringbuf[rb_pos],
                       sizeof(float complex) * d->fft_size);
            } else {
                size_t first = d->ringbuf_size - rb_pos;
                memcpy(dst, &d->ringbuf[rb_pos],
                       first * sizeof(float complex));
                memcpy(dst + first * 2, d->ringbuf,
                       (d->fft_size - first) * sizeof(float complex));
            }

            read_idx += d->fft_size;
            d->gpu_batch_count++;

            if (d->gpu_batch_count >= d->gpu_batch_size) {
                gpu_flush_batch(d);
                /* gpu_flush_batch advances d->index by batch_count * fft_size */

                if (d->num_gone_bursts > 0)
                    emit_gone_bursts(d, cb, user);
            }
        }

        /* Flush remaining partial batch */
        if (d->gpu_batch_count > 0)
            gpu_flush_batch(d);
    } else {
        /* CPU path: process one frame at a time */
        while (d->index + d->fft_size <= d->sample_count) {
            size_t rb_pos = (size_t)(d->index % d->ringbuf_size);

            if (rb_pos + d->fft_size <= d->ringbuf_size) {
                process_fft_frame(d, &d->ringbuf[rb_pos]);
            } else {
                float complex tmp[d->fft_size];
                size_t first = d->ringbuf_size - rb_pos;
                memcpy(tmp, &d->ringbuf[rb_pos], first * sizeof(float complex));
                memcpy(tmp + first, d->ringbuf,
                       (d->fft_size - first) * sizeof(float complex));
                process_fft_frame(d, tmp);
            }

            d->index += d->fft_size;
        }
    }

    /* Emit any completed bursts */
    if (d->num_gone_bursts > 0)
        emit_gone_bursts(d, cb, user);
}

/* ---- Public: feed float32 samples (no int8 quantization) ---- */

void burst_detector_feed_cf32(burst_detector_t *d, const float *iq,
                              size_t num_samples, burst_callback_t cb, void *user) {
    /* Track timestamp: prefer hardware timestamp from SDR if available */
    if (d->start_time_ns == 0) {
        if (d->pending_hw_ts != 0) {
            d->start_time_ns = d->pending_hw_ts;
        } else {
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            d->start_time_ns = ts.tv_sec * 1000000000ULL + ts.tv_nsec;
        }
        d->start_time_ns += d->pending_skip_ns;
        d->pending_skip_ns = 0;
    }
    d->pending_hw_ts = 0;

    /* Convert interleaved float pairs to float complex and write to ringbuffer */
    if (num_samples > d->convert_buf_size) {
        free(d->convert_buf);
        d->convert_buf_size = num_samples;
        d->convert_buf = malloc(sizeof(float complex) * d->convert_buf_size);
    }

    for (size_t i = 0; i < num_samples; i++)
        d->convert_buf[i] = iq[2 * i] + iq[2 * i + 1] * I;

    /* Write to ringbuffer */
    ringbuf_write(d, d->convert_buf, num_samples);
    d->sample_count += num_samples;

    /* Process complete FFT frames (identical to int8 path) */
    if (d->gpu) {
        uint64_t read_idx = d->index;

        while (read_idx + d->fft_size <= d->sample_count) {
            size_t rb_pos = (size_t)(read_idx % d->ringbuf_size);
            float *dst = d->gpu_batch_input
                         + d->gpu_batch_count * d->fft_size * 2;

            if (rb_pos + d->fft_size <= d->ringbuf_size) {
                memcpy(dst, &d->ringbuf[rb_pos],
                       sizeof(float complex) * d->fft_size);
            } else {
                size_t first = d->ringbuf_size - rb_pos;
                memcpy(dst, &d->ringbuf[rb_pos],
                       first * sizeof(float complex));
                memcpy(dst + first * 2, d->ringbuf,
                       (d->fft_size - first) * sizeof(float complex));
            }

            read_idx += d->fft_size;
            d->gpu_batch_count++;

            if (d->gpu_batch_count >= d->gpu_batch_size) {
                gpu_flush_batch(d);
                if (d->num_gone_bursts > 0)
                    emit_gone_bursts(d, cb, user);
            }
        }

        if (d->gpu_batch_count > 0)
            gpu_flush_batch(d);
    } else {
        while (d->index + d->fft_size <= d->sample_count) {
            size_t rb_pos = (size_t)(d->index % d->ringbuf_size);

            if (rb_pos + d->fft_size <= d->ringbuf_size) {
                process_fft_frame(d, &d->ringbuf[rb_pos]);
            } else {
                float complex tmp[d->fft_size];
                size_t first = d->ringbuf_size - rb_pos;
                memcpy(tmp, &d->ringbuf[rb_pos], first * sizeof(float complex));
                memcpy(tmp + first, d->ringbuf,
                       (d->fft_size - first) * sizeof(float complex));
                process_fft_frame(d, tmp);
            }

            d->index += d->fft_size;
        }
    }

    if (d->num_gone_bursts > 0)
        emit_gone_bursts(d, cb, user);
}

/* ---- Thread integration: callback that pushes to burst_queue ---- */

static void burst_to_queue(burst_data_t *burst, void *user) {
    Blocking_Queue *queue = (Blocking_Queue *)user;
    int ret = blocking_queue_put(queue, burst);
    if (ret != 0) {
        burst_release_samples(burst);
        free(burst);
        atomic_fetch_add(&stat_n_dropped, 1);
    }
}

/* ---- Thread function ---- */

void *burst_detector_thread(void *arg) {
    burst_detector_t *det = (burst_detector_t *)arg;

    while (1) {
        sample_buf_t *samples;
        if (blocking_queue_take(&samples_queue, &samples) != 0)
            break;

        /* Pass hardware timestamp to detector (used on first buffer only) */
        if (samples->hw_timestamp_ns != 0)
            det->pending_hw_ts = samples->hw_timestamp_ns;

        if (samples->format == SAMPLE_FMT_FLOAT)
            burst_detector_feed_cf32(det, (const float *)samples->samples,
                                     samples->num, burst_to_queue, &burst_queue);
        else
            burst_detector_feed(det, samples->samples, samples->num,
                               burst_to_queue, &burst_queue);
        free(samples);
    }

    burst_detector_destroy(det);
    return NULL;
}


/* ---- Public: reset noise estimate (IridiScope) ---- */
void burst_detector_reset_noise(burst_detector_t *d) {
    if (!d) return;
    d->num_bursts = 0;
    update_burst_mask(d);
    d->history_index = 0;
    d->history_primed = 0;
    memset(d->baseline_history, 0, sizeof(float) * d->fft_size * d->history_size);
    memset(d->baseline_sum, 0, sizeof(float) * d->fft_size);
    d->squelch_count = 0;
}

/* ---- IridiScope additions: stream time keeping ---- */

/* Samples were lost between two feeds (dropped by the SDR driver or by the
 * input buffer): advance the time base so later bursts keep wall-clock time
 * instead of falling behind by the lost duration. */
void burst_detector_skip_time(burst_detector_t *d, uint64_t ns) {
    if (!d || ns == 0) return;
    if (d->start_time_ns) d->start_time_ns += ns;
    else d->pending_skip_ns += ns;
}

/* Set the wall-clock time of the first sample (e.g. from a recording's file
 * name). Must be called before the first feed. */
void burst_detector_set_start_time(burst_detector_t *d, uint64_t ns) {
    if (!d || ns == 0) return;
    d->start_time_ns = ns + d->pending_skip_ns;
    d->pending_skip_ns = 0;
}
