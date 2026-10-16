// C wrapper around the vendored iridium-sniffer DSP pipeline. See pipeline_c.h.
//
// The burst detector runs on the caller's thread and enqueues completed bursts;
// a worker pool performs downmix + DQPSK demodulation in parallel. A Blackman-
// windowed power spectrum (with exponential averaging and percentile auto-range)
// is computed alongside, following the AirScope spectrum approach.
#include "pipeline_c.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdatomic.h>
#include <pthread.h>
#include <unistd.h>

#include <fftw3.h>

#include "burst_detect.h"
#include "burst_downmix.h"
#include "qpsk_demod.h"
#include "fftw_lock.h"
#include "simd_kernels.h"
#include "blocking_queue.h"

#define IR_DEFAULT_FFT 4096
#define IR_MAX_WORKERS 8

// Defined in engine_globals.c / qpsk_demod.c.
extern int use_gardner;
extern int use_chase;

struct ir_pipeline;

typedef struct {
    struct ir_pipeline *p;
    int                 idx;
} ir_worker_arg_t;

struct ir_pipeline {
    burst_detector_t *det;
    ir_frame_cb       cb;
    void             *user;

    Blocking_Queue    burst_q;
    pthread_t         workers[IR_MAX_WORKERS];
    burst_downmix_t  *dm[IR_MAX_WORKERS];
    ir_worker_arg_t   wargs[IR_MAX_WORKERS];
    int               n_workers;

    atomic_ulong bursts;
    atomic_ulong frames_ok;
    atomic_ulong frames_fail;

    /* spectrum */
    atomic_int       spec_req;      /* requested FFT size */
    int              spec_n;        /* active FFT size */
    float            avg_alpha;
    int              dc_block;
    int              spec_init;
    fftwf_plan       spec_plan;
    fftwf_complex   *spec_in;
    fftwf_complex   *spec_out;
    float           *spec_win;
    float           *spec_inst;
    float           *spec_avg;
    float           *spec_sort;
    float            floor_db;
    float            peak_db;
};

static void ir_global_init(void) {
    static int done = 0;
    if (done) return;
    done = 1;
    simd_init(0);
    fftw_lock_init();
    use_gardner = 1;
    use_chase = 0;
}

/* ---- burst worker pool ---------------------------------------------------- */

static void emit_demod(ir_pipeline_t *p, demod_frame_t *demod) {
    atomic_fetch_add(&p->frames_ok, 1);
    if (p->cb) {
        ir_frame_info_t info;
        memset(&info, 0, sizeof(info));
        info.id = demod->id;
        info.timestamp_ns = demod->timestamp;
        info.frequency = demod->center_frequency;
        info.direction = (int)demod->direction;
        info.magnitude = demod->magnitude;
        info.noise = demod->noise;
        info.level = demod->level;
        info.confidence = demod->confidence;
        info.n_symbols = demod->n_symbols;
        info.n_payload_symbols = demod->n_payload_symbols;
        info.n_bits = demod->n_bits;
        info.bits = demod->bits;
        p->cb(&info, p->user);
    }
    free(demod->bits);
    free(demod->llr);
    free(demod);
}

static void handle_burst(ir_pipeline_t *p, int idx, burst_data_t *burst) {
    downmix_frame_t *frames = NULL;
    int nf = burst_downmix_process(p->dm[idx], burst, &frames);
    for (int i = 0; i < nf; i++) {
        demod_frame_t *demod = NULL;
        if (qpsk_demod(&frames[i], &demod))
            emit_demod(p, demod);
        else
            atomic_fetch_add(&p->frames_fail, 1);
        free(frames[i].samples);
    }
    free(frames);
    free(burst->samples);
    free(burst);
}

static void *worker_main(void *arg) {
    ir_worker_arg_t *wa = (ir_worker_arg_t *)arg;
    ir_pipeline_t *p = wa->p;
    for (;;) {
        void *item = NULL;
        if (blocking_queue_take(&p->burst_q, &item) != 0) break;
        if (!item) break;
        handle_burst(p, wa->idx, (burst_data_t *)item);
    }
    return NULL;
}

static void on_burst_enqueue(burst_data_t *burst, void *user) {
    ir_pipeline_t *p = (ir_pipeline_t *)user;
    atomic_fetch_add(&p->bursts, 1);
    blocking_queue_put(&p->burst_q, burst);
}

/* ---- spectrum ------------------------------------------------------------- */

static void build_spectrum(ir_pipeline_t *p, int n) {
    if (p->spec_plan) { fftwf_destroy_plan(p->spec_plan); p->spec_plan = NULL; }
    if (p->spec_in)  { fftwf_free(p->spec_in);  p->spec_in = NULL; }
    if (p->spec_out) { fftwf_free(p->spec_out); p->spec_out = NULL; }
    free(p->spec_win);  p->spec_win = NULL;
    free(p->spec_inst); p->spec_inst = NULL;
    free(p->spec_avg);  p->spec_avg = NULL;
    free(p->spec_sort); p->spec_sort = NULL;

    p->spec_n = n;
    p->spec_init = 0;
    p->spec_in  = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex) * n);
    p->spec_out = (fftwf_complex *)fftwf_malloc(sizeof(fftwf_complex) * n);
    p->spec_win  = (float *)malloc(sizeof(float) * n);
    p->spec_inst = (float *)malloc(sizeof(float) * n);
    p->spec_avg  = (float *)malloc(sizeof(float) * n);
    p->spec_sort = (float *)malloc(sizeof(float) * n);
    if (!p->spec_in || !p->spec_out || !p->spec_win || !p->spec_inst ||
        !p->spec_avg || !p->spec_sort)
        return;

    // Blackman window: low sidelobes -> clean noise floor.
    for (int i = 0; i < n; i++) {
        double x = 2.0 * M_PI * i / (double)(n - 1);
        p->spec_win[i] = (float)(0.42 - 0.5 * cos(x) + 0.08 * cos(2.0 * x));
        p->spec_inst[i] = -120.0f;
        p->spec_avg[i]  = -120.0f;
    }
    fftw_lock();
    p->spec_plan = fftwf_plan_dft_1d(n, p->spec_in, p->spec_out,
                                     FFTW_FORWARD, FFTW_ESTIMATE);
    fftw_unlock();
}

static void patch_dc(float *a, int n, int w) {
    int c = n / 2, span = 2 * w + 1, src = c - w - span;
    if (src < 0 || c + w >= n) return;
    for (int k = 0; k < span; ++k) a[c - w + k] = a[src + k];
}

static int cmp_float(const void *a, const void *b) {
    float x = *(const float *)a, y = *(const float *)b;
    return (x > y) - (x < y);
}

static void compute_spectrum(ir_pipeline_t *p, const float *iq, size_t n) {
    int req = atomic_load(&p->spec_req);
    if (req != p->spec_n) { build_spectrum(p, req); if (!p->spec_plan) return; }
    int N = p->spec_n;
    if (n < (size_t)N || !p->spec_plan) return;

    const size_t start = (n - (size_t)N) * 2;

    double meanI = 0.0, meanQ = 0.0;
    if (p->dc_block) {
        for (int i = 0; i < N; i++) {
            meanI += iq[start + 2 * i];
            meanQ += iq[start + 2 * i + 1];
        }
        meanI /= N; meanQ /= N;
    }
    for (int i = 0; i < N; i++) {
        p->spec_in[i][0] = (float)(iq[start + 2 * i]     - meanI) * p->spec_win[i];
        p->spec_in[i][1] = (float)(iq[start + 2 * i + 1] - meanQ) * p->spec_win[i];
    }
    fftwf_execute(p->spec_plan);

    const double invN = 1.0 / (double)N;
    const float alpha = p->avg_alpha;
    for (int i = 0; i < N; i++) {
        int src = (i + N / 2) % N; // fftshift: center DC
        float re = p->spec_out[src][0], im = p->spec_out[src][1];
        double pw = (double)(re * re + im * im) * invN;
        float db = (float)(10.0 * log10(pw + 1e-20));
        p->spec_inst[i] = db;
        p->spec_avg[i] = p->spec_init ? (alpha * p->spec_avg[i] + (1.0f - alpha) * db) : db;
    }
    p->spec_init = 1;

    patch_dc(p->spec_inst, N, 4);
    patch_dc(p->spec_avg, N, 4);

    memcpy(p->spec_sort, p->spec_inst, sizeof(float) * N);
    qsort(p->spec_sort, N, sizeof(float), cmp_float);
    p->floor_db = p->spec_sort[(int)(0.30 * N)];
    p->peak_db  = p->spec_sort[(int)(0.995 * N)];
}

/* ---- public API ----------------------------------------------------------- */

ir_pipeline_t *ir_pipeline_create(double center_freq, int sample_rate,
                                  float threshold, int num_workers,
                                  ir_frame_cb cb, void *user) {
    ir_global_init();

    ir_pipeline_t *p = (ir_pipeline_t *)calloc(1, sizeof(*p));
    if (!p) return NULL;
    p->cb = cb;
    p->user = user;
    p->avg_alpha = 0.6f;
    p->dc_block = 1;
    atomic_store(&p->spec_req, IR_DEFAULT_FFT);

    burst_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.center_frequency = center_freq;
    cfg.sample_rate = sample_rate;
    cfg.threshold = threshold > 0 ? threshold : 16.0f;
    cfg.use_gpu = 0;
    p->det = burst_detector_create(&cfg);
    if (!p->det) { free(p); return NULL; }

    if (num_workers <= 0) {
        long hw = 4;
#ifdef _SC_NPROCESSORS_ONLN
        long n = sysconf(_SC_NPROCESSORS_ONLN);
        if (n > 1) hw = n;
#endif
        num_workers = (int)hw - 1;
        if (num_workers < 1) num_workers = 1;
        if (num_workers > IR_MAX_WORKERS) num_workers = IR_MAX_WORKERS;
    }
    p->n_workers = num_workers;
    blocking_queue_init(&p->burst_q, 4096);
    for (int i = 0; i < p->n_workers; i++) {
        p->dm[i] = burst_downmix_create(NULL);
        if (!p->dm[i]) { ir_pipeline_destroy(p); return NULL; }
        p->wargs[i].p = p;
        p->wargs[i].idx = i;
        pthread_create(&p->workers[i], NULL, worker_main, &p->wargs[i]);
    }

    build_spectrum(p, IR_DEFAULT_FFT);
    if (!p->spec_plan) { ir_pipeline_destroy(p); return NULL; }
    return p;
}

uint64_t ir_pipeline_feed(ir_pipeline_t *p, const float *iq, size_t num_samples) {
    if (!p || !iq || num_samples == 0) return 0;
    compute_spectrum(p, iq, num_samples);
    uint64_t before = atomic_load(&p->frames_ok) + atomic_load(&p->frames_fail);
    burst_detector_feed_cf32(p->det, iq, num_samples, on_burst_enqueue, p);
    uint64_t after = atomic_load(&p->frames_ok) + atomic_load(&p->frames_fail);
    return after - before;
}

const float *ir_pipeline_spectrum_inst(ir_pipeline_t *p) { return p ? p->spec_inst : NULL; }
const float *ir_pipeline_spectrum_avg(ir_pipeline_t *p)  { return p ? p->spec_avg : NULL; }
int ir_pipeline_spectrum_size(ir_pipeline_t *p)          { return p ? p->spec_n : 0; }
float ir_pipeline_spectrum_floor(ir_pipeline_t *p)       { return p ? p->floor_db : -120.0f; }
float ir_pipeline_spectrum_peak(ir_pipeline_t *p)        { return p ? p->peak_db : -120.0f; }

void ir_pipeline_set_fft_size(ir_pipeline_t *p, int n) {
    if (!p || n < 256) return;
    atomic_store(&p->spec_req, n);
}
int ir_pipeline_get_fft_size(ir_pipeline_t *p) { return p ? p->spec_n : 0; }
void ir_pipeline_set_avg_alpha(ir_pipeline_t *p, float alpha) {
    if (p) p->avg_alpha = alpha < 0 ? 0 : (alpha > 0.999f ? 0.999f : alpha);
}
void ir_pipeline_set_dc_block(ir_pipeline_t *p, int on) { if (p) p->dc_block = on; }

uint64_t ir_pipeline_bursts(ir_pipeline_t *p)     { return p ? atomic_load(&p->bursts) : 0; }
uint64_t ir_pipeline_frames_ok(ir_pipeline_t *p)  { return p ? atomic_load(&p->frames_ok) : 0; }
uint64_t ir_pipeline_frames_fail(ir_pipeline_t *p){ return p ? atomic_load(&p->frames_fail) : 0; }

void ir_pipeline_destroy(ir_pipeline_t *p) {
    if (!p) return;
    blocking_queue_close(&p->burst_q);
    for (int i = 0; i < p->n_workers; i++)
        pthread_join(p->workers[i], NULL);
    blocking_queue_destroy(&p->burst_q);
    for (int i = 0; i < p->n_workers; i++)
        if (p->dm[i]) burst_downmix_destroy(p->dm[i]);

    if (p->spec_plan) fftwf_destroy_plan(p->spec_plan);
    if (p->spec_in)  fftwf_free(p->spec_in);
    if (p->spec_out) fftwf_free(p->spec_out);
    free(p->spec_win);
    free(p->spec_inst);
    free(p->spec_avg);
    free(p->spec_sort);
    if (p->det) burst_detector_destroy(p->det);
    free(p);
}
