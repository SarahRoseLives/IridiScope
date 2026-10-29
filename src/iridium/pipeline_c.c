// C wrapper around the vendored iridium-sniffer DSP pipeline. See pipeline_c.h.
//
// The burst detector runs on the caller's thread and enqueues completed bursts;
// a worker pool performs downmix + DQPSK demodulation in parallel. A Blackman-
// windowed power spectrum (with exponential averaging and percentile auto-range)
// is computed alongside, following the AirScope spectrum approach.
#include "pipeline_c.h"
#include "core/priority.h"

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

#include <math.h>
#include <time.h>
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
#define IR_MAX_VOICE_TARGETS 64
#define IR_DET_REC 4096
#define IR_PROBE_FLAG (1ULL << 63)
#define IR_SLOT_NS 90000000ULL

typedef struct {
    struct ir_pipeline *p;
    int                 idx;
} ir_worker_arg_t;

struct ir_pipeline {
    burst_detector_t *det;
    ir_frame_cb       cb;
    void             *user;

    atomic_llong      center_hz;    /* current VFO center for frequency reporting */
    atomic_int        noise_reset;  /* request: re-learn the detector noise floor */

    Blocking_Queue    burst_q;
    pthread_t         workers[IR_MAX_WORKERS];
    burst_downmix_t  *dm[IR_MAX_WORKERS];
    ir_worker_arg_t   wargs[IR_MAX_WORKERS];
    int               n_workers;

    atomic_ulong bursts;
    atomic_ullong queue_full;    /* times the detector had to wait for a free worker */
    atomic_ullong queue_wait_ns; /* time the detector spent waiting for queue space */
    int sample_rate;
    int queue_cap;               /* burst queue capacity (memory-budgeted) */
    int live;                    /* live radio: shared ring, never wait */
    atomic_ullong skipped;       /* bursts skipped: decoders beyond the backlog */

    /* Voice targeting (live): for calls in progress, read the burst at each
     * expected slot time even when the detector didn't trigger on it. */
    pthread_mutex_t vt_mtx;
    struct { double freq, phase; uint64_t last_ns, cursor_ns; int used; } vt[IR_MAX_VOICE_TARGETS];
    struct { uint64_t start, stop; int bin; } det_rec[IR_DET_REC];   /* recent detections */
    unsigned det_rec_n;
    uint64_t probe_seq;
    atomic_ullong probes, probe_frames;
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
    if (demod->id & IR_PROBE_FLAG) atomic_fetch_add(&p->probe_frames, 1);
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
    /* Already overwritten in the ring (decoders too far behind): skip. */
    if (!burst_samples_intact(burst)) {
        atomic_fetch_add(&p->skipped, 1);
        burst_release_samples(burst);
        free(burst);
        return;
    }
    int nf = burst_downmix_process(p->dm[idx], burst, &frames);
    /* The burst's samples are only read during downmix: if the ring
     * overwrote them meanwhile, the result can't be trusted. */
    if (!burst_samples_intact(burst)) {
        for (int i = 0; i < nf; i++) free(frames[i].samples);
        nf = 0;
        atomic_fetch_add(&p->skipped, 1);
    }
    for (int i = 0; i < nf; i++) {
        demod_frame_t *demod = NULL;
        if (qpsk_demod(&frames[i], &demod))
            emit_demod(p, demod);
        else
            atomic_fetch_add(&p->frames_fail, 1);
        free(frames[i].samples);
    }
    free(frames);
    burst_release_samples(burst);
    free(burst);
}

static void *worker_main(void *arg) {
    ir_worker_arg_t *wa = (ir_worker_arg_t *)arg;
    ir_pipeline_t *p = wa->p;
    /* Decoder workers never outrank the radio driver or the detector: if they
     * can't keep up, the oldest bursts are skipped (live) rather than anything
     * upstream being starved. See core/priority.h. */
    int prio_gen = 0;
    iridi_refresh_thread_priority(IRIDI_PRIO_BACKGROUND, &prio_gen);
    for (;;) {
        iridi_refresh_thread_priority(IRIDI_PRIO_BACKGROUND, &prio_gen);
        void *item = NULL;
        if (blocking_queue_take(&p->burst_q, &item) != 0) break;
        if (!item) break;
        handle_burst(p, wa->idx, (burst_data_t *)item);
    }
    return NULL;
}

static void on_burst_enqueue(burst_data_t *burst, void *user) {
    ir_pipeline_t *p = (ir_pipeline_t *)user;
    /* The SDR may have been retuned after the detector was created. */
    burst->center_frequency = (double)atomic_load(&p->center_hz);
    atomic_fetch_add(&p->bursts, 1);
    if (p->live) {
        unsigned k = p->det_rec_n++ % IR_DET_REC;
        p->det_rec[k].start = burst->ring_start;
        p->det_rec[k].stop = burst->ring_start + burst->num_samples;
        p->det_rec[k].bin = burst->info.center_bin;
        /* Never wait on live radio: a full queue means the decoders are far
         * behind, so this burst is skipped (counted) and the radio keeps
         * flowing. */
        if (blocking_queue_add(&p->burst_q, burst) == BQ_FULL) {
            atomic_fetch_add(&p->skipped, 1);
            burst_release_samples(burst);
            free(burst);
        }
        return;
    }
    /* Try without blocking first so a full queue (workers behind) can be
     * counted; then wait as before. */
    if (blocking_queue_add(&p->burst_q, burst) == BQ_FULL) {
        atomic_fetch_add(&p->queue_full, 1);
        struct timespec a, b;
        clock_gettime(CLOCK_MONOTONIC, &a);
        blocking_queue_put(&p->burst_q, burst);
        clock_gettime(CLOCK_MONOTONIC, &b);
        atomic_fetch_add(&p->queue_wait_ns, (unsigned long long)((b.tv_sec - a.tv_sec) * 1000000000LL +
                                                                 (b.tv_nsec - a.tv_nsec)));
    }
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



/* k-th smallest of a[0..n-1] (0-based), partially reordering a: quickselect
 * with median-of-three. Returns exactly what a full sort would put at a[k]. */
static float select_kth(float *a, int n, int k) {
    int lo = 0, hi = n - 1;
    while (hi > lo) {
        const int mid = lo + (hi - lo) / 2;
        if (a[mid] < a[lo]) { float t = a[mid]; a[mid] = a[lo]; a[lo] = t; }
        if (a[hi] < a[lo]) { float t = a[hi]; a[hi] = a[lo]; a[lo] = t; }
        if (a[hi] < a[mid]) { float t = a[hi]; a[hi] = a[mid]; a[mid] = t; }
        const float pivot = a[mid];
        int i = lo, j = hi;
        while (i <= j) {
            while (a[i] < pivot) i++;
            while (a[j] > pivot) j--;
            if (i <= j) { float t = a[i]; a[i] = a[j]; a[j] = t; i++; j--; }
        }
        if (k <= j) hi = j;
        else if (k >= i) lo = i;
        else return a[k];
    }
    return a[k];
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

    /* Only two order statistics are needed (30th and 99.5th percentile), so
     * select them instead of sorting all N bins: same values, O(N). */
    memcpy(p->spec_sort, p->spec_inst, sizeof(float) * N);
    const int k_floor = (int)(0.30 * N), k_peak = (int)(0.995 * N);
    p->peak_db  = select_kth(p->spec_sort, N, k_peak);
    /* After selecting k_peak, everything below index k_peak is <= it, so the
     * floor can be selected within that prefix. */
    p->floor_db = select_kth(p->spec_sort, k_peak, k_floor);
}

/* ---- public API ----------------------------------------------------------- */

static double g_queue_budget_bytes = 4.0e9; /* burst queue memory budget (playback) */
void ir_pipeline_set_queue_budget(double bytes) {
    if (bytes >= 1.0e8) g_queue_budget_bytes = bytes;
}
/* Live radio: seconds of backlog the decoders may fall behind before the
 * oldest bursts are skipped (0 = playback: the detector waits instead). */
static double g_live_backlog_s = 0.0;
static int g_voice_targeting = 1;
void ir_pipeline_set_voice_targeting(int on) { g_voice_targeting = on ? 1 : 0; }
void ir_pipeline_set_live_backlog(double seconds) { g_live_backlog_s = seconds > 0 ? seconds : 0.0; }

/* ---- voice targeting (live) ---------------------------------------------- */

static double slot_phase(uint64_t ns) { return (double)(ns % IR_SLOT_NS); }
static double slot_delta(double a, double b) {   /* signed b - a around the frame */
    double d = fmod(b - a, (double)IR_SLOT_NS);
    if (d > IR_SLOT_NS / 2.0) d -= IR_SLOT_NS;
    if (d < -(double)IR_SLOT_NS / 2.0) d += IR_SLOT_NS;
    return d;
}

/* A voice frame was decoded (any thread): track its call's slot. */
void ir_pipeline_voice_seen(ir_pipeline_t *p, uint64_t ts_ns, double freq_hz) {
    if (!p || !p->live) return;
    const double ph = slot_phase(ts_ns);
    pthread_mutex_lock(&p->vt_mtx);
    int free_i = -1;
    for (int i = 0; i < IR_MAX_VOICE_TARGETS; i++) {
        if (!p->vt[i].used) { if (free_i < 0) free_i = i; continue; }
        if (fabs(p->vt[i].freq - freq_hz) < 20000.0 && fabs(slot_delta(p->vt[i].phase, ph)) <= 2.5e6) {
            if (ts_ns > p->vt[i].last_ns) p->vt[i].last_ns = ts_ns;
            p->vt[i].freq = freq_hz;
            p->vt[i].phase = fmod(p->vt[i].phase + 0.2 * slot_delta(p->vt[i].phase, ph) + IR_SLOT_NS,
                                  (double)IR_SLOT_NS);
            pthread_mutex_unlock(&p->vt_mtx);
            return;
        }
    }
    if (free_i >= 0) {
        p->vt[free_i].used = 1;
        p->vt[free_i].freq = freq_hz;
        p->vt[free_i].phase = ph;
        p->vt[free_i].last_ns = ts_ns;
        /* Also look back ~2 s for the start of the call. */
        p->vt[free_i].cursor_ns = ts_ns > 22 * IR_SLOT_NS ? ts_ns - 23 * IR_SLOT_NS : ts_ns;
    }
    pthread_mutex_unlock(&p->vt_mtx);
}

/* Detector thread: queue a read of the expected burst at slot time t. */
static void probe_slot(ir_pipeline_t *p, uint64_t t, double freq) {
    const uint64_t t0 = burst_detector_start_time(p->det);
    const double rate = (double)p->sample_rate;
    /* A voice burst starts ~0.25 ms before its frame time and lasts ~8.3 ms;
     * the slot before it can be occupied right up to its start, so the read
     * begins just 0.6 ms early. */
    const uint64_t pre = (uint64_t)(0.0006 * rate), len = (uint64_t)(0.0095 * rate);
    if (t < t0 + 2000000ULL) return;
    const uint64_t idx = (uint64_t)((double)(t - t0) * rate / 1e9);
    if (idx < pre) return;
    const uint64_t start = idx - pre;
    const int fft = burst_detector_fft_size(p->det);
    const double center = (double)atomic_load(&p->center_hz);
    const int bin = (int)lround((freq - center) / rate * fft) + fft / 2;
    if (bin < 0 || bin >= fft) return;
    /* Skip if the detector already caught this burst: a detection that
     * STARTS at this slot (its start includes 2 FFT frames of lead-in). A
     * detection that merely overlaps is usually the previous slot's burst
     * run together with this one, and only that first burst gets decoded. */
    const int tol = (int)(15000.0 / (rate / fft)) + 1;
    const double lead = 2.0 * fft, slack = 0.0025 * rate;
    const double want = (double)idx - lead;
    const unsigned n = p->det_rec_n < IR_DET_REC ? p->det_rec_n : IR_DET_REC;
    for (unsigned i = 0; i < n; i++) {
        const unsigned k = (p->det_rec_n - 1 - i) % IR_DET_REC;
        const double ds = (double)p->det_rec[k].start;
        if (ds + 0.3 * rate < want) break;   /* older than anything relevant */
        if (fabs(ds - want) <= slack && abs(p->det_rec[k].bin - bin) <= tol) return;
    }
    burst_data_t *bd = (burst_data_t *)malloc(sizeof(*bd));
    if (!bd) return;
    if (!burst_detector_ring_burst(p->det, start, (size_t)len, bin, bd)) { free(bd); return; }
    bd->center_frequency = center;
    bd->info.id = IR_PROBE_FLAG | ++p->probe_seq;
    if (blocking_queue_add(&p->burst_q, bd) == BQ_FULL) { free(bd); return; }
    atomic_fetch_add(&p->probes, 1);
}

static void voice_targeting(ir_pipeline_t *p) {
    const uint64_t t0 = burst_detector_start_time(p->det);
    if (!t0 || p->sample_rate <= 0) return;
    const uint64_t now = t0 + (uint64_t)((double)burst_detector_sample_count(p->det) * 1e9 / p->sample_rate);
    struct { uint64_t t; double f; } todo[256];
    int nt = 0;
    pthread_mutex_lock(&p->vt_mtx);
    for (int i = 0; i < IR_MAX_VOICE_TARGETS; i++) {
        if (!p->vt[i].used) continue;
        /* Call silent for a while (ended, or the decoders are far behind). */
        if (now > p->vt[i].last_ns + 8000000000ULL) { p->vt[i].used = 0; continue; }
        uint64_t next = p->vt[i].cursor_ns + IR_SLOT_NS;
        /* Stay on the call's slot phase (it drifts slowly with Doppler). */
        next = (uint64_t)((double)next + slot_delta(slot_phase(next), p->vt[i].phase));
        int k = 0;
        /* Wait until the detector would have finished (and emitted) a burst
         * there: burst 8.3 ms + its 16 ms tail, with margin. */
        while (next + 60000000ULL <= now && k < 24 && nt < 256) {
            todo[nt].t = next; todo[nt].f = p->vt[i].freq; nt++; k++;
            p->vt[i].cursor_ns = next;
            next += IR_SLOT_NS;
        }
    }
    pthread_mutex_unlock(&p->vt_mtx);
    for (int i = 0; i < nt; i++) probe_slot(p, todo[i].t, todo[i].f);
}

void ir_pipeline_probe_stats(ir_pipeline_t *p, uint64_t *probes, uint64_t *frames) {
    if (probes) *probes = p ? (uint64_t)atomic_load(&p->probes) : 0;
    if (frames) *frames = p ? (uint64_t)atomic_load(&p->probe_frames) : 0;
}

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
    atomic_store(&p->center_hz, (long long)llround(center_freq));
    atomic_store(&p->spec_req, IR_DEFAULT_FFT);

    burst_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.center_frequency = center_freq;
    cfg.sample_rate = sample_rate;
    p->sample_rate = sample_rate;
    cfg.threshold = threshold > 0 ? threshold : 16.0f;
    cfg.use_gpu = 0;
    /* Live: bursts point into a shared ring holding the backlog (no copy,
     * fixed memory, the detector never waits). Playback: copy per burst and
     * let the detector wait for the decoders, so a file loses nothing. */
    p->live = g_live_backlog_s > 0;
    pthread_mutex_init(&p->vt_mtx, NULL);
    burst_detector_set_shared_ring(p->live ? g_live_backlog_s : 0.0);
    p->det = burst_detector_create(&cfg);
    burst_detector_set_shared_ring(0.0);
    if (!p->det) { free(p); return NULL; }

    if (num_workers <= 0) {
        long hw = 4;
#if defined(_WIN32)
        /* MinGW has no sysconf(_SC_NPROCESSORS_ONLN); without this the pool
         * was always 3 workers on Windows. */
        {
            DWORD n = GetActiveProcessorCount(ALL_PROCESSOR_GROUPS);
            if (n > 1) hw = (long)n;
        }
#elif defined(_SC_NPROCESSORS_ONLN)
        long n = sysconf(_SC_NPROCESSORS_ONLN);
        if (n > 1) hw = n;
#endif
        num_workers = (int)hw - 1;
        if (num_workers < 1) num_workers = 1;
        if (num_workers > IR_MAX_WORKERS) num_workers = IR_MAX_WORKERS;
    }
    p->n_workers = num_workers;
    /* Each queued burst carries full-rate IQ for its lifetime plus pre/post
     * padding (~26 ms typical: ~2 MB at 10 Msps). A fixed 4096-entry queue
     * could grow to ~8+ GB when workers fall behind. Size it to a memory
     * budget instead (default 4 GB, user-adjustable); when full the detector
     * waits (and the sample buffer absorbs/drops), which is visible in Stats. */
    {
        const double burst_bytes = (double)sample_rate * 0.026 * 8.0;
        double cap = burst_bytes > 0 ? g_queue_budget_bytes / burst_bytes : 4096.0;
        if (cap < 64) cap = 64;
        if (cap > 65536) cap = 65536;
        if (p->live) cap = g_live_backlog_s * 4000.0; /* entries are small */
        p->queue_cap = (int)cap;
        /* Keep freed burst buffers for reuse up to the same budget, and
         * give every typical burst (~28 ms of IQ) the same buffer size. */
        if (!p->live)
            burst_buf_configure((size_t)g_queue_budget_bytes, (size_t)((double)sample_rate * 0.028 * 8.0));
    }
    blocking_queue_init(&p->burst_q, (unsigned)p->queue_cap);
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
    if (atomic_exchange(&p->noise_reset, 0)) burst_detector_reset_noise(p->det);
    compute_spectrum(p, iq, num_samples);
    burst_buf_maintain();
    uint64_t before = atomic_load(&p->frames_ok) + atomic_load(&p->frames_fail);
    burst_detector_feed_cf32(p->det, iq, num_samples, on_burst_enqueue, p);
    if (p->live && g_voice_targeting) voice_targeting(p);
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

void ir_pipeline_set_center(ir_pipeline_t *p, double hz) {
    if (p) atomic_store(&p->center_hz, (long long)llround(hz));
}

void ir_pipeline_reset_noise(ir_pipeline_t *p) {
    if (p) atomic_store(&p->noise_reset, 1);
}

/* Call from the feeding thread only (between feeds). */
void ir_pipeline_skip_samples(ir_pipeline_t *p, uint64_t n) {
    if (!p || !n || p->sample_rate <= 0) return;
    burst_detector_skip_time(p->det, (uint64_t)((double)n * 1e9 / p->sample_rate));
}
void ir_pipeline_set_start_time(ir_pipeline_t *p, uint64_t unix_ns) {
    if (p) burst_detector_set_start_time(p->det, unix_ns);
}
void ir_pipeline_buffer_stats(uint64_t *reused, uint64_t *fresh, size_t *live, size_t *idle) {
    burst_buf_stats(reused, fresh);
    burst_buf_memory(live, idle);
}

uint64_t ir_pipeline_skipped(ir_pipeline_t *p) { return p ? (uint64_t)atomic_load(&p->skipped) : 0; }
float ir_pipeline_noise_floor(ir_pipeline_t *p) { return p && p->det ? burst_detector_noise_floor(p->det) : 0.0f; }
void ir_set_reject_wide(int on) { burst_detector_set_reject_wide(on); }
int ir_reject_wide(void) { return burst_detector_reject_wide(); }
uint64_t ir_wide_dropped(void) { return burst_detector_wide_dropped(); }

uint64_t ir_pipeline_queue_wait_ns(ir_pipeline_t *p) {
    return p ? (uint64_t)atomic_load(&p->queue_wait_ns) : 0;
}

uint64_t ir_pipeline_bursts(ir_pipeline_t *p)     { return p ? atomic_load(&p->bursts) : 0; }
uint64_t ir_pipeline_frames_ok(ir_pipeline_t *p)  { return p ? atomic_load(&p->frames_ok) : 0; }
uint64_t ir_pipeline_frames_fail(ir_pipeline_t *p){ return p ? atomic_load(&p->frames_fail) : 0; }
uint64_t ir_pipeline_queue_full(ir_pipeline_t *p) { return p ? (uint64_t)atomic_load(&p->queue_full) : 0; }
int ir_pipeline_workers(ir_pipeline_t *p)          { return p ? p->n_workers : 0; }
int ir_pipeline_queue_capacity(ir_pipeline_t *p)   { return p ? p->queue_cap : 0; }

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
    pthread_mutex_destroy(&p->vt_mtx);
    free(p);
}
