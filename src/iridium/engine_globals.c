// Definitions for the globals referenced by the vendored iridium-sniffer C
// core. IridiScope drives the pipeline through the direct functions
// (burst_detector_feed / burst_downmix_process / qpsk_demod) rather than the
// thread helpers, but the objects must still exist to satisfy the linker.
#include <stdatomic.h>
#include <signal.h>
#include <pthread.h>

#define C_FEK_BLOCKING_QUEUE_IMPLEMENTATION
#define C_FEK_FAIR_LOCK_IMPLEMENTATION

#include "blocking_queue.h"
#include "fftw_lock.h"
#include "sdr.h"

Blocking_Queue samples_queue;
Blocking_Queue burst_queue;
Blocking_Queue frame_queue;

volatile sig_atomic_t running = 1;
int verbose = 0;

atomic_ulong stat_n_detected = 0;
atomic_ulong stat_n_dropped = 0;
atomic_ulong stat_n_frame_drops = 0;

pthread_mutex_t fftw_planner_mutex;

char *save_bursts_dir = NULL;
int use_gardner = 1;
int use_chase = 0;

// Provided by SDR backends in iridium-sniffer; unused here (file input only).
void push_samples(sample_buf_t *buf) { (void)buf; }
