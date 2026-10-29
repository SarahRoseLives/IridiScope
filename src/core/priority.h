// Process/thread priorities (Windows; no-ops elsewhere).
//
// "Protect radio" (boost) raises the IridiScope process to ABOVE_NORMAL so
// the threads that move samples from the radio (libairspy's USB and
// conversion threads, the burst detector, all at HIGHEST) outrank every
// normal-priority program. Everything else in IridiScope is lowered by the
// same step so it still sits level with other programs, not above them.
#pragma once
#ifdef __cplusplus
extern "C" {
#endif

enum {
    IRIDI_PRIO_BACKGROUND = 0, // decoders, WAV writer, sorting: below normal when not boosted
    IRIDI_PRIO_NORMAL = 1      // UI, database, map tiles: normal when not boosted
};

void iridi_set_boost(int on);   // applies at once (process + calling thread is not changed)
int iridi_boost(void);
// Set the calling thread's priority for its role and the current mode.
void iridi_apply_thread_priority(int role);
// Same, only if the mode changed since *seen_gen (cheap; call in a loop).
void iridi_refresh_thread_priority(int role, int *seen_gen);

#ifdef __cplusplus
}
#endif
