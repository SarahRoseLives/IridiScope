#include "core/priority.h"

#include <stdatomic.h>
#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

static atomic_int g_boost = 0;
static atomic_int g_gen = 1;

void iridi_set_boost(int on) {
    on = on ? 1 : 0;
#if defined(_WIN32)
    SetPriorityClass(GetCurrentProcess(), on ? ABOVE_NORMAL_PRIORITY_CLASS : NORMAL_PRIORITY_CLASS);
#endif
    if (atomic_exchange(&g_boost, on) != on) atomic_fetch_add(&g_gen, 1);
}

int iridi_boost(void) { return atomic_load(&g_boost); }

void iridi_apply_thread_priority(int role) {
#if defined(_WIN32)
    int prio;
    if (atomic_load(&g_boost))
        prio = THREAD_PRIORITY_LOWEST;   /* ABOVE_NORMAL class: base 10 - 2 = 8, same as a normal program */
    else
        prio = role == IRIDI_PRIO_BACKGROUND ? THREAD_PRIORITY_BELOW_NORMAL : THREAD_PRIORITY_NORMAL;
    SetThreadPriority(GetCurrentThread(), prio);
#else
    (void)role;
#endif
}

void iridi_refresh_thread_priority(int role, int *seen_gen) {
    const int g = atomic_load(&g_gen);
    if (*seen_gen == g) return;
    *seen_gen = g;
    iridi_apply_thread_priority(role);
}
