// Windows/MinGW compatibility shim for the vendored iridium-sniffer C core.
//
// This header is force-included (-include) into every C core translation unit
// so the Linux-oriented code compiles unchanged on mingw-w64. It provides the
// BSD <err.h> functions, a 2-argument mkdir, and posix_memalign.
#ifndef IRIDISCOPE_WIN_COMPAT_H
#define IRIDISCOPE_WIN_COMPAT_H

#ifdef _WIN32

#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <direct.h>

// ---- <err.h> (BSD) ---------------------------------------------------------
static inline void iridi_vwarnx(const char *fmt, va_list ap) {
    fputs("iridium: ", stderr);
    if (fmt) vfprintf(stderr, fmt, ap);
    fputc('\n', stderr);
}

static inline void iridi_vwarn(const char *fmt, va_list ap) {
    int e = errno;
    fputs("iridium: ", stderr);
    if (fmt) vfprintf(stderr, fmt, ap);
    fputs(": ", stderr);
    fputs(strerror(e), stderr);
    fputc('\n', stderr);
}

static inline void warnx(const char *fmt, ...) {
    va_list ap; va_start(ap, fmt); iridi_vwarnx(fmt, ap); va_end(ap);
}
static inline void warn(const char *fmt, ...) {
    va_list ap; va_start(ap, fmt); iridi_vwarn(fmt, ap); va_end(ap);
}
static inline void errx(int code, const char *fmt, ...) {
    va_list ap; va_start(ap, fmt); iridi_vwarnx(fmt, ap); va_end(ap);
    exit(code);
}
static inline void err(int code, const char *fmt, ...) {
    va_list ap; va_start(ap, fmt); iridi_vwarn(fmt, ap); va_end(ap);
    exit(code);
}

// ---- 2-argument mkdir ------------------------------------------------------
#define mkdir(path, mode) _mkdir(path)

// ---- posix_memalign --------------------------------------------------------
// The SIMD kernels free these buffers with free(), and all SIMD accesses are
// unaligned, so a plain malloc-backed implementation is sufficient here. The
// macro avoids clashing with GCC's posix_memalign builtin.
static inline int iridi_posix_memalign(void **memptr, size_t alignment, size_t size) {
    (void)alignment;
    void *p = malloc(size ? size : 1);
    if (!p) return ENOMEM;
    *memptr = p;
    return 0;
}
#define posix_memalign(memptr, alignment, size) \
    iridi_posix_memalign((memptr), (alignment), (size))

#endif // _WIN32
#endif // IRIDISCOPE_WIN_COMPAT_H
