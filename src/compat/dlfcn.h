// Minimal <dlfcn.h> stub for Windows. GPU acceleration is disabled in
// IridiScope, so dynamic loading always fails gracefully (the burst detector
// falls back to the CPU FFTW path).
#ifndef IRIDISCOPE_COMPAT_DLFCN_H
#define IRIDISCOPE_COMPAT_DLFCN_H

#ifdef _WIN32

#ifndef RTLD_NOW
#define RTLD_NOW 2
#endif

static inline void *dlopen(const char *filename, int flags) {
    (void)filename; (void)flags;
    return 0;
}
static inline void *dlsym(void *handle, const char *symbol) {
    (void)handle; (void)symbol;
    return 0;
}
static inline int dlclose(void *handle) {
    (void)handle;
    return 0;
}
static inline char *dlerror(void) { return (char *)"dynamic loading disabled"; }

#endif // _WIN32
#endif // IRIDISCOPE_COMPAT_DLFCN_H
