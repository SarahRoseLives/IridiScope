/*
 * GPU-accelerated burst detection FFT
 *
 * Backend-agnostic interface. Batches multiple FFT frames and processes
 * them on GPU: window multiply -> forward FFT -> fftshift + magnitude squared
 *
 * Copyright (c) 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/*
 * GPU-accelerated burst detection FFT
 *
 * Backend-agnostic interface. Implemented by:
 *   - opencl/burst_fft.c  (OpenCL + VkFFT, VKFFT_BACKEND=3)
 *   - vulkan/burst_fft.c  (Vulkan + VkFFT, VKFFT_BACKEND=0)
 *
 * Batches multiple FFT frames and processes them on GPU:
 *   window multiply -> forward FFT -> fftshift + magnitude squared
 *
 * CPU handles the burst state machine on the returned magnitudes.
 */

#ifndef __BURST_FFT_H__
#define __BURST_FFT_H__

#define GPU_API __attribute__((visibility("default")))

typedef struct gpu_burst_fft gpu_burst_fft_t;

/* Create GPU FFT context.
 * fft_size: FFT length (must be power of 2)
 * batch_size: max frames per GPU dispatch (e.g. 16)
 * window: Blackman window coefficients (fft_size floats, copied to GPU) */
GPU_API gpu_burst_fft_t *gpu_burst_fft_create(int fft_size, int batch_size,
                                               const float *window);

/* Destroy GPU FFT context and release all resources. */
GPU_API void gpu_burst_fft_destroy(gpu_burst_fft_t *g);

/* Process a batch of FFT frames on GPU.
 * input: batch_count * fft_size interleaved float pairs (re, im)
 * output: batch_count * fft_size floats (magnitude squared, DC-shifted)
 * batch_count: number of frames in this batch (<= batch_size)
 * Returns 0 on success, -1 on error. */
GPU_API int gpu_burst_fft_process(gpu_burst_fft_t *g, const float *input,
                                   float *output, int batch_count);

#endif /* __BURST_FFT_H__ */
