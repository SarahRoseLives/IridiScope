/*
 * GPU-accelerated burst detection FFT via Vulkan + VkFFT
 *
 * Pipeline per batch:
 *   1. CPU: window multiply (write directly to mapped GPU buffer)
 *   2. GPU: VkFFT batched forward FFT (in-place)
 *   3. CPU: fftshift + magnitude squared (read from mapped GPU buffer)
 *
 * Uses host-visible coherent memory for zero-copy transfers.
 * Suitable for shared-memory GPUs (Pi5 VideoCore VII) and discrete GPUs.
 *
 * Copyright (c) 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/*
 * GPU-accelerated burst detection FFT via Vulkan + VkFFT
 *
 * Pipeline per batch:
 *   1. CPU: window multiply (write directly to mapped GPU buffer)
 *   2. GPU: VkFFT batched forward FFT (in-place)
 *   3. CPU: fftshift + magnitude squared (read from mapped GPU buffer)
 *
 * Uses host-visible coherent memory for zero-copy transfers.
 * Suitable for shared-memory GPUs (Pi5 VideoCore VII) and discrete GPUs.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <vulkan/vulkan.h>
#include <glslang/Include/glslang_c_interface.h>

#define VKFFT_BACKEND 0
#include "vkFFT.h"

#include "burst_fft.h"

/* ---- GPU context ---- */

struct gpu_burst_fft {
    int fft_size;
    int batch_size;
    float *window;          /* CPU-side window coefficients */

    /* Vulkan objects */
    VkInstance instance;
    VkPhysicalDevice physical_device;
    VkDevice device;
    VkQueue queue;
    uint32_t queue_family;
    VkCommandPool command_pool;
    VkCommandBuffer command_buffer;
    VkFence fence;

    /* Buffer (host-visible, coherent) */
    VkBuffer buffer;
    VkDeviceMemory memory;
    float *mapped;          /* permanently mapped pointer */
    uint64_t buffer_size;

    /* VkFFT */
    VkFFTApplication vkfft_app;
};

/* ---- Find suitable memory type ---- */

static uint32_t find_memory_type(VkPhysicalDevice phys, uint32_t type_filter,
                                  VkMemoryPropertyFlags properties) {
    VkPhysicalDeviceMemoryProperties mem_props;
    vkGetPhysicalDeviceMemoryProperties(phys, &mem_props);

    for (uint32_t i = 0; i < mem_props.memoryTypeCount; i++) {
        if ((type_filter & (1 << i)) &&
            (mem_props.memoryTypes[i].propertyFlags & properties) == properties)
            return i;
    }
    return UINT32_MAX;
}

/* ---- Find compute-capable queue family ---- */

static int find_compute_queue_family(VkPhysicalDevice phys) {
    uint32_t count = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(phys, &count, NULL);

    VkQueueFamilyProperties *props = malloc(count * sizeof(*props));
    vkGetPhysicalDeviceQueueFamilyProperties(phys, &count, props);

    int idx = -1;

    /* Prefer compute-only queue (avoids contention with graphics) */
    for (uint32_t i = 0; i < count; i++) {
        if ((props[i].queueFlags & VK_QUEUE_COMPUTE_BIT) &&
            !(props[i].queueFlags & VK_QUEUE_GRAPHICS_BIT)) {
            idx = (int)i;
            break;
        }
    }

    /* Fall back to any compute queue */
    if (idx < 0) {
        for (uint32_t i = 0; i < count; i++) {
            if (props[i].queueFlags & VK_QUEUE_COMPUTE_BIT) {
                idx = (int)i;
                break;
            }
        }
    }

    free(props);
    return idx;
}

/* ---- Create ---- */

GPU_API gpu_burst_fft_t *gpu_burst_fft_create(int fft_size, int batch_size,
                                               const float *window) {
    VkResult vk;

    gpu_burst_fft_t *g = calloc(1, sizeof(*g));
    if (!g) return NULL;

    g->fft_size = fft_size;
    g->batch_size = batch_size;

    /* Save window coefficients for CPU-side multiply */
    g->window = malloc(sizeof(float) * fft_size);
    memcpy(g->window, window, sizeof(float) * fft_size);

    /* ---- Create Vulkan instance ---- */
    VkApplicationInfo app_info = {
        .sType = VK_STRUCTURE_TYPE_APPLICATION_INFO,
        .pApplicationName = "iridium-sniffer",
        .applicationVersion = VK_MAKE_VERSION(1, 0, 0),
        .apiVersion = VK_API_VERSION_1_1,
    };

    /* Since Vulkan SDK 1.3.216, portability drivers (e.g. MoltenVK on macOS)
     * mark themselves with is_portability_driver=true.  The loader hides them
     * from apps that don't explicitly opt in, returning
     * VK_ERROR_INCOMPATIBLE_DRIVER (-9).  Probe for the extension at runtime
     * so the same binary works on Linux without it. */
#ifdef VK_KHR_portability_enumeration
    const char *portability_ext = VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME;
    uint32_t ext_count = 0;
    vkEnumerateInstanceExtensionProperties(NULL, &ext_count, NULL);
    VkExtensionProperties *exts = malloc(ext_count * sizeof(*exts));
    vkEnumerateInstanceExtensionProperties(NULL, &ext_count, exts);
    int has_portability = 0;
    for (uint32_t e = 0; e < ext_count; e++) {
        if (strcmp(exts[e].extensionName, portability_ext) == 0) {
            has_portability = 1;
            break;
        }
    }
    free(exts);
#else
    int has_portability = 0;
#endif

    VkInstanceCreateInfo inst_info = {
        .sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO,
        .pApplicationInfo = &app_info,
#ifdef VK_KHR_portability_enumeration
        .flags = has_portability ? VK_INSTANCE_CREATE_ENUMERATE_PORTABILITY_BIT_KHR : 0,
        .enabledExtensionCount = has_portability ? 1 : 0,
        .ppEnabledExtensionNames = has_portability ? &portability_ext : NULL,
#endif
    };

    vk = vkCreateInstance(&inst_info, NULL, &g->instance);
    if (vk != VK_SUCCESS) {
        fprintf(stderr, "Vulkan: create instance error %d\n", vk);
        goto err_free;
    }

    /* ---- Find physical device ---- */
    uint32_t dev_count = 0;
    vkEnumeratePhysicalDevices(g->instance, &dev_count, NULL);
    if (dev_count == 0) {
        fprintf(stderr, "Vulkan: no devices found\n");
        goto err_instance;
    }

    VkPhysicalDevice *devices = malloc(dev_count * sizeof(VkPhysicalDevice));
    vkEnumeratePhysicalDevices(g->instance, &dev_count, devices);

    /* Prefer discrete GPU, fall back to any */
    g->physical_device = devices[0];
    for (uint32_t i = 0; i < dev_count; i++) {
        VkPhysicalDeviceProperties props;
        vkGetPhysicalDeviceProperties(devices[i], &props);
        if (props.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU) {
            g->physical_device = devices[i];
            break;
        }
    }

    VkPhysicalDeviceProperties dev_props;
    vkGetPhysicalDeviceProperties(g->physical_device, &dev_props);
    fprintf(stderr, "Vulkan GPU: %s\n", dev_props.deviceName);
    free(devices);

    /* ---- Find compute queue family ---- */
    int qf = find_compute_queue_family(g->physical_device);
    if (qf < 0) {
        fprintf(stderr, "Vulkan: no compute queue found\n");
        goto err_instance;
    }
    g->queue_family = (uint32_t)qf;

    /* ---- Create logical device ---- */
    float queue_priority = 1.0f;
    VkDeviceQueueCreateInfo queue_info = {
        .sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO,
        .queueFamilyIndex = g->queue_family,
        .queueCount = 1,
        .pQueuePriorities = &queue_priority,
    };
    VkDeviceCreateInfo dev_info = {
        .sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO,
        .queueCreateInfoCount = 1,
        .pQueueCreateInfos = &queue_info,
    };

    vk = vkCreateDevice(g->physical_device, &dev_info, NULL, &g->device);
    if (vk != VK_SUCCESS) {
        fprintf(stderr, "Vulkan: create device error %d\n", vk);
        goto err_instance;
    }

    vkGetDeviceQueue(g->device, g->queue_family, 0, &g->queue);

    /* ---- Command pool + buffer ---- */
    VkCommandPoolCreateInfo pool_info = {
        .sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO,
        .flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT,
        .queueFamilyIndex = g->queue_family,
    };

    vk = vkCreateCommandPool(g->device, &pool_info, NULL, &g->command_pool);
    if (vk != VK_SUCCESS) {
        fprintf(stderr, "Vulkan: create command pool error %d\n", vk);
        goto err_device;
    }

    VkCommandBufferAllocateInfo alloc_info = {
        .sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO,
        .commandPool = g->command_pool,
        .level = VK_COMMAND_BUFFER_LEVEL_PRIMARY,
        .commandBufferCount = 1,
    };

    vk = vkAllocateCommandBuffers(g->device, &alloc_info, &g->command_buffer);
    if (vk != VK_SUCCESS) {
        fprintf(stderr, "Vulkan: allocate command buffer error %d\n", vk);
        goto err_pool;
    }

    /* ---- Fence ---- */
    VkFenceCreateInfo fence_info = {
        .sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO,
    };

    vk = vkCreateFence(g->device, &fence_info, NULL, &g->fence);
    if (vk != VK_SUCCESS) {
        fprintf(stderr, "Vulkan: create fence error %d\n", vk);
        goto err_pool;
    }

    /* ---- Allocate GPU buffer ---- */
    g->buffer_size = (uint64_t)batch_size * fft_size * 2 * sizeof(float);

    VkBufferCreateInfo buf_info = {
        .sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO,
        .size = g->buffer_size,
        .usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT
               | VK_BUFFER_USAGE_TRANSFER_SRC_BIT
               | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
        .sharingMode = VK_SHARING_MODE_EXCLUSIVE,
    };

    vk = vkCreateBuffer(g->device, &buf_info, NULL, &g->buffer);
    if (vk != VK_SUCCESS) {
        fprintf(stderr, "Vulkan: create buffer error %d\n", vk);
        goto err_fence;
    }

    /* Find host-visible, coherent memory */
    VkMemoryRequirements mem_reqs;
    vkGetBufferMemoryRequirements(g->device, g->buffer, &mem_reqs);

    uint32_t mem_type = find_memory_type(g->physical_device, mem_reqs.memoryTypeBits,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    if (mem_type == UINT32_MAX) {
        fprintf(stderr, "Vulkan: no suitable memory type found\n");
        goto err_buffer;
    }

    VkMemoryAllocateInfo mem_info = {
        .sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO,
        .allocationSize = mem_reqs.size,
        .memoryTypeIndex = mem_type,
    };

    vk = vkAllocateMemory(g->device, &mem_info, NULL, &g->memory);
    if (vk != VK_SUCCESS) {
        fprintf(stderr, "Vulkan: allocate memory error %d\n", vk);
        goto err_buffer;
    }

    vk = vkBindBufferMemory(g->device, g->buffer, g->memory, 0);
    if (vk != VK_SUCCESS) {
        fprintf(stderr, "Vulkan: bind buffer memory error %d\n", vk);
        goto err_memory;
    }

    /* Map buffer permanently */
    vk = vkMapMemory(g->device, g->memory, 0, g->buffer_size, 0,
                     (void **)&g->mapped);
    if (vk != VK_SUCCESS) {
        fprintf(stderr, "Vulkan: map memory error %d\n", vk);
        goto err_memory;
    }

    /* ---- Initialize glslang (VkFFT needs it for shader compilation) ---- */
    glslang_initialize_process();

    /* ---- Initialize VkFFT ---- */
    VkFFTConfiguration config = {};
    config.FFTdim = 1;
    config.size[0] = (uint64_t)fft_size;
    config.numberBatches = (uint64_t)batch_size;
    config.physicalDevice = &g->physical_device;
    config.device = &g->device;
    config.queue = &g->queue;
    config.commandPool = &g->command_pool;
    config.fence = &g->fence;
    config.isCompilerInitialized = 1;
    config.buffer = &g->buffer;
    config.bufferSize = &g->buffer_size;

    VkFFTResult vk_res = initializeVkFFT(&g->vkfft_app, config);
    if (vk_res != VKFFT_SUCCESS) {
        fprintf(stderr, "VkFFT init error: %d\n", vk_res);
        goto err_glslang;
    }

    /* ---- Validation: run a test FFT to verify GPU actually works ----
     * Some GPUs (e.g. Pi5 VideoCore VII) report Vulkan support but
     * cannot execute VkFFT compute shaders correctly.  Run a single
     * DC-input FFT and check the output makes sense. */
    {
        /* Fill first frame with DC signal: all (1.0, 0.0) */
        for (int i = 0; i < fft_size; i++) {
            g->mapped[2 * i]     = 1.0f;
            g->mapped[2 * i + 1] = 0.0f;
        }

        VkCommandBufferBeginInfo tbi = {
            .sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO,
            .flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT,
        };
        vkResetCommandBuffer(g->command_buffer, 0);
        vkBeginCommandBuffer(g->command_buffer, &tbi);

        VkFFTLaunchParams tp = {};
        tp.commandBuffer = &g->command_buffer;
        tp.buffer = &g->buffer;
        VkFFTResult tr = VkFFTAppend(&g->vkfft_app, -1, &tp);

        if (tr != VKFFT_SUCCESS) {
            fprintf(stderr, "GPU validation: VkFFT append failed (%d), disabling GPU\n", tr);
            goto err_glslang;
        }

        vkEndCommandBuffer(g->command_buffer);

        VkSubmitInfo tsi = {
            .sType = VK_STRUCTURE_TYPE_SUBMIT_INFO,
            .commandBufferCount = 1,
            .pCommandBuffers = &g->command_buffer,
        };

        vk = vkQueueSubmit(g->queue, 1, &tsi, g->fence);
        if (vk != VK_SUCCESS) {
            fprintf(stderr, "GPU validation: submit failed (%d), disabling GPU\n", vk);
            goto err_glslang;
        }

        /* Wait with 2-second timeout instead of forever */
        vk = vkWaitForFences(g->device, 1, &g->fence, VK_TRUE,
                             (uint64_t)2000000000);
        if (vk == VK_TIMEOUT) {
            fprintf(stderr, "GPU validation: FFT timed out (GPU compute broken), disabling GPU\n");
            /* Device may be stuck -- skip cleanup that could hang */
            vkDestroyInstance(g->instance, NULL);
            free(g->window);
            free(g);
            return NULL;
        }
        if (vk != VK_SUCCESS) {
            fprintf(stderr, "GPU validation: fence wait error (%d), disabling GPU\n", vk);
            goto err_glslang;
        }
        vkResetFences(g->device, 1, &g->fence);

        /* DC input FFT: bin 0 should have all the energy (re = fft_size, im = 0).
         * Just check that bin 0 magnitude >> other bins. */
        float dc_re = g->mapped[0];
        float dc_im = g->mapped[1];
        float dc_mag = dc_re * dc_re + dc_im * dc_im;
        float expected = (float)fft_size * (float)fft_size;

        if (dc_mag < expected * 0.5f || dc_mag > expected * 2.0f) {
            fprintf(stderr, "GPU validation: DC test failed (expected ~%.0f, got %.0f), disabling GPU\n",
                    expected, dc_mag);
            goto err_glslang;
        }
    }

    fprintf(stderr, "GPU burst FFT: %d-point, batch %d, VkFFT ready\n",
            fft_size, batch_size);
    return g;

err_glslang:
    glslang_finalize_process();
    vkUnmapMemory(g->device, g->memory);
err_memory:
    vkFreeMemory(g->device, g->memory, NULL);
err_buffer:
    vkDestroyBuffer(g->device, g->buffer, NULL);
err_fence:
    vkDestroyFence(g->device, g->fence, NULL);
err_pool:
    vkDestroyCommandPool(g->device, g->command_pool, NULL);
err_device:
    vkDestroyDevice(g->device, NULL);
err_instance:
    vkDestroyInstance(g->instance, NULL);
err_free:
    free(g->window);
    free(g);
    return NULL;
}

/* ---- Destroy ---- */

GPU_API void gpu_burst_fft_destroy(gpu_burst_fft_t *g) {
    if (!g) return;

    deleteVkFFT(&g->vkfft_app);
    glslang_finalize_process();

    vkUnmapMemory(g->device, g->memory);
    vkFreeMemory(g->device, g->memory, NULL);
    vkDestroyBuffer(g->device, g->buffer, NULL);
    vkDestroyFence(g->device, g->fence, NULL);
    vkDestroyCommandPool(g->device, g->command_pool, NULL);
    vkDestroyDevice(g->device, NULL);
    vkDestroyInstance(g->instance, NULL);

    free(g->window);
    free(g);
}

/* ---- Process batch ---- */

GPU_API int gpu_burst_fft_process(gpu_burst_fft_t *g, const float *input,
                                   float *output, int batch_count) {
    if (batch_count <= 0 || batch_count > g->batch_size)
        return -1;

    int n = batch_count * g->fft_size;

    /* Step 1: CPU window multiply, write to mapped GPU buffer */
    for (int i = 0; i < n; i++) {
        int win_idx = i % g->fft_size;
        float w = g->window[win_idx];
        g->mapped[2 * i]     = input[2 * i]     * w;
        g->mapped[2 * i + 1] = input[2 * i + 1] * w;
    }

    /* Step 2: Record and submit VkFFT forward FFT */
    VkCommandBufferBeginInfo begin_info = {
        .sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO,
        .flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT,
    };

    vkResetCommandBuffer(g->command_buffer, 0);
    vkBeginCommandBuffer(g->command_buffer, &begin_info);

    VkFFTLaunchParams params = {};
    params.commandBuffer = &g->command_buffer;
    params.buffer = &g->buffer;

    VkFFTResult vk_res = VkFFTAppend(&g->vkfft_app, -1, &params);
    if (vk_res != VKFFT_SUCCESS) {
        fprintf(stderr, "GPU: VkFFT error %d\n", vk_res);
        return -1;
    }

    vkEndCommandBuffer(g->command_buffer);

    VkSubmitInfo submit_info = {
        .sType = VK_STRUCTURE_TYPE_SUBMIT_INFO,
        .commandBufferCount = 1,
        .pCommandBuffers = &g->command_buffer,
    };

    VkResult vk = vkQueueSubmit(g->queue, 1, &submit_info, g->fence);
    if (vk != VK_SUCCESS) {
        fprintf(stderr, "GPU: queue submit error %d\n", vk);
        return -1;
    }

    /* 5-second timeout prevents hanging on broken GPUs */
    vk = vkWaitForFences(g->device, 1, &g->fence, VK_TRUE,
                         (uint64_t)5000000000);
    if (vk != VK_SUCCESS) {
        fprintf(stderr, "GPU: fence wait %s\n",
                vk == VK_TIMEOUT ? "timed out" : "failed");
        return -1;
    }
    vkResetFences(g->device, 1, &g->fence);

    /* Step 3: CPU fftshift + magnitude squared */
    int half_n = g->fft_size / 2;
    for (int i = 0; i < n; i++) {
        int frame = i / g->fft_size;
        int bin = i % g->fft_size;
        int src_bin = (bin + half_n) % g->fft_size;
        int src_idx = frame * g->fft_size + src_bin;
        float re = g->mapped[2 * src_idx];
        float im = g->mapped[2 * src_idx + 1];
        output[i] = re * re + im * im;
    }

    return 0;
}

