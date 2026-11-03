#include "edev_camera_io.h"
#include "../src/video_ddr.h"
#include "../src/video_crc.h"
#include "../src/video_host_direct.h"

#ifndef BIT
#define BIT(nr) (1UL << (nr))
#endif
#ifndef __packed
#define __packed __attribute__((packed))
#endif
#ifndef __aligned
#define __aligned(x) __attribute__((aligned(x)))
#endif
#include "edev_sgdma_driver.h"

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>
#include <dlfcn.h>

#ifndef MAP_FAILED
#define MAP_FAILED ((void *)-1)
#endif

#define CAMERA_CSR_MAP_SIZE 0x1000u
#define CAMERA_DMA_ALIGN 4096u
#define CAMERA_DMA_PUMP_WAIT_MS 5u
#define CAMERA_DMA_MAX_SYNC_QUEUES 8u
#define CAMERA_DROP_REASON_RING_FULL 16u
#define CAMERA_DROP_REASON_LINE_MISMATCH 3u
#define CAMERA_DROP_REASON_FRAME_MISMATCH 4u
#define CAMERA_DROP_REASON_HEIGHT_MISMATCH 5u
#define CAMERA_DROP_REASON_FINAL_SIZE_MISMATCH 9u
#define CAMERA_DROP_REASON_CMLK_SYNC_MISMATCH 13u

enum camera_dma_buffer_state {
    CAMERA_DMA_FREE = 0,
    CAMERA_DMA_SUBMITTED = 1,
    CAMERA_DMA_DONE = 2,
    CAMERA_DMA_READY = 3,
    CAMERA_DMA_CHECKED_OUT = 4,
};

struct camera_dma_buffer {
    void *ptr;
    size_t size;
    uint32_t buf_id;
    uint32_t lane_buf_id[CAMERA_DMA_MAX_SYNC_QUEUES];
    int state;
    int locked;
    int cuda_host_alloc;
    int coherent_owner;
    int coherent_fd;
    uint64_t coherent_iova, direct_release_token;
    uint64_t op_id;
    size_t len;
    struct camera_frame_info info;
};

struct camera_context;

struct camera_dma_lane {
    struct camera_context *ctx;
    int fd;
    uint32_t qid;
    pthread_t thread;
    int active;
};

struct camera_context {
    int ctrl_fd;
    int queue_fd;
    char queue_path[PATH_MAX];
    unsigned int dma_lane_count;
    struct camera_dma_lane dma_lanes[CAMERA_DMA_MAX_SYNC_QUEUES];
    volatile uint8_t *csr;
    size_t csr_map_size;
    uint64_t csr_base;
    uint64_t ring_base;
    uint32_t frame_stride;
    uint32_t ring_slots;
    uint32_t qid;
    struct camera_dma_buffer *dma_buffers;
    uint32_t *dma_ready_ring;
    unsigned int dma_buffer_count;
    unsigned int dma_inflight_limit;
    unsigned int dma_inflight;
    unsigned int dma_ready_head;
    unsigned int dma_ready_tail;
    unsigned int dma_ready_count;
    uint32_t dma_next_submit_idx;
    uint32_t dma_next_ddr_release_idx;
    struct ddr_ring *ddr;
    int ddr_fd;
    int host_direct, host_armed;
    uint32_t direct_cursor, direct_release_cursor, direct_ddr_publish;
    uint32_t direct_cmlk_drop_budget;
    uint32_t ddr_release_idx;
    int dma_latest_mode;
    uint32_t dma_latest_ddr_skipped_pending;
    uint32_t dma_drop_baseline;
    uint32_t dma_hls_resync_baseline;
    uint32_t dma_producer_baseline;
    uint32_t dma_frame_seq_baseline;
    uint32_t hw_build_id;
    uint32_t hw_build_version;
    uint32_t dma_expected_width;
    uint32_t dma_expected_height;
    uint32_t dma_expected_frame_bytes;
    uint32_t configured_control_flags;
    uint32_t configured_bytes_per_clock;
    uint32_t configured_tap_order;
    int configured_tap_order_set;
    uint32_t configured_processing_mode;
    uint32_t configured_dma_queue_count;
    int configured_dma_frame_irq;
    int configured_driver_frame_dma;
    int dma_driver_frame_mode;
    uint32_t configured_expected_line_bytes;
    uint32_t configured_expected_height;
    uint32_t configured_expected_frame_bytes;
    int dma_strided;
    uint32_t dma_src_line_bytes;      /* useful RGB8 bytes copied from each source row */
    uint32_t dma_src_stride_learned;  /* actual DDR source line stride learned from FPGA metadata; 0 until first valid frame */
    uint32_t dma_dst_offset;          /* NvBufSurface plane offset inside the imported DMA-BUF */
    uint32_t dma_dst_stride;          /* NvBufSurface destination pitch */
    uint32_t dma_rows;
    size_t dma_buffer_size;
    size_t dma_transfer_size;
    pthread_mutex_t dma_lock;
    pthread_cond_t dma_cond;
    pthread_t dma_thread;
    int dma_sync_ready;
    int dma_thread_active;
    int dma_thread_stop;
    int dma_thread_error;
    int dma_frame_waiter_active;
    char camera_start_cmd[160];
    char camera_stop_cmd[160];
    int camera_control_timeout_ms;
    int camera_remote_started;
    char last_error[1024];
};

static int camera_dma_start_pump(struct camera_context *ctx);
static void camera_dma_stop_pump(struct camera_context *ctx);
static unsigned int camera_dma_env_u32(const char *name, unsigned int fallback,
                                        unsigned int min_v, unsigned int max_v);
/* forward */ static unsigned int camera_dma_env_u32(const char *name, unsigned int fallback, unsigned int min_v, unsigned int max_v);
static unsigned int camera_dma_env_u32(const char *name, unsigned int fallback,
                                       unsigned int min_v, unsigned int max_v);

static void camera_set_error(struct camera_context *ctx, const char *fmt, ...)
{
    va_list ap;

    if (!ctx)
        return;

    va_start(ap, fmt);
    vsnprintf(ctx->last_error, sizeof(ctx->last_error), fmt, ap);
    va_end(ap);
}

static void camera_save_error(struct camera_context *ctx, char *buf, size_t len, int *errp)
{
    if (errp)
        *errp = errno;
    if (buf && len) {
        if (ctx && ctx->last_error[0])
            snprintf(buf, len, "%s", ctx->last_error);
        else
            buf[0] = '\0';
    }
}

static void camera_restore_error(struct camera_context *ctx, const char *buf, int err)
{
    if (ctx && buf && buf[0])
        snprintf(ctx->last_error, sizeof(ctx->last_error), "%s", buf);
    if (err)
        errno = err;
}

static uint32_t camera_read32(const struct camera_context *ctx, uint32_t reg)
{
    volatile uint32_t *p = (volatile uint32_t *)(ctx->csr + reg);
    return *p;
}

static void camera_write32(const struct camera_context *ctx, uint32_t reg,
                           uint32_t value)
{
    volatile uint32_t *p = (volatile uint32_t *)(ctx->csr + reg);
    *p = value;
}


static void camera_mmio_flush(const struct camera_context *ctx)
{
    (void)camera_read32(ctx, CAMERA_REG_CONTROL);
}

/* Program the camera UART from the clock actually implemented in RTL.
 * New RTL reports the AXI-Lite clock at CAMERA_REG_SERIAL_CLOCK_HZ; older
 * bitstreams return zero, in which case the OpenNIC 125 MHz control clock is
 * used as the compatibility fallback.  The divider is cycles per UART bit.
 */
static int camera_serial_configure_default_baud(struct camera_context *ctx)
{
    uint32_t clk_hz;
    uint32_t divisor;
    uint32_t readback;

    if (!ctx || ctx->csr == MAP_FAILED) {
        errno = EINVAL;
        return -1;
    }

    clk_hz = camera_read32(ctx, CAMERA_REG_SERIAL_CLOCK_HZ);
    if (!clk_hz || clk_hz == UINT32_MAX)
        clk_hz = CAMERA_DEFAULT_AXIL_CLOCK_HZ;

    divisor = (clk_hz + CAMERA_DEFAULT_SERIAL_BAUD / 2u) /
              CAMERA_DEFAULT_SERIAL_BAUD;
    if (divisor < CAMERA_SERIAL_MIN_BAUD_DIV) {
        camera_set_error(ctx,
                         "camera UART clock %u Hz cannot generate %u baud",
                         clk_hz, CAMERA_DEFAULT_SERIAL_BAUD);
        errno = ERANGE;
        return -1;
    }

    camera_write32(ctx, CAMERA_REG_SERIAL_BAUD_DIV, divisor);
    camera_mmio_flush(ctx);
    usleep(1000);

    readback = camera_read32(ctx, CAMERA_REG_SERIAL_BAUD_DIV);
    if (readback != divisor) {
        camera_set_error(ctx,
                         "camera UART divider readback mismatch: wrote %u read %u",
                         divisor, readback);
        errno = EIO;
        return -1;
    }
    if (camera_read32(ctx, CAMERA_REG_SERIAL_STATUS) &
        CAMERA_SERIAL_STATUS_BAUD_DIV_ERR) {
        camera_set_error(ctx,
                         "camera UART rejected divider %u for %u Hz",
                         divisor, clk_hz);
        errno = EIO;
        return -1;
    }
    return 0;
}

static const char *camera_status_bits(uint32_t status, char *buf, size_t len)
{
    int n;

    if (!buf || !len)
        return "";

    n = snprintf(buf, len, "%s%s%s%s%s%s%s%s%s%s%s",
                 (status & CAMERA_STATUS_FIFO_OVERFLOW) ? " fifo_overflow" : "",
                 (status & CAMERA_STATUS_META_OVERFLOW) ? " meta_overflow" : "",
                 (status & CAMERA_STATUS_AXI_ERROR) ? " axi_error" : "",
                 (status & CAMERA_STATUS_CFG_ERROR) ? " cfg_error" : "",
                 (status & CAMERA_STATUS_FRAME_SIZE_ERROR) ? " frame_size_error" : "",
                 (status & CAMERA_STATUS_LINK_READY) ? " link_ready" : "",
                 (status & CAMERA_STATUS_FRAME_VALID) ? " frame_valid" : "",
                 (status & CAMERA_STATUS_CFG_READY) ? " cfg_ready" : "",
                 (status & CAMERA_STATUS_FVAL_SEEN) ? " fval_seen" : "",
                 (status & CAMERA_STATUS_LVAL_SEEN) ? " lval_seen" : "",
                 (status & CAMERA_STATUS_DVAL_SEEN) ? " dval_seen" : "");
    if (n < 0)
        buf[0] = '\0';
    return buf;
}

static int camera_check_csr_healthy(struct camera_context *ctx,
                                    const struct camera_frame_info *info,
                                    const char *where)
{
    char status_desc[128];

    if (!ctx || !info) {
        errno = EINVAL;
        return -1;
    }

    if (info->status & CAMERA_STATUS_ERROR_MASK) {
        camera_set_error(ctx,
                         "%s: status=0x%08x%s producer=%u consumer=%u "
                         "width=%u height=%u frame_bytes=%u seq=%u drop=%u",
                         where ? where : "camera CSR error",
                         info->status,
                         camera_status_bits(info->status, status_desc, sizeof(status_desc)),
                         info->producer_idx, info->consumer_idx,
                         info->width, info->height, info->frame_bytes,
                         info->frame_seq, info->drop_count);
        errno = EPIPE;
        return -1;
    }

    return 0;
}

static int camera_validate_frame_info(struct camera_context *ctx,
                                      const struct camera_frame_info *info,
                                      size_t expected_frame_size,
                                      const char *where)
{
    if (camera_check_csr_healthy(ctx, info, where) < 0)
        return -1;

    if (!info->width || !info->height || !info->frame_bytes) {
        camera_set_error(ctx,
                         "%s: invalid frame metadata: producer=%u consumer=%u "
                         "width=%u height=%u frame_bytes=%u seq=%u drop=%u status=0x%08x",
                         where ? where : "camera frame",
                         info->producer_idx, info->consumer_idx,
                         info->width, info->height, info->frame_bytes,
                         info->frame_seq, info->drop_count, info->status);
        errno = EPROTO;
        return -1;
    }

    if (expected_frame_size && expected_frame_size > info->frame_bytes) {
        camera_set_error(ctx,
                         "%s: requested frame bytes %zu exceed CSR frame_bytes=%u "
                         "width=%u height=%u producer=%u consumer=%u status=0x%08x",
                         where ? where : "camera frame",
                         expected_frame_size, info->frame_bytes,
                         info->width, info->height,
                         info->producer_idx, info->consumer_idx, info->status);
        errno = EMSGSIZE;
        return -1;
    }

    return 0;
}

static long long camera_now_ms(void)
{
    struct timespec ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000ll + (long long)ts.tv_nsec / 1000000ll;
}

static uint32_t camera_slot_mask_or_mod(uint32_t idx, uint32_t slots)
{
    if (slots && ((slots & (slots - 1u)) == 0))
        return idx & (slots - 1u);
    return slots ? (idx % slots) : 0;
}


static size_t camera_round_up(size_t value, size_t align)
{
    if (!align)
        return value;
    return (value + align - 1u) & ~(align - 1u);
}

struct camera_cuda_api {
    void *handle;
    int (*cudaHostAlloc)(void **, size_t, unsigned int);
    int (*cudaFreeHost)(void *);
};

static struct camera_cuda_api g_cuda_api;
static pthread_mutex_t g_cuda_api_lock = PTHREAD_MUTEX_INITIALIZER;

static int camera_cuda_load_runtime(void)
{
    static const char *names[] = {
        "libcudart.so", "libcudart.so.12", "libcudart.so.11.0", NULL
    };
    int i;

    pthread_mutex_lock(&g_cuda_api_lock);
    if (g_cuda_api.handle) {
        pthread_mutex_unlock(&g_cuda_api_lock);
        return 0;
    }
    for (i = 0; names[i]; i++) {
        g_cuda_api.handle = dlopen(names[i], RTLD_LAZY | RTLD_LOCAL);
        if (g_cuda_api.handle)
            break;
    }
    if (!g_cuda_api.handle) {
        pthread_mutex_unlock(&g_cuda_api_lock);
        return -1;
    }
    *(void **)(&g_cuda_api.cudaHostAlloc) = dlsym(g_cuda_api.handle, "cudaHostAlloc");
    *(void **)(&g_cuda_api.cudaFreeHost) = dlsym(g_cuda_api.handle, "cudaFreeHost");
    if (!g_cuda_api.cudaHostAlloc || !g_cuda_api.cudaFreeHost) {
        dlclose(g_cuda_api.handle);
        memset(&g_cuda_api, 0, sizeof(g_cuda_api));
        pthread_mutex_unlock(&g_cuda_api_lock);
        return -1;
    }
    pthread_mutex_unlock(&g_cuda_api_lock);
    return 0;
}

static int camera_use_cuda_pinned_buffers(void)
{
    const char *e = getenv("CAMERA_DMA_CUDA_PINNED");
    return e && *e && strcmp(e, "0") != 0;
}

static int camera_dma_alloc_host_buffer(struct camera_dma_buffer *buf, size_t size)
{
    if (camera_use_cuda_pinned_buffers() && camera_cuda_load_runtime() == 0) {
        enum { CUDA_HOST_ALLOC_MAPPED = 2 };
        void *ptr = NULL;
        int ret = g_cuda_api.cudaHostAlloc(&ptr, size, CUDA_HOST_ALLOC_MAPPED);
        if (ret == 0 && ptr) {
            buf->ptr = ptr;
            buf->cuda_host_alloc = 1;
            return 0;
        }
    }
    return posix_memalign(&buf->ptr, CAMERA_DMA_ALIGN, size);
}

static void camera_dma_free_host_buffer(struct camera_dma_buffer *buf)
{
    if (!buf || !buf->ptr)
        return;
    if (buf->coherent_owner) {
        munmap(buf->ptr,buf->size);
        close(buf->coherent_fd);
        buf->coherent_owner=0;
    } else if (buf->cuda_host_alloc && g_cuda_api.cudaFreeHost) {
        (void)g_cuda_api.cudaFreeHost(buf->ptr);
    } else {
        free(buf->ptr);
    }
    buf->ptr = NULL;
    buf->cuda_host_alloc = 0;
}

static void camera_dma_signal(struct camera_context *ctx)
{
    if (ctx && ctx->dma_sync_ready)
        pthread_cond_broadcast(&ctx->dma_cond);
}

static int camera_ready_push(struct camera_context *ctx, uint32_t buffer_index)
{
    if (!ctx->dma_ready_ring || ctx->dma_ready_count >= ctx->dma_buffer_count) {
        camera_set_error(ctx, "host ready queue overflow (%u/%u)",
                         ctx->dma_ready_count, ctx->dma_buffer_count);
        errno = ENOSPC;
        return -1;
    }

    ctx->dma_ready_ring[ctx->dma_ready_tail] = buffer_index;
    ctx->dma_ready_tail = (ctx->dma_ready_tail + 1u) % ctx->dma_buffer_count;
    ctx->dma_ready_count++;
    camera_dma_signal(ctx);
    return 0;
}

static int camera_ready_pop(struct camera_context *ctx)
{
    uint32_t buffer_index;

    if (!ctx->dma_ready_ring || !ctx->dma_ready_count)
        return -1;

    buffer_index = ctx->dma_ready_ring[ctx->dma_ready_head];
    ctx->dma_ready_head = (ctx->dma_ready_head + 1u) % ctx->dma_buffer_count;
    ctx->dma_ready_count--;
    return (int)buffer_index;
}

static int camera_dma_pool_is_idle(struct camera_context *ctx)
{
    unsigned int i;

    if (!ctx || !ctx->dma_buffers)
        return 1;

    if (ctx->dma_inflight || ctx->dma_ready_count) {
        camera_set_error(ctx,
                         "DMA pool is busy before strict capture start: inflight=%u ready=%u",
                         ctx->dma_inflight, ctx->dma_ready_count);
        errno = EBUSY;
        return 0;
    }

    for (i = 0; i < ctx->dma_buffer_count; i++) {
        if (ctx->dma_buffers[i].state != CAMERA_DMA_FREE) {
            camera_set_error(ctx,
                             "DMA buffer %u is not free before strict capture start (state=%d)",
                             i, ctx->dma_buffers[i].state);
            errno = EBUSY;
            return 0;
        }
    }
    return 1;
}

static void camera_dma_reset_runtime_indices(struct camera_context *ctx,
                                             uint32_t consumer_idx,
                                             uint32_t drop_baseline,
                                             uint32_t expected_line_bytes,
                                             uint32_t expected_height,
                                             uint32_t expected_frame_bytes)
{
    unsigned int i;

    if (!ctx || !ctx->dma_buffers)
        return;

    ctx->dma_inflight = 0;
    ctx->dma_ready_head = 0;
    ctx->dma_ready_tail = 0;
    ctx->dma_ready_count = 0;
    ctx->dma_next_submit_idx = consumer_idx;
    ctx->dma_next_ddr_release_idx = consumer_idx;
    ctx->dma_latest_ddr_skipped_pending = 0;
    ctx->dma_drop_baseline = drop_baseline;
    ctx->dma_hls_resync_baseline = camera_read32(ctx, CAMERA_REG_HLS_RESYNC_COUNT);
    ctx->dma_producer_baseline = consumer_idx;
    ctx->dma_frame_seq_baseline = camera_read32(ctx, CAMERA_REG_FRAME_SEQ);
    ctx->dma_frame_waiter_active = 0;
    ctx->dma_expected_width = expected_line_bytes;
    ctx->dma_expected_height = expected_height;
    ctx->dma_expected_frame_bytes = expected_frame_bytes;

    for (i = 0; i < ctx->dma_buffer_count; i++) {
        ctx->dma_buffers[i].state = CAMERA_DMA_FREE;
        ctx->dma_buffers[i].op_id = 0;
        ctx->dma_buffers[i].len = 0;
        memset(&ctx->dma_buffers[i].info, 0, sizeof(ctx->dma_buffers[i].info));
    }
}

/*
 * Build 3.9 can isolate a short/long physical frame and resume at the next
 * genuine SOF. Build 3.12 additionally tags a frame containing replicated
 * Camera Link sync disagreement with reason 13. Only these hardware-proven,
 * isolated cases are recoverable; FIFO, AXI and configuration failures remain
 * fatal.
 */
static int camera_accept_hls_resync(struct camera_context *ctx,
                                    const struct camera_frame_info *info)
{
    uint32_t error_bits;
    uint32_t reason;
    uint32_t resync_count;
    int hls_recovered;
    int cmlk_frame_rejected;

    if (!ctx || !info || ctx->hw_build_version < 0x00030009u)
        return 0;

    error_bits = info->status & CAMERA_STATUS_ERROR_MASK;
    if ((error_bits & ~CAMERA_STATUS_FRAME_SIZE_ERROR) != 0u)
        return 0;
    if (error_bits == 0u && info->drop_count == ctx->dma_drop_baseline)
        return 0;

    resync_count = camera_read32(ctx, CAMERA_REG_HLS_RESYNC_COUNT);
    reason = camera_read32(ctx, CAMERA_REG_DIAG_DROP_REASON);
    hls_recovered = resync_count != ctx->dma_hls_resync_baseline;
    cmlk_frame_rejected =
        ctx->hw_build_version >= CAMERA_ISP_DYNAMIC_VERSION_MIN &&
        reason == CAMERA_DROP_REASON_CMLK_SYNC_MISMATCH;

    if (!hls_recovered && !cmlk_frame_rejected)
        return 0;
    if (!cmlk_frame_rejected &&
        reason != CAMERA_DROP_REASON_LINE_MISMATCH &&
        reason != CAMERA_DROP_REASON_FRAME_MISMATCH &&
        reason != CAMERA_DROP_REASON_HEIGHT_MISMATCH &&
        reason != CAMERA_DROP_REASON_FINAL_SIZE_MISMATCH)
        return 0;

    fprintf(stderr,
            "camera: discarded one malformed input frame and restored SOF alignment "
            "(hls_resync %u->%u, drop %u->%u, reason=%u)\n",
            ctx->dma_hls_resync_baseline, resync_count,
            ctx->dma_drop_baseline, info->drop_count, reason);

    if (camera_clear_status(ctx) < 0)
        return -1;
    ctx->dma_hls_resync_baseline =
        camera_read32(ctx, CAMERA_REG_HLS_RESYNC_COUNT);
    ctx->dma_drop_baseline = camera_read32(ctx, CAMERA_REG_DROP_COUNT);
    return 1;
}

static int camera_validate_drop_count(struct camera_context *ctx,
                                      const struct camera_frame_info *info,
                                      const char *where)
{
    uint32_t reason;

    if (info->drop_count == ctx->dma_drop_baseline)
        return 0;

    reason = camera_read32(ctx, CAMERA_REG_DIAG_DROP_REASON);
    if (reason == CAMERA_DROP_REASON_RING_FULL) {
        /* The DDR ring drops the newly arriving frame when all 32 slots are
         * reserved; committed slots remain valid. Advance only the diagnostic
         * baseline and continue draining those slots. Data-integrity drop
         * reasons remain fatal.
         */
        ctx->dma_drop_baseline = info->drop_count;
        return 0;
    }

    camera_set_error(ctx,
                     "%s: FPGA drop counter changed from %u to %u, reason=%u; strict delivery stopped",
                     where ? where : "camera DMA",
                     ctx->dma_drop_baseline, info->drop_count, reason);
    errno = EOVERFLOW;
    return -1;
}

static int camera_validate_dma_stream(struct camera_context *ctx,
                                      const struct camera_frame_info *info,
                                      const char *where)
{
    uint32_t occupancy;
    int recovered = 0;
    if (ctx->ddr) {
        uint32_t reason = camera_read32(ctx, CAMERA_REG_DIAG_DROP_REASON);
        if (reason == CAMERA_DROP_REASON_CMLK_SYNC_MISMATCH &&
            ((info->status & CAMERA_STATUS_ERROR_MASK) ||
             info->drop_count != ctx->dma_drop_baseline)) {
            /* One Camera Link frame can be rejected without invalidating
             * already committed DDR slots. Serialize recovery under dma_lock;
             * the ioctl waiter must not clear the diagnostic first. */
            recovered = camera_accept_hls_resync(ctx, info);
            if (recovered < 0)
                return -1;
        }
        if (!recovered && ((info->status & CAMERA_STATUS_ERROR_MASK) ||
            (info->drop_count != ctx->dma_drop_baseline &&
             reason != CAMERA_DROP_REASON_RING_FULL))) {
            camera_set_error(ctx,
                             "DDR direct: capture fault, status=0x%08x drop=%u->%u reason=%u; preserve source ownership",
                             info->status, ctx->dma_drop_baseline,
                             info->drop_count, reason);
            errno = EPIPE;
            return -1;
        }
        /* Ring-full rejects a NEW input frame; committed DDR slots stay valid.
         * Do not reset capture or change either reader's ownership. Keep
         * draining those slots, and retain DROP_COUNT for the GUI statistics. */
        if (!recovered && info->drop_count != ctx->dma_drop_baseline)
            ctx->dma_drop_baseline = info->drop_count;
    }

    if (!recovered && ((info->status & CAMERA_STATUS_ERROR_MASK) ||
        info->drop_count != ctx->dma_drop_baseline)) {
        recovered = camera_accept_hls_resync(ctx, info);
        if (recovered < 0)
            return -1;
    }

    if (!recovered && camera_check_csr_healthy(ctx, info, where) < 0)
        return -1;

    if (!recovered && camera_validate_drop_count(ctx, info, where) < 0)
        return -1;

    /* A correct bitstream publishes exactly once per physical frame. Compare
     * deltas rather than absolute values because the camera sequence and DDR
     * producer may start from different offsets. The legacy RTL advanced the
     * producer twice, yielding producer delta 2N and frame-sequence delta N.
     */
    {
        uint32_t observed_seq = info->reserved ? info->reserved : info->frame_seq;
        uint32_t producer_delta = info->producer_idx - ctx->dma_producer_baseline;
        uint32_t sequence_delta = observed_seq - ctx->dma_frame_seq_baseline;
        if (producer_delta > sequence_delta + 1u) {
            camera_set_error(ctx,
                             "%s: duplicate FPGA frame publication detected: producer delta=%u frame-sequence delta=%u build=0x%08x version=0x%08x; rebuild and reload the supplied RTL",
                             where ? where : "camera DMA", producer_delta,
                             sequence_delta, ctx->hw_build_id, ctx->hw_build_version);
            errno = EPROTO;
            return -1;
        }
    }

    /* Producer is the CSR that says a DDR slot is visible, but
     * width/height/frame_bytes are independent metadata CSRs updated by the RTL
     * on the same publish event.  During strict start userspace can observe the
     * new producer before all metadata registers are nonzero.  Do not kill the
     * DMA pump on that transient; submitted slots use the configured expected
     * geometry.  Once metadata is nonzero, strict equality checks are restored.
     */
    if (info->width && info->height && info->frame_bytes) {
        if (ctx->dma_expected_width && info->width != ctx->dma_expected_width) {
            camera_set_error(ctx, "%s: line-byte width changed from %u to %u",
                             where ? where : "camera DMA",
                             ctx->dma_expected_width, info->width);
            errno = EPROTO;
            return -1;
        }
        if (ctx->dma_expected_height && info->height != ctx->dma_expected_height) {
            camera_set_error(ctx, "%s: height changed from %u to %u",
                             where ? where : "camera DMA",
                             ctx->dma_expected_height, info->height);
            errno = EPROTO;
            return -1;
        }
        if (ctx->dma_expected_frame_bytes &&
            info->frame_bytes != ctx->dma_expected_frame_bytes) {
            camera_set_error(ctx, "%s: frame bytes changed from %u to %u",
                             where ? where : "camera DMA",
                             ctx->dma_expected_frame_bytes, info->frame_bytes);
            errno = EPROTO;
            return -1;
        }
    }

    occupancy = info->producer_idx - info->consumer_idx;
    if (occupancy > ctx->ring_slots) {
        camera_set_error(ctx,
                         "%s: DDR ring occupancy %u exceeds %u slots",
                         where ? where : "camera DMA", occupancy,
                         ctx->ring_slots);
        errno = EOVERFLOW;
        return -1;
    }
    return 0;
}

static void camera_fill_info_for_idx(struct camera_context *ctx, uint32_t idx,
                                     uint32_t producer,
                                     struct camera_frame_info *info)
{
    uint32_t slot = camera_slot_mask_or_mod(idx, ctx->ring_slots);

    memset(info, 0, sizeof(*info));
    info->producer_idx = producer;
    info->consumer_idx = idx;
    info->slot = slot;
    info->width = ctx->dma_expected_width;
    info->height = ctx->dma_expected_height;
    info->frame_bytes = ctx->dma_expected_frame_bytes;
    /* The monotonic DDR index is the stable per-slot sequence identifier. */
    info->frame_seq = idx;
    info->drop_count = camera_read32(ctx, CAMERA_REG_DROP_COUNT);
    info->status = camera_read32(ctx, CAMERA_REG_STATUS);
    info->dma_buffer_index = CAMERA_DMA_BUFFER_INVALID;
    info->reserved = camera_read32(ctx, CAMERA_REG_FRAME_SEQ);
    info->dev_addr = ctx->ring_base + (uint64_t)slot * ctx->frame_stride;
}


int camera_reset_capture(struct camera_context *ctx)
{
    if (!ctx || ctx->csr == MAP_FAILED) {
        errno = EINVAL;
        return -1;
    }

    camera_write32(ctx, CAMERA_REG_CONTROL, CAMERA_CTRL_RESET);
    camera_mmio_flush(ctx);
    usleep(1000);
    camera_write32(ctx, CAMERA_REG_CONTROL, 0);
    camera_mmio_flush(ctx);
    usleep(1000);
    ctx->last_error[0] = '\0';
    return 0;
}

int camera_clear_status(struct camera_context *ctx)
{
    uint32_t enable;

    if (!ctx || ctx->csr == MAP_FAILED) {
        errno = EINVAL;
        return -1;
    }

    enable = camera_read32(ctx, CAMERA_REG_CONTROL) &
             (CAMERA_CTRL_ENABLE | CAMERA_CTRL_OPTION_MASK);
    camera_write32(ctx, CAMERA_REG_CONTROL, enable | CAMERA_CTRL_CLEAR_STATUS);
    camera_mmio_flush(ctx);
    usleep(1000);
    camera_write32(ctx, CAMERA_REG_CONTROL, enable);
    camera_mmio_flush(ctx);
    usleep(1000);
    ctx->last_error[0] = '\0';
    return 0;
}


int camera_enable_capture(struct camera_context *ctx, uint32_t control_flags)
{
    if (!ctx || ctx->csr == MAP_FAILED) {
        errno = EINVAL;
        return -1;
    }
    camera_write32(ctx, CAMERA_REG_CONTROL,
                   CAMERA_CTRL_ENABLE | (control_flags & CAMERA_CTRL_OPTION_MASK));
    camera_mmio_flush(ctx);
    return 0;
}

int camera_set_output_mode(struct camera_context *ctx, int raw_bayer)
{
    uint32_t v;
    if (camera_read32(ctx, CAMERA_REG_CONTROL) & CAMERA_CTRL_ENABLE) {
        camera_set_error(ctx, "stop capture before changing output mode");
        errno = EBUSY; return -1;
    }
    v = camera_read32(ctx, 0x3cu);
    if ((v & 0xffff0000u) != 0x42590000u) {
        if (!raw_bayer) return 0;
        camera_set_error(ctx, "RAW Bayer requires new FPGA bitstream with output-mode CSR 0x3c; current hardware is RGB-only");
        errno = ENOTSUP; return -1;
    }
    camera_write32(ctx, 0x3cu, raw_bayer ? 1u : 0u);
    camera_mmio_flush(ctx);
    if ((camera_read32(ctx, 0x3cu) & 1u) != (raw_bayer ? 1u : 0u)) {
        camera_set_error(ctx, "output-mode readback failed"); errno=EIO; return -1;
    }
    usleep(1000);
    return 0;
}

int camera_set_capture_format(struct camera_context *ctx, uint32_t bytes_per_clock,
                              uint32_t tap_order, int tap_order_set)
{
    if (!ctx || ctx->csr == MAP_FAILED) {
        errno = EINVAL;
        return -1;
    }
    /* Build 3.16 exposes a fixed Camera Link 80-bit A..J stream. The RTL
     * trims invalid taps at each line end and repacks ten input bytes onto the
     * existing 64-bit HLS stream; mode switching is intentionally not dynamic.
     * Reject incompatible requests explicitly. */
    if (bytes_per_clock && bytes_per_clock != CAMERA_DEFAULT_BYTES_PER_CLOCK) {
        errno = EINVAL;
        camera_set_error(ctx,
                         "current FPGA image supports only 10 Camera Link bytes/clock, got %u",
                         bytes_per_clock);
        return -1;
    }
    if (tap_order_set && tap_order != CAMERA_DEFAULT_TAP_ORDER) {
        errno = EINVAL;
        camera_set_error(ctx,
                         "current FPGA image uses fixed A..J Deca mapping; legacy tap field must remain 0x%08x, got 0x%08x",
                         CAMERA_DEFAULT_TAP_ORDER, tap_order);
        return -1;
    }
    return 0;
}

int camera_capture_is_enabled(struct camera_context *ctx)
{
    if (!ctx || ctx->csr == MAP_FAILED)
        return -1;
    uint32_t control = camera_read32(ctx, CAMERA_REG_CONTROL);
    return control == UINT32_MAX ? -1 : !!(control & CAMERA_CTRL_ENABLE);
}

int camera_disable_capture(struct camera_context *ctx)
{
    uint32_t control;

    if (!ctx || ctx->csr == MAP_FAILED) {
        errno = EINVAL;
        return -1;
    }

    control = camera_read32(ctx, CAMERA_REG_CONTROL) & CAMERA_CTRL_OPTION_MASK;
    camera_write32(ctx, CAMERA_REG_CONTROL, control);
    camera_mmio_flush(ctx);
    return 0;
}

int camera_read_diag(struct camera_context *ctx, struct camera_diag_info *diag)
{
    if (!ctx || !diag || ctx->csr == MAP_FAILED) {
        errno = EINVAL;
        return -1;
    }

    diag->fval_edges      = camera_read32(ctx, CAMERA_REG_DIAG_FVAL_EDGES);
    diag->lval_edges      = camera_read32(ctx, CAMERA_REG_DIAG_LVAL_EDGES);
    diag->dval_edges      = camera_read32(ctx, CAMERA_REG_DIAG_DVAL_EDGES);
    diag->pix_valid_words = camera_read32(ctx, CAMERA_REG_DIAG_PIX_VALID_WORDS);
    diag->bad_frames      = camera_read32(ctx, CAMERA_REG_DIAG_BAD_FRAMES);
    diag->last_line_bytes = camera_read32(ctx, CAMERA_REG_DIAG_LAST_LINE_BYTES);
    diag->drop_frame_bytes = camera_read32(ctx, CAMERA_REG_DIAG_DROP_FRAME_BYTES);
    diag->drop_reason      = camera_read32(ctx, CAMERA_REG_DIAG_DROP_REASON);
    return 0;
}

int camera_read_csr_snapshot(struct camera_context *ctx, struct camera_csr_snapshot *snap)
{
    if (!ctx || !snap || ctx->csr == MAP_FAILED) {
        errno = EINVAL;
        return -1;
    }

    memset(snap, 0, sizeof(*snap));
    snap->control = camera_read32(ctx, CAMERA_REG_CONTROL);
    snap->status = camera_read32(ctx, CAMERA_REG_STATUS);
    snap->producer_idx = camera_read32(ctx, CAMERA_REG_PRODUCER_IDX);
    snap->consumer_idx = camera_read32(ctx, CAMERA_REG_CONSUMER_IDX);
    snap->ring_fill = snap->producer_idx - snap->consumer_idx;
    snap->width = camera_read32(ctx, CAMERA_REG_WIDTH);
    snap->height = camera_read32(ctx, CAMERA_REG_HEIGHT);
    snap->frame_bytes = camera_read32(ctx, CAMERA_REG_FRAME_BYTES);
    snap->frame_seq = camera_read32(ctx, CAMERA_REG_FRAME_SEQ);
    snap->drop_count = camera_read32(ctx, CAMERA_REG_DROP_COUNT);
    snap->debug_state = 0;
    snap->debug_delta = snap->ring_fill;
    snap->debug_cdc_flags = 0;
    snap->bytes_per_clock = CAMERA_DEFAULT_BYTES_PER_CLOCK;
    snap->tap_order = CAMERA_DEFAULT_TAP_ORDER;
    snap->processing_mode = (camera_read32(ctx, 0x3c) == 0x42590001u) ? CAMERA_PROCESS_RAW : CAMERA_PROCESS_BAYERRG8_RGB888;
    snap->build_id = camera_read32(ctx, CAMERA_REG_BUILD_ID);
    snap->build_version = camera_read32(ctx, CAMERA_REG_BUILD_VERSION);
    snap->duplicate_commit_count = camera_read32(ctx, CAMERA_REG_DUP_COMMIT_COUNT);
    snap->hls_stall_cycles = camera_read32(ctx, CAMERA_REG_HLS_STALL_CYCLES);
    snap->axi_stall_cycles = camera_read32(ctx, CAMERA_REG_AXI_STALL_CYCLES);
    snap->cmlk_sync_mismatch_cycles =
        camera_read32(ctx, CAMERA_REG_CMLK_SYNC_MISMATCH_CYCLES);
    snap->cmlk_sync_mismatch_events =
        camera_read32(ctx, CAMERA_REG_CMLK_SYNC_MISMATCH_EVENTS);
    snap->cmlk_sync_mismatch_mask =
        camera_read32(ctx, CAMERA_REG_CMLK_SYNC_MISMATCH_MASK);
    snap->cmlk_align_loss_count =
        camera_read32(ctx, CAMERA_REG_CMLK_ALIGN_LOSS_COUNT);
    snap->hls_resync_count = camera_read32(ctx, CAMERA_REG_HLS_RESYNC_COUNT);
    snap->isp_enable = camera_read32(ctx, CAMERA_REG_ISP_ENABLE) & 0x0fu;
    snap->isp_black_level = camera_read32(ctx, CAMERA_REG_ISP_BLACK_LEVEL) & 0xffu;
    snap->isp_wb_gain[0] = camera_read32(ctx, CAMERA_REG_ISP_WB_GAIN_R) & 0xffffu;
    snap->isp_wb_gain[1] = camera_read32(ctx, CAMERA_REG_ISP_WB_GAIN_G) & 0xffffu;
    snap->isp_wb_gain[2] = camera_read32(ctx, CAMERA_REG_ISP_WB_GAIN_B) & 0xffffu;
    snap->isp_ccm[0] = (int16_t)(camera_read32(ctx, CAMERA_REG_ISP_CCM_00) & 0xffffu);
    snap->isp_ccm[1] = (int16_t)(camera_read32(ctx, CAMERA_REG_ISP_CCM_01) & 0xffffu);
    snap->isp_ccm[2] = (int16_t)(camera_read32(ctx, CAMERA_REG_ISP_CCM_02) & 0xffffu);
    snap->isp_ccm[3] = (int16_t)(camera_read32(ctx, CAMERA_REG_ISP_CCM_10) & 0xffffu);
    snap->isp_ccm[4] = (int16_t)(camera_read32(ctx, CAMERA_REG_ISP_CCM_11) & 0xffffu);
    snap->isp_ccm[5] = (int16_t)(camera_read32(ctx, CAMERA_REG_ISP_CCM_12) & 0xffffu);
    snap->isp_ccm[6] = (int16_t)(camera_read32(ctx, CAMERA_REG_ISP_CCM_20) & 0xffffu);
    snap->isp_ccm[7] = (int16_t)(camera_read32(ctx, CAMERA_REG_ISP_CCM_21) & 0xffffu);
    snap->isp_ccm[8] = (int16_t)(camera_read32(ctx, CAMERA_REG_ISP_CCM_22) & 0xffffu);
    return camera_read_diag(ctx, &snap->diag);
}

int camera_set_expected_frame(struct camera_context *ctx, uint32_t line_bytes,
                              uint32_t frame_bytes)
{
    if (!ctx || ctx->csr == MAP_FAILED) {
        errno = EINVAL;
        return -1;
    }
    camera_write32(ctx, CAMERA_REG_EXPECTED_LINE_BYTES, line_bytes);
    camera_write32(ctx, CAMERA_REG_EXPECTED_FRAME_BYTES, frame_bytes);
    camera_mmio_flush(ctx);
    uint32_t actual_line = camera_read32(ctx, CAMERA_REG_EXPECTED_LINE_BYTES);
    uint32_t actual_frame = camera_read32(ctx, CAMERA_REG_EXPECTED_FRAME_BYTES);
    if (actual_line != line_bytes || actual_frame != frame_bytes) {
        camera_set_error(ctx, "FPGA geometry write/readback mismatch: requested line=%u frame=%u "
                         "actual line=%u frame=%u; capture must be stopped before reconfiguration",
                         line_bytes, frame_bytes, actual_line, actual_frame);
        errno = EPROTO;
        return -1;
    }
    return 0;
}

int camera_set_isp_controls(struct camera_context *ctx, uint32_t enable_mask,
                            uint32_t black_level)
{
    uint32_t control;
    uint32_t enable_readback;
    uint32_t black_readback;

    if (!ctx || ctx->csr == MAP_FAILED || (enable_mask & ~0x0fu) ||
        black_level > 255u) {
        errno = EINVAL;
        return -1;
    }

    control = camera_read32(ctx, CAMERA_REG_CONTROL);
    if (control & CAMERA_CTRL_ENABLE) {
        camera_set_error(ctx,
                         "ISP controls can only be changed while capture is disabled");
        errno = EBUSY;
        return -1;
    }

    camera_write32(ctx, CAMERA_REG_ISP_ENABLE, enable_mask);
    camera_write32(ctx, CAMERA_REG_ISP_BLACK_LEVEL, black_level);
    camera_mmio_flush(ctx);
    enable_readback = camera_read32(ctx, CAMERA_REG_ISP_ENABLE) & 0x0fu;
    black_readback = camera_read32(ctx, CAMERA_REG_ISP_BLACK_LEVEL) & 0xffu;
    if (enable_readback != enable_mask || black_readback != black_level) {
        camera_set_error(ctx,
                         "ISP control readback failed: enable=0x%x/%x black=%u/%u",
                         enable_readback, enable_mask, black_readback, black_level);
        errno = EIO;
        return -1;
    }
    return 0;
}

int camera_set_isp_config(struct camera_context *ctx,
                          const struct camera_isp_config *config)
{
    static const uint32_t profile_regs[12] = {
        CAMERA_REG_ISP_WB_GAIN_R, CAMERA_REG_ISP_WB_GAIN_G,
        CAMERA_REG_ISP_WB_GAIN_B,
        CAMERA_REG_ISP_CCM_00, CAMERA_REG_ISP_CCM_01, CAMERA_REG_ISP_CCM_02,
        CAMERA_REG_ISP_CCM_10, CAMERA_REG_ISP_CCM_11, CAMERA_REG_ISP_CCM_12,
        CAMERA_REG_ISP_CCM_20, CAMERA_REG_ISP_CCM_21, CAMERA_REG_ISP_CCM_22
    };
    uint16_t expected[12];
    uint32_t control;
    unsigned int i;

    if (!ctx || !config || ctx->csr == MAP_FAILED ||
        (config->enable_mask & ~0x0fu) || config->black_level > 255u) {
        errno = EINVAL;
        return -1;
    }
    if (ctx->hw_build_version < CAMERA_ISP_DYNAMIC_VERSION_MIN) {
        camera_set_error(ctx,
                         "dynamic WB/CCM needs FPGA build >=0x%08x, found 0x%08x",
                         CAMERA_ISP_DYNAMIC_VERSION_MIN, ctx->hw_build_version);
        errno = EPROTO;
        return -1;
    }

    control = camera_read32(ctx, CAMERA_REG_CONTROL);
    if (control & CAMERA_CTRL_ENABLE) {
        camera_set_error(ctx,
                         "ISP profile can only be changed while capture is disabled");
        errno = EBUSY;
        return -1;
    }

    expected[0] = config->wb_gain[0];
    expected[1] = config->wb_gain[1];
    expected[2] = config->wb_gain[2];
    for (i = 0; i < 9; i++)
        expected[i + 3] = (uint16_t)config->ccm[i];

    camera_write32(ctx, CAMERA_REG_ISP_ENABLE, config->enable_mask);
    camera_write32(ctx, CAMERA_REG_ISP_BLACK_LEVEL, config->black_level);
    for (i = 0; i < 12; i++)
        camera_write32(ctx, profile_regs[i], expected[i]);
    camera_mmio_flush(ctx);

    if ((camera_read32(ctx, CAMERA_REG_ISP_ENABLE) & 0x0fu) != config->enable_mask ||
        (camera_read32(ctx, CAMERA_REG_ISP_BLACK_LEVEL) & 0xffu) !=
            config->black_level) {
        camera_set_error(ctx, "ISP enable/black-level readback failed");
        errno = EIO;
        return -1;
    }
    for (i = 0; i < 12; i++) {
        uint16_t actual = (uint16_t)(camera_read32(ctx, profile_regs[i]) & 0xffffu);
        if (actual != expected[i]) {
            camera_set_error(ctx,
                             "ISP profile readback failed at CSR 0x%02x: 0x%04x/0x%04x",
                             profile_regs[i], actual, expected[i]);
            errno = EIO;
            return -1;
        }
    }
    return 0;
}

int camera_serial_set_baud_div(struct camera_context *ctx, uint32_t baud_div)
{
    if (!ctx || ctx->csr == MAP_FAILED || baud_div == 0) {
        errno = EINVAL;
        return -1;
    }
    camera_write32(ctx, CAMERA_REG_SERIAL_BAUD_DIV, baud_div);
    camera_write32(ctx, CAMERA_REG_SERIAL_CTRL, CAMERA_SERIAL_CTRL_RESET | CAMERA_SERIAL_CTRL_CLEAR);
    camera_mmio_flush(ctx);
    usleep(1000);
    return 0;
}

int camera_serial_get_baud_div(struct camera_context *ctx, uint32_t *baud_div)
{
    if (!ctx || !baud_div || ctx->csr == MAP_FAILED) {
        errno = EINVAL;
        return -1;
    }
    *baud_div = camera_read32(ctx, CAMERA_REG_SERIAL_BAUD_DIV);
    return 0;
}

uint32_t camera_serial_status(struct camera_context *ctx)
{
    if (!ctx || ctx->csr == MAP_FAILED)
        return 0xffffffffu;
    return camera_read32(ctx, CAMERA_REG_SERIAL_STATUS);
}

static int camera_serial_wait_tx_ready(struct camera_context *ctx, int timeout_ms)
{
    long long deadline;

    if (timeout_ms < 0)
        timeout_ms = CAMERA_DEFAULT_TIMEOUT_MS;
    deadline = camera_now_ms() + timeout_ms;

    for (;;) {
        uint32_t st = camera_read32(ctx, CAMERA_REG_SERIAL_STATUS);
        if (st & CAMERA_SERIAL_STATUS_TX_READY)
            return 0;
        if (timeout_ms == 0 || camera_now_ms() >= deadline) {
            camera_set_error(ctx, "timed out waiting for Camera Link serial TX ready, status=0x%08x", st);
            errno = ETIMEDOUT;
            return -1;
        }
        usleep(200);
    }
}

static int camera_response_success(const char *response);

static int camera_serial_drain_window_impl(struct camera_context *ctx, char *response,
                                           size_t response_len, int idle_ms,
                                           int total_ms, int stop_on_ack)
{
    long long deadline;
    long long idle_deadline;
    size_t used = 0;
    int saw_response = 0;

    if (total_ms < 0)
        total_ms = CAMERA_DEFAULT_TIMEOUT_MS;
    if (idle_ms < 0)
        idle_ms = total_ms;
    if (idle_ms > total_ms && total_ms > 0)
        idle_ms = total_ms;

    deadline = camera_now_ms() + total_ms;
    idle_deadline = camera_now_ms() + idle_ms;

    if (response && response_len)
        response[0] = '\0';

    for (;;) {
        long long now;
        uint32_t st = camera_read32(ctx, CAMERA_REG_SERIAL_STATUS);
        if (st & CAMERA_SERIAL_STATUS_RX_VALID) {
            uint32_t r = camera_read32(ctx, CAMERA_REG_SERIAL_RXDATA);
            if (r & CAMERA_SERIAL_RXDATA_VALID) {
                saw_response = 1;
                if (response && response_len > 1 && used + 1 < response_len) {
                    response[used++] = (char)(r & 0xffu);
                    response[used] = '\0';
                    // A complete acknowledgement ends the start transaction.
                    // Do not retain ready video slots for the 100 ms UART
                    // silence window; capability reads still use that window.
                    if (stop_on_ack && (char)(r & 0xffu) == '\n' &&
                        camera_response_success(response))
                        return (int)used;
                }
            }
            idle_deadline = camera_now_ms() + idle_ms;
            continue;
        }

        now = camera_now_ms();
        if (total_ms == 0 || now >= deadline)
            return (int)used;
        if (saw_response && now >= idle_deadline)
            return (int)used;
        usleep(1000);
    }
}

static int camera_serial_drain_window(struct camera_context *ctx, char *response,
                                      size_t response_len, int idle_ms, int total_ms)
{
    return camera_serial_drain_window_impl(ctx, response, response_len,
                                          idle_ms, total_ms, 0);
}

static int camera_serial_drain(struct camera_context *ctx, char *response,
                               size_t response_len, int timeout_ms)
{
    return camera_serial_drain_window(ctx, response, response_len,
                                      timeout_ms, timeout_ms);
}

int camera_serial_set_cc(struct camera_context *ctx, uint32_t cc_mask)
{
    if (!ctx || ctx->csr == MAP_FAILED) {
        errno = EINVAL;
        return -1;
    }
    camera_write32(ctx, CAMERA_REG_SERIAL_CC, cc_mask & 0x0fu);
    camera_mmio_flush(ctx);
    return 0;
}

int camera_serial_set_polarity(struct camera_context *ctx, uint32_t polarity)
{
    uint32_t value;

    if (!ctx || ctx->csr == MAP_FAILED) {
        errno = EINVAL;
        return -1;
    }

    value = polarity & (CAMERA_SERIAL_POLARITY_RX_INVERT |
                        CAMERA_SERIAL_POLARITY_TX_INVERT |
                        CAMERA_SERIAL_POLARITY_CC_INVERT_MASK);
    /* Polarity changes deliberately reset the small UART and RX FIFO.  This
     * recovers from a wrong RX inversion that otherwise holds the receiver in
     * a permanent break/busy state.
     */
    camera_write32(ctx, CAMERA_REG_SERIAL_POLARITY, value);
    camera_write32(ctx, CAMERA_REG_SERIAL_CTRL, CAMERA_SERIAL_CTRL_RESET | CAMERA_SERIAL_CTRL_CLEAR);
    camera_mmio_flush(ctx);
    usleep(1000);
    return 0;
}

int camera_serial_get_polarity(struct camera_context *ctx, uint32_t *polarity)
{
    if (!ctx || !polarity || ctx->csr == MAP_FAILED) {
        errno = EINVAL;
        return -1;
    }

    *polarity = camera_read32(ctx, CAMERA_REG_SERIAL_POLARITY);
    return 0;
}

static int camera_serial_send_command_impl(struct camera_context *ctx, const char *cmd,
                                          int timeout_ms, char *response,
                                          size_t response_len, int stop_on_ack)
{
    size_t i;
    size_t len;

    if (!ctx || !cmd || ctx->csr == MAP_FAILED) {
        errno = EINVAL;
        return -1;
    }

    len = strlen(cmd);
    camera_write32(ctx, CAMERA_REG_SERIAL_CTRL, CAMERA_SERIAL_CTRL_CLEAR);
    camera_mmio_flush(ctx);
    (void)camera_serial_drain(ctx, NULL, 0, 0);

    for (i = 0; i < len; i++) {
        if (camera_serial_wait_tx_ready(ctx, timeout_ms) < 0)
            return -1;
        camera_write32(ctx, CAMERA_REG_SERIAL_TXDATA, (uint8_t)cmd[i]);
        camera_mmio_flush(ctx);
    }
    if (len == 0 || cmd[len - 1] != '\r') {
        if (camera_serial_wait_tx_ready(ctx, timeout_ms) < 0)
            return -1;
        camera_write32(ctx, CAMERA_REG_SERIAL_TXDATA, (uint8_t)'\r');
        camera_mmio_flush(ctx);
    }

    return camera_serial_drain_window_impl(ctx, response, response_len,
                                          100, timeout_ms, stop_on_ack) >= 0 ? 0 : -1;
}


int camera_serial_send_command(struct camera_context *ctx, const char *cmd,
                               int timeout_ms, char *response, size_t response_len)
{
    return camera_serial_send_command_impl(ctx, cmd, timeout_ms,
                                          response, response_len, 0);
}

static int camera_response_success(const char *response)
{
    return response && strstr(response, "Success!") != NULL;
}

int camera_set_remote_acquisition_control(struct camera_context *ctx,
                                          const char *start_cmd,
                                          const char *stop_cmd,
                                          int timeout_ms)
{
    if (!ctx) {
        errno = EINVAL;
        return -1;
    }
    ctx->camera_start_cmd[0] = '\0';
    ctx->camera_stop_cmd[0] = '\0';
    if (start_cmd && *start_cmd) {
        if (strlen(start_cmd) >= sizeof(ctx->camera_start_cmd)) {
            camera_set_error(ctx, "camera start command is too long");
            errno = ENAMETOOLONG;
            return -1;
        }
        strcpy(ctx->camera_start_cmd, start_cmd);
    }
    if (stop_cmd && *stop_cmd) {
        if (strlen(stop_cmd) >= sizeof(ctx->camera_stop_cmd)) {
            camera_set_error(ctx, "camera stop command is too long");
            errno = ENAMETOOLONG;
            return -1;
        }
        strcpy(ctx->camera_stop_cmd, stop_cmd);
    }
    ctx->camera_control_timeout_ms = timeout_ms > 0 ? timeout_ms : CAMERA_DEFAULT_TIMEOUT_MS;
    return 0;
}

int camera_remote_acquisition_start(struct camera_context *ctx)
{
    char response[512];

    if (!ctx) {
        errno = EINVAL;
        return -1;
    }
    if (!ctx->camera_start_cmd[0])
        return 0;

    long long start_ms = camera_now_ms();
    if (camera_serial_send_command_impl(ctx, ctx->camera_start_cmd,
                                       ctx->camera_control_timeout_ms,
                                       response, sizeof(response), 1) < 0)
        return -1;
    if (!camera_response_success(response)) {
        camera_set_error(ctx, "camera start command failed: cmd='%s' response='%s'",
                         ctx->camera_start_cmd, response[0] ? response : "<empty>");
        errno = EIO;
        return -1;
    }
    ctx->camera_remote_started = 1;
    fprintf(stderr, "CAMERA_START ack_ms=%lld release_on_complete_ack=1\n",
            camera_now_ms() - start_ms);
    return 0;
}

int camera_remote_acquisition_stop(struct camera_context *ctx)
{
    char response[512];

    if (!ctx) {
        errno = EINVAL;
        return -1;
    }
    if (!ctx->camera_stop_cmd[0])
        return 0;

    if (camera_serial_send_command(ctx, ctx->camera_stop_cmd,
                                   ctx->camera_control_timeout_ms,
                                   response, sizeof(response)) < 0)
        return -1;
    if (!camera_response_success(response)) {
        camera_set_error(ctx, "camera stop command failed: cmd='%s' response='%s'",
                         ctx->camera_stop_cmd, response[0] ? response : "<empty>");
        errno = EIO;
        return -1;
    }
    ctx->camera_remote_started = 0;
    return 0;
}

int camera_serial_hw_init(struct camera_context *ctx, int timeout_ms,
                          char *response, size_t response_len)
{
    long long deadline;

    if (!ctx || ctx->csr == MAP_FAILED) {
        errno = EINVAL;
        return -1;
    }
    if (timeout_ms < 0)
        timeout_ms = 5000;
    deadline = camera_now_ms() + timeout_ms;

    camera_write32(ctx, CAMERA_REG_SERIAL_CTRL,
                   CAMERA_SERIAL_CTRL_CLEAR | CAMERA_SERIAL_CTRL_START_INIT);
    camera_mmio_flush(ctx);

    for (;;) {
        uint32_t st = camera_read32(ctx, CAMERA_REG_SERIAL_STATUS);
        if (!(st & CAMERA_SERIAL_STATUS_INIT_BUSY) &&
            (st & CAMERA_SERIAL_STATUS_INIT_DONE))
            break;
        if (camera_now_ms() >= deadline) {
            camera_set_error(ctx, "timed out waiting for hardware camera init, status=0x%08x", st);
            errno = ETIMEDOUT;
            return -1;
        }
        usleep(1000);
    }

    return camera_serial_drain(ctx, response, response_len, 250) >= 0 ? 0 : -1;
}

int camera_open(struct camera_context **out, const struct camera_open_opts *opts)
{
    const char *ctrl_path;
    const char *queue_path;
    struct camera_context *ctx;
    uint64_t csr_base;
    uint32_t producer;

    if (!out)
        return -1;
    *out = NULL;

    ctx = calloc(1, sizeof(*ctx));
    if (!ctx)
        return -1;

    if (pthread_mutex_init(&ctx->dma_lock, NULL) != 0) {
        free(ctx);
        errno = ENOMEM;
        return -1;
    }
    if (pthread_cond_init(&ctx->dma_cond, NULL) != 0) {
        pthread_mutex_destroy(&ctx->dma_lock);
        free(ctx);
        errno = ENOMEM;
        return -1;
    }
    ctx->dma_sync_ready = 1;

    ctx->ctrl_fd = -1;
    ctx->queue_fd = -1;
    ctx->csr = MAP_FAILED;
    ctx->csr_map_size = CAMERA_CSR_MAP_SIZE;

    ctrl_path = opts && opts->ctrl_path ? opts->ctrl_path : CAMERA_DEFAULT_CTRL_DEVICE;
    queue_path = opts && opts->queue_path ? opts->queue_path : CAMERA_DEFAULT_QUEUE_DEVICE;
    csr_base = opts && opts->csr_base ? opts->csr_base : CAMERA_DEFAULT_CSR_BASE;

    if (csr_base & (CAMERA_CSR_MAP_SIZE - 1u)) {
        camera_set_error(ctx, "CSR base 0x%llx is not 4 KiB aligned",
                         (unsigned long long)csr_base);
        goto fail;
    }

    ctx->ctrl_fd = open(ctrl_path, O_RDWR | O_CLOEXEC);
    if (ctx->ctrl_fd < 0) {
        camera_set_error(ctx, "open(%s) failed: %s", ctrl_path, strerror(errno));
        goto fail;
    }

    snprintf(ctx->queue_path, sizeof(ctx->queue_path), "%s", queue_path);

    ctx->queue_fd = open(queue_path, O_RDONLY | O_CLOEXEC);
    if (ctx->queue_fd < 0) {
        camera_set_error(ctx, "open(%s) failed: %s", queue_path, strerror(errno));
        goto fail;
    }

    ctx->csr = mmap(NULL, ctx->csr_map_size, PROT_READ | PROT_WRITE,
                    MAP_SHARED, ctx->ctrl_fd, (off_t)csr_base);
    if (ctx->csr == MAP_FAILED) {
        camera_set_error(ctx, "mmap BAR2 CSR at 0x%llx failed: %s",
                         (unsigned long long)csr_base, strerror(errno));
        goto fail;
    }

    ctx->csr_base = csr_base;

    /* Reject an unfixed data FIFO before resetting capture or programming
     * any camera state. UART-only opens do not request frame geometry. */
    if (opts && opts->expected_line_bytes) {
        uint32_t id = camera_read32(ctx, CAMERA_REG_BUILD_ID);
        uint32_t version = camera_read32(ctx, CAMERA_REG_BUILD_VERSION);
        if (id != CAMERA_RGB888_BUILD_ID || version == UINT32_MAX ||
            version < CAMERA_FIFO_SAFE_BUILD_VERSION_MIN) {
            camera_set_error(ctx,
                             "capture requires DDR FIFO integrity fix: build=0x%08x version=0x%08x; compile and load version>=0x%08x",
                             id, version, CAMERA_FIFO_SAFE_BUILD_VERSION_MIN);
            errno = EPROTO;
            goto fail;
        }
    }

    /* Start every userspace session from a quiesced capture core.  Older RTL
     * revisions ignore CAMERA_CTRL_RESET and simply leave capture disabled;
     * fixed RTL also clears sticky status, producer metadata and internal FIFOs.
     */
    if (camera_reset_capture(ctx) < 0)
        goto fail;

    /* Normalize the UART divider before any remote camera command.  This also
     * recovers systems upgraded from the old 250 MHz parameterization, which
     * produced approximately 4800 baud when driven by the 125 MHz AXI-Lite
     * clock.
     */
    if (camera_serial_configure_default_baud(ctx) < 0)
        goto fail;

    ctx->ring_base = opts ? opts->ring_base : 0;
    ctx->frame_stride = opts ? opts->frame_stride : 0;
    ctx->ring_slots = opts && opts->ring_slots ? opts->ring_slots : CAMERA_DEFAULT_RING_SLOTS;
    ctx->qid = opts && opts->qid_set ? opts->qid : CAMERA_DEFAULT_QID;
    ctx->configured_control_flags = opts ? (opts->control_flags & CAMERA_CTRL_OPTION_MASK) : 0;
    ctx->configured_bytes_per_clock = opts ? opts->bytes_per_clock : 0;
    ctx->configured_tap_order = opts ? opts->tap_order : 0;
    ctx->configured_tap_order_set = opts ? opts->tap_order_set : 0;
    ctx->configured_processing_mode = (opts && opts->processing_mode) ? opts->processing_mode : CAMERA_PROCESS_RAW;
    ctx->configured_dma_queue_count = opts ? opts->dma_queue_count : 0;
    ctx->configured_dma_frame_irq = opts ? opts->dma_frame_irq : -1;
    ctx->configured_driver_frame_dma = (opts && opts->driver_frame_irq_dma) ? 1 : camera_dma_env_u32("CAMERA_DRIVER_FRAME_IRQ_DMA", 0, 0, 1);
    ctx->dma_driver_frame_mode = ctx->configured_driver_frame_dma;
    ctx->configured_expected_line_bytes = opts ? opts->expected_line_bytes : 0;
    ctx->configured_expected_height = opts ? opts->expected_height : 0;
    ctx->configured_expected_frame_bytes = opts ? opts->expected_frame_bytes : 0;

    ctx->hw_build_id = camera_read32(ctx, CAMERA_REG_BUILD_ID);
    ctx->hw_build_version = camera_read32(ctx, CAMERA_REG_BUILD_VERSION);
    const char *host_path=getenv("CAMERA_HOST_PATH");
    if(host_path && strcmp(host_path,"direct") && strcmp(host_path,"ddr") && strcmp(host_path,"auto")) {
        camera_set_error(ctx,"CAMERA_HOST_PATH must be auto, direct or ddr");
        errno=EINVAL;goto fail;
    }
    const char *direct_ring_path=getenv("CAMERA_DDR_RING");
    int want_direct=(host_path && !strcmp(host_path,"direct")) ||
        ((!host_path || !strcmp(host_path,"auto")) && direct_ring_path && *direct_ring_path);
    ctx->host_direct=want_direct && ctx->hw_build_id==CAMERA_RGB888_BUILD_ID &&
        ctx->hw_build_version>=VIDEO_HOST_BUILD_MIN && ctx->hw_build_version!=UINT32_MAX;
    /* A local preview can use the ISP->PCIe writer without an RDMA sender. */
    if(want_direct && !ctx->host_direct) {
        camera_set_error(ctx,"ISP direct-host path requires FPGA build >=0x%08x",VIDEO_HOST_BUILD_MIN);
        errno=EPROTO;goto fail;
    }
    if(ctx->host_direct && ctx->configured_driver_frame_dma) {
        camera_set_error(ctx,"direct-host path owns completion itself; CAMERA_DRIVER_FRAME_IRQ_DMA must be 0");
        errno=EINVAL;goto fail;
    }
    if (ctx->configured_processing_mode == CAMERA_PROCESS_BAYERRG8_RGB888 &&
        (ctx->hw_build_id != CAMERA_RGB888_BUILD_ID ||
         ctx->hw_build_version < CAMERA_RGB888_BUILD_VERSION_MIN)) {
        camera_set_error(ctx,
                         "incompatible/stale FPGA camera bitstream: build_id=0x%08x version=0x%08x, expected compact-RGB888 id=0x%08x version>=0x%08x",
                         ctx->hw_build_id, ctx->hw_build_version,
                         CAMERA_RGB888_BUILD_ID, CAMERA_RGB888_BUILD_VERSION_MIN);
        errno = EPROTO;
        goto fail;
    }

    /* Reject the known burst-overflow configuration before allocating/arming
     * host DMA, including launches that bypass the GUI/controller preflight. */
    if (ctx->configured_processing_mode == CAMERA_PROCESS_BAYERRG8_RGB888 &&
        ctx->configured_expected_line_bytes >= 2400 &&
        ctx->configured_expected_height >= 2048 &&
        (ctx->hw_build_version < CAMERA_FULL_FRAME_BURST_BUILD_MIN ||
         ctx->hw_build_version == UINT32_MAX)) {
        camera_set_error(ctx,
                         "full-frame RGB burst buffering requires FPGA build >=0x%08x, current=0x%08x; use 1920x1080 until the updated bitstream is loaded (DMA not armed)",
                         CAMERA_FULL_FRAME_BURST_BUILD_MIN, ctx->hw_build_version);
        errno = EPROTO;
        goto fail;
    }

    if (ctx->ring_slots != CAMERA_DEFAULT_RING_SLOTS) {
        camera_set_error(ctx, "camera ring slots must be %u, got %u",
                         CAMERA_DEFAULT_RING_SLOTS, ctx->ring_slots);
        errno = EINVAL;
        goto fail;
    }

    if (ctx->ring_base > UINT32_MAX) {
        camera_set_error(ctx, "ring base 0x%llx exceeds 32-bit DDR address space",
                         (unsigned long long)ctx->ring_base);
        errno = EINVAL;
        goto fail;
    }

    if (opts && opts->ring_base_set) {
        /* DDR address 0 is a valid frame-ring base on systems that map the
         * image buffer at the beginning of the FPGA DDR aperture.  Do not use
         * ring_base==0 as a sentinel. */
        camera_write32(ctx, CAMERA_REG_RING_BASE_LO,
                       (uint32_t)(ctx->ring_base & 0xffffffffu));
        camera_write32(ctx, CAMERA_REG_RING_BASE_HI, 0);
    } else {
        ctx->ring_base = ((uint64_t)camera_read32(ctx, CAMERA_REG_RING_BASE_HI) << 32) |
                         camera_read32(ctx, CAMERA_REG_RING_BASE_LO);
    }

    if ((opts && opts->frame_stride_set) || ctx->frame_stride)
        camera_write32(ctx, CAMERA_REG_FRAME_STRIDE, ctx->frame_stride);
    else
        ctx->frame_stride = camera_read32(ctx, CAMERA_REG_FRAME_STRIDE);

    camera_write32(ctx, CAMERA_REG_RING_SLOTS, ctx->ring_slots);
    camera_write32(ctx, CAMERA_REG_IRQ_ENABLE, opts ? opts->irq_enable : 0);

    producer = camera_read32(ctx, CAMERA_REG_PRODUCER_IDX);
    camera_write32(ctx, CAMERA_REG_CONSUMER_IDX, producer);

    if (!ctx->frame_stride) {
        uint32_t frame_bytes = camera_read32(ctx, CAMERA_REG_FRAME_BYTES);
        ctx->frame_stride = frame_bytes;
    }

    if (opts && (opts->expected_line_bytes || opts->expected_frame_bytes)) {
        camera_write32(ctx, CAMERA_REG_EXPECTED_HEIGHT, opts->expected_height);
        (void)camera_set_expected_frame(ctx, opts->expected_line_bytes,
                                        opts->expected_frame_bytes);
    } else {
        camera_write32(ctx, CAMERA_REG_EXPECTED_HEIGHT, 0);
        (void)camera_set_expected_frame(ctx, 0, 0);
    }

    if (opts && (opts->bytes_per_clock || opts->tap_order_set)) {
        if (camera_set_capture_format(ctx, opts->bytes_per_clock,
                                      opts->tap_order, opts->tap_order_set) < 0)
            goto fail;
    }
    if (ctx->configured_processing_mode > CAMERA_PROCESS_MAX) {
        camera_set_error(ctx, "invalid processing mode %u", ctx->configured_processing_mode);
        errno = EINVAL;
        goto fail;
    }

    if (camera_clear_status(ctx) < 0)
        goto fail;

    if (opts && opts->enable_capture) {
        uint32_t control = CAMERA_CTRL_ENABLE |
                           (opts->control_flags & CAMERA_CTRL_OPTION_MASK);
        /* Give the AXI-domain configuration CDC a quiet window before asserting
         * capture enable.  The RTL also samples enable and config coherently; this
         * delay keeps older bitstreams from seeing a transient zero ring_base/stride.
         */
        usleep(1000);
        camera_write32(ctx, CAMERA_REG_CONTROL, control);
        camera_mmio_flush(ctx);
    }

    *out = ctx;
    return 0;

fail:
    if (ctx->csr != MAP_FAILED)
        munmap((void *)ctx->csr, ctx->csr_map_size);
    if (ctx->queue_fd >= 0)
        close(ctx->queue_fd);
    if (ctx->ctrl_fd >= 0)
        close(ctx->ctrl_fd);
    errno = errno ? errno : EINVAL;
    fprintf(stderr, "%s\n", ctx->last_error[0] ? ctx->last_error : "camera_open failed");
    if (ctx->dma_sync_ready) {
        pthread_cond_destroy(&ctx->dma_cond);
        pthread_mutex_destroy(&ctx->dma_lock);
    }
    free(ctx);
    return -1;
}

void camera_close(struct camera_context *ctx)
{
    if (!ctx)
        return;

    camera_dma_stop_pump(ctx);

    if (ctx->csr != MAP_FAILED && ctx->camera_stop_cmd[0])
        (void)camera_remote_acquisition_stop(ctx);

    if (ctx->csr != MAP_FAILED)
        (void)camera_disable_capture(ctx);

    /* Unregister/pin-release the userspace DMA buffers while the queue file is
     * still open.  The driver also cleans up on file release, but doing it here
     * gives deterministic error handling and avoids leaving duplicate buffer
     * registrations on long-running test loops.
     */
    camera_dma_close_pool(ctx);
    if (ctx->ddr) {
        atomic_store_explicit(&ctx->ddr->owner_stop, 1, memory_order_release);
        munmap(ctx->ddr, sizeof(*ctx->ddr));
        close(ctx->ddr_fd);
        ctx->ddr = NULL;
    }
    if (ctx->queue_fd >= 0) {
        close(ctx->queue_fd);
        ctx->queue_fd = -1;
    }

    if (ctx->csr != MAP_FAILED)
        munmap((void *)ctx->csr, ctx->csr_map_size);
    if (ctx->ctrl_fd >= 0)
        close(ctx->ctrl_fd);
    if (ctx->dma_sync_ready) {
        pthread_cond_destroy(&ctx->dma_cond);
        pthread_mutex_destroy(&ctx->dma_lock);
    }
    free(ctx);
}

int camera_refresh_info(struct camera_context *ctx, struct camera_frame_info *info)
{
    uint32_t producer_before = 0;
    uint32_t producer_after = 0;
    uint32_t consumer = 0;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t frame_bytes = 0;
    uint32_t frame_seq = 0;
    uint32_t drop_count = 0;
    uint32_t status = 0;
    uint32_t slot;
    unsigned int attempt;

    if (!ctx || !info) {
        errno = EINVAL;
        return -1;
    }

    /* producer_idx is the publication fence.  Read it on both sides of the
     * metadata registers so userspace never combines a newly published index
     * with metadata from the preceding frame during an AXI/AXI-Lite CDC update.
     */
    for (attempt = 0; attempt < 4; attempt++) {
        producer_before = camera_read32(ctx, CAMERA_REG_PRODUCER_IDX);
        consumer = camera_read32(ctx, CAMERA_REG_CONSUMER_IDX);
        width = camera_read32(ctx, CAMERA_REG_WIDTH);
        height = camera_read32(ctx, CAMERA_REG_HEIGHT);
        frame_bytes = camera_read32(ctx, CAMERA_REG_FRAME_BYTES);
        frame_seq = camera_read32(ctx, CAMERA_REG_FRAME_SEQ);
        drop_count = camera_read32(ctx, CAMERA_REG_DROP_COUNT);
        status = camera_read32(ctx, CAMERA_REG_STATUS);
        producer_after = camera_read32(ctx, CAMERA_REG_PRODUCER_IDX);
        if (producer_before == producer_after)
            break;
    }
    if (producer_before != producer_after) {
        camera_set_error(ctx,
                         "camera CSR publication changed during snapshot: before=%u after=%u",
                         producer_before, producer_after);
        errno = EAGAIN;
        return -1;
    }

    slot = camera_slot_mask_or_mod(consumer, ctx->ring_slots);
    memset(info, 0, sizeof(*info));
    info->producer_idx = producer_after;
    info->consumer_idx = consumer;
    info->slot = slot;
    info->width = width;
    info->height = height;
    info->frame_bytes = frame_bytes;
    info->frame_seq = frame_seq;
    info->drop_count = drop_count;
    info->status = status;
    info->dma_buffer_index = CAMERA_DMA_BUFFER_INVALID;
    info->reserved = 0;
    info->dev_addr = ctx->ring_base + (uint64_t)slot * ctx->frame_stride;
    return 0;
}


int camera_wait_ready(struct camera_context *ctx, size_t expected_frame_size,
                      int timeout_ms, struct camera_frame_info *info)
{
    long long deadline;

    if (!ctx || !info) {
        errno = EINVAL;
        return -1;
    }

    if (timeout_ms < 0)
        timeout_ms = CAMERA_DEFAULT_TIMEOUT_MS;
    deadline = camera_now_ms() + timeout_ms;

    for (;;) {
        if (camera_refresh_info(ctx, info) < 0)
            return -1;

        if (camera_check_csr_healthy(ctx, info, "camera CSR error before DMA") < 0)
            return -1;

        if (info->producer_idx != info->consumer_idx &&
            info->width != 0 && info->height != 0 && info->frame_bytes != 0)
            return camera_validate_frame_info(ctx, info, expected_frame_size,
                                              "camera ready check");

        if (timeout_ms == 0 || camera_now_ms() >= deadline) {
            camera_set_error(ctx,
                             "camera did not produce a valid frame before DMA: "
                             "producer=%u consumer=%u width=%u height=%u frame_bytes=%u "
                             "seq=%u drop=%u status=0x%08x%s",
                             info->producer_idx, info->consumer_idx,
                             info->width, info->height, info->frame_bytes,
                             info->frame_seq, info->drop_count, info->status,
                             camera_status_bits(info->status, (char[128]){0}, 128));
            errno = ETIMEDOUT;
            return -1;
        }

        usleep(1000);
    }
}

static int camera_program_static_capture_config(struct camera_context *ctx)
{
    if (!ctx || ctx->csr == MAP_FAILED) {
        errno = EINVAL;
        return -1;
    }

    /* Always program the base, including 0.  A zero DDR base is valid on
     * designs whose frame-ring aperture starts at address 0. */
    camera_write32(ctx, CAMERA_REG_RING_BASE_LO,
                   (uint32_t)(ctx->ring_base & 0xffffffffu));
    camera_write32(ctx, CAMERA_REG_RING_BASE_HI, 0);
    if (ctx->frame_stride)
        camera_write32(ctx, CAMERA_REG_FRAME_STRIDE, ctx->frame_stride);
    camera_write32(ctx, CAMERA_REG_RING_SLOTS, ctx->ring_slots);
    camera_write32(ctx, CAMERA_REG_EXPECTED_HEIGHT,
                   ctx->configured_expected_height);
    /* Enable capture-core frame-ready IRQ.  The SGDMA driver turns this into a
     * blocking EDEV_IOC_CAMERA_WAIT_FRAME wakeup, so userspace workers do not
     * need to spin on producer_idx. */
    camera_write32(ctx, CAMERA_REG_IRQ_ENABLE, 1u);

    if (ctx->dma_strided) {
        /* In the BayerRG8-to-RGB888 architecture these registers are not merely
         * diagnostic equality guards.  Updated RTL uses them as the active
         * row/frame clip geometry: copy only expected_line_bytes from each
         * LVAL row and publish exactly expected_frame_bytes per frame.
         * This removes Camera Link row padding before the DDR ring while the
         * SGDMA2D path handles NvBufSurface destination pitch separately.
         */
        if (camera_set_expected_frame(ctx, ctx->configured_expected_line_bytes,
                                      ctx->configured_expected_frame_bytes) < 0)
            return -1;
    } else if (ctx->configured_expected_line_bytes ||
               ctx->configured_expected_frame_bytes) {
        if (camera_set_expected_frame(ctx,
                                      ctx->configured_expected_line_bytes,
                                      ctx->configured_expected_frame_bytes) < 0)
            return -1;
    }

    if (ctx->configured_bytes_per_clock ||
        ctx->configured_tap_order_set) {
        if (camera_set_capture_format(ctx, ctx->configured_bytes_per_clock,
                                      ctx->configured_tap_order,
                                      ctx->configured_tap_order_set) < 0)
            return -1;
    }
    if (ctx->configured_processing_mode > CAMERA_PROCESS_MAX) {
        camera_set_error(ctx, "invalid processing mode %u", ctx->configured_processing_mode);
        errno = EINVAL;
        return -1;
    }

    camera_mmio_flush(ctx);
    return 0;
}

/* Program coherent host ownership while capture is disabled. Both strict-start
 * entry points must arm the same writer before any camera pixel is released. */
static int camera_direct_arm(struct camera_context *ctx, uint32_t producer)
{
    if (!ctx->host_direct)
        return 0;
    if (!ctx->dma_buffers || !ctx->dma_buffer_count || ctx->dma_buffer_count > 32 ||
        (ctx->dma_buffer_count & (ctx->dma_buffer_count - 1))) {
        camera_set_error(ctx, "direct-host arm requires allocated power-of-two DMA buffers");
        errno = EINVAL;
        return -1;
    }
    if(ctx->host_direct) {
        ctx->direct_cursor=ctx->direct_release_cursor=0;
        ctx->direct_ddr_publish=producer;
        ctx->dma_next_submit_idx=ctx->dma_next_ddr_release_idx=0;
        camera_write32(ctx,VIDEO_HOST_CONTROL,0);
        camera_write32(ctx,VIDEO_HOST_SLOTS,ctx->dma_buffer_count);
        camera_write32(ctx,VIDEO_HOST_CONSUMER,0);
        for(unsigned int j=0;j<ctx->dma_buffer_count;j++) {
            struct video_host_footer *footer=(struct video_host_footer *)((uint8_t*)ctx->dma_buffers[j].ptr+camera_round_up(ctx->dma_transfer_size,64));
            __atomic_store_n(&footer->token,0,__ATOMIC_RELEASE);
            ctx->dma_buffers[j].direct_release_token=0;
            uint64_t addr=ctx->dma_buffers[j].coherent_iova;
            camera_write32(ctx,VIDEO_HOST_ADDRESSES+j*8,(uint32_t)addr);
            camera_write32(ctx,VIDEO_HOST_ADDRESSES+j*8+4,(uint32_t)(addr>>32));
        }
        camera_mmio_flush(ctx);
        usleep(1000);
        camera_write32(ctx,VIDEO_HOST_CONTROL,1);
        camera_mmio_flush(ctx);
        ctx->host_armed=1;
        fprintf(stderr,"HOST_DIRECT enabled=1 buffers=%u source=ISP no_DDR_read=1 build=0x%08x\n",
            ctx->dma_buffer_count,ctx->hw_build_version);
    }
    if (!(camera_read32(ctx, VIDEO_HOST_CONTROL) & 1U) ||
        camera_read32(ctx, VIDEO_HOST_SLOTS) != ctx->dma_buffer_count) {
        camera_set_error(ctx, "direct-host arm CSR readback failed before camera start");
        errno = EIO;
        return -1;
    }
    return 0;
}

int camera_start_strict(struct camera_context *ctx, uint32_t control_flags,
                        uint32_t expected_line_bytes, uint32_t expected_height,
                        uint32_t expected_frame_bytes, int timeout_ms,
                        struct camera_frame_info *info)
{
    struct camera_frame_info ready_info;
    uint32_t producer;

    if (!ctx || ctx->csr == MAP_FAILED) {
        errno = EINVAL;
        return -1;
    }
    if (!expected_line_bytes || !expected_height || !expected_frame_bytes) {
        camera_set_error(ctx,
                         "strict start requires explicit line bytes, height and frame bytes");
        errno = EINVAL;
        return -1;
    }
    if (ctx->frame_stride && ctx->frame_stride < expected_frame_bytes) {
        camera_set_error(ctx,
                         "frame stride %u is smaller than expected frame size %u",
                         ctx->frame_stride, expected_frame_bytes);
        errno = EMSGSIZE;
        return -1;
    }
    if (!camera_dma_pool_is_idle(ctx))
        return -1;

    ctx->configured_control_flags = control_flags & CAMERA_CTRL_OPTION_MASK;
    ctx->configured_expected_line_bytes = expected_line_bytes;
    ctx->configured_expected_height = expected_height;
    ctx->configured_expected_frame_bytes = expected_frame_bytes;

    /* The display program does all heavy allocation before this point.  This
     * reset marks the start of strict accounting: frames produced while the
     * host was still allocating X11/MIT-SHM/DMA resources are deliberately not
     * part of the delivered sequence. */
    if (camera_reset_capture(ctx) < 0)
        return -1;
    if (camera_program_static_capture_config(ctx) < 0)
        return -1;
    if (camera_clear_status(ctx) < 0)
        return -1;

    producer = camera_read32(ctx, CAMERA_REG_PRODUCER_IDX);
    camera_write32(ctx, CAMERA_REG_CONSUMER_IDX, producer);
    camera_mmio_flush(ctx);
    camera_dma_reset_runtime_indices(ctx, producer, 0u, expected_line_bytes,
                                     expected_height, expected_frame_bytes);

    /* Give AXI-domain CDC copies of ring/stride/format/processing a quiet window before ENABLE. */
    if (camera_direct_arm(ctx, producer) < 0)
        return -1;
    usleep(1000);
    if (camera_enable_capture(ctx, control_flags & CAMERA_CTRL_OPTION_MASK) < 0)
        return -1;

    /* If a remote camera-start command is configured, the acquisition device is
     * still parked at this point.  Start it only after DDR ring, DMA buffers,
     * X11 surfaces and FPGA capture are ready.  This prevents enabling the FPGA
     * in the middle of an already-running frame and avoids startup FIFO bursts.
     */
    if (camera_remote_acquisition_start(ctx) < 0) {
        (void)camera_disable_capture(ctx);
        return -1;
    }
    usleep(2000);

    if (camera_wait_ready(ctx, expected_frame_bytes, timeout_ms, &ready_info) < 0) {
        (void)camera_remote_acquisition_stop(ctx);
        (void)camera_disable_capture(ctx);
        return -1;
    }

    if (ready_info.drop_count != 0u ||
        ready_info.width != expected_line_bytes ||
        ready_info.height != expected_height ||
        ready_info.frame_bytes != expected_frame_bytes) {
        camera_set_error(ctx,
                         "strict start metadata mismatch: expected line=%u height=%u frame=%u drop=0; "
                         "got line=%u height=%u frame=%u drop=%u",
                         expected_line_bytes, expected_height, expected_frame_bytes,
                         ready_info.width, ready_info.height,
                         ready_info.frame_bytes, ready_info.drop_count);
        errno = EPROTO;
        return -1;
    }

    camera_dma_reset_runtime_indices(ctx, ready_info.consumer_idx,
                                     ready_info.drop_count, expected_line_bytes,
                                     expected_height, expected_frame_bytes);
    if (info)
        *info = ready_info;
    return 0;
}

int camera_wait_frame(struct camera_context *ctx, int timeout_ms,
                      struct camera_frame_info *info)
{
    long long deadline;

    if (!ctx || !info) {
        errno = EINVAL;
        return -1;
    }

    if (timeout_ms < 0)
        timeout_ms = CAMERA_DEFAULT_TIMEOUT_MS;
    deadline = camera_now_ms() + timeout_ms;

    for (;;) {
        if (camera_refresh_info(ctx, info) < 0)
            return -1;

        if (camera_check_csr_healthy(ctx, info,
                                     "camera CSR error while waiting for frame") < 0)
            return -1;

        if (info->producer_idx != info->consumer_idx)
            return camera_validate_frame_info(ctx, info, 0,
                                              "camera frame wait");

        if (timeout_ms == 0 || camera_now_ms() >= deadline) {
            camera_set_error(ctx,
                             "timed out waiting for camera frame: producer=%u consumer=%u "
                             "width=%u height=%u frame_bytes=%u seq=%u drop=%u status=0x%08x%s",
                             info->producer_idx, info->consumer_idx,
                             info->width, info->height,
                             info->frame_bytes, info->frame_seq,
                             info->drop_count, info->status,
                             camera_status_bits(info->status, (char[128]){0}, 128));
            errno = ETIMEDOUT;
            return -1;
        }
        usleep(1000);
    }
}

ssize_t camera_read_frame(struct camera_context *ctx,
                          const struct camera_frame_info *info,
                          void *buf, size_t len)
{
    uint8_t *p = buf;
    size_t done = 0;

    if (!ctx || !info || !buf) {
        errno = EINVAL;
        return -1;
    }
    if (!ctx->frame_stride) {
        camera_set_error(ctx, "ring stride is not configured");
        errno = EINVAL;
        return -1;
    }
    if (camera_validate_frame_info(ctx, info, len, "camera read") < 0)
        return -1;

    while (done < len) {
        ssize_t ret = pread(ctx->queue_fd, p + done, len - done,
                            (off_t)(info->dev_addr + done));
        if (ret < 0) {
            if (errno == EINTR)
                continue;
            camera_set_error(ctx, "pread frame at 0x%llx failed after %zu/%zu bytes: %s",
                             (unsigned long long)(info->dev_addr + done),
                             done, len, strerror(errno));
            return -1;
        }
        if (ret == 0) {
            camera_set_error(ctx, "pread frame returned EOF after %zu/%zu bytes",
                             done, len);
            errno = EIO;
            return -1;
        }
        done += (size_t)ret;
    }

    return (ssize_t)done;
}

int camera_release_frame(struct camera_context *ctx,
                         const struct camera_frame_info *info)
{
    if (!ctx || !info) {
        errno = EINVAL;
        return -1;
    }
    camera_write32(ctx, CAMERA_REG_CONSUMER_IDX, info->consumer_idx + 1u);
    camera_mmio_flush(ctx);
    return 0;
}


static unsigned int camera_dma_env_u32(const char *name, unsigned int fallback,
                                       unsigned int min_v, unsigned int max_v)
{
    const char *s = getenv(name);
    char *end = NULL;
    unsigned long v;

    if (!s || !*s)
        return fallback;
    errno = 0;
    v = strtoul(s, &end, 0);
    if (errno || end == s || *end != '\0')
        return fallback;
    if (v < min_v)
        v = min_v;
    if (v > max_v)
        v = max_v;
    return (unsigned int)v;
}

static __attribute__((unused)) int camera_parse_dmabuf_fd_env(int *fds, unsigned int max_fds)
{
    const char *env = getenv("CAMERA_DMA_DMABUF_FDS");
    const char *p;
    unsigned int n = 0;

    if (!env || !*env)
        return 0;
    p = env;
    while (*p && n < max_fds) {
        char *end = NULL;
        long v;
        errno = 0;
        v = strtol(p, &end, 0);
        if (errno || end == p || v < 0 || v > INT_MAX)
            return -1;
        fds[n++] = (int)v;
        p = end;
        if (*p == ',')
            p++;
        else if (*p != '\0')
            return -1;
    }
    return (int)n;
}

static int camera_wait_frame_event(struct camera_context *ctx, int timeout_ms)
{
#ifdef EDEV_IOC_CAMERA_WAIT_FRAME
    struct edev_ioc_camera_wait wait_arg;

    if (!ctx || ctx->ctrl_fd < 0) {
        errno = EINVAL;
        return -1;
    }
    memset(&wait_arg, 0, sizeof(wait_arg));
    wait_arg.csr_base = ctx->csr_base;
    wait_arg.timeout_ms = timeout_ms > 0 ? (uint32_t)timeout_ms : 1000u;
    if (ioctl(ctx->ctrl_fd, EDEV_IOC_CAMERA_WAIT_FRAME, &wait_arg) == 0)
        return 0;

    /* The Orin RGB8 path must not depend on a frame-ready MSI-X wakeup for
     * correctness.  Some platform/bitstream combinations expose producer_idx
     * correctly but do not wake the kernel wait queue for every capture frame.
     * Treat a wait timeout as "no frame yet" and let the userspace DMA pump
     * re-read producer_idx after a short sleep.  Real camera/capture errors are
     * still fatal below.
     */
    if ((errno == ETIMEDOUT || wait_arg.result == -ETIMEDOUT) &&
        !(wait_arg.status & CAMERA_STATUS_ERROR_MASK))
        return 0;

    /* Compatibility with an older kernel module/RTL pair: the kernel wait ioctl
     * may report -EPIPE solely because the diagnostic FIFO_OVERFLOW bit is
     * sticky.  If no fatal bits remain after applying the userspace fatal mask,
     * let the DMA worker re-read producer_idx and continue.
     */
    if ((errno == EPIPE || wait_arg.result == -EPIPE) &&
        !(wait_arg.status & CAMERA_STATUS_ERROR_MASK))
        return 0;

    if ((errno == EPIPE || wait_arg.result == -EPIPE) &&
        ctx->hw_build_version >= 0x00030009u) {
        if (ctx->ddr &&
            ctx->hw_build_version >= CAMERA_ISP_DYNAMIC_VERSION_MIN &&
            camera_read32(ctx, CAMERA_REG_DIAG_DROP_REASON) ==
                CAMERA_DROP_REASON_CMLK_SYNC_MISMATCH &&
            !(wait_arg.status & CAMERA_STATUS_ERROR_MASK &
              ~CAMERA_STATUS_FRAME_SIZE_ERROR)) {
            /* The pump owns dma_lock in its claim path. Wake it to account
             * for this rejected frame before any other lane sees the drop. */
            return 0;
        }
        struct camera_frame_info recovered_info;
        int recovered;

        memset(&recovered_info, 0, sizeof(recovered_info));
        recovered_info.producer_idx = wait_arg.producer_idx;
        recovered_info.consumer_idx = wait_arg.consumer_idx;
        recovered_info.width = wait_arg.width;
        recovered_info.height = wait_arg.height;
        recovered_info.frame_bytes = wait_arg.frame_bytes;
        recovered_info.drop_count = wait_arg.drop_count;
        recovered_info.status = wait_arg.status;
        recovered = camera_accept_hls_resync(ctx, &recovered_info);
        if (recovered > 0)
            return 0;
        if (recovered < 0)
            return -1;
    }

    if (errno != ENOTTY && errno != ENOSYS && errno != EINVAL) {
        struct camera_diag_info diag;
        char status_desc[128];
        memset(&diag, 0, sizeof(diag));
        (void)camera_read_diag(ctx, &diag);
        /* Do not hide FRAME_SIZE_ERROR in the fixed RGB8 path.  It means the
         * RTL geometry and host geometry are inconsistent and must be reported
         * immediately instead of causing an apparent black-screen hang.
         */
        camera_set_error(ctx,
                         "camera frame-ready wait reported capture error: result=%d producer=%u consumer=%u "
                         "status=0x%08x%s width=%u height=%u frame=%u drop=%u "
                         "diag: last_line=%u drop_reason=%u drop_bytes=%u bad_frames=%u "
                         "fval_edges=%u lval_edges=%u dval_edges=%u pix_words=%u",
                         wait_arg.result, wait_arg.producer_idx, wait_arg.consumer_idx,
                         wait_arg.status, camera_status_bits(wait_arg.status, status_desc, sizeof(status_desc)),
                         wait_arg.width, wait_arg.height, wait_arg.frame_bytes, wait_arg.drop_count,
                         diag.last_line_bytes, diag.drop_reason, diag.drop_frame_bytes, diag.bad_frames,
                         diag.fval_edges, diag.lval_edges, diag.dval_edges, diag.pix_valid_words);
        return -1;
    }
#endif
    /* Compatibility fallback for older drivers: keep the sleep long enough to
     * avoid aggressive MMIO polling, but short enough to preserve latency.
     */
    usleep(1000);
    return 0;
}

static void camera_ddr_release_locked(struct camera_context *ctx)
{
    if (!ctx->ddr)
        return;
    uint32_t next = ddr_release_ready(ctx->ddr, ctx->ddr_release_idx);
    if (next != ctx->ddr_release_idx) {
        camera_write32(ctx, CAMERA_REG_CONSUMER_IDX, next);
        camera_mmio_flush(ctx);
        ctx->ddr_release_idx = next;
    }
}

static void camera_direct_reclaim_locked(struct camera_context *ctx);

static void camera_dma_flush_completed_locked(struct camera_context *ctx)
{
    int progress = 1;

    while (progress) {
        unsigned int i;
        progress = 0;
        for (i = 0; i < ctx->dma_buffer_count; i++) {
            struct camera_dma_buffer *buf = &ctx->dma_buffers[i];
            if (buf->state != CAMERA_DMA_DONE ||
                buf->info.consumer_idx != ctx->dma_next_ddr_release_idx)
                continue;

            if(ctx->host_direct && !buf->len) {
                buf->state=CAMERA_DMA_FREE;
                buf->direct_release_token=(uint64_t)buf->info.consumer_idx+1;
                ctx->dma_next_ddr_release_idx++;
                camera_direct_reclaim_locked(ctx);
                progress=1;break;
            }
            buf->state = CAMERA_DMA_READY;
            if (camera_ready_push(ctx, i) < 0) {
                ctx->dma_thread_error = 1;
                ctx->dma_thread_stop = 1;
                return;
            }
            ctx->dma_next_ddr_release_idx++;
            if (!ctx->configured_driver_frame_dma && !ctx->ddr) {
                camera_write32(ctx, CAMERA_REG_CONSUMER_IDX,
                               ctx->dma_next_ddr_release_idx);
                camera_mmio_flush(ctx);
            }
            progress = 1;
            break;
        }
    }
}

static int camera_dma_find_free_buffer_locked(struct camera_context *ctx)
{
    unsigned int i;
    for (i = 0; i < ctx->dma_buffer_count; i++) {
        if (ctx->dma_buffers[i].state == CAMERA_DMA_FREE)
            return (int)i;
    }
    return -1;
}

static int camera_dma_claim_work_locked(struct camera_context *ctx, int *buf_index_out,
                                        struct camera_frame_info *info_out)
{
    struct camera_frame_info current_info;
    uint32_t producer;
    int buf_index;
    size_t len = ctx->dma_transfer_size;

    camera_ddr_release_locked(ctx);
    if (ctx->ddr && atomic_load_explicit(&ctx->ddr->sender_state, memory_order_acquire) != DDR_SENDER_ACTIVE) {
        pthread_cond_broadcast(&ctx->dma_cond);
        errno = EAGAIN;
        return 0;
    }
    if (camera_refresh_info(ctx, &current_info) < 0)
        return -1;

    producer = current_info.producer_idx;
    if (producer == ctx->dma_next_submit_idx) {
        uint32_t occupancy;
        if (ctx->dma_strided && !ctx->dma_src_stride_learned &&
            ((current_info.status & CAMERA_STATUS_ERROR_MASK) ||
             current_info.drop_count != ctx->dma_drop_baseline)) {
            camera_write32(ctx, CAMERA_REG_CONSUMER_IDX, producer);
            camera_mmio_flush(ctx);
            if (camera_clear_status(ctx) < 0)
                return -1;
            ctx->dma_drop_baseline =
                camera_read32(ctx, CAMERA_REG_DROP_COUNT);
            ctx->dma_hls_resync_baseline =
                camera_read32(ctx, CAMERA_REG_HLS_RESYNC_COUNT);
            errno = EAGAIN;
            return 0;
        }
        if (camera_validate_dma_stream(ctx, &current_info,
                                       "camera idle before DMA submit") < 0)
            return -1;
        occupancy = current_info.producer_idx - current_info.consumer_idx;
        if (occupancy > ctx->ring_slots) {
            camera_set_error(ctx,
                             "camera idle before DMA submit: DDR ring occupancy %u exceeds %u slots",
                             occupancy, ctx->ring_slots);
            errno = EOVERFLOW;
            return -1;
        }
        errno = EAGAIN;
        return 0;
    }

    if (camera_validate_dma_stream(ctx, &current_info,
                                   "camera state before DMA submit") < 0)
        return -1;
    /* In newest-frame mode, discard stale DDR ownership tokens before
     * submitting another DMA.  producer-1 is the newest complete slot; the
     * RTL publishes producer only after the final AXI write response.  The
     * no-inflight/matched-index guard prevents skipping across a DMA which
     * may still be reading an older slot.
     */
    if (!ctx->ddr && ctx->dma_latest_mode && ctx->dma_inflight == 0u &&
        ctx->dma_next_submit_idx == ctx->dma_next_ddr_release_idx) {
        uint32_t pending = producer - ctx->dma_next_submit_idx;

        if (pending > ctx->ring_slots) {
            camera_set_error(ctx,
                             "latest-frame pre-submit occupancy %u exceeds %u slots",
                             pending, ctx->ring_slots);
            errno = EOVERFLOW;
            return -1;
        }
        if (pending > 1u) {
            uint32_t newest_complete = producer - 1u;
            uint32_t skipped = newest_complete - ctx->dma_next_submit_idx;

            ctx->dma_next_submit_idx = newest_complete;
            ctx->dma_next_ddr_release_idx = newest_complete;
            ctx->dma_latest_ddr_skipped_pending += skipped;
            camera_write32(ctx, CAMERA_REG_CONSUMER_IDX, newest_complete);
            camera_mmio_flush(ctx);
        }
    }
    if (ctx->dma_inflight >= ctx->dma_inflight_limit) {
        errno = EAGAIN;
        return 0;
    }

    buf_index = camera_dma_find_free_buffer_locked(ctx);
    if (buf_index < 0) {
        errno = EAGAIN;
        return 0;
    }

    camera_fill_info_for_idx(ctx, ctx->dma_next_submit_idx, producer, info_out);
    if (ctx->dma_strided && current_info.width && current_info.height &&
        current_info.frame_bytes) {
        info_out->width = current_info.width;
        info_out->height = current_info.height;
        info_out->frame_bytes = current_info.frame_bytes;
        if (info_out->width != ctx->dma_src_line_bytes ||
            info_out->height != ctx->dma_rows ||
            info_out->frame_bytes != ctx->dma_transfer_size) {
            camera_set_error(ctx,
                             "camera DMA submit: FPGA RGB888 metadata mismatch: got line=%u height=%u frame=%u, expected line=%u height=%u frame=%zu",
                             info_out->width, info_out->height, info_out->frame_bytes,
                             ctx->dma_src_line_bytes, ctx->dma_rows, ctx->dma_transfer_size);
            errno = EPROTO;
            return -1;
        }
        ctx->dma_src_stride_learned = ctx->dma_src_line_bytes;
    }
    if (camera_validate_frame_info(ctx, info_out, len,
                                   "camera DMA submit") < 0 ||
        camera_validate_dma_stream(ctx, info_out,
                                   "camera DMA submit") < 0)
        return -1;
    if (info_out->dev_addr > UINT32_MAX ||
        info_out->dev_addr + len > (uint64_t)UINT32_MAX + 1ull) {
        camera_set_error(ctx, "frame DDR address 0x%llx + %zu exceeds 32-bit aperture",
                         (unsigned long long)info_out->dev_addr, len);
        errno = EINVAL;
        return -1;
    }

    info_out->dma_buffer_index = (uint32_t)buf_index;
    info_out->ready_ns = ddr_ns();
    if (ctx->ddr && ddr_publish(ctx->ddr, info_out->consumer_idx,
                               info_out->frame_seq, info_out->ready_ns,
                               info_out->dev_addr) < 0) {
        camera_set_error(ctx, "DDR direct: descriptor still owned by reader");
        return -1;
    }
    ctx->dma_buffers[buf_index].state = CAMERA_DMA_SUBMITTED;
    ctx->dma_buffers[buf_index].info = *info_out;
    ctx->dma_buffers[buf_index].len = len;
    ctx->dma_buffers[buf_index].op_id = 0;
    ctx->dma_inflight++;
    ctx->dma_next_submit_idx++;
    *buf_index_out = buf_index;
    return 1;
}


static void camera_direct_reclaim_locked(struct camera_context *ctx)
{
    while(ctx->direct_release_cursor<ctx->direct_cursor) {
        struct camera_dma_buffer *b=&ctx->dma_buffers[ctx->direct_release_cursor%ctx->dma_buffer_count];
        if(b->direct_release_token!=(uint64_t)ctx->direct_release_cursor+1)break;
        b->direct_release_token=0;ctx->direct_release_cursor++;
        /* One-slot Gray transitions, including out-of-order CPU completion. */
        camera_write32(ctx,VIDEO_HOST_CONSUMER,ctx->direct_release_cursor);
        camera_mmio_flush(ctx);
    }
}
static int camera_direct_publish_locked(struct camera_context *ctx)
{
    if(!ctx->ddr)return 0;
    uint32_t prod=camera_read32(ctx,CAMERA_REG_PRODUCER_IDX);
    if(prod==UINT32_MAX || prod<ctx->direct_ddr_publish ||
       prod-ctx->direct_ddr_publish>DDR_RING_SLOTS) {
        camera_set_error(ctx,"direct-host: invalid DDR producer=%u cursor=%u",prod,ctx->direct_ddr_publish);
        return -1;
    }
    while(ctx->direct_ddr_publish<prod) {
        uint32_t i=ctx->direct_ddr_publish;
        uint32_t seq=camera_read32(ctx,VIDEO_DDR_FRAME_IDS+(i%DDR_RING_SLOTS)*4);
        if(ddr_publish(ctx->ddr,i,seq,ddr_ns(),ctx->ring_base+(uint64_t)(i%DDR_RING_SLOTS)*ctx->frame_stride)<0)
            return -1;
        ctx->direct_ddr_publish++;
    }
    camera_ddr_release_locked(ctx);
    return 0;
}
/* dma_lock held on entry/exit. Host bytes are written directly by FPGA;
 * no EDEV D2H descriptor and no DDR read are submitted here. */
static int camera_direct_step_locked(struct camera_context *ctx)
{
    if(camera_direct_publish_locked(ctx)<0)return -1;
    uint32_t err=camera_read32(ctx,VIDEO_HOST_ERROR);
    if(err) {camera_set_error(ctx,"direct-host hardware fault=%u; preserve DMA owner",err);return -1;}
    if(ctx->dma_inflight>=ctx->dma_inflight_limit)return 0;
    uint32_t cur=ctx->direct_cursor;
    if(cur==UINT32_MAX)return -1;
    unsigned int bi=cur%ctx->dma_buffer_count;
    struct camera_dma_buffer *b=&ctx->dma_buffers[bi];
    if(b->state!=CAMERA_DMA_FREE)return 0;
    struct video_host_footer *mapped=(struct video_host_footer *)((uint8_t *)b->ptr+camera_round_up(ctx->dma_transfer_size,64));
    uint64_t token=__atomic_load_n(&mapped->token,__ATOMIC_ACQUIRE);
    if(token!=(uint64_t)cur+1)return 0;
    struct video_host_footer f;memcpy(&f,mapped,sizeof(f));
    uint32_t fmt=ctx->configured_processing_mode==CAMERA_PROCESS_RAW?0x42474752U:0x33424752U;
    uint32_t width=ctx->configured_expected_line_bytes;
    if(!video_host_footer_valid(&f,token,width,ctx->configured_expected_height,fmt,ctx->dma_transfer_size)) {
        camera_set_error(ctx,"direct-host footer ABI/frame geometry mismatch");return -1;
    }
    b->state=CAMERA_DMA_SUBMITTED;ctx->direct_cursor++;ctx->dma_inflight++;
    memset(&b->info,0,sizeof(b->info));
    b->info.consumer_idx=cur;b->info.producer_idx=cur+1;b->info.frame_seq=(uint32_t)f.frame_id;
    b->info.width=ctx->dma_expected_width;b->info.height=f.height;b->info.frame_bytes=f.bytes;
    b->info.ready_ns=ddr_ns();b->info.dma_buffer_index=bi;
    b->info.drop_count=camera_read32(ctx,CAMERA_REG_DROP_COUNT);
    b->info.status=camera_read32(ctx,CAMERA_REG_STATUS);
    b->info.slot=f.ddr_index%DDR_RING_SLOTS;
    b->info.dev_addr=ctx->ring_base+(uint64_t)b->info.slot*ctx->frame_stride;
    b->len=ctx->dma_transfer_size;
    pthread_mutex_unlock(&ctx->dma_lock);
    uint64_t begin=ddr_ns();
    int reference_required=ctx->ddr && ctx->ddr->crc_mode==VIDEO_CRC_FULL;
    uint32_t crc=(f.flags&1) && reference_required ?
        video_crc32(b->ptr,b->len):0;
    uint64_t elapsed=ddr_ns()-begin;
    pthread_mutex_lock(&ctx->dma_lock);
    if(camera_direct_publish_locked(ctx)<0)return -1;
    /* Transport mode releases DDR after RDMA alone. Its ring slot may be
     * reused before this independent local preview frame is consumed. */
    /* A deliberately gated source may close one incomplete terminal frame.
     * Its footer proves that all host writes completed; retire that slot
     * without publishing a nonexistent DDR reference or relaxing CRC for
     * any complete frame while acquisition is running. */
    int stop_tail_bad=reference_required && (f.flags&3)!=3 &&
        (camera_read32(ctx,CAMERA_REG_CONTROL)&0x10U);
    if(stop_tail_bad)
        fprintf(stderr,"STOP_TAIL retired host_frame=%u flags=0x%x frame_id=%llu\n",
                cur,f.flags,(unsigned long long)f.frame_id);
    /* A Camera Link synchronization slip is a source-frame erasure.  RTL
     * completes the host footer with flags=0 and does not commit that frame
     * to DDR. Retire only that host slot; never publish its bytes or weaken
     * CRC checks on the next complete frame. Other incomplete footers remain
     * fatal, including a host/DDR disagreement. */
    int cmlk_drop_bad=0;
    if(reference_required && !stop_tail_bad && f.flags==0 &&
       camera_read32(ctx,CAMERA_REG_DIAG_DROP_REASON)==CAMERA_DROP_REASON_CMLK_SYNC_MISMATCH &&
       !(camera_read32(ctx,CAMERA_REG_STATUS)&CAMERA_STATUS_ERROR_MASK &
         ~CAMERA_STATUS_FRAME_SIZE_ERROR)) {
        uint32_t drops=camera_read32(ctx,CAMERA_REG_DROP_COUNT);
        uint32_t delta=drops-ctx->dma_drop_baseline;
        uint32_t prod=camera_read32(ctx,CAMERA_REG_PRODUCER_IDX);
        if(delta>0 && delta<=16 && ctx->direct_cmlk_drop_budget<=16-delta) {
            ctx->direct_cmlk_drop_budget+=delta;
            ctx->dma_drop_baseline=drops;
        }
        if(ctx->direct_cmlk_drop_budget && f.ddr_index<=prod &&
           (f.ddr_index==prod ||
            camera_read32(ctx,VIDEO_DDR_FRAME_IDS+(f.ddr_index%DDR_RING_SLOTS)*4)!=(uint32_t)f.frame_id)) {
            ctx->direct_cmlk_drop_budget--;
            cmlk_drop_bad=1;
            fprintf(stderr,"CMLK_FRAME_DROP host_frame=%u frame_id=%llu ddr_index=%u drop_count=%u pending=%u reason=13\n",
                cur,(unsigned long long)f.frame_id,f.ddr_index,drops,ctx->direct_cmlk_drop_budget);
            if(camera_clear_status(ctx)<0)return -1;
            /* CLEAR_STATUS also resets the diagnostic drop counter on this
             * RTL. Rebase immediately or the next isolated slip appears to
             * have delta zero and is incorrectly treated as fatal. */
            ctx->dma_drop_baseline=camera_read32(ctx,CAMERA_REG_DROP_COUNT);
        }
    }
    int ring_full_host_only=0;
    if(reference_required && !stop_tail_bad && !cmlk_drop_bad && f.flags==1) {
        uint32_t producer=camera_read32(ctx,CAMERA_REG_PRODUCER_IDX);
        uint32_t committed_id=camera_read32(ctx,VIDEO_DDR_FRAME_IDS+
                                             (f.ddr_index%DDR_RING_SLOTS)*4);
        ring_full_host_only=video_host_footer_ring_full(&f,
            camera_read32(ctx,CAMERA_REG_DIAG_DROP_REASON),
            camera_read32(ctx,CAMERA_REG_STATUS)&CAMERA_STATUS_ERROR_MASK,
            producer,committed_id);
        if(ring_full_host_only) {
            ctx->dma_drop_baseline=camera_read32(ctx,CAMERA_REG_DROP_COUNT);
            fprintf(stderr,"DDR_FULL_FRAME host_frame=%u frame_id=%llu ddr_index=%u "
                    "drop_count=%u host_complete=1 ddr_committed=0 reason=16\n",
                    cur,(unsigned long long)f.frame_id,f.ddr_index,ctx->dma_drop_baseline);
        }
    }
    if(reference_required && (f.flags&3)!=3 && !stop_tail_bad &&
       !cmlk_drop_bad && !ring_full_host_only) {
        camera_set_error(ctx,"direct-host frame rejected: flags=0x%x host_frame=%u frame_id=%llu ddr_index=%u host_prod=%u host_cons=%u host_stall=%u ddr_prod=%u ddr_cons=%u drop=%u baseline=%u reason=%u; full CRC requires every frame",
            f.flags,cur,(unsigned long long)f.frame_id,f.ddr_index,
            camera_read32(ctx,VIDEO_HOST_PRODUCER),camera_read32(ctx,VIDEO_HOST_CONSUMER),
            camera_read32(ctx,VIDEO_HOST_STALL),
            camera_read32(ctx,CAMERA_REG_PRODUCER_IDX),camera_read32(ctx,CAMERA_REG_CONSUMER_IDX),
            camera_read32(ctx,CAMERA_REG_DROP_COUNT),ctx->dma_drop_baseline,
            camera_read32(ctx,CAMERA_REG_DIAG_DROP_REASON));
        return -1;
    }
    if((f.flags&3)==3 && reference_required) {
        uint64_t commit_deadline=ddr_ns()+5000000000ULL;
        while(f.ddr_index>=ctx->direct_ddr_publish) {
            pthread_mutex_unlock(&ctx->dma_lock);
            usleep(100);
            pthread_mutex_lock(&ctx->dma_lock);
            if(camera_direct_publish_locked(ctx)<0)return -1;
            if(ddr_ns()>commit_deadline) {
                camera_set_error(ctx,"direct-host: DDR commit timeout index=%u",f.ddr_index);
                return -1;
            }
        }
        struct ddr_frame *d=ddr_frame_at(ctx->ddr,f.ddr_index);
        if(atomic_load_explicit(&d->token,memory_order_acquire)!=(uint64_t)f.ddr_index+1 ||
           d->frame_id!=f.frame_id) {
            camera_set_error(ctx,"direct-host/video frame-ID mismatch");return -1;
        }
        b->info.ready_ns=d->ready_ns; /* Identical timestamp on video and YOLO paths. */
        d->crc=crc;
        atomic_store_explicit(&d->dma_done,(uint64_t)f.ddr_index+1,memory_order_release);
        camera_ddr_release_locked(ctx);
    }
    if((cur&255U)==0)
        fprintf(stderr,"HOST_DIRECT frame=%u id=%llu crc_ms=%.3f no_DDR_read=1\n",
            cur,(unsigned long long)f.frame_id,elapsed/1e6);
    ctx->dma_inflight--;
    if(!(f.flags&1) || stop_tail_bad || cmlk_drop_bad) {
        /* Keep ordered completion; invalid frames are retired by flush below. */
        b->len=0;
    }
    b->state=CAMERA_DMA_DONE;
    camera_dma_flush_completed_locked(ctx);
    pthread_cond_broadcast(&ctx->dma_cond);
    return 1;
}

struct camera_dma_worker_arg {
    struct camera_context *ctx;
    unsigned int lane;
};


static int __attribute__((unused)) camera_dma_driver_frame_worker_once(struct camera_context *ctx,
                                               unsigned int lane,
                                               int *submit_errno_out)
{
#ifdef EDEV_IOC_CAMERA_SUBMIT_FRAME
    struct edev_ioc_camera_submit_frame req;
    struct camera_dma_buffer *buf;
    int buf_index;
    int submit_errno = 0;

    if (submit_errno_out)
        *submit_errno_out = 0;

    buf_index = camera_dma_find_free_buffer_locked(ctx);
    if (buf_index < 0 || ctx->dma_inflight >= ctx->dma_inflight_limit) {
        errno = EAGAIN;
        return 0;
    }

    buf = &ctx->dma_buffers[buf_index];
    buf->state = CAMERA_DMA_SUBMITTED;
    buf->len = ctx->dma_transfer_size;
    buf->op_id = 0;
    memset(&buf->info, 0, sizeof(buf->info));
    ctx->dma_inflight++;

    memset(&req, 0, sizeof(req));
    req.csr_base = ctx->csr_base;
    req.ring_base = ctx->ring_base;
    req.frame_stride = ctx->frame_stride;
    req.ring_slots = ctx->ring_slots;
    req.qid = ctx->dma_lanes[lane].qid;
    req.buf_id = buf->lane_buf_id[lane];
    req.flags = ((ctx->configured_dma_frame_irq >= 0) ?
                 (unsigned int)ctx->configured_dma_frame_irq :
                 camera_dma_env_u32("CAMERA_DMA_IRQ", 0, 0, 1)) ?
                0 : EDEV_SUBMIT_F_NO_IRQ;
    req.timeout_ms = CAMERA_DEFAULT_TIMEOUT_MS;
    req.len = buf->len;
    req.buf_offset = 0;

    pthread_mutex_unlock(&ctx->dma_lock);
    if (ioctl(ctx->dma_lanes[lane].fd, EDEV_IOC_CAMERA_SUBMIT_FRAME, &req) < 0)
        submit_errno = errno;
    else if (req.result)
        submit_errno = req.result < 0 ? -req.result : EIO;
    pthread_mutex_lock(&ctx->dma_lock);

    if (submit_errno) {
        buf->state = CAMERA_DMA_CHECKED_OUT;
        if (ctx->dma_inflight)
            ctx->dma_inflight--;
        if (submit_errno_out)
            *submit_errno_out = submit_errno;
        camera_set_error(ctx,
                         "kernel camera IRQ+D2H failed on q%u: %s result=%d desc_count=%u producer=%u consumer=%u status=0x%08x",
                         ctx->dma_lanes[lane].qid, strerror(submit_errno),
                         req.result, req.desc_count, req.producer_idx,
                         req.consumer_idx, req.status);
        errno = submit_errno;
        return -1;
    }

    buf->info.producer_idx = req.producer_idx;
    buf->info.consumer_idx = req.consumer_idx;
    buf->info.slot = req.slot;
    buf->info.width = req.width ? req.width : ctx->dma_expected_width;
    buf->info.height = req.height ? req.height : ctx->dma_expected_height;
    buf->info.frame_bytes = req.frame_bytes ? req.frame_bytes : ctx->dma_expected_frame_bytes;
    buf->info.frame_seq = req.frame_seq;
    buf->info.drop_count = req.drop_count;
    buf->info.status = req.status;
    buf->info.dma_buffer_index = (uint32_t)buf_index;
    buf->info.dev_addr = req.dev_addr;
    buf->len = (size_t)req.len;
    buf->state = CAMERA_DMA_DONE;
    if (ctx->dma_inflight)
        ctx->dma_inflight--;
    camera_dma_flush_completed_locked(ctx);
    pthread_cond_broadcast(&ctx->dma_cond);
    return 1;
#else
    (void)ctx; (void)lane; (void)submit_errno_out;
    errno = ENOTSUP;
    return -1;
#endif
}

static void *camera_dma_sync_worker_main(void *arg)
{
    struct camera_dma_worker_arg *wa = arg;
    struct camera_context *ctx = wa->ctx;
    unsigned int lane = wa->lane;
    uint64_t profile_dma_ns=0,profile_crc_ns=0,profile_report_ns=ddr_ns();
    unsigned int profile_samples=0;
    free(wa);

    pthread_mutex_lock(&ctx->dma_lock);
    while (!ctx->dma_thread_stop) {
        struct camera_frame_info info;
        struct edev_ioc_dma_submit submit;
        struct camera_dma_buffer *buf;
        int buf_index = -1;
        int claim;
        int submit_errno = 0;

#ifdef EDEV_IOC_CAMERA_SUBMIT_FRAME
        if (ctx->configured_driver_frame_dma && !ctx->dma_strided) {
            int kerrno = 0;
            int kret = camera_dma_driver_frame_worker_once(ctx, lane, &kerrno);
            if (kret < 0) {
                if (kerrno == ENOTTY || kerrno == ENOSYS || kerrno == EINVAL || kerrno == ENOTSUP) {
                    ctx->configured_driver_frame_dma = 0;
                } else {
                    goto worker_error;
                }
            } else if (kret > 0) {
                continue;
            } else {
                pthread_cond_wait(&ctx->dma_cond, &ctx->dma_lock);
                continue;
            }
        }
#endif

        if(ctx->host_direct) {
            int direct_ret=camera_direct_step_locked(ctx);
            if(direct_ret<0)goto worker_error;
            if(!direct_ret) {
                pthread_mutex_unlock(&ctx->dma_lock);
                usleep(100);
                pthread_mutex_lock(&ctx->dma_lock);
            }
            continue;
        }
        claim = camera_dma_claim_work_locked(ctx, &buf_index, &info);
        if (claim < 0)
            goto worker_error;
        if (claim == 0) {
            int wait_ret;

            /* Frame-ready may already be asserted while every host buffer is
             * READY/CHECKED_OUT. Polling that level would return immediately
             * and repeatedly reacquire dma_lock, starving the FIFO consumer.
             * Only completion/recycle can provide a host slot in this case. */
            if (ctx->dma_inflight >= ctx->dma_inflight_limit ||
                camera_dma_find_free_buffer_locked(ctx) < 0) {
                pthread_cond_wait(&ctx->dma_cond, &ctx->dma_lock);
                continue;
            }

            /* A single waiter owns the frame-ready ioctl. The other DMA lanes
             * sleep on dma_cond instead of creating a thundering herd of MMIO
             * polls and wait-queue ioctls.
             */
            if (ctx->dma_frame_waiter_active) {
                pthread_cond_wait(&ctx->dma_cond, &ctx->dma_lock);
                continue;
            }
            ctx->dma_frame_waiter_active = 1;
            pthread_mutex_unlock(&ctx->dma_lock);
            wait_ret = camera_wait_frame_event(ctx, 2);
            pthread_mutex_lock(&ctx->dma_lock);
            ctx->dma_frame_waiter_active = 0;
            pthread_cond_broadcast(&ctx->dma_cond);
            if (wait_ret < 0)
                goto worker_error;
            continue;
        }

        buf = &ctx->dma_buffers[buf_index];
        memset(&submit, 0, sizeof(submit));
        submit.qid = ctx->dma_lanes[lane].qid;
        submit.buf_id = buf->lane_buf_id[lane];
        submit.dir = EDEV_DMA_DIR_D2H;
        submit.flags = ((ctx->configured_dma_frame_irq >= 0) ?
                        (unsigned int)ctx->configured_dma_frame_irq :
                        camera_dma_env_u32("CAMERA_DMA_IRQ", 0, 0, 1)) ?
                       0 : EDEV_SUBMIT_F_NO_IRQ;
        /* Imported NvBufSurface DMA-BUFs are written by PCIe SGDMA and then
         * consumed by GPU/EGL, with no CPU map/write in this path.  Avoid the
         * CPU-cache-maintenance path in the SGDMA driver, which is only valid
         * for CPU access and can trigger NvMapMemCacheMaint errors on Jetson
         * SURFACE_ARRAY buffers.  Host-pinned bounce/staging buffers have ptr
         * != NULL and keep the normal sync behavior.
         */
        if (buf->ptr == NULL && buf->buf_id != 0)
            submit.flags |= EDEV_SUBMIT_F_SKIP_CPU_SYNC;
        submit.buf_offset = 0;
        submit.len = buf->len;
        submit.dev_addr = info.dev_addr;

        pthread_mutex_unlock(&ctx->dma_lock);
        uint64_t dma_begin_ns=ddr_ns();
        if (ctx->dma_strided) {
#ifdef EDEV_IOC_SUBMIT_2D
            struct edev_ioc_dma_submit_2d submit2d;
            memset(&submit2d, 0, sizeof(submit2d));
            submit2d.qid = submit.qid;
            submit2d.buf_id = submit.buf_id;
            submit2d.dir = EDEV_DMA_DIR_D2H;
            submit2d.flags = submit.flags | EDEV_SUBMIT_F_SKIP_CPU_SYNC;
            submit2d.buf_offset = ctx->dma_dst_offset;
            submit2d.dev_addr = info.dev_addr;
            submit2d.row_bytes = ctx->dma_src_line_bytes;
            submit2d.rows = ctx->dma_rows;
            submit2d.dev_stride = ctx->dma_src_line_bytes;
            submit2d.buf_stride = ctx->dma_dst_stride;
            if (submit2d.dev_stride < submit2d.row_bytes) {
                submit_errno = EMSGSIZE;
            } else if ((uint64_t)(submit2d.rows - 1u) * submit2d.dev_stride +
                       submit2d.row_bytes > ctx->frame_stride) {
                submit_errno = EMSGSIZE;
            } else if (ioctl(ctx->dma_lanes[lane].fd, EDEV_IOC_SUBMIT_2D, &submit2d) < 0)
                submit_errno = errno;
            else if (submit2d.status)
                submit_errno = submit2d.status < 0 ? -submit2d.status : EIO;
            else if (submit2d.bytes_done != buf->len)
                submit_errno = EIO;
            submit.desc_count = submit2d.desc_count;
            submit.status = submit2d.status;
            submit.len = (uint64_t)submit2d.bytes_done;
#else
            submit_errno = ENOTSUP;
#endif
        } else {
            if (ioctl(ctx->dma_lanes[lane].fd, EDEV_IOC_SUBMIT_BUFFER, &submit) < 0)
                submit_errno = errno;
            else if (submit.status)
                submit_errno = submit.status < 0 ? -submit.status : EIO;
            else if (submit.len != buf->len)
                submit_errno = EIO;
        }
        /* End-to-end reference CRC is computed from completed D2H memory.
         * It does not hold the scheduler lock or delay ERNIC's concurrent DDR read. */
        if (!submit_errno && ctx->ddr) {
            struct ddr_frame *desc = ddr_frame_at(ctx->ddr, info.consumer_idx);
            uint64_t crc_begin_ns=ddr_ns();
            /* D2H completion/cache synchronization are still mandatory in both modes. */
            desc->crc = ctx->ddr->crc_mode == VIDEO_CRC_FULL ?
                        video_crc32(buf->ptr, buf->len) : 0;
            if ((info.consumer_idx & 255u) == 0)
                fprintf(stderr,"DDR_DMA frame=%u dma_ms=%.3f crc_ms=%.3f\n",
                        info.consumer_idx,(crc_begin_ns-dma_begin_ns)/1e6,(ddr_ns()-crc_begin_ns)/1e6);
            uint64_t crc_end_ns=ddr_ns();
            profile_dma_ns+=crc_begin_ns-dma_begin_ns;
            profile_crc_ns+=crc_end_ns-crc_begin_ns;
            ++profile_samples;
            atomic_store_explicit(&desc->dma_done,
                                  (uint64_t)info.consumer_idx + 1, memory_order_release);
            if(crc_end_ns-profile_report_ns>=1000000000ULL) {
                fprintf(stderr,"DMA_STAGES lane=%u samples=%u dma_ms=%.3f crc_ms=%.3f backend=%s\n",
                        lane,profile_samples,profile_dma_ns/1e6/profile_samples,
                        profile_crc_ns/1e6/profile_samples,video_crc_backend());
                profile_dma_ns=profile_crc_ns=0;profile_samples=0;profile_report_ns=crc_end_ns;
            }
        }
        pthread_mutex_lock(&ctx->dma_lock);

        if (submit_errno) {
            buf->state = CAMERA_DMA_CHECKED_OUT;
            if (ctx->dma_inflight)
                ctx->dma_inflight--;
            camera_set_error(ctx,
                             "%s D2H DMA failed on q%u frame=%u: %s status=%d desc_count=%u line=%u rows=%u pitch=%u",
                             ctx->dma_strided ? "strided" : "sync",
                             ctx->dma_lanes[lane].qid, info.consumer_idx,
                             strerror(submit_errno), submit.status, submit.desc_count,
                             ctx->dma_src_line_bytes, ctx->dma_rows,
                             ctx->dma_dst_stride);
            errno = submit_errno;
            goto worker_error;
        }

        buf->len = ctx->dma_transfer_size;
        buf->info = info;
        buf->info.dma_buffer_index = (uint32_t)buf_index;
        buf->state = CAMERA_DMA_DONE;
        if (ctx->dma_inflight)
            ctx->dma_inflight--;
        camera_dma_flush_completed_locked(ctx);
        pthread_cond_broadcast(&ctx->dma_cond);
    }

    pthread_cond_broadcast(&ctx->dma_cond);
    pthread_mutex_unlock(&ctx->dma_lock);
    return NULL;

worker_error:
    ctx->dma_thread_error = 1;
    ctx->dma_thread_stop = 1;
    pthread_cond_broadcast(&ctx->dma_cond);
    pthread_mutex_unlock(&ctx->dma_lock);
    return NULL;
}

void camera_dma_close_pool(struct camera_context *ctx)
{
    unsigned int i, q;

    if (!ctx)
        return;
    camera_dma_stop_pump(ctx);
    if(ctx->host_direct && ctx->host_armed) {
        if(!(camera_read32(ctx,VIDEO_HOST_STATUS)&1U)) {
            fprintf(stderr,"FAIL: direct-host DMA safety hold PID %ld; reset FPGA before releasing host buffers\n",(long)getpid());
            for(;;)pause();
        }
        camera_write32(ctx,VIDEO_HOST_CONTROL,0);
        camera_mmio_flush(ctx);
        usleep(1000);
        if(!(camera_read32(ctx,VIDEO_HOST_STATUS)&1U)) {
            fprintf(stderr,"FAIL: direct-host writer did not quiesce; preserve DMA allocation\n");
            for(;;)pause();
        }
        ctx->host_armed=0;
    }

    if (ctx->dma_buffers) {
        for (i = 0; i < ctx->dma_buffer_count; i++) {
            if (ctx->dma_buffers[i].buf_id) {
                uint32_t id = ctx->dma_buffers[i].buf_id;
                (void)ioctl(ctx->queue_fd, EDEV_IOC_UNREGISTER_BUFFER, &id);
            }
            ctx->dma_buffers[i].buf_id = 0;
            memset(ctx->dma_buffers[i].lane_buf_id, 0,
                   sizeof(ctx->dma_buffers[i].lane_buf_id));
            if (ctx->dma_buffers[i].locked)
                (void)munlock(ctx->dma_buffers[i].ptr, ctx->dma_buffers[i].size);
            camera_dma_free_host_buffer(&ctx->dma_buffers[i]);
        }
    }

    for (q = 1; q < ctx->dma_lane_count; q++) {
        if (ctx->dma_lanes[q].fd >= 0 &&
            ctx->dma_lanes[q].fd != ctx->queue_fd) {
            close(ctx->dma_lanes[q].fd);
        }
        ctx->dma_lanes[q].fd = -1;
    }

    free(ctx->dma_ready_ring);
    free(ctx->dma_buffers);
    ctx->dma_ready_ring = NULL;
    ctx->dma_buffers = NULL;
    ctx->dma_lane_count = 0;
    ctx->dma_buffer_count = 0;
    ctx->dma_inflight_limit = 0;
    ctx->dma_inflight = 0;
    ctx->dma_ready_head = 0;
    ctx->dma_ready_tail = 0;
    ctx->dma_ready_count = 0;
    ctx->dma_next_submit_idx = 0;
    ctx->dma_next_ddr_release_idx = 0;
    ctx->dma_drop_baseline = 0;
    ctx->dma_expected_width = 0;
    ctx->dma_expected_height = 0;
    ctx->dma_expected_frame_bytes = 0;
    ctx->dma_strided = 0;
    ctx->dma_src_line_bytes = 0;
    ctx->dma_src_stride_learned = 0;
    ctx->dma_dst_stride = 0;
    ctx->dma_rows = 0;
    ctx->dma_buffer_size = 0;
    ctx->dma_transfer_size = 0;
}

int camera_dma_open_pool(struct camera_context *ctx, unsigned int buffer_count,
                         size_t buffer_size, unsigned int inflight)
{
    uint32_t start_idx;
    unsigned int i, q;
    unsigned int lane_count;
    unsigned int irq_enable;
    int dmabuf_fds[256];
    int dmabuf_count = 0;

    dmabuf_count = camera_parse_dmabuf_fd_env(dmabuf_fds, 256);
    if (dmabuf_count < 0) {
        camera_set_error(ctx, "invalid CAMERA_DMA_DMABUF_FDS list");
        errno = EINVAL;
        return -1;
    }

    if (!ctx || !buffer_size || ctx->queue_fd < 0) {
        errno = EINVAL;
        return -1;
    }
    if (!ctx->frame_stride) {
        camera_set_error(ctx, "ring stride is not configured");
        errno = EINVAL;
        return -1;
    }
    if (ctx->frame_stride < buffer_size) {
        camera_set_error(ctx,
                         "frame stride %u is smaller than DMA frame size %zu",
                         ctx->frame_stride, buffer_size);
        errno = EMSGSIZE;
        return -1;
    }

    if (!buffer_count)
        buffer_count = CAMERA_DEFAULT_HOST_BUFFERS;
    if (!inflight)
        inflight = CAMERA_DEFAULT_INFLIGHT;
    if (buffer_count < 2)
        buffer_count = 2;
    if (buffer_count > 256)
        buffer_count = 256;
    if (dmabuf_count > 0) {
        if ((unsigned int)dmabuf_count < buffer_count)
            buffer_count = (unsigned int)dmabuf_count;
        if ((unsigned int)dmabuf_count > buffer_count)
            dmabuf_count = (int)buffer_count;
    }
    if (inflight > buffer_count)
        inflight = buffer_count;

    lane_count = ctx->configured_dma_queue_count ? ctx->configured_dma_queue_count : inflight;
    if (lane_count > CAMERA_DMA_MAX_SYNC_QUEUES)
        lane_count = CAMERA_DMA_MAX_SYNC_QUEUES;
#ifdef EDEV_MM_QUEUE_COUNT
    if (lane_count > EDEV_MM_QUEUE_COUNT)
        lane_count = EDEV_MM_QUEUE_COUNT;
#endif
    lane_count = camera_dma_env_u32("CAMERA_DMA_QUEUES", lane_count, 1, CAMERA_DMA_MAX_SYNC_QUEUES);
    if (lane_count > buffer_count)
        lane_count = buffer_count;
    if (!lane_count)
        lane_count = 1;

    camera_dma_close_pool(ctx);
    if (ctx->dma_buffers) {
        errno = EBUSY;
        return -1;
    }

    ctx->dma_transfer_size = buffer_size;
    if(ctx->host_direct && (dmabuf_count || buffer_count>32 || (buffer_count&(buffer_count-1)))) {
        camera_set_error(ctx,"direct-host requires 2/4/8/16/32 ordinary host buffers");
        errno=EINVAL;return -1;
    }
    ctx->dma_buffer_size = camera_round_up(buffer_size+(ctx->host_direct?64:0), CAMERA_DMA_ALIGN);
    ctx->dma_buffers = calloc(buffer_count, sizeof(ctx->dma_buffers[0]));
    ctx->dma_ready_ring = calloc(buffer_count, sizeof(ctx->dma_ready_ring[0]));
    if (!ctx->dma_buffers || !ctx->dma_ready_ring) {
        camera_set_error(ctx,
                         "failed to allocate DMA metadata for %u buffers",
                         buffer_count);
        camera_dma_close_pool(ctx);
        return -1;
    }

    ctx->dma_buffer_count = buffer_count;
    ctx->dma_lane_count = lane_count;
    ctx->dma_inflight_limit = lane_count;
    start_idx = camera_read32(ctx, CAMERA_REG_CONSUMER_IDX);
    ctx->dma_next_submit_idx = start_idx;
    ctx->dma_next_ddr_release_idx = start_idx;
    ctx->dma_drop_baseline = camera_read32(ctx, CAMERA_REG_DROP_COUNT);
    ctx->dma_expected_width = camera_read32(ctx, CAMERA_REG_WIDTH);
    ctx->dma_expected_height = camera_read32(ctx, CAMERA_REG_HEIGHT);
    ctx->dma_expected_frame_bytes = camera_read32(ctx, CAMERA_REG_FRAME_BYTES);
    if (ctx->configured_processing_mode == CAMERA_PROCESS_BAYERRG8_RGB888) {
        /* Capture is normally still disabled while the host pool is created,
         * so the published DDR metadata can legitimately be zero here.  The
         * configured values describe the BayerRG8 input; derive the RGB888
         * host geometry used by the HLS-enabled bitstream. */
        if (!ctx->dma_expected_width)
            ctx->dma_expected_width = ctx->configured_expected_line_bytes +
                                      (ctx->configured_expected_line_bytes << 1);
        if (!ctx->dma_expected_height)
            ctx->dma_expected_height = ctx->configured_expected_height;
        if (!ctx->dma_expected_frame_bytes)
            ctx->dma_expected_frame_bytes = ctx->configured_expected_frame_bytes +
                                            (ctx->configured_expected_frame_bytes << 1);
    } else {
        if (!ctx->dma_expected_width)
            ctx->dma_expected_width = ctx->configured_expected_line_bytes;
        if (!ctx->dma_expected_height)
            ctx->dma_expected_height = ctx->configured_expected_height;
        if (!ctx->dma_expected_frame_bytes)
            ctx->dma_expected_frame_bytes = ctx->configured_expected_frame_bytes;
    }
    if (!ctx->dma_expected_width || !ctx->dma_expected_height ||
        !ctx->dma_expected_frame_bytes) {
        camera_set_error(ctx,
                         "camera metadata is not ready and no explicit host metadata was supplied: width=%u height=%u bytes=%u",
                         ctx->dma_expected_width, ctx->dma_expected_height,
                         ctx->dma_expected_frame_bytes);
        errno = EPROTO;
        camera_dma_close_pool(ctx);
        return -1;
    }
    if ((size_t)ctx->dma_expected_frame_bytes != buffer_size) {
        camera_set_error(ctx,
                         "host frame size %zu does not match RTL frame size %u",
                         buffer_size, ctx->dma_expected_frame_bytes);
        errno = EMSGSIZE;
        camera_dma_close_pool(ctx);
        return -1;
    }

    irq_enable = (ctx->configured_dma_frame_irq >= 0) ?
                 (unsigned int)ctx->configured_dma_frame_irq :
                 camera_dma_env_u32("CAMERA_DMA_IRQ", 0, 0, 1);
    memset(ctx->dma_lanes, 0, sizeof(ctx->dma_lanes));
    for (q = 0; q < lane_count; q++) {
        ctx->dma_lanes[q].ctx = ctx;
        ctx->dma_lanes[q].fd = ctx->queue_fd;
        ctx->dma_lanes[q].qid = ctx->qid + q;
    }
    {
        uint32_t irq_val = irq_enable ? 1u : 0u;
        if (ioctl(ctx->queue_fd, EDEV_IOC_SET_IRQ_MODE, &irq_val) < 0) {
            /* IRQ can only be enabled if the driver was probed with MSI-X vectors.
             * Polling remains a safe fallback and avoids any IRQ storm. */
            ctx->last_error[0] = '\0';
        }
    }

    for (i = 0; i < buffer_count; i++) {
        int saved_errno;
        int ret;

        ctx->dma_buffers[i].size = ctx->dma_buffer_size;
        ctx->dma_buffers[i].state = CAMERA_DMA_FREE;

        if (dmabuf_count > 0) {
#ifdef EDEV_IOC_REGISTER_DMABUF
            ctx->dma_buffers[i].ptr = NULL;
            struct edev_ioc_dmabuf dbuf;
            memset(&dbuf, 0, sizeof(dbuf));
            dbuf.fd = dmabuf_fds[i];
            dbuf.size = ctx->dma_transfer_size;
            dbuf.dir = EDEV_DMA_DIR_D2H;
            if (ioctl(ctx->queue_fd, EDEV_IOC_REGISTER_DMABUF, &dbuf) < 0) {
                camera_set_error(ctx,
                                 "register DMA-BUF fd %d as buffer %u failed: %s",
                                 dmabuf_fds[i], i, strerror(errno));
                camera_dma_close_pool(ctx);
                return -1;
            }
            ctx->dma_buffers[i].buf_id = dbuf.buf_id;
            for (q = 0; q < lane_count; q++)
                ctx->dma_buffers[i].lane_buf_id[q] = dbuf.buf_id;
            continue;
#else
            camera_set_error(ctx, "driver header lacks EDEV_IOC_REGISTER_DMABUF");
            errno = ENOSYS;
            camera_dma_close_pool(ctx);
            return -1;
#endif
        }

        if(ctx->host_direct) {
            struct reconic_dma_alloc_req req={.size=ctx->dma_buffer_size};
            int fd=open(ctx->queue_path,O_RDWR|O_CLOEXEC);
            if(fd<0 || ioctl(fd,RECONIC_DMA_IOC_ALLOC_COHERENT,&req)<0 ||
               req.mmap_offset!=0 || req.size<ctx->dma_buffer_size ||
               req.dma_addr>UINT64_C(0xffffffffff)-req.size) {
                if(fd>=0)close(fd);
                camera_set_error(ctx,"direct-host coherent allocation %u failed: %s",i,strerror(errno));
                camera_dma_close_pool(ctx);return -1;
            }
            void *mem=mmap(NULL,req.size,PROT_READ|PROT_WRITE,MAP_SHARED,fd,0);
            if(mem==MAP_FAILED) {
                close(fd);camera_dma_close_pool(ctx);return -1;
            }
            ctx->dma_buffers[i].ptr=mem;ctx->dma_buffers[i].size=req.size;
            ctx->dma_buffers[i].coherent_owner=1;ctx->dma_buffers[i].coherent_fd=fd;
            ctx->dma_buffers[i].coherent_iova=req.dma_addr;
            memset(mem,0,req.size);
            continue;
        }
        ret = camera_dma_alloc_host_buffer(&ctx->dma_buffers[i],
                                           ctx->dma_buffer_size);
        if (ret) {
            errno = ret;
            camera_set_error(ctx, "host DMA buffer allocation (%zu bytes) failed: %s",
                             ctx->dma_buffer_size, strerror(errno));
            camera_dma_close_pool(ctx);
            return -1;
        }
        memset(ctx->dma_buffers[i].ptr, 0, ctx->dma_buffer_size);
        saved_errno = errno;
        if (mlock(ctx->dma_buffers[i].ptr, ctx->dma_buffer_size) == 0)
            ctx->dma_buffers[i].locked = 1;
        errno = saved_errno;

        struct edev_ioc_user_buffer ubuf;
        memset(&ubuf, 0, sizeof(ubuf));
        ubuf.user_va = (uint64_t)(uintptr_t)ctx->dma_buffers[i].ptr;
        ubuf.size = ctx->dma_buffers[i].size;
        ubuf.dir = EDEV_DMA_DIR_D2H;
        if (ioctl(ctx->queue_fd, EDEV_IOC_REGISTER_BUFFER, &ubuf) < 0) {
            camera_set_error(ctx,
                             "register DMA buffer %u failed: %s",
                             i, strerror(errno));
            camera_dma_close_pool(ctx);
            return -1;
        }
        ctx->dma_buffers[i].buf_id = ubuf.buf_id;
        for (q = 0; q < lane_count; q++)
            ctx->dma_buffers[i].lane_buf_id[q] = ubuf.buf_id;
    }
    return 0;
}


int camera_dma_open_dmabuf_pool_strided(struct camera_context *ctx,
                                        const int *dmabuf_fds,
                                        unsigned int buffer_count,
                                        size_t dmabuf_size,
                                        uint32_t dst_buffer_offset,
                                        uint32_t src_line_bytes,
                                        uint32_t dst_line_stride,
                                        uint32_t height,
                                        unsigned int inflight)
{
#ifndef EDEV_IOC_REGISTER_DMABUF
    (void)ctx; (void)dmabuf_fds; (void)buffer_count; (void)dmabuf_size;
    (void)dst_buffer_offset; (void)src_line_bytes; (void)dst_line_stride; (void)height; (void)inflight;
    errno = ENOTSUP;
    return -1;
#else
#ifndef EDEV_IOC_SUBMIT_2D
    (void)ctx; (void)dmabuf_fds; (void)buffer_count; (void)dmabuf_size;
    (void)dst_buffer_offset; (void)src_line_bytes; (void)dst_line_stride; (void)height; (void)inflight;
    errno = ENOTSUP;
    return -1;
#else
    uint32_t start_idx;
    unsigned int i, q;
    unsigned int lane_count;
    unsigned int irq_enable;
    size_t useful_frame;
    size_t required_surface;

    if (!ctx || !dmabuf_fds || !dmabuf_size || !src_line_bytes ||
        !dst_line_stride || !height || ctx->queue_fd < 0) {
        errno = EINVAL;
        return -1;
    }
    if (dst_line_stride < src_line_bytes) {
        camera_set_error(ctx, "NvBufSurface pitch %u is smaller than RGB8 line %u",
                         dst_line_stride, src_line_bytes);
        errno = EINVAL;
        return -1;
    }
    useful_frame = (size_t)src_line_bytes * (size_t)height;
    if (height > 0 && useful_frame / (size_t)height != (size_t)src_line_bytes) {
        errno = EOVERFLOW;
        return -1;
    }
    required_surface = (size_t)(height - 1u) * (size_t)dst_line_stride + src_line_bytes;
    if ((size_t)dst_buffer_offset > dmabuf_size ||
        required_surface > dmabuf_size - (size_t)dst_buffer_offset) {
        camera_set_error(ctx,
                         "NvBufSurface DMA-BUF span %zu is smaller than plane offset + pitch layout %zu + %zu (offset=%u pitch=%u height=%u line=%u)",
                         dmabuf_size, (size_t)dst_buffer_offset, required_surface,
                         dst_buffer_offset, dst_line_stride, height, src_line_bytes);
        errno = EMSGSIZE;
        return -1;
    }
    if (!ctx->frame_stride || ctx->frame_stride < useful_frame) {
        camera_set_error(ctx,
                         "DDR frame stride %u is smaller than FPGA RGB888 source frame %zu (line=%u height=%u)",
                         ctx ? ctx->frame_stride : 0, useful_frame,
                         src_line_bytes, height);
        errno = EINVAL;
        return -1;
    }
    if (useful_frame > UINT32_MAX) {
        camera_set_error(ctx, "FPGA RGB888 useful frame %zu exceeds 32-bit DMA accounting", useful_frame);
        errno = EMSGSIZE;
        return -1;
    }

    if (!buffer_count)
        buffer_count = 2;
    if (!inflight)
        inflight = CAMERA_DEFAULT_INFLIGHT;
    if (buffer_count < 2)
        buffer_count = 2;
    if (buffer_count > 256)
        buffer_count = 256;
    if (inflight > buffer_count)
        inflight = buffer_count;

    lane_count = ctx->configured_dma_queue_count ? ctx->configured_dma_queue_count : inflight;
    if (lane_count > CAMERA_DMA_MAX_SYNC_QUEUES)
        lane_count = CAMERA_DMA_MAX_SYNC_QUEUES;
#ifdef EDEV_MM_QUEUE_COUNT
    if (lane_count > EDEV_MM_QUEUE_COUNT)
        lane_count = EDEV_MM_QUEUE_COUNT;
#endif
    if (lane_count > buffer_count)
        lane_count = buffer_count;
    if (!lane_count)
        lane_count = 1;

    camera_dma_close_pool(ctx);
    ctx->dma_strided = 1;
    ctx->dma_src_line_bytes = src_line_bytes;
    ctx->dma_dst_offset = dst_buffer_offset;
    ctx->dma_dst_stride = dst_line_stride;
    ctx->dma_rows = height;
    ctx->dma_transfer_size = useful_frame;
    ctx->dma_buffer_size = dmabuf_size;
    ctx->dma_buffers = calloc(buffer_count, sizeof(ctx->dma_buffers[0]));
    ctx->dma_ready_ring = calloc(buffer_count, sizeof(ctx->dma_ready_ring[0]));
    if (!ctx->dma_buffers || !ctx->dma_ready_ring) {
        camera_set_error(ctx, "failed to allocate strided dmabuf DMA metadata");
        camera_dma_close_pool(ctx);
        return -1;
    }

    ctx->dma_buffer_count = buffer_count;
    ctx->dma_lane_count = lane_count;
    ctx->dma_inflight_limit = lane_count;
    start_idx = camera_read32(ctx, CAMERA_REG_CONSUMER_IDX);
    ctx->dma_next_submit_idx = start_idx;
    ctx->dma_next_ddr_release_idx = start_idx;
    ctx->dma_drop_baseline = camera_read32(ctx, CAMERA_REG_DROP_COUNT);
    /* Updated capture RTL publishes FPGA RGB888 source frames using the
     * expanded RGB888 geometry produced by the capture RTL.  The DMA-BUF destination
     * remains pitched, but the DDR source stride is src_line_bytes.
     */
    ctx->dma_expected_width = src_line_bytes;
    ctx->dma_expected_height = height;
    ctx->dma_expected_frame_bytes = (uint32_t)useful_frame;
    ctx->dma_src_stride_learned = src_line_bytes;

    /* A pitch-aware DMA-BUF path cannot use the old kernel CAMERA_SUBMIT_FRAME
     * helper, which only copies one contiguous range.  Use userspace frame IRQ
     * wait plus EDEV_IOC_SUBMIT_2D instead.
     */
    ctx->configured_driver_frame_dma = 0;

    irq_enable = (ctx->configured_dma_frame_irq >= 0) ?
                 (unsigned int)ctx->configured_dma_frame_irq : 0u;
    memset(ctx->dma_lanes, 0, sizeof(ctx->dma_lanes));
    for (q = 0; q < lane_count; q++) {
        ctx->dma_lanes[q].ctx = ctx;
        ctx->dma_lanes[q].fd = ctx->queue_fd;
        ctx->dma_lanes[q].qid = ctx->qid + q;
    }
    {
        uint32_t irq_val = irq_enable ? 1u : 0u;
        (void)ioctl(ctx->queue_fd, EDEV_IOC_SET_IRQ_MODE, &irq_val);
    }
    for (i = 0; i < buffer_count; i++) {
        ctx->dma_buffers[i].ptr = NULL;
        ctx->dma_buffers[i].size = dmabuf_size;
        ctx->dma_buffers[i].state = CAMERA_DMA_FREE;
        struct edev_ioc_dmabuf dbuf;
        memset(&dbuf, 0, sizeof(dbuf));
        dbuf.fd = dmabuf_fds[i];
        dbuf.dir = EDEV_DMA_DIR_D2H;
        dbuf.size = dmabuf_size;
        dbuf.offset = 0;
        if (ioctl(ctx->queue_fd, EDEV_IOC_REGISTER_DMABUF, &dbuf) < 0) {
            camera_set_error(ctx, "register pitched NvBufSurface dmabuf buffer %u failed: %s",
                             i, strerror(errno));
            camera_dma_close_pool(ctx);
            return -1;
        }
        ctx->dma_buffers[i].buf_id = dbuf.buf_id;
        for (q = 0; q < lane_count; q++)
            ctx->dma_buffers[i].lane_buf_id[q] = dbuf.buf_id;
    }
    return 0;
#endif
#endif
}

int camera_dma_open_dmabuf_pool(struct camera_context *ctx, const int *dmabuf_fds,
                                unsigned int buffer_count, size_t buffer_size,
                                unsigned int inflight)
{
#ifndef EDEV_IOC_REGISTER_DMABUF
    (void)ctx; (void)dmabuf_fds; (void)buffer_count; (void)buffer_size; (void)inflight;
    errno = ENOTSUP;
    return -1;
#else
    uint32_t start_idx;
    unsigned int i, q;
    unsigned int lane_count;
    unsigned int irq_enable;

    if (!ctx || !dmabuf_fds || !buffer_size || ctx->queue_fd < 0) {
        errno = EINVAL;
        return -1;
    }
    if (!ctx->frame_stride || ctx->frame_stride < buffer_size) {
        camera_set_error(ctx, "invalid ring stride %u for dmabuf frame size %zu",
                         ctx ? ctx->frame_stride : 0, buffer_size);
        errno = EINVAL;
        return -1;
    }
    if (!buffer_count)
        buffer_count = 2;
    if (!inflight)
        inflight = CAMERA_DEFAULT_INFLIGHT;
    if (buffer_count < 2)
        buffer_count = 2;
    if (buffer_count > 256)
        buffer_count = 256;
    if (inflight > buffer_count)
        inflight = buffer_count;

    lane_count = ctx->configured_dma_queue_count ? ctx->configured_dma_queue_count : inflight;
    if (lane_count > CAMERA_DMA_MAX_SYNC_QUEUES)
        lane_count = CAMERA_DMA_MAX_SYNC_QUEUES;
#ifdef EDEV_MM_QUEUE_COUNT
    if (lane_count > EDEV_MM_QUEUE_COUNT)
        lane_count = EDEV_MM_QUEUE_COUNT;
#endif
    if (lane_count > buffer_count)
        lane_count = buffer_count;
    if (!lane_count)
        lane_count = 1;

    camera_dma_close_pool(ctx);
    ctx->dma_transfer_size = buffer_size;
    ctx->dma_buffer_size = camera_round_up(buffer_size, CAMERA_DMA_ALIGN);
    ctx->dma_buffers = calloc(buffer_count, sizeof(ctx->dma_buffers[0]));
    ctx->dma_ready_ring = calloc(buffer_count, sizeof(ctx->dma_ready_ring[0]));
    if (!ctx->dma_buffers || !ctx->dma_ready_ring) {
        camera_set_error(ctx, "failed to allocate dmabuf DMA metadata");
        camera_dma_close_pool(ctx);
        return -1;
    }
    ctx->dma_buffer_count = buffer_count;
    ctx->dma_lane_count = lane_count;
    ctx->dma_inflight_limit = lane_count;
    start_idx = camera_read32(ctx, CAMERA_REG_CONSUMER_IDX);
    ctx->dma_next_submit_idx = start_idx;
    ctx->dma_next_ddr_release_idx = start_idx;
    ctx->dma_drop_baseline = camera_read32(ctx, CAMERA_REG_DROP_COUNT);
    if (ctx->configured_processing_mode == CAMERA_PROCESS_BAYERRG8_RGB888) {
        /* The RTL consumes BayerRG8 input geometry from camera_open_opts but
         * publishes expanded RGB888-sized DDR metadata.  For a contiguous
         * NvBufSurface DMA-BUF, validate against the actual DMA frame size here;
         * camera_dma_start_strict() will keep the RGB8 expected registers for
         * the RTL while resetting runtime delivery checks to its RGB888 arguments.
         */
        ctx->dma_expected_height = ctx->configured_expected_height;
        ctx->dma_expected_frame_bytes = (uint32_t)buffer_size;
        ctx->dma_expected_width = (ctx->dma_expected_height != 0) ?
                                  (uint32_t)(buffer_size / ctx->dma_expected_height) : 0;
    } else {
        ctx->dma_expected_width = ctx->configured_expected_line_bytes;
        ctx->dma_expected_height = ctx->configured_expected_height;
        ctx->dma_expected_frame_bytes = ctx->configured_expected_frame_bytes;
        if ((size_t)ctx->dma_expected_frame_bytes != buffer_size) {
            camera_set_error(ctx, "dmabuf frame size %zu does not match configured frame size %u",
                             buffer_size, ctx->dma_expected_frame_bytes);
            errno = EMSGSIZE;
            camera_dma_close_pool(ctx);
            return -1;
        }
    }

    irq_enable = (ctx->configured_dma_frame_irq >= 0) ?
                 (unsigned int)ctx->configured_dma_frame_irq : 0u;
    memset(ctx->dma_lanes, 0, sizeof(ctx->dma_lanes));
    for (q = 0; q < lane_count; q++) {
        ctx->dma_lanes[q].ctx = ctx;
        ctx->dma_lanes[q].fd = ctx->queue_fd;
        ctx->dma_lanes[q].qid = ctx->qid + q;
    }
    {
        uint32_t irq_val = irq_enable ? 1u : 0u;
        (void)ioctl(ctx->queue_fd, EDEV_IOC_SET_IRQ_MODE, &irq_val);
    }
    for (i = 0; i < buffer_count; i++) {
        ctx->dma_buffers[i].ptr = NULL;
        ctx->dma_buffers[i].size = buffer_size;
        ctx->dma_buffers[i].state = CAMERA_DMA_FREE;
        struct edev_ioc_dmabuf dbuf;
        memset(&dbuf, 0, sizeof(dbuf));
        dbuf.fd = dmabuf_fds[i];
        dbuf.dir = EDEV_DMA_DIR_D2H;
        dbuf.size = buffer_size;
        dbuf.offset = 0;
        if (ioctl(ctx->queue_fd, EDEV_IOC_REGISTER_DMABUF, &dbuf) < 0) {
            camera_set_error(ctx, "register DRM dmabuf buffer %u failed: %s",
                             i, strerror(errno));
            camera_dma_close_pool(ctx);
            return -1;
        }
        ctx->dma_buffers[i].buf_id = dbuf.buf_id;
        for (q = 0; q < lane_count; q++)
            ctx->dma_buffers[i].lane_buf_id[q] = dbuf.buf_id;
    }
    return 0;
#endif
}

static int camera_dma_wait_first_ready(struct camera_context *ctx, int timeout_ms,
                                       struct camera_frame_info *info)
{
    struct timespec abs;

    if (!ctx || !ctx->dma_buffers || !ctx->dma_sync_ready) {
        errno = EINVAL;
        return -1;
    }
    if (timeout_ms < 0)
        timeout_ms = CAMERA_DEFAULT_TIMEOUT_MS;
    if (timeout_ms == 0) {
        camera_set_error(ctx, "no timeout budget for first DMA frame");
        errno = EAGAIN;
        return -1;
    }
    {
        uint64_t nsec;
        if (clock_gettime(CLOCK_REALTIME, &abs) < 0)
            return -1;
        nsec = (uint64_t)abs.tv_nsec + (uint64_t)timeout_ms * 1000000ull;
        abs.tv_sec += (time_t)(nsec / 1000000000ull);
        abs.tv_nsec = (long)(nsec % 1000000000ull);
    }

    pthread_mutex_lock(&ctx->dma_lock);
    for (;;) {
        if (ctx->dma_ready_count) {
            uint32_t buffer_index = ctx->dma_ready_ring[ctx->dma_ready_head];
            if (buffer_index < ctx->dma_buffer_count &&
                ctx->dma_buffers[buffer_index].state == CAMERA_DMA_READY) {
                if (info)
                    *info = ctx->dma_buffers[buffer_index].info;
                pthread_mutex_unlock(&ctx->dma_lock);
                return 0;
            }
            pthread_mutex_unlock(&ctx->dma_lock);
            camera_set_error(ctx, "invalid first DMA-ready buffer index/state");
            errno = EIO;
            return -1;
        }
        if (ctx->dma_thread_error) {
            pthread_mutex_unlock(&ctx->dma_lock);
            if (!ctx->last_error[0])
                camera_set_error(ctx, "DMA pump stopped before first frame");
            errno = EIO;
            return -1;
        }
        if (!ctx->dma_thread_active) {
            pthread_mutex_unlock(&ctx->dma_lock);
            camera_set_error(ctx, "DMA pump thread is not running");
            errno = EIO;
            return -1;
        }
        {
            int ret = pthread_cond_timedwait(&ctx->dma_cond, &ctx->dma_lock, &abs);
            if (ret == ETIMEDOUT) {
                struct camera_csr_snapshot snap;
                pthread_mutex_unlock(&ctx->dma_lock);
                memset(&snap, 0, sizeof(snap));
                (void)camera_read_csr_snapshot(ctx, &snap);
                camera_set_error(ctx,
                                 "timed out waiting for first DMA frame: prod=%u cons=%u status=0x%08x%s width=%u height=%u frame=%u drop=%u diag:last_line=%u fval=%u lval=%u dval=%u pix_words=%u bad=%u reason=0x%08x hls_stall=%u axi_stall=%u host_prod=%u host_cons=%u host_stall=%u host_error=%u build=0x%08x",
                                 snap.producer_idx, snap.consumer_idx, snap.status,
                                 camera_status_bits(snap.status, (char[128]){0}, 128),
                                 snap.width, snap.height, snap.frame_bytes, snap.drop_count,
                                 snap.diag.last_line_bytes, snap.diag.fval_edges,
                                 snap.diag.lval_edges, snap.diag.dval_edges,
                                 snap.diag.pix_valid_words, snap.diag.bad_frames,
                                 snap.diag.drop_reason,
                                 snap.hls_stall_cycles, snap.axi_stall_cycles,
                                 ctx->host_direct ? camera_read32(ctx, VIDEO_HOST_PRODUCER) : 0,
                                 ctx->host_direct ? camera_read32(ctx, VIDEO_HOST_CONSUMER) : 0,
                                 ctx->host_direct ? camera_read32(ctx, VIDEO_HOST_STALL) : 0,
                                 ctx->host_direct ? camera_read32(ctx, VIDEO_HOST_ERROR) : 0,
                                 ctx->hw_build_version);
                errno = ETIMEDOUT;
                return -1;
            }
            if (ret) {
                pthread_mutex_unlock(&ctx->dma_lock);
                errno = ret;
                camera_set_error(ctx, "first DMA frame wait failed: %s", strerror(ret));
                return -1;
            }
        }
    }
}

int camera_dma_set_latest_mode(struct camera_context *ctx, int enable)
{
    if (!ctx || !ctx->dma_sync_ready) {
        errno = EINVAL;
        return -1;
    }

    pthread_mutex_lock(&ctx->dma_lock);
    if (ctx->dma_thread_active || ctx->dma_inflight != 0u) {
        pthread_mutex_unlock(&ctx->dma_lock);
        camera_set_error(ctx, "latest-frame mode must be selected before DMA start");
        errno = EBUSY;
        return -1;
    }
    ctx->dma_latest_mode = enable ? 1 : 0;
    ctx->dma_latest_ddr_skipped_pending = 0;
    pthread_mutex_unlock(&ctx->dma_lock);
    return 0;
}

int camera_dma_start_strict(struct camera_context *ctx, uint32_t control_flags,
                            uint32_t expected_line_bytes, uint32_t expected_height,
                            uint32_t expected_frame_bytes, int timeout_ms,
                            struct camera_frame_info *info)
{
    struct camera_frame_info ready_info;
    uint32_t producer;
    char saved_error[sizeof(ctx->last_error)];
    int saved_errno = 0;

    if (!ctx || !ctx->dma_buffers) {
        errno = EINVAL;
        return -1;
    }
    if (!expected_line_bytes || !expected_height || !expected_frame_bytes) {
        camera_set_error(ctx,
                         "strict DMA start requires explicit line bytes, height and frame bytes");
        errno = EINVAL;
        return -1;
    }
    if (ctx->frame_stride && ctx->frame_stride < expected_frame_bytes) {
        camera_set_error(ctx,
                         "frame stride %u is smaller than expected frame size %u",
                         ctx->frame_stride, expected_frame_bytes);
        errno = EMSGSIZE;
        return -1;
    }
    if (!camera_dma_pool_is_idle(ctx))
        return -1;

    ctx->configured_control_flags = control_flags & CAMERA_CTRL_OPTION_MASK;
    if ((ctx->configured_processing_mode == CAMERA_PROCESS_BAYERRG8_RGB888) &&
        ctx->configured_expected_line_bytes && ctx->configured_expected_frame_bytes) {
        /* Strided Orin display validates the expanded RGB888 DDR metadata through
         * the strict-start arguments below, but the capture RTL must be
         * programmed with the Camera Link BayerRG8 input geometry that it clips
         * before expanding to RGB888.  Preserve the values supplied in
         * camera_open_opts for the RTL registers.
         */
        ctx->configured_expected_height = expected_height;
    } else {
        ctx->configured_expected_line_bytes = expected_line_bytes;
        ctx->configured_expected_height = expected_height;
        ctx->configured_expected_frame_bytes = expected_frame_bytes;
    }

    if (camera_reset_capture(ctx) < 0)
        return -1;
    if (camera_program_static_capture_config(ctx) < 0)
        return -1;
    if (camera_clear_status(ctx) < 0)
        return -1;

    producer = camera_read32(ctx, CAMERA_REG_PRODUCER_IDX);
    camera_write32(ctx, CAMERA_REG_CONSUMER_IDX, producer);
    camera_mmio_flush(ctx);
    camera_dma_reset_runtime_indices(ctx, producer, 0u,
                                     expected_line_bytes,
                                     expected_height,
                                     expected_frame_bytes);

    const char *ddr_path = getenv("CAMERA_DDR_RING");
    if (ddr_path && *ddr_path) {
        if (ctx->ddr || ctx->dma_strided || ctx->configured_driver_frame_dma ||
            ctx->ring_slots != DDR_RING_SLOTS) {
            camera_set_error(ctx, "DDR direct requires ordinary multi-queue host DMA");
            errno = EINVAL;
            return -1;
        }
        int crc_mode=video_crc_mode_env();
        if (crc_mode < 0) {
            camera_set_error(ctx, "invalid VIDEO_CRC_MODE (full or transport)");
            errno=EINVAL;return -1;
        }
        ctx->ddr = ddr_map(ddr_path, 1, &ctx->ddr_fd);
        if (!ctx->ddr) {
            camera_set_error(ctx, "DDR descriptor create failed: %s", strerror(errno));
            return -1;
        }
        uint32_t bpp = ctx->configured_processing_mode == CAMERA_PROCESS_RAW ? 1u : 3u;
        uint64_t box_ddr_addr=(ctx->ring_base+(uint64_t)ctx->frame_stride*DDR_RING_SLOTS+4095)&~UINT64_C(4095);
        if(ctx->host_direct && box_ddr_addr>UINT64_C(0xffffffff)-4096) {
            camera_set_error(ctx,"no non-overlapping DDR arena for boxes");errno=ENOSPC;return -1;
        }
        ddr_init_paths(ctx->ddr, expected_line_bytes / bpp, expected_height,
                 bpp == 1 ? 0x42474752U : 0x33424752U, expected_frame_bytes, producer, (uint32_t)crc_mode,ctx->host_direct,ctx->host_direct?box_ddr_addr:0);
        ctx->ddr_release_idx = producer;
        fprintf(stderr, "DDR_RING ready=1 slots=%u bytes=%u path=%s\n",
                DDR_RING_SLOTS, expected_frame_bytes, ddr_path);
        uint64_t deadline = ddr_ns() + 20000000000ULL;
        while (atomic_load_explicit(&ctx->ddr->sender_state, memory_order_acquire) == DDR_SENDER_WAIT) {
            if (ddr_ns() > deadline) {
                camera_set_error(ctx, "DDR sender did not become ready before camera arm");
                errno = ETIMEDOUT;
                return -1;
            }
            usleep(1000);
        }
        if (atomic_load(&ctx->ddr->sender_state) != DDR_SENDER_ACTIVE) {
            camera_set_error(ctx, "DDR sender failed before camera arm");
            errno = EPIPE;
            return -1;
        }
    }
    if (camera_direct_arm(ctx, producer) < 0)
        return -1;
    usleep(1000);
    if (camera_enable_capture(ctx, control_flags & CAMERA_CTRL_OPTION_MASK) < 0)
        return -1;

    /* Arm all host DMA workers before the serial command releases the camera.
     * The first FVAL can then be serviced immediately instead of accumulating
     * frames while software is still starting its worker pool.
     */
    if (camera_dma_start_pump(ctx) < 0) {
        camera_save_error(ctx, saved_error, sizeof(saved_error), &saved_errno);
        (void)camera_disable_capture(ctx);
        camera_restore_error(ctx, saved_error, saved_errno);
        return -1;
    }
    camera_dma_signal(ctx);

    if (camera_remote_acquisition_start(ctx) < 0) {
        camera_save_error(ctx, saved_error, sizeof(saved_error), &saved_errno);
        camera_dma_stop_pump(ctx);
        (void)camera_disable_capture(ctx);
        camera_restore_error(ctx, saved_error, saved_errno);
        return -1;
    }

    if (camera_dma_wait_first_ready(ctx, timeout_ms, &ready_info) < 0) {
        camera_save_error(ctx, saved_error, sizeof(saved_error), &saved_errno);
        (void)camera_remote_acquisition_stop(ctx);
        (void)camera_disable_capture(ctx);
        goto fail_stop_pump;
    }

    if (ctx->dma_strided) {
        if (ready_info.width != expected_line_bytes ||
            ready_info.height != expected_height ||
            ready_info.frame_bytes != expected_frame_bytes) {
            camera_set_error(ctx,
                             "strict DMA start metadata mismatch for FPGA RGB8-to-RGB888 RTL: expected line=%u height=%u frame=%u; got line=%u height=%u frame=%u drop=%u",
                             expected_line_bytes, expected_height, expected_frame_bytes,
                             ready_info.width, ready_info.height,
                             ready_info.frame_bytes, ready_info.drop_count);
            errno = EPROTO;
            camera_save_error(ctx, saved_error, sizeof(saved_error), &saved_errno);
            (void)camera_remote_acquisition_stop(ctx);
            (void)camera_disable_capture(ctx);
            goto fail_stop_pump;
        }
        ctx->dma_src_stride_learned = expected_line_bytes;
        ctx->dma_drop_baseline = ready_info.drop_count;
    } else if (ready_info.drop_count != ctx->dma_drop_baseline ||
        ready_info.width != expected_line_bytes ||
        ready_info.height != expected_height ||
        ready_info.frame_bytes != expected_frame_bytes) {
        camera_set_error(ctx,
                         "strict DMA start metadata mismatch: expected line=%u height=%u frame=%u drop=0; got line=%u height=%u frame=%u drop=%u",
                         expected_line_bytes, expected_height, expected_frame_bytes,
                         ready_info.width, ready_info.height,
                         ready_info.frame_bytes, ready_info.drop_count);
        errno = EPROTO;
        camera_save_error(ctx, saved_error, sizeof(saved_error), &saved_errno);
        (void)camera_remote_acquisition_stop(ctx);
        (void)camera_disable_capture(ctx);
        goto fail_stop_pump;
    }

    if (info)
        *info = ready_info;
    return 0;

fail_stop_pump:
    camera_dma_stop_pump(ctx);
    camera_restore_error(ctx, saved_error, saved_errno);
    return -1;
}

int camera_dma_submit_ready(struct camera_context *ctx)
{
    if (!ctx || !ctx->dma_buffers) {
        errno = EINVAL;
        return -1;
    }
    pthread_cond_broadcast(&ctx->dma_cond);
    return 0;
}

static int camera_dma_start_pump(struct camera_context *ctx)
{
    unsigned int i;
    int ret;

    if (!ctx || !ctx->dma_sync_ready || !ctx->dma_lane_count) {
        errno = EINVAL;
        return -1;
    }

    pthread_mutex_lock(&ctx->dma_lock);
    if (ctx->dma_thread_active) {
        pthread_mutex_unlock(&ctx->dma_lock);
        return 0;
    }
    ctx->dma_thread_stop = 0;
    ctx->dma_thread_error = 0;
    ctx->dma_frame_waiter_active = 0;
    for (i = 0; i < ctx->dma_lane_count; i++) {
        struct camera_dma_worker_arg *wa = calloc(1, sizeof(*wa));
        if (!wa) {
            ret = ENOMEM;
            goto fail;
        }
        wa->ctx = ctx;
        wa->lane = i;
        ret = pthread_create(&ctx->dma_lanes[i].thread, NULL,
                             camera_dma_sync_worker_main, wa);
        if (ret) {
            free(wa);
            goto fail;
        }
        ctx->dma_lanes[i].active = 1;
    }
    ctx->dma_thread_active = 1;
    pthread_mutex_unlock(&ctx->dma_lock);
    return 0;

fail:
    ctx->dma_thread_stop = 1;
    pthread_cond_broadcast(&ctx->dma_cond);
    pthread_mutex_unlock(&ctx->dma_lock);
    for (i = 0; i < ctx->dma_lane_count; i++) {
        if (ctx->dma_lanes[i].active) {
            (void)pthread_join(ctx->dma_lanes[i].thread, NULL);
            ctx->dma_lanes[i].active = 0;
        }
    }
    errno = ret;
    camera_set_error(ctx, "failed to start sync multi-queue DMA worker: %s", strerror(ret));
    return -1;
}

static void camera_dma_stop_pump(struct camera_context *ctx)
{
    unsigned int i;
    int join_needed = 0;

    if (!ctx || !ctx->dma_sync_ready)
        return;

    pthread_mutex_lock(&ctx->dma_lock);
    if (ctx->dma_thread_active) {
        ctx->dma_thread_stop = 1;
        join_needed = 1;
        pthread_cond_broadcast(&ctx->dma_cond);
    }
    pthread_mutex_unlock(&ctx->dma_lock);

    if (join_needed) {
        for (i = 0; i < ctx->dma_lane_count; i++) {
            if (ctx->dma_lanes[i].active) {
                (void)pthread_join(ctx->dma_lanes[i].thread, NULL);
                ctx->dma_lanes[i].active = 0;
            }
        }
        pthread_mutex_lock(&ctx->dma_lock);
        ctx->dma_thread_active = 0;
        ctx->dma_thread_stop = 0;
        ctx->dma_frame_waiter_active = 0;
        pthread_mutex_unlock(&ctx->dma_lock);
    }
}

static int camera_realtime_after_ms(struct timespec *ts, int timeout_ms)
{
    uint64_t nsec;

    if (clock_gettime(CLOCK_REALTIME, ts) < 0)
        return -1;
    if (timeout_ms < 0)
        timeout_ms = CAMERA_DEFAULT_TIMEOUT_MS;
    nsec = (uint64_t)ts->tv_nsec + (uint64_t)timeout_ms * 1000000ull;
    ts->tv_sec += (time_t)(nsec / 1000000000ull);
    ts->tv_nsec = (long)(nsec % 1000000000ull);
    return 0;
}

static int camera_dma_reap_frame_from_pump(struct camera_context *ctx, int timeout_ms,
                                           struct camera_frame_info *info,
                                           void **data, size_t *len)
{
    struct timespec abs;
    int buf_index;

    if (timeout_ms < 0)
        timeout_ms = CAMERA_DEFAULT_TIMEOUT_MS;
    if (camera_realtime_after_ms(&abs, timeout_ms) < 0)
        return -1;

    pthread_mutex_lock(&ctx->dma_lock);
    for (;;) {
        if (ctx->dma_ready_count)
            break;
        if (ctx->ddr && atomic_load_explicit(&ctx->ddr->sender_state, memory_order_acquire) == DDR_SENDER_DRAINED) {
            pthread_mutex_unlock(&ctx->dma_lock);
            errno = ECANCELED;
            return -1;
        }
        if (ctx->dma_thread_error) {
            pthread_mutex_unlock(&ctx->dma_lock);
            if (!ctx->last_error[0])
                camera_set_error(ctx, "DMA pump stopped before a frame was ready");
            errno = EIO;
            return -1;
        }
        if (!ctx->dma_thread_active) {
            pthread_mutex_unlock(&ctx->dma_lock);
            camera_set_error(ctx, "DMA pump thread is not running");
            errno = EIO;
            return -1;
        }
        if (timeout_ms == 0) {
            pthread_mutex_unlock(&ctx->dma_lock);
            camera_set_error(ctx, "no DMA frame is ready");
            errno = EAGAIN;
            return -1;
        }
        {
            int ret = pthread_cond_timedwait(&ctx->dma_cond, &ctx->dma_lock, &abs);
            if (ret == ETIMEDOUT) {
                pthread_mutex_unlock(&ctx->dma_lock);
                struct camera_csr_snapshot snap;
                memset(&snap, 0, sizeof(snap));
                (void)camera_read_csr_snapshot(ctx, &snap);
                camera_set_error(ctx,
                    "timed out waiting for DMA pump frame: prod=%u cons=%u status=0x%08x drop=%u reason=0x%08x hls_resync=%u host_prod=%u host_cons=%u host_stall=%u host_error=%u build=0x%08x",
                    snap.producer_idx, snap.consumer_idx, snap.status,
                    snap.drop_count, snap.diag.drop_reason, snap.hls_resync_count,
                    ctx->host_direct ? camera_read32(ctx, VIDEO_HOST_PRODUCER) : 0,
                    ctx->host_direct ? camera_read32(ctx, VIDEO_HOST_CONSUMER) : 0,
                    ctx->host_direct ? camera_read32(ctx, VIDEO_HOST_STALL) : 0,
                    ctx->host_direct ? camera_read32(ctx, VIDEO_HOST_ERROR) : 0,
                    ctx->hw_build_version);
                errno = ETIMEDOUT;
                return -1;
            }
            if (ret) {
                pthread_mutex_unlock(&ctx->dma_lock);
                errno = ret;
                camera_set_error(ctx, "DMA pump wait failed: %s", strerror(ret));
                return -1;
            }
        }
    }

    buf_index = camera_ready_pop(ctx);
    if (buf_index < 0 || (unsigned int)buf_index >= ctx->dma_buffer_count ||
        ctx->dma_buffers[buf_index].state != CAMERA_DMA_READY) {
        pthread_mutex_unlock(&ctx->dma_lock);
        camera_set_error(ctx, "invalid host-ready buffer index/state from DMA pump");
        errno = EIO;
        return -1;
    }

    ctx->dma_buffers[buf_index].state = CAMERA_DMA_CHECKED_OUT;
    *info = ctx->dma_buffers[buf_index].info;
    *data = ctx->dma_buffers[buf_index].ptr;
    *len = ctx->dma_buffers[buf_index].len;
    pthread_mutex_unlock(&ctx->dma_lock);
    return 0;
}

int camera_dma_reap_frame(struct camera_context *ctx, int timeout_ms,
                          struct camera_frame_info *info,
                          void **data, size_t *len)
{
    if (!ctx || !ctx->dma_buffers || !info || !data || !len) {
        errno = EINVAL;
        return -1;
    }

    if (!ctx->dma_thread_active) {
        if (camera_dma_start_pump(ctx) < 0)
            return -1;
    }
    return camera_dma_reap_frame_from_pump(ctx, timeout_ms, info, data, len);
}

static void camera_dma_recycle_buffer_locked(struct camera_context *ctx,
                                             unsigned int idx)
{
    struct camera_dma_buffer *buf = &ctx->dma_buffers[idx];

    buf->state = CAMERA_DMA_FREE;
    if(ctx->host_direct) {
        buf->direct_release_token=(uint64_t)buf->info.consumer_idx+1;
        camera_direct_reclaim_locked(ctx);
    }
    buf->op_id = 0;
    buf->len = 0;
    memset(&buf->info, 0, sizeof(buf->info));
}

/*
 * Return the newest frame which has already completed D2H DMA. Older host
 * frames are staging buffers, not DDR ownership tokens, so they can be safely
 * recycled here. If every submitted DMA has completed, also discard stale DDR
 * slots and retain producer-1: producer is advanced by the RTL only after the
 * complete frame has been committed and all AXI write responses arrived.
 * Never release an in-flight DMA slot and never select slot producer, which
 * may currently be written by the FPGA.
 */
int camera_dma_reap_latest_frame(struct camera_context *ctx, int timeout_ms,
                                 struct camera_frame_info *info,
                                 void **data, size_t *len,
                                 uint32_t *host_skipped,
                                 uint32_t *ddr_skipped)
{
    struct timespec abs;
    uint32_t host_drop = 0;
    uint32_t ddr_drop = 0;
    int selected = -1;

    if (host_skipped)
        *host_skipped = 0;
    if (ddr_skipped)
        *ddr_skipped = 0;
    if (!ctx || !ctx->dma_buffers || !info || !data || !len) {
        errno = EINVAL;
        return -1;
    }
    if (!ctx->dma_thread_active && camera_dma_start_pump(ctx) < 0)
        return -1;
    if (timeout_ms < 0)
        timeout_ms = CAMERA_DEFAULT_TIMEOUT_MS;
    if (camera_realtime_after_ms(&abs, timeout_ms) < 0)
        return -1;

    pthread_mutex_lock(&ctx->dma_lock);
    for (;;) {
        if (ctx->dma_ready_count)
            break;
        if (ctx->ddr && atomic_load_explicit(&ctx->ddr->sender_state, memory_order_acquire) == DDR_SENDER_DRAINED) {
            pthread_mutex_unlock(&ctx->dma_lock);
            errno = ECANCELED;
            return -1;
        }
        if (ctx->dma_thread_error) {
            pthread_mutex_unlock(&ctx->dma_lock);
            if (!ctx->last_error[0])
                camera_set_error(ctx, "DMA pump stopped before a frame was ready");
            errno = EIO;
            return -1;
        }
        if (!ctx->dma_thread_active) {
            pthread_mutex_unlock(&ctx->dma_lock);
            camera_set_error(ctx, "DMA pump thread is not running");
            errno = EIO;
            return -1;
        }
        if (timeout_ms == 0) {
            pthread_mutex_unlock(&ctx->dma_lock);
            camera_set_error(ctx, "no DMA frame is ready");
            errno = EAGAIN;
            return -1;
        }
        {
            int ret = pthread_cond_timedwait(&ctx->dma_cond, &ctx->dma_lock, &abs);
            if (ret == ETIMEDOUT) {
                pthread_mutex_unlock(&ctx->dma_lock);
                camera_set_error(ctx, "timed out waiting for latest DMA frame");
                errno = ETIMEDOUT;
                return -1;
            }
            if (ret) {
                pthread_mutex_unlock(&ctx->dma_lock);
                errno = ret;
                camera_set_error(ctx, "latest DMA frame wait failed: %s", strerror(ret));
                return -1;
            }
        }
    }

    /* Include stale DDR slots skipped by the DMA workers before submit. */
    ddr_drop = ctx->dma_latest_ddr_skipped_pending;
    ctx->dma_latest_ddr_skipped_pending = 0;

    /* READY buffers are inserted in monotonically increasing DDR-index order,
     * therefore the final entry is the freshest completed host frame.
     */
    while (ctx->dma_ready_count) {
        int next = camera_ready_pop(ctx);

        if (next < 0 || (unsigned int)next >= ctx->dma_buffer_count ||
            ctx->dma_buffers[next].state != CAMERA_DMA_READY) {
            pthread_mutex_unlock(&ctx->dma_lock);
            camera_set_error(ctx, "invalid latest host-ready buffer index/state");
            errno = EIO;
            return -1;
        }
        if (selected >= 0) {
            camera_dma_recycle_buffer_locked(ctx, (unsigned int)selected);
            host_drop++;
        }
        selected = next;
    }

    /* Do not create a hole across an outstanding DMA. Once submit and DDR
     * release indices meet, all earlier slots are resident in host memory.
     */
    if (!ctx->ddr && !ctx->configured_driver_frame_dma && ctx->dma_inflight == 0u &&
        ctx->dma_next_submit_idx == ctx->dma_next_ddr_release_idx) {
        uint32_t producer = camera_read32(ctx, CAMERA_REG_PRODUCER_IDX);
        uint32_t pending = producer - ctx->dma_next_submit_idx;

        if (pending > ctx->ring_slots) {
            camera_dma_recycle_buffer_locked(ctx, (unsigned int)selected);
            pthread_cond_broadcast(&ctx->dma_cond);
            pthread_mutex_unlock(&ctx->dma_lock);
            camera_set_error(ctx,
                             "latest-frame DDR occupancy %u exceeds %u slots",
                             pending, ctx->ring_slots);
            errno = EOVERFLOW;
            return -1;
        }
        if (pending > 1u) {
            uint32_t newest_complete = producer - 1u;

            ddr_drop += newest_complete - ctx->dma_next_submit_idx;
            ctx->dma_next_submit_idx = newest_complete;
            ctx->dma_next_ddr_release_idx = newest_complete;
            camera_write32(ctx, CAMERA_REG_CONSUMER_IDX, newest_complete);
            camera_mmio_flush(ctx);
        }
    }

    ctx->dma_buffers[selected].state = CAMERA_DMA_CHECKED_OUT;
    *info = ctx->dma_buffers[selected].info;
    *data = ctx->dma_buffers[selected].ptr;
    *len = ctx->dma_buffers[selected].len;
    if (host_skipped)
        *host_skipped = host_drop;
    if (ddr_skipped)
        *ddr_skipped = ddr_drop;
    pthread_cond_broadcast(&ctx->dma_cond);
    pthread_mutex_unlock(&ctx->dma_lock);
    return 0;
}

int camera_dma_release_frame(struct camera_context *ctx,
                             const struct camera_frame_info *info)
{
    struct camera_dma_buffer *buf;
    uint32_t idx;

    if (!ctx || !info) {
        errno = EINVAL;
        return -1;
    }

    idx = info->dma_buffer_index;
    if (!ctx->dma_buffers || idx == CAMERA_DMA_BUFFER_INVALID ||
        idx >= ctx->dma_buffer_count)
        return camera_release_frame(ctx, info);

    pthread_mutex_lock(&ctx->dma_lock);
    buf = &ctx->dma_buffers[idx];
    if (buf->state != CAMERA_DMA_CHECKED_OUT ||
        buf->info.consumer_idx != info->consumer_idx) {
        pthread_mutex_unlock(&ctx->dma_lock);
        camera_set_error(ctx,
                         "invalid host-buffer release: buffer=%u state=%d frame=%u expected=%u",
                         idx, buf->state, info->consumer_idx,
                         buf->info.consumer_idx);
        errno = EINVAL;
        return -1;
    }

    camera_dma_recycle_buffer_locked(ctx, idx);
    pthread_cond_broadcast(&ctx->dma_cond);
    pthread_mutex_unlock(&ctx->dma_lock);
    return 0;
}

const char *camera_last_error(const struct camera_context *ctx)
{
    if (!ctx || !ctx->last_error[0])
        return "";
    return ctx->last_error;
}
