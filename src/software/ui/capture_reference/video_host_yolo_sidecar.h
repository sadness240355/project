#ifndef VIDEO_HOST_YOLO_SIDECAR_H
#define VIDEO_HOST_YOLO_SIDECAR_H
#include "edev_sgdma_driver.h"
#include "../src/video_host_direct.h"
#include <pthread.h>
#include <sys/ioctl.h>
struct host_yolo_sidecar {
    int fd;
    uint8_t *buffer;
    size_t buffer_bytes;
    uint32_t frame_bytes, width, height, csr_base;
    uint32_t producer, demand, last_request;
    struct frame_shm_publisher *publisher;
    pthread_t thread;
    _Atomic int stop;
    _Atomic uint64_t published;
    int started, fault;
};
static int host_yolo_reg(struct host_yolo_sidecar *s, uint32_t reg,
                         uint32_t *value, int write)
{
    struct reconic_bar2_access access = {s->csr_base + reg, *value};
    if (ioctl(s->fd, write ? RECONIC_DMA_IOC_BAR2_WRITE :
                         RECONIC_DMA_IOC_BAR2_READ, &access) < 0) return -1;
    if (!write) *value = access.value;
    return 0;
}
static int host_yolo_write(struct host_yolo_sidecar *s, uint32_t reg, uint32_t value)
{ return host_yolo_reg(s, reg, &value, 1); }
static int host_yolo_read(struct host_yolo_sidecar *s, uint32_t reg, uint32_t *value)
{ return host_yolo_reg(s, reg, value, 0); }
static void *host_yolo_worker(void *arg)
{
    struct host_yolo_sidecar *s = arg;
    struct video_host_footer *footer = (void *)(s->buffer +
                                             ((s->frame_bytes + 63u) & ~63u));
    unsigned rejected = 0;
    while (!atomic_load_explicit(&s->stop, memory_order_acquire)) {
        uint32_t request = atomic_load_explicit(&s->publisher->header->request_seq,
                                                memory_order_acquire);
        if (request == s->last_request) { usleep(100); continue; }
        atomic_store_explicit((_Atomic uint64_t *)&footer->token, 0,
                              memory_order_release);
        if (host_yolo_write(s, VIDEO_HOST_DEMAND, ++s->demand) < 0) {
            s->fault = 1; break;
        }
        uint64_t deadline = monotonic_ns() + 2000000000ull;
        uint64_t token = (uint64_t)s->producer + 1u;
        while (!atomic_load_explicit(&s->stop, memory_order_acquire) &&
               atomic_load_explicit((_Atomic uint64_t *)&footer->token,
                                    memory_order_acquire) != token) {
            uint32_t error = 0;
            if (host_yolo_read(s, VIDEO_HOST_ERROR, &error) < 0 || error ||
                monotonic_ns() > deadline) {
                fprintf(stderr, "Error: YOLO direct-host DMA timeout/fault; coherent memory retained\n");
                s->fault = 1; break;
            }
            usleep(50);
        }
        if (s->fault || atomic_load_explicit(&s->stop, memory_order_acquire)) break;
        struct video_host_footer f;
        memcpy(&f, footer, sizeof(f));
        if (!video_host_footer_valid(&f, token, s->width, s->height,
                                     FRAME_SHM_PIXEL_RGB888, s->frame_bytes)) {
            fprintf(stderr, "Error: YOLO direct-host footer ABI invalid; coherent memory retained\n");
            s->fault = 1; break;
        }
        if ((f.flags & 3u) != 3u) {
            /* This optional branch may lose a frame under host congestion.
             * The DDR/RDMA frame stays valid; release only the host slot. */
            s->producer++;
            if (host_yolo_write(s, VIDEO_HOST_CONSUMER, s->producer) < 0) {
                s->fault = 1; break;
            }
            if (++rejected >= 8) {
                fprintf(stderr, "Error: eight consecutive YOLO direct-host frames rejected\n");
                s->fault = 1; break;
            }
            continue;
        }
        rejected = 0;
        request = atomic_load_explicit(&s->publisher->header->request_seq,
                                       memory_order_acquire);
        frame_shm_publish_letterbox_rgb(s->publisher, s->buffer, s->width,
                                        s->height, s->width * 3u, f.frame_id,
                                        monotonic_ns());
        atomic_store_explicit(&s->publisher->header->served_request_seq,
                              request, memory_order_release);
        frame_shm_notify(s->publisher->header);
        s->last_request = request;
        atomic_fetch_add_explicit(&s->published, 1, memory_order_relaxed);
        s->producer++;
        if (host_yolo_write(s, VIDEO_HOST_CONSUMER, s->producer) < 0) {
            s->fault = 1; break;
        }
    }
    return NULL;
}
/* Returns 1 for dual-path bit, 0 for an old bit, -1 on setup failure. */
static int host_yolo_start(struct host_yolo_sidecar *s, const char *ctrl_path,
                           uint32_t csr_base, uint32_t width, uint32_t height,
                           uint32_t frame_bytes, struct frame_shm_publisher *pub)
{
    struct reconic_dma_alloc_req allocation;
    uint32_t version = 0, verify = 0;
    memset(s, 0, sizeof(*s));
    s->fd = -1; s->csr_base = csr_base;
    s->width = width; s->height = height; s->frame_bytes = frame_bytes;
    s->publisher = pub;
    s->fd = open(ctrl_path, O_RDWR | O_CLOEXEC);
    if (s->fd < 0) return -1;
    if (host_yolo_read(s, CAMERA_REG_BUILD_VERSION, &version) < 0) return -1;
    if (version < 0x00030027u) { close(s->fd); s->fd = -1; return 0; }
    allocation.size = (uint64_t)((frame_bytes + 63u) & ~63u) + 64u;
    allocation.dma_addr = allocation.mmap_offset = 0;
    if (ioctl(s->fd, RECONIC_DMA_IOC_ALLOC_COHERENT, &allocation) < 0 ||
        allocation.mmap_offset != 0 || allocation.size < (uint64_t)frame_bytes + 64u ||
        allocation.dma_addr > UINT64_C(0xffffffffff) - allocation.size)
        return -1;
    s->buffer_bytes = (size_t)allocation.size;
    s->buffer = mmap(NULL, s->buffer_bytes, PROT_READ | PROT_WRITE,
                     MAP_SHARED, s->fd, 0);
    if (s->buffer == MAP_FAILED) { s->buffer = NULL; return -1; }
    memset(s->buffer, 0, s->buffer_bytes);
    if (host_yolo_write(s, VIDEO_HOST_CONTROL, 0) < 0 ||
        host_yolo_write(s, VIDEO_HOST_SLOTS, 1) < 0 ||
        host_yolo_write(s, VIDEO_HOST_CONSUMER, 0) < 0 ||
        host_yolo_write(s, VIDEO_HOST_DEMAND, 0) < 0 ||
        host_yolo_write(s, VIDEO_HOST_ADDRESSES, (uint32_t)allocation.dma_addr) < 0 ||
        host_yolo_write(s, VIDEO_HOST_ADDRESSES + 4,
                        (uint32_t)(allocation.dma_addr >> 32)) < 0 ||
        host_yolo_write(s, VIDEO_HOST_CONTROL, 1) < 0 ||
        host_yolo_read(s, VIDEO_HOST_CONTROL, &verify) < 0 || !(verify & 1u))
        return -1;
    if (pthread_create(&s->thread, NULL, host_yolo_worker, s) != 0) return -1;
    s->started = 1;
    fprintf(stderr, "HOST_YOLO enabled=1 source=ISP-PCIe direct slot=1 build=0x%08x\n", version);
    return 1;
}
static void host_yolo_stop(struct host_yolo_sidecar *s)
{
    if (s->started) {
        atomic_store_explicit(&s->stop, 1, memory_order_release);
        pthread_join(s->thread, NULL);
        uint32_t status = 0;
        uint64_t deadline = monotonic_ns() + 2000000000ull;
        while (host_yolo_read(s, VIDEO_HOST_STATUS, &status) == 0 &&
               !(status & 1u) && monotonic_ns() < deadline) usleep(1000);
        if (!(status & 1u)) {
            fprintf(stderr, "FAIL: YOLO direct-host DMA outstanding; safety hold PID %ld. Reset FPGA before killing.\n", (long)getpid());
            for (;;) pause();
        }
        (void)host_yolo_write(s, VIDEO_HOST_CONTROL, 0);
    }
    if (s->buffer) munmap(s->buffer, s->buffer_bytes);
    if (s->fd >= 0) close(s->fd);
    s->fd = -1; s->buffer = NULL; s->started = 0;
}
#endif
