/* SPDX-License-Identifier: MIT */
/*
 * BayerRG8 Camera Link viewer for Jetson/Linux.
 *
 * Data path:
 *   camera BayerRG8 -> FPGA capture -> HLS bilinear demosaic -> RGB888
 *   -> ISP output fork -> direct PCIe host writer -> host buffer -> GL texture.
 *   The video branch writes FPGA DDR for ERNIC. Older bitstreams keep DDR D2H.
 *
 * Demosaicing is performed in the FPGA.  The CPU only transfers each complete
 * RGB888 frame to the OpenGL ES texture; there is no per-pixel CPU conversion.
 */

#include "edev_camera_io.h"
#include "../src/video_source_park.h"
#include "../src/video_display.h"

#include <EGL/egl.h>
#include <GLES2/gl2.h>
#include <X11/Xlib.h>

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <linux/futex.h>
#include <math.h>
#include <signal.h>
#include <stdatomic.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

#define FRAME_SHM_PIXEL_BAYER 0x42474752u
#define VIEWER_DEFAULT_WIDTH 2432u
#define VIEWER_DEFAULT_HEIGHT 2048u
#define VIEWER_DEFAULT_FPS 140.0
#define VIEWER_DEFAULT_BUFFERS 16u
#define VIEWER_DEFAULT_DMA_QUEUES 8u
#define VIEWER_MAX_WIDTH 2432u
#define VIEWER_MAX_HEIGHT 2048u
#define VIEWER_RING_SLOTS CAMERA_DEFAULT_RING_SLOTS


/*
 * Cross-process, latest-frame RGB888 mailbox consumed by track_shm.py.
 * The first page is a fixed ABI header; two complete frames follow it. A
 * per-slot seqlock prevents the reader from accepting a slot while it is being
 * overwritten. Keep these offsets in sync with SHM_HEADER_STRUCT in Python.
 */
#define FRAME_SHM_MAGIC        0x4d485343u /* bytes "CSHM" */
#define FRAME_SHM_VERSION      3u
#define FRAME_SHM_VERSION_DEMAND 4u
#define FRAME_SHM_HEADER_BYTES 4096u
#define FRAME_SHM_PIXEL_RGB888 0x33424752u /* little-endian "RGB3" */
#define FRAME_SHM_PIXEL_BGR888 0x33524742u /* little-endian "BGR3" */
#define FRAME_SHM_BUFFERS      2u

#define BOX_SHM_MAGIC          0x584f4259u /* little-endian bytes "YBOX" */
#define BOX_SHM_VERSION        1u
#define BOX_SHM_HEADER_BYTES   4096u
#define BOX_SHM_BUFFERS        2u
#define BOX_SHM_MAX_BOXES      256u
#define BOX_SHM_STALE_NS       500000000ull
#define BOX_SHM_REOPEN_NS      2000000000ull

struct frame_shm_header {
    uint32_t magic;
    uint32_t version;
    uint32_t header_bytes;
    uint32_t pixel_format;
    uint32_t width;
    uint32_t height;
    uint32_t stride_bytes;
    uint32_t frame_bytes;
    uint32_t buffer_count;
    uint32_t reserved0;
    _Atomic uint64_t publish_seq;
    _Atomic uint32_t active_index;
    uint32_t reserved1;
    _Atomic uint64_t slot_guard[FRAME_SHM_BUFFERS];
    uint64_t frame_seq[FRAME_SHM_BUFFERS];
    uint64_t timestamp_ns[FRAME_SHM_BUFFERS];
    uint32_t source_width;
    uint32_t source_height;
    uint32_t content_x;
    uint32_t content_y;
    uint32_t content_width;
    uint32_t content_height;
    _Atomic uint32_t notify_seq;
    _Atomic uint32_t request_seq;
    _Atomic uint32_t served_request_seq;
};

_Static_assert(offsetof(struct frame_shm_header, request_seq) == 132,
               "shared-memory request ABI changed");
_Static_assert(offsetof(struct frame_shm_header, served_request_seq) == 136,
               "shared-memory served-request ABI changed");
_Static_assert(offsetof(struct frame_shm_header, publish_seq) == 40,
               "shared-memory publish_seq ABI changed");
_Static_assert(offsetof(struct frame_shm_header, active_index) == 48,
               "shared-memory active_index ABI changed");
_Static_assert(offsetof(struct frame_shm_header, slot_guard) == 56,
               "shared-memory slot_guard ABI changed");
_Static_assert(offsetof(struct frame_shm_header, frame_seq) == 72,
               "shared-memory frame_seq ABI changed");
_Static_assert(offsetof(struct frame_shm_header, timestamp_ns) == 88,
               "shared-memory timestamp ABI changed");
_Static_assert(offsetof(struct frame_shm_header, source_width) == 104,
               "shared-memory source geometry ABI changed");
_Static_assert(offsetof(struct frame_shm_header, notify_seq) == 128,
               "shared-memory notification ABI changed");
_Static_assert(sizeof(struct frame_shm_header) <= FRAME_SHM_HEADER_BYTES,
               "shared-memory header exceeds reserved page");

struct box_shm_record {
    float x1;
    float y1;
    float x2;
    float y2;
    float confidence;
    int32_t class_id;
    int32_t track_id;
    int32_t reserved;
};

struct box_shm_header {
    uint32_t magic;
    uint32_t version;
    uint32_t header_bytes;
    uint32_t record_bytes;
    uint32_t max_boxes;
    uint32_t buffer_count;
    uint32_t source_width;
    uint32_t source_height;
    _Atomic uint64_t publish_seq;
    _Atomic uint32_t active_index;
    uint32_t reserved0;
    _Atomic uint64_t slot_guard[BOX_SHM_BUFFERS];
    uint64_t frame_seq[BOX_SHM_BUFFERS];
    uint64_t timestamp_ns[BOX_SHM_BUFFERS];
    uint32_t box_count[BOX_SHM_BUFFERS];
    uint32_t reserved1[6];
};

_Static_assert(sizeof(struct box_shm_record) == 32,
               "box shared-memory record ABI changed");
_Static_assert(offsetof(struct box_shm_header, publish_seq) == 32,
               "box shared-memory publish_seq ABI changed");
_Static_assert(offsetof(struct box_shm_header, slot_guard) == 48,
               "box shared-memory slot_guard ABI changed");
_Static_assert(offsetof(struct box_shm_header, box_count) == 96,
               "box shared-memory box_count ABI changed");
_Static_assert(sizeof(struct box_shm_header) == 128,
               "box shared-memory header ABI changed");
_Static_assert(sizeof(struct box_shm_header) <= BOX_SHM_HEADER_BYTES,
               "box shared-memory header exceeds reserved page");

struct frame_shm_publisher {
    int fd;
    void *mapping;
    size_t mapping_bytes;
    struct frame_shm_header *header;
    uint8_t *frames[FRAME_SHM_BUFFERS];
    uint32_t *map_x;
    uint32_t *map_y;
    char *name;
};

struct box_shm_reader {
    int fd;
    void *mapping;
    size_t mapping_bytes;
    const struct box_shm_header *header;
    const struct box_shm_record *slots[BOX_SHM_BUFFERS];
    struct box_shm_record cached[BOX_SHM_MAX_BOXES];
    char *name;
    uint64_t last_open_attempt_ns;
    uint64_t last_publish_seq;
    uint64_t last_update_ns;
    uint64_t frame_seq;
    uint64_t timestamp_ns;
    uint32_t count;
    uint32_t source_width;
    uint32_t source_height;
    uint64_t fps_window_started_ns;
    uint32_t fps_updates;
    double detect_fps;
};

static volatile sig_atomic_t running = 1;

struct display_state {
    Display *xdpy;
    Window win;
    EGLDisplay edpy;
    EGLSurface surface;
    EGLContext context;
    GLuint program;
    GLuint texture;
    GLint a_pos;
    GLint a_uv;
    GLint u_tex;
    GLuint overlay_program;
    GLint overlay_a_pos;
    GLint overlay_u_color;
    GLint overlay_u_point_size;
    uint32_t image_width;
    uint32_t image_height;
};

static void stop_handler(int signo)
{
    (void)signo;
    running = 0;
}

static uint64_t monotonic_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

static void frame_shm_notify(struct frame_shm_header *header)
{
    if (!header)
        return;
    atomic_fetch_add_explicit(&header->notify_seq, 1u, memory_order_release);
#ifdef SYS_futex
    (void)syscall(SYS_futex, (uint32_t *)&header->notify_seq,
                  FUTEX_WAKE, INT_MAX, NULL, NULL, 0);
#endif
}


static int frame_shm_name_valid(const char *name)
{
    const char *p;

    if (!name || name[0] != '/' || !name[1])
        return 0;
    /* POSIX shm names used here are a leading slash plus one path component. */
    for (p = name + 1; *p; p++) {
        if (*p == '/')
            return 0;
    }
    return 1;
}

static void frame_shm_destroy(struct frame_shm_publisher *pub)
{
    if (!pub)
        return;
    if (pub->mapping && pub->mapping != MAP_FAILED)
        munmap(pub->mapping, pub->mapping_bytes);
    if (pub->fd >= 0)
        close(pub->fd);
    free(pub->map_x);
    free(pub->map_y);
    if (pub->name) {
        /* Existing readers retain their mapping but no new reader opens stale data. */
        shm_unlink(pub->name);
        free(pub->name);
    }
    memset(pub, 0, sizeof(*pub));
    pub->fd = -1;
}

static int frame_shm_init(struct frame_shm_publisher *pub, const char *name,
                          uint32_t width, uint32_t height,
                          uint32_t stride_bytes, uint32_t frame_bytes,
                          uint32_t pixel_format,
                          uint32_t source_width, uint32_t source_height,
                          uint32_t content_x, uint32_t content_y,
                          uint32_t content_width, uint32_t content_height,
                          int on_demand)
{
    size_t mapping_bytes;
    unsigned int i;
    int needs_resample;

    if (!pub || !frame_shm_name_valid(name) || !width || !height ||
        !stride_bytes || frame_bytes != stride_bytes * (uint64_t)height ||
        (pixel_format != FRAME_SHM_PIXEL_RGB888 &&
         pixel_format != FRAME_SHM_PIXEL_BGR888 && pixel_format != FRAME_SHM_PIXEL_BAYER) ||
        !source_width || !source_height || !content_width || !content_height ||
        content_x + (uint64_t)content_width > width ||
        content_y + (uint64_t)content_height > height) {
        errno = EINVAL;
        return -1;
    }

    needs_resample = source_width != width || source_height != height ||
                     content_x != 0 || content_y != 0 ||
                     content_width != width || content_height != height;

    memset(pub, 0, sizeof(*pub));
    pub->fd = -1;
    pub->name = strdup(name);
    if (!pub->name)
        return -1;

    mapping_bytes = FRAME_SHM_HEADER_BYTES +
                    (size_t)frame_bytes * FRAME_SHM_BUFFERS;
    if (shm_unlink(name) < 0 && errno != ENOENT)
        goto fail;
    pub->fd = shm_open(name, O_CREAT | O_EXCL | O_RDWR, 0660);
    if (pub->fd < 0)
        goto fail;
    /* UI readers use this lifetime lock to distinguish live frames from a
     * stale mailbox left behind by a terminated camera process. */
    if (flock(pub->fd, LOCK_EX | LOCK_NB) < 0)
        goto fail;
    if (ftruncate(pub->fd, (off_t)mapping_bytes) < 0)
        goto fail;
    /*
     * The camera process commonly runs as root while YOLO runs as nvidia.
     * Readers need no write access; make the mailbox readable without a
     * manual chmod after every camera restart.
     */
    if (fchmod(pub->fd, on_demand ? 0600 : 0644) < 0)
        goto fail;
    pub->mapping = mmap(NULL, mapping_bytes, PROT_READ | PROT_WRITE,
                        MAP_SHARED, pub->fd, 0);
    if (pub->mapping == MAP_FAILED)
        goto fail;

    pub->mapping_bytes = mapping_bytes;
    memset(pub->mapping, 0, mapping_bytes);
    pub->header = (struct frame_shm_header *)pub->mapping;
    pub->header->magic = FRAME_SHM_MAGIC;
    pub->header->version = on_demand ? FRAME_SHM_VERSION_DEMAND : FRAME_SHM_VERSION;
    pub->header->header_bytes = FRAME_SHM_HEADER_BYTES;
    pub->header->pixel_format = pixel_format;
    pub->header->width = width;
    pub->header->height = height;
    pub->header->stride_bytes = stride_bytes;
    pub->header->frame_bytes = frame_bytes;
    pub->header->buffer_count = FRAME_SHM_BUFFERS;
    pub->header->source_width = source_width;
    pub->header->source_height = source_height;
    pub->header->content_x = content_x;
    pub->header->content_y = content_y;
    pub->header->content_width = content_width;
    pub->header->content_height = content_height;
    atomic_init(&pub->header->publish_seq, 0);
    atomic_init(&pub->header->active_index, 0);
    atomic_init(&pub->header->notify_seq, 0);
    atomic_init(&pub->header->request_seq, 0);
    atomic_init(&pub->header->served_request_seq, 0);
    for (i = 0; i < FRAME_SHM_BUFFERS; i++) {
        atomic_init(&pub->header->slot_guard[i], 0);
        pub->frames[i] = (uint8_t *)pub->mapping + FRAME_SHM_HEADER_BYTES +
                         (size_t)i * frame_bytes;
    }
    if (needs_resample) {
        /* Geometry is fixed for the mailbox lifetime, so padding never needs
         * to be cleared again. Every publish overwrites the complete content
         * rectangle and leaves these preinitialized letterbox margins intact. */
        for (i = 0; i < FRAME_SHM_BUFFERS; i++)
            memset(pub->frames[i], 114, frame_bytes);
        pub->map_x = calloc(content_width, sizeof(*pub->map_x));
        pub->map_y = calloc(content_height, sizeof(*pub->map_y));
        if (!pub->map_x || !pub->map_y)
            goto fail;
        for (i = 0; i < content_width; i++) {
            uint64_t mapped = ((uint64_t)(2u * i + 1u) * source_width) /
                              (2ull * content_width);
            pub->map_x[i] = mapped < source_width ?
                            (uint32_t)mapped : source_width - 1u;
        }
        for (i = 0; i < content_height; i++) {
            uint64_t mapped = ((uint64_t)(2u * i + 1u) * source_height) /
                              (2ull * content_height);
            pub->map_y[i] = mapped < source_height ?
                            (uint32_t)mapped : source_height - 1u;
        }
    }
    return 0;

fail:
    frame_shm_destroy(pub);
    return -1;
}

static void frame_shm_publish(struct frame_shm_publisher *pub,
                              const void *rgb, size_t len,
                              uint64_t frame_seq, uint64_t timestamp_ns)
{
    struct frame_shm_header *h;
    uint32_t active, index;
    uint64_t guard;

    if (!pub || !pub->header || !rgb)
        return;
    h = pub->header;
    if (len < h->frame_bytes)
        return;

    active = atomic_load_explicit(&h->active_index, memory_order_acquire);
    index = (active ^ 1u) & 1u;
    guard = atomic_load_explicit(&h->slot_guard[index], memory_order_relaxed);
    if (guard & 1u)
        guard++;

    /* Odd means writer owns this slot; even means a complete frame is visible. */
    atomic_store_explicit(&h->slot_guard[index], guard + 1u,
                          memory_order_seq_cst);
    atomic_thread_fence(memory_order_seq_cst);
    memcpy(pub->frames[index], rgb, h->frame_bytes);
    h->frame_seq[index] = frame_seq;
    h->timestamp_ns[index] = timestamp_ns;
    atomic_thread_fence(memory_order_release);
    atomic_store_explicit(&h->slot_guard[index], guard + 2u,
                          memory_order_release);
    atomic_store_explicit(&h->active_index, index, memory_order_release);
    atomic_fetch_add_explicit(&h->publish_seq, 1u, memory_order_release);
    frame_shm_notify(h);
}

/*
 * Publish a detector-only RGB888 image. The camera-sized RGB frame remains on
 * the direct OpenGL path; only this small, aspect-preserving letterbox crosses
 * into Python. Center sampling is exact for the common 1920x1080 -> 640x360
 * 3:1 reduction. Keeping RGB order lets the Python fast path submit a tensor
 * directly without an RGB/BGR/RGB round trip.
 */
static void frame_shm_publish_letterbox_rgb(struct frame_shm_publisher *pub,
                                            const uint8_t *rgb,
                                            uint32_t source_width,
                                            uint32_t source_height,
                                            uint32_t source_stride,
                                            uint64_t frame_seq,
                                            uint64_t timestamp_ns)
{
    struct frame_shm_header *h;
    uint32_t active, index, dx, dy;
    uint64_t guard;
    uint8_t *dst;

    if (!pub || !pub->header || !rgb)
        return;
    h = pub->header;
    if (h->pixel_format != FRAME_SHM_PIXEL_RGB888 ||
        h->source_width != source_width || h->source_height != source_height ||
        source_stride < source_width * 3u || !pub->map_x || !pub->map_y)
        return;

    active = atomic_load_explicit(&h->active_index, memory_order_acquire);
    index = (active ^ 1u) & 1u;
    guard = atomic_load_explicit(&h->slot_guard[index], memory_order_relaxed);
    if (guard & 1u)
        guard++;
    atomic_store_explicit(&h->slot_guard[index], guard + 1u,
                          memory_order_seq_cst);
    atomic_thread_fence(memory_order_seq_cst);

    dst = pub->frames[index];
    for (dy = 0; dy < h->content_height; dy++) {
        uint32_t sy = pub->map_y[dy];
        uint8_t *dst_row = dst + (size_t)(h->content_y + dy) * h->stride_bytes +
                           (size_t)h->content_x * 3u;
        const uint8_t *src_row;

        src_row = rgb + (size_t)sy * source_stride;
        for (dx = 0; dx < h->content_width; dx++) {
            uint32_t sx = pub->map_x[dx];
            const uint8_t *s;
            uint8_t *d;

            s = src_row + (size_t)sx * 3u;
            d = dst_row + (size_t)dx * 3u;
            d[0] = s[0];
            d[1] = s[1];
            d[2] = s[2];
        }
    }

    h->frame_seq[index] = frame_seq;
    h->timestamp_ns[index] = timestamp_ns;
    atomic_thread_fence(memory_order_release);
    atomic_store_explicit(&h->slot_guard[index], guard + 2u,
                          memory_order_release);
    atomic_store_explicit(&h->active_index, index, memory_order_release);
    atomic_fetch_add_explicit(&h->publish_seq, 1u, memory_order_release);
    frame_shm_notify(h);
}

#include "video_host_yolo_sidecar.h"

static void box_shm_close_mapping(struct box_shm_reader *reader)
{
    if (!reader)
        return;
    if (reader->mapping && reader->mapping != MAP_FAILED)
        munmap(reader->mapping, reader->mapping_bytes);
    if (reader->fd >= 0)
        close(reader->fd);
    reader->fd = -1;
    reader->mapping = NULL;
    reader->mapping_bytes = 0;
    reader->header = NULL;
    reader->slots[0] = NULL;
    reader->slots[1] = NULL;
    reader->last_publish_seq = 0;
    reader->last_update_ns = 0;
    reader->frame_seq = 0;
    reader->timestamp_ns = 0;
    reader->count = 0;
    reader->fps_window_started_ns = 0;
    reader->fps_updates = 0;
    reader->detect_fps = 0.0;
}

static void box_shm_destroy(struct box_shm_reader *reader)
{
    if (!reader)
        return;
    box_shm_close_mapping(reader);
    free(reader->name);
    memset(reader, 0, sizeof(*reader));
    reader->fd = -1;
}

static int box_shm_init(struct box_shm_reader *reader, const char *name,
                        uint32_t source_width, uint32_t source_height)
{
    if (!reader || !frame_shm_name_valid(name) ||
        !source_width || !source_height) {
        errno = EINVAL;
        return -1;
    }
    memset(reader, 0, sizeof(*reader));
    reader->fd = -1;
    reader->name = strdup(name);
    if (!reader->name)
        return -1;
    reader->source_width = source_width;
    reader->source_height = source_height;
    return 0;
}

static int box_shm_try_open(struct box_shm_reader *reader, uint64_t now_ns)
{
    struct stat st;
    const struct box_shm_header *h;
    size_t required;
    unsigned int i;

    if (!reader || !reader->name || reader->header)
        return reader && reader->header ? 0 : -1;
    if (reader->last_open_attempt_ns &&
        now_ns - reader->last_open_attempt_ns < 250000000ull)
        return -1;
    reader->last_open_attempt_ns = now_ns;
    reader->fd = shm_open(reader->name, O_RDONLY, 0);
    if (reader->fd < 0)
        return -1;
    if (fstat(reader->fd, &st) < 0 ||
        st.st_size < (off_t)sizeof(struct box_shm_header))
        goto fail;
    reader->mapping_bytes = (size_t)st.st_size;
    reader->mapping = mmap(NULL, reader->mapping_bytes, PROT_READ,
                           MAP_SHARED, reader->fd, 0);
    if (reader->mapping == MAP_FAILED)
        goto fail;
    h = (const struct box_shm_header *)reader->mapping;
    required = BOX_SHM_HEADER_BYTES +
               (size_t)BOX_SHM_BUFFERS * BOX_SHM_MAX_BOXES *
               sizeof(struct box_shm_record);
    if (h->magic != BOX_SHM_MAGIC || h->version != BOX_SHM_VERSION ||
        h->header_bytes != BOX_SHM_HEADER_BYTES ||
        h->record_bytes != sizeof(struct box_shm_record) ||
        h->max_boxes != BOX_SHM_MAX_BOXES ||
        h->buffer_count != BOX_SHM_BUFFERS ||
        h->source_width != reader->source_width ||
        h->source_height != reader->source_height ||
        reader->mapping_bytes < required) {
        errno = EPROTO;
        goto fail;
    }
    reader->header = h;
    for (i = 0; i < BOX_SHM_BUFFERS; i++)
        reader->slots[i] = (const struct box_shm_record *)
            ((const uint8_t *)reader->mapping + BOX_SHM_HEADER_BYTES +
             (size_t)i * BOX_SHM_MAX_BOXES * sizeof(struct box_shm_record));
    fprintf(stderr,
            "YOLO overlay connected: name=%s source=%ux%u max_boxes=%u\n",
            reader->name, reader->source_width, reader->source_height,
            BOX_SHM_MAX_BOXES);
    return 0;

fail:
    box_shm_close_mapping(reader);
    return -1;
}

static void box_shm_refresh(struct box_shm_reader *reader, uint64_t now_ns)
{
    const struct box_shm_header *h;
    uint64_t publish_seq, guard_before, guard_after;
    uint32_t index, count;

    if (!reader || !reader->name)
        return;
    if (!reader->header) {
        (void)box_shm_try_open(reader, now_ns);
        if (!reader->header)
            return;
    }
    h = reader->header;
    publish_seq = atomic_load_explicit(&h->publish_seq, memory_order_acquire);
    if (!publish_seq || publish_seq == reader->last_publish_seq) {
        if (reader->last_update_ns &&
            now_ns - reader->last_update_ns > BOX_SHM_STALE_NS) {
            reader->count = 0;
            reader->detect_fps = 0.0;
        }
        if (reader->last_update_ns &&
            now_ns - reader->last_update_ns > BOX_SHM_REOPEN_NS)
            box_shm_close_mapping(reader);
        return;
    }
    index = atomic_load_explicit(&h->active_index, memory_order_acquire);
    if (index >= BOX_SHM_BUFFERS)
        return;
    guard_before = atomic_load_explicit(&h->slot_guard[index],
                                        memory_order_acquire);
    if (guard_before & 1u)
        return;
    count = h->box_count[index];
    if (count > BOX_SHM_MAX_BOXES)
        return;
    memcpy(reader->cached, reader->slots[index],
           (size_t)count * sizeof(struct box_shm_record));
    atomic_thread_fence(memory_order_acquire);
    guard_after = atomic_load_explicit(&h->slot_guard[index],
                                       memory_order_acquire);
    if (guard_before != guard_after || (guard_after & 1u))
        return;
    reader->frame_seq = h->frame_seq[index];
    reader->timestamp_ns = h->timestamp_ns[index];
    reader->count = count;
    reader->last_publish_seq = publish_seq;
    reader->last_update_ns = now_ns;
    if (!reader->fps_window_started_ns)
        reader->fps_window_started_ns = now_ns;
    reader->fps_updates++;
    if (now_ns - reader->fps_window_started_ns >= 1000000000ull) {
        reader->detect_fps =
            (double)reader->fps_updates * 1000000000.0 /
            (double)(now_ns - reader->fps_window_started_ns);
        reader->fps_window_started_ns = now_ns;
        reader->fps_updates = 0;
    }
}

static int parse_u32(const char *s, uint32_t *out)
{
    char *end = NULL;
    unsigned long v;

    errno = 0;
    v = strtoul(s, &end, 0);
    if (errno || !s || end == s || *end || v > UINT32_MAX)
        return -1;
    *out = (uint32_t)v;
    return 0;
}

static int parse_u64(const char *s, uint64_t *out)
{
    char *end = NULL;
    unsigned long long v;

    errno = 0;
    v = strtoull(s, &end, 0);
    if (errno || !s || end == s || *end)
        return -1;
    *out = (uint64_t)v;
    return 0;
}

static int parse_double(const char *s, double *out, int allow_zero)
{
    char *end = NULL;
    double v;

    errno = 0;
    v = strtod(s, &end);
    if (errno || !s || end == s || *end || !isfinite(v) ||
        (allow_zero ? v < 0.0 : v <= 0.0))
        return -1;
    *out = v;
    return 0;
}

static int parse_uq12(const char *s, uint16_t *out)
{
    char *end = NULL;
    double v;
    long scaled;

    errno = 0;
    v = strtod(s, &end);
    if (errno || !s || end == s || *end || !isfinite(v) ||
        v < 0.0 || v > (65535.0 / 4096.0))
        return -1;
    scaled = (long)(v * 4096.0 + 0.5);
    if (scaled < 0 || scaled > 65535)
        return -1;
    *out = (uint16_t)scaled;
    return 0;
}

static int parse_ccm_q12(const char *s, int16_t out[9])
{
    const char *p = s;
    unsigned int i;

    if (!s)
        return -1;
    for (i = 0; i < 9; i++) {
        char *end = NULL;
        double v;
        long scaled;

        errno = 0;
        v = strtod(p, &end);
        if (errno || end == p || !isfinite(v) || v < -8.0 ||
            v > (32767.0 / 4096.0))
            return -1;
        scaled = (long)(v * 4096.0 + (v >= 0.0 ? 0.5 : -0.5));
        if (scaled < -32768 || scaled > 32767)
            return -1;
        out[i] = (int16_t)scaled;
        if (i != 8) {
            if (*end != ',')
                return -1;
            p = end + 1;
        } else if (*end) {
            return -1;
        }
    }
    return 0;
}

static double q12_to_double(int32_t value)
{
    return (double)value / 4096.0;
}

static int parse_size(const char *s, uint32_t *width, uint32_t *height)
{
    char *end = NULL;
    unsigned long w, h;

    errno = 0;
    w = strtoul(s, &end, 10);
    if (errno || end == s || (*end != 'x' && *end != 'X'))
        return -1;
    s = end + 1;
    errno = 0;
    h = strtoul(s, &end, 10);
    if (errno || end == s || *end || !w || !h ||
        w > UINT32_MAX || h > UINT32_MAX)
        return -1;
    *width = (uint32_t)w;
    *height = (uint32_t)h;
    return 0;
}

static void usage(const char *prog)
{
    fprintf(stderr,
            "Usage: %s [options]\n"
            "Displays or capture-tests FPGA-generated RGB888 frames converted from BayerRG8.\n"
            "  --size WxH          active image size, default %ux%u\n"
            "  --fps FPS           reporting target, default %.1f\n"
            "  --duration SEC      stop after SEC; 0 runs until Ctrl-C\n"
            "  --ctrl PATH         BAR2 control device, default %s\n"
            "  --device PATH       first SGDMA queue, default %s\n"
            "  --csr-base ADDR     camera CSR offset, default 0x%llx\n"
            "  --ring-base ADDR    FPGA DDR ring base, default 0\n"
            "  --qid N             first DMA queue id, default %u\n"
            "  --buffers N         host DMA buffers, default %u\n"
            "  --dma-queues N      parallel SGDMA queues, default %u\n"
            "  --swap-interval N   EGL swap interval, default 1\n"
            "  --isp-enable MASK   ISP stages: bit0 BLC, bit1 WB, bit2 CCM, bit3 Gamma; default 15\n"
            "  --black-level N     Bayer black-level subtraction, 0..255; default 0\n"
            "  --wb-r GAIN         red Bayer-site gain, Q4.12 value as decimal; default 1.0\n"
            "  --wb-g GAIN         green Bayer-site gain; default 1.0\n"
            "  --wb-b GAIN         blue Bayer-site gain; default 1.0\n"
            "  --ccm 9_VALUES      row-major signed 3x3 matrix, comma separated; default identity\n"
            "  --shm-name NAME     publish full-resolution RGB888 frame (compatibility)\n"
            "  --detect-shm-name NAME publish low-copy letterboxed RGB detector input\n"
            "  --detect-size WxH   detector input size; N remains square, default 640x384\n"
            "  --detect-publish-fps FPS limit detector mailbox generation; 0 means every frame, default 0\n"
            "  --detect-on-demand  generate one detector input per YOLO request (ABI 4)\n"
            "  --boxes-shm-name NAME read YOLO boxes and overlay them on direct display\n"
            "  --dump-frame FILE   save a post-DMA/pre-OpenGL frame as binary PPM\n"
            "  --dump-frame-at N   wait until N frames were captured before saving\n"
            "  --gl-finish         wait for GPU completion before recycling DMA buffer (default)\n"
            "  --no-gl-finish      disable the GPU completion wait for performance comparison\n"
            "  --raw-bayer         transmit untouched BayerRG8; requires RAW-capable bit and --no-display\n"
            "  --no-display        skip X11/EGL/OpenGL and release each DMA frame immediately\n"
            "  --latest-frame      display newest complete frame; discard stale frames (default)\n"
            "  --fifo-display      display every frame in capture order (diagnostic/high latency)\n"
            "  --no-remote-start   do not send camera run/stop commands\n"
            "  --debug-csr         print capture diagnostics once per second\n",
            prog, VIEWER_DEFAULT_WIDTH, VIEWER_DEFAULT_HEIGHT,
            VIEWER_DEFAULT_FPS, CAMERA_DEFAULT_CTRL_DEVICE,
            CAMERA_DEFAULT_QUEUE_DEVICE,
            (unsigned long long)CAMERA_DEFAULT_CSR_BASE,
            CAMERA_DEFAULT_QID, VIEWER_DEFAULT_BUFFERS,
            VIEWER_DEFAULT_DMA_QUEUES);
}

static GLuint compile_shader(GLenum type, const char *source)
{
    GLuint shader = glCreateShader(type);
    GLint ok = 0;

    glShaderSource(shader, 1, &source, NULL);
    glCompileShader(shader);
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[1024];
        GLsizei n = 0;
        glGetShaderInfoLog(shader, sizeof(log), &n, log);
        fprintf(stderr, "shader compile failed: %.*s\n", (int)n, log);
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}

static void display_set_viewport(struct display_state *d,
                                 int window_width, int window_height)
{
    int viewport_x = 0;
    int viewport_y = 0;
    int viewport_width = window_width;
    int viewport_height = window_height;

    if (!d || window_width <= 0 || window_height <= 0 ||
        !d->image_width || !d->image_height)
        return;

    /*
     * Preserve the camera aspect ratio. The old implementation stretched a
     * camera-sized texture over whatever dimensions the window manager chose.
     */
    if ((uint64_t)window_width * d->image_height >
        (uint64_t)window_height * d->image_width) {
        viewport_width = (int)((uint64_t)window_height * d->image_width /
                               d->image_height);
        viewport_x = (window_width - viewport_width) / 2;
    } else {
        viewport_height = (int)((uint64_t)window_width * d->image_height /
                                d->image_width);
        viewport_y = (window_height - viewport_height) / 2;
    }

    if (viewport_width < 1)
        viewport_width = 1;
    if (viewport_height < 1)
        viewport_height = 1;
    glViewport(viewport_x, viewport_y, viewport_width, viewport_height);
}

static int create_program(struct display_state *d)
{
    static const char vertex_source[] =
        "attribute vec2 a_pos;\n"
        "attribute vec2 a_uv;\n"
        "varying highp vec2 v_uv;\n"
        "void main(){ gl_Position=vec4(a_pos,0.0,1.0); v_uv=a_uv; }\n";
    static const char fragment_source[] =
        "precision highp float;\n"
        "varying highp vec2 v_uv;\n"
        "uniform sampler2D u_tex;\n"
        "void main(){ gl_FragColor=texture2D(u_tex,v_uv); }\n";
    GLuint vs = compile_shader(GL_VERTEX_SHADER, vertex_source);
    GLuint fs = compile_shader(GL_FRAGMENT_SHADER, fragment_source);
    GLint ok = 0;

    if (!vs || !fs)
        goto fail;
    d->program = glCreateProgram();
    glAttachShader(d->program, vs);
    glAttachShader(d->program, fs);
    glLinkProgram(d->program);
    glGetProgramiv(d->program, GL_LINK_STATUS, &ok);
    glDeleteShader(vs);
    glDeleteShader(fs);
    if (!ok) {
        char log[1024];
        GLsizei n = 0;
        glGetProgramInfoLog(d->program, sizeof(log), &n, log);
        fprintf(stderr, "program link failed: %.*s\n", (int)n, log);
        goto fail;
    }
    d->a_pos = glGetAttribLocation(d->program, "a_pos");
    d->a_uv = glGetAttribLocation(d->program, "a_uv");
    d->u_tex = glGetUniformLocation(d->program, "u_tex");
    return 0;

fail:
    if (vs)
        glDeleteShader(vs);
    if (fs)
        glDeleteShader(fs);
    return -1;
}

static int create_overlay_program(struct display_state *d)
{
    static const char vertex_source[] =
        "attribute vec2 a_pos;\n"
        "uniform float u_point_size;\n"
        "void main(){ gl_Position=vec4(a_pos,0.0,1.0); "
        "gl_PointSize=u_point_size; }\n";
    static const char fragment_source[] =
        "precision mediump float;\n"
        "uniform vec4 u_color;\n"
        "void main(){ gl_FragColor=u_color; }\n";
    GLuint vs = compile_shader(GL_VERTEX_SHADER, vertex_source);
    GLuint fs = compile_shader(GL_FRAGMENT_SHADER, fragment_source);
    GLint ok = 0;

    if (!vs || !fs)
        goto fail;
    d->overlay_program = glCreateProgram();
    glAttachShader(d->overlay_program, vs);
    glAttachShader(d->overlay_program, fs);
    glLinkProgram(d->overlay_program);
    glGetProgramiv(d->overlay_program, GL_LINK_STATUS, &ok);
    glDeleteShader(vs);
    glDeleteShader(fs);
    if (!ok) {
        char log[1024];
        GLsizei n = 0;
        glGetProgramInfoLog(d->overlay_program, sizeof(log), &n, log);
        fprintf(stderr, "overlay program link failed: %.*s\n", (int)n, log);
        goto fail;
    }
    d->overlay_a_pos = glGetAttribLocation(d->overlay_program, "a_pos");
    d->overlay_u_color = glGetUniformLocation(d->overlay_program, "u_color");
    d->overlay_u_point_size =
        glGetUniformLocation(d->overlay_program, "u_point_size");
    return 0;

fail:
    if (vs)
        glDeleteShader(vs);
    if (fs)
        glDeleteShader(fs);
    return -1;
}

static int display_init(struct display_state *d, uint32_t width, uint32_t height,
                        int swap_interval)
{
    EGLint config_attrs[] = {
        EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
        EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
        EGL_RED_SIZE, 8, EGL_GREEN_SIZE, 8, EGL_BLUE_SIZE, 8,
        EGL_NONE
    };
    EGLint context_attrs[] = { EGL_CONTEXT_CLIENT_VERSION, 2, EGL_NONE };
    EGLint surface_attrs[] = {
        EGL_RENDER_BUFFER, EGL_BACK_BUFFER,
        EGL_NONE
    };
    EGLConfig config;
    EGLint count = 0;
    EGLint min_swap_interval = 0;
    EGLint max_swap_interval = 0;
    EGLint render_buffer = EGL_NONE;
    XSetWindowAttributes attrs;
    int screen;
    uint32_t max_window_width;
    uint32_t max_window_height;
    uint32_t window_width = width;
    uint32_t window_height = height;

    memset(d, 0, sizeof(*d));
    d->edpy = EGL_NO_DISPLAY;
    d->surface = EGL_NO_SURFACE;
    d->context = EGL_NO_CONTEXT;
    d->xdpy = XOpenDisplay(NULL);
    if (!d->xdpy) {
        fprintf(stderr, "XOpenDisplay failed; check DISPLAY/Xauthority\n");
        return -1;
    }
    d->image_width = width;
    d->image_height = height;
    screen = DefaultScreen(d->xdpy);
    max_window_width = (uint32_t)DisplayWidth(d->xdpy, screen) * 9u / 10u;
    max_window_height = (uint32_t)DisplayHeight(d->xdpy, screen) * 9u / 10u;
    if (!max_window_width)
        max_window_width = 1;
    if (!max_window_height)
        max_window_height = 1;

    /*
     * Fit a camera image larger than the monitor into the visible desktop.
     * The source texture remains at native camera resolution.
     */
    if (window_width > max_window_width) {
        window_height = (uint32_t)((uint64_t)height * max_window_width / width);
        window_width = max_window_width;
    }
    if (window_height > max_window_height) {
        window_width = (uint32_t)((uint64_t)width * max_window_height / height);
        window_height = max_window_height;
    }
    if (!window_width)
        window_width = 1;
    if (!window_height)
        window_height = 1;

    attrs.event_mask = StructureNotifyMask | ExposureMask | KeyPressMask;
    d->win = XCreateWindow(d->xdpy, DefaultRootWindow(d->xdpy), 0, 0,
                           window_width, window_height, 0,
                           CopyFromParent, InputOutput,
                           CopyFromParent, CWEventMask, &attrs);
    XStoreName(d->xdpy, d->win, "BayerRG8 to RGB888 Camera Link");
    XMapWindow(d->xdpy, d->win);

    d->edpy = eglGetDisplay((EGLNativeDisplayType)d->xdpy);
    if (d->edpy == EGL_NO_DISPLAY || !eglInitialize(d->edpy, NULL, NULL) ||
        !eglChooseConfig(d->edpy, config_attrs, &config, 1, &count) || count != 1) {
        fprintf(stderr, "EGL display/config initialization failed (0x%x)\n", eglGetError());
        return -1;
    }
    d->surface = eglCreateWindowSurface(d->edpy, config,
                                        (EGLNativeWindowType)d->win,
                                        surface_attrs);
    d->context = eglCreateContext(d->edpy, config, EGL_NO_CONTEXT, context_attrs);
    if (d->surface == EGL_NO_SURFACE || d->context == EGL_NO_CONTEXT ||
        !eglMakeCurrent(d->edpy, d->surface, d->surface, d->context)) {
        fprintf(stderr, "EGL surface/context creation failed (0x%x)\n", eglGetError());
        return -1;
    }
    if (!eglQuerySurface(d->edpy, d->surface, EGL_RENDER_BUFFER,
                         &render_buffer) ||
        render_buffer != EGL_BACK_BUFFER) {
        fprintf(stderr,
                "EGL window is not double-buffered (render buffer 0x%x, error 0x%x)\n",
                render_buffer, eglGetError());
        return -1;
    }
    if (!eglGetConfigAttrib(d->edpy, config, EGL_MIN_SWAP_INTERVAL,
                            &min_swap_interval) ||
        !eglGetConfigAttrib(d->edpy, config, EGL_MAX_SWAP_INTERVAL,
                            &max_swap_interval) ||
        swap_interval < min_swap_interval || swap_interval > max_swap_interval ||
        !eglSwapInterval(d->edpy, swap_interval)) {
        fprintf(stderr,
                "EGL swap interval %d is unsupported (reported range %d..%d, error 0x%x)\n",
                swap_interval, min_swap_interval, max_swap_interval,
                eglGetError());
        return -1;
    }
    if (swap_interval == 0)
        fprintf(stderr,
                "warning: swap interval 0 permits visible frame tearing; use 1 for camera display\n");
    if (create_program(d) < 0 || create_overlay_program(d) < 0)
        return -1;

    glGenTextures(1, &d->texture);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, d->texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, (GLsizei)width,
                 (GLsizei)height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
    if (glGetError() != GL_NO_ERROR) {
        fprintf(stderr, "GL_RGB texture allocation failed\n");
        return -1;
    }
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    display_set_viewport(d, (int)window_width, (int)window_height);
    glUseProgram(d->program);
    glUniform1i(d->u_tex, 0);
    return 0;
}

static void display_events(struct display_state *d)
{
    while (XPending(d->xdpy)) {
        XEvent ev;
        XNextEvent(d->xdpy, &ev);
        if (ev.type == KeyPress)
            running = 0;
        else if (ev.type == ConfigureNotify)
            display_set_viewport(d, ev.xconfigure.width,
                                 ev.xconfigure.height);
    }
}

static void display_set_status_title(struct display_state *d,
                                     double video_fps,
                                     const struct box_shm_reader *boxes)
{
    char title[256];

    if (!d || !d->xdpy || !d->win)
        return;
    if (!boxes || !boxes->name) {
        snprintf(title, sizeof(title),
                 "FPGA Camera | Video %.1f FPS", video_fps);
    } else if (!boxes->header) {
        snprintf(title, sizeof(title),
                 "FPGA Camera | Video %.1f FPS | YOLO waiting",
                 video_fps);
    } else {
        snprintf(title, sizeof(title),
                 "FPGA Camera | Video %.1f FPS | YOLO %.1f FPS | Objects %u",
                 video_fps, boxes->detect_fps, boxes->count);
    }
    XStoreName(d->xdpy, d->win, title);
    XFlush(d->xdpy);
}

static float clamp_float(float value, float low, float high)
{
    if (value < low)
        return low;
    if (value > high)
        return high;
    return value;
}

static void overlay_rectangle_vertices(GLfloat vertices[12],
                                       float x0, float y0,
                                       float x1, float y1,
                                       uint32_t width, uint32_t height)
{
    const GLfloat left = -1.0f + 2.0f * x0 / (float)width;
    const GLfloat right = -1.0f + 2.0f * x1 / (float)width;
    const GLfloat top = 1.0f - 2.0f * y0 / (float)height;
    const GLfloat bottom = 1.0f - 2.0f * y1 / (float)height;

    vertices[0] = left;  vertices[1] = top;
    vertices[2] = right; vertices[3] = top;
    vertices[4] = left;  vertices[5] = bottom;
    vertices[6] = left;  vertices[7] = bottom;
    vertices[8] = right; vertices[9] = top;
    vertices[10] = right; vertices[11] = bottom;
}

#define FONT_GLYPH(r0, r1, r2, r3, r4, r5, r6) \
    ((uint64_t)(r0) | ((uint64_t)(r1) << 5) | \
     ((uint64_t)(r2) << 10) | ((uint64_t)(r3) << 15) | \
     ((uint64_t)(r4) << 20) | ((uint64_t)(r5) << 25) | \
     ((uint64_t)(r6) << 30))

static uint64_t overlay_font_glyph(char ch)
{
    switch (ch) {
    case '0': return FONT_GLYPH(14,17,19,21,25,17,14);
    case '1': return FONT_GLYPH(4,12,4,4,4,4,14);
    case '2': return FONT_GLYPH(14,17,1,2,4,8,31);
    case '3': return FONT_GLYPH(30,1,1,14,1,1,30);
    case '4': return FONT_GLYPH(2,6,10,18,31,2,2);
    case '5': return FONT_GLYPH(31,16,16,30,1,1,30);
    case '6': return FONT_GLYPH(14,16,16,30,17,17,14);
    case '7': return FONT_GLYPH(31,1,2,4,8,8,8);
    case '8': return FONT_GLYPH(14,17,17,14,17,17,14);
    case '9': return FONT_GLYPH(14,17,17,15,1,1,14);
    case 'A': return FONT_GLYPH(14,17,17,31,17,17,17);
    case 'B': return FONT_GLYPH(30,17,17,30,17,17,30);
    case 'C': return FONT_GLYPH(14,17,16,16,16,17,14);
    case 'D': return FONT_GLYPH(30,17,17,17,17,17,30);
    case 'E': return FONT_GLYPH(31,16,16,30,16,16,31);
    case 'F': return FONT_GLYPH(31,16,16,30,16,16,16);
    case 'G': return FONT_GLYPH(14,17,16,23,17,17,15);
    case 'H': return FONT_GLYPH(17,17,17,31,17,17,17);
    case 'I': return FONT_GLYPH(14,4,4,4,4,4,14);
    case 'J': return FONT_GLYPH(7,2,2,2,18,18,12);
    case 'K': return FONT_GLYPH(17,18,20,24,20,18,17);
    case 'L': return FONT_GLYPH(16,16,16,16,16,16,31);
    case 'M': return FONT_GLYPH(17,27,21,21,17,17,17);
    case 'N': return FONT_GLYPH(17,25,21,19,17,17,17);
    case 'O': return FONT_GLYPH(14,17,17,17,17,17,14);
    case 'P': return FONT_GLYPH(30,17,17,30,16,16,16);
    case 'Q': return FONT_GLYPH(14,17,17,17,21,18,13);
    case 'R': return FONT_GLYPH(30,17,17,30,20,18,17);
    case 'S': return FONT_GLYPH(15,16,16,14,1,1,30);
    case 'T': return FONT_GLYPH(31,4,4,4,4,4,4);
    case 'U': return FONT_GLYPH(17,17,17,17,17,17,14);
    case 'V': return FONT_GLYPH(17,17,17,17,17,10,4);
    case 'W': return FONT_GLYPH(17,17,17,21,21,21,10);
    case 'X': return FONT_GLYPH(17,17,10,4,10,17,17);
    case 'Y': return FONT_GLYPH(17,17,10,4,4,4,4);
    case 'Z': return FONT_GLYPH(31,1,2,4,8,16,31);
    default: return 0;
    }
}

#undef FONT_GLYPH

static const char *overlay_coco_class_name(int32_t class_id)
{
    static const char *const names[] = {
        "PERSON", "BICYCLE", "CAR", "MOTORCYCLE", "AIRPLANE",
        "BUS", "TRAIN", "TRUCK", "BOAT", "TRAFFIC LIGHT",
        "FIRE HYDRANT", "STOP SIGN", "PARKING METER", "BENCH", "BIRD",
        "CAT", "DOG", "HORSE", "SHEEP", "COW",
        "ELEPHANT", "BEAR", "ZEBRA", "GIRAFFE", "BACKPACK",
        "UMBRELLA", "HANDBAG", "TIE", "SUITCASE", "FRISBEE",
        "SKIS", "SNOWBOARD", "SPORTS BALL", "KITE", "BASEBALL BAT",
        "BASEBALL GLOVE", "SKATEBOARD", "SURFBOARD", "TENNIS RACKET",
        "BOTTLE", "WINE GLASS", "CUP", "FORK", "KNIFE",
        "SPOON", "BOWL", "BANANA", "APPLE", "SANDWICH",
        "ORANGE", "BROCCOLI", "CARROT", "HOT DOG", "PIZZA",
        "DONUT", "CAKE", "CHAIR", "COUCH", "POTTED PLANT",
        "BED", "DINING TABLE", "TOILET", "TV", "LAPTOP",
        "MOUSE", "REMOTE", "KEYBOARD", "CELL PHONE", "MICROWAVE",
        "OVEN", "TOASTER", "SINK", "REFRIGERATOR", "BOOK",
        "CLOCK", "VASE", "SCISSORS", "TEDDY BEAR", "HAIR DRIER",
        "TOOTHBRUSH"
    };

    if (class_id < 0 ||
        (uint32_t)class_id >= sizeof(names) / sizeof(names[0]))
        return NULL;
    return names[class_id];
}

static void display_overlay_text(struct display_state *d, const char *text,
                                 float x, float y, float cell_size,
                                 const GLfloat color[3])
{
    enum { MAX_CHARS = 63, MAX_POINTS = MAX_CHARS * 35 };
    GLfloat points[MAX_POINTS * 2];
    size_t char_index;
    GLsizei point_count = 0;

    if (!text || !*text)
        return;
    for (char_index = 0; text[char_index] && char_index < MAX_CHARS;
         char_index++) {
        char ch = text[char_index];
        uint64_t glyph;
        unsigned int row, column;

        if (ch >= 'a' && ch <= 'z')
            ch = (char)(ch - 'a' + 'A');
        glyph = overlay_font_glyph(ch);
        for (row = 0; row < 7; row++) {
            for (column = 0; column < 5; column++) {
                float px, py;

                if (!(glyph & (1ull << (row * 5u + (4u - column)))))
                    continue;
                if (point_count >= MAX_POINTS)
                    break;
                px = x + ((float)char_index * 6.0f +
                          (float)column + 0.5f) * cell_size;
                py = y + ((float)row + 0.5f) * cell_size;
                if (px < 0.0f || py < 0.0f ||
                    px > d->image_width || py > d->image_height)
                    continue;
                points[point_count * 2] =
                    -1.0f + 2.0f * px / (float)d->image_width;
                points[point_count * 2 + 1] =
                    1.0f - 2.0f * py / (float)d->image_height;
                point_count++;
            }
        }
    }
    if (!point_count)
        return;
    glUniform4f(d->overlay_u_color, color[0], color[1], color[2], 1.0f);
    glUniform1f(d->overlay_u_point_size, 3.0f);
    glVertexAttribPointer((GLuint)d->overlay_a_pos, 2, GL_FLOAT, GL_FALSE,
                          0, points);
    glDrawArrays(GL_POINTS, 0, point_count);
}

static void display_overlay_boxes(struct display_state *d,
                                  const struct box_shm_reader *reader)
{
    static const GLfloat colors[][3] = {
        { 0.00f, 1.00f, 0.20f }, { 1.00f, 0.20f, 0.10f },
        { 0.10f, 0.70f, 1.00f }, { 1.00f, 0.85f, 0.05f },
        { 0.90f, 0.10f, 1.00f }, { 0.00f, 1.00f, 0.90f },
        { 1.00f, 0.45f, 0.00f }, { 0.45f, 0.30f, 1.00f }
    };
    const float thickness = 4.0f;
    uint32_t i;

    if (!reader || !reader->count)
        return;
    glUseProgram(d->overlay_program);
    glEnableVertexAttribArray((GLuint)d->overlay_a_pos);
    for (i = 0; i < reader->count; i++) {
        const struct box_shm_record *box = &reader->cached[i];
        GLfloat vertices[12];
        float x1, y1, x2, y2, t;
        float label_x, label_y, label_width, label_height;
        unsigned int key;
        const GLfloat *color;
        const char *class_name;
        char class_fallback[24];
        char label[64];
        size_t label_length;
        int confidence;

        if (!isfinite(box->x1) || !isfinite(box->y1) ||
            !isfinite(box->x2) || !isfinite(box->y2))
            continue;
        x1 = clamp_float(box->x1, 0.0f, (float)d->image_width);
        y1 = clamp_float(box->y1, 0.0f, (float)d->image_height);
        x2 = clamp_float(box->x2, 0.0f, (float)d->image_width);
        y2 = clamp_float(box->y2, 0.0f, (float)d->image_height);
        if (x2 <= x1 || y2 <= y1)
            continue;
        key = (unsigned int)(box->track_id >= 0 ?
              box->track_id : box->class_id);
        color = colors[key % (sizeof(colors) / sizeof(colors[0]))];
        glUniform4f(d->overlay_u_color, color[0], color[1], color[2], 1.0f);

        t = y2 - y1 < thickness ? y2 - y1 : thickness;
        overlay_rectangle_vertices(vertices, x1, y1, x2, y1 + t,
                                   d->image_width, d->image_height);
        glVertexAttribPointer((GLuint)d->overlay_a_pos, 2, GL_FLOAT, GL_FALSE,
                              0, vertices);
        glDrawArrays(GL_TRIANGLES, 0, 6);
        overlay_rectangle_vertices(vertices, x1, y2 - t, x2, y2,
                                   d->image_width, d->image_height);
        glVertexAttribPointer((GLuint)d->overlay_a_pos, 2, GL_FLOAT, GL_FALSE,
                              0, vertices);
        glDrawArrays(GL_TRIANGLES, 0, 6);

        t = x2 - x1 < thickness ? x2 - x1 : thickness;
        overlay_rectangle_vertices(vertices, x1, y1, x1 + t, y2,
                                   d->image_width, d->image_height);
        glVertexAttribPointer((GLuint)d->overlay_a_pos, 2, GL_FLOAT, GL_FALSE,
                              0, vertices);
        glDrawArrays(GL_TRIANGLES, 0, 6);
        overlay_rectangle_vertices(vertices, x2 - t, y1, x2, y2,
                                   d->image_width, d->image_height);
        glVertexAttribPointer((GLuint)d->overlay_a_pos, 2, GL_FLOAT, GL_FALSE,
                              0, vertices);
        glDrawArrays(GL_TRIANGLES, 0, 6);

        class_name = overlay_coco_class_name(box->class_id);
        if (!class_name) {
            snprintf(class_fallback, sizeof(class_fallback),
                     "CLASS%d", box->class_id);
            class_name = class_fallback;
        }
        confidence = (int)(clamp_float(box->confidence, 0.0f, 1.0f) *
                           100.0f + 0.5f);
        if (box->track_id >= 0)
            snprintf(label, sizeof(label), "%s %d ID%d",
                     class_name, confidence, box->track_id);
        else
            snprintf(label, sizeof(label), "%s %d",
                     class_name, confidence);
        label_length = strlen(label);
        label_width = (float)(label_length * 6u + 2u) * 4.0f;
        label_height = 9.0f * 4.0f;
        if (label_width > d->image_width)
            label_width = (float)d->image_width;
        if (label_height > d->image_height)
            label_height = (float)d->image_height;
        label_x = x1;
        if (label_x + label_width > d->image_width)
            label_x = (float)d->image_width - label_width;
        label_y = y1 >= label_height ? y1 - label_height : y1 + 4.0f;
        if (label_y + label_height > d->image_height)
            label_y = (float)d->image_height - label_height;
        glUniform4f(d->overlay_u_color, 0.02f, 0.02f, 0.02f, 1.0f);
        overlay_rectangle_vertices(vertices, label_x, label_y,
                                   label_x + label_width,
                                   label_y + label_height,
                                   d->image_width, d->image_height);
        glVertexAttribPointer((GLuint)d->overlay_a_pos, 2, GL_FLOAT, GL_FALSE,
                              0, vertices);
        glDrawArrays(GL_TRIANGLES, 0, 6);
        display_overlay_text(d, label, label_x + 4.0f, label_y + 4.0f,
                             4.0f, color);
    }
    glDisableVertexAttribArray((GLuint)d->overlay_a_pos);
}

static int display_frame(struct display_state *d, const void *rgb,
                         uint32_t width, uint32_t height, int gl_finish,
                         const struct box_shm_reader *boxes)
{
    /*
     * One oversized triangle covers the complete viewport. This removes the
     * shared edge and gives every displayed pixel one continuous UV equation.
     */
    static const GLfloat vertices[] = {
        -1.0f, -1.0f, 0.0f,  1.0f,
         3.0f, -1.0f, 2.0f,  1.0f,
        -1.0f,  3.0f, 0.0f, -1.0f
    };

    glUseProgram(d->program);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, d->texture);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, (GLsizei)width,
                    (GLsizei)height, GL_RGB, GL_UNSIGNED_BYTE, rgb);
    glVertexAttribPointer((GLuint)d->a_pos, 2, GL_FLOAT, GL_FALSE,
                          4 * sizeof(GLfloat), vertices);
    glVertexAttribPointer((GLuint)d->a_uv, 2, GL_FLOAT, GL_FALSE,
                          4 * sizeof(GLfloat), vertices + 2);
    glEnableVertexAttribArray((GLuint)d->a_pos);
    glEnableVertexAttribArray((GLuint)d->a_uv);
    glClear(GL_COLOR_BUFFER_BIT);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    display_overlay_boxes(d, boxes);
    if (gl_finish)
        glFinish();
    if (glGetError() != GL_NO_ERROR) {
        fprintf(stderr, "OpenGL upload/draw failed\n");
        return -1;
    }
    if (!eglSwapBuffers(d->edpy, d->surface)) {
        fprintf(stderr, "eglSwapBuffers failed (0x%x)\n", eglGetError());
        return -1;
    }
    return 0;
}


static int dump_rgb_ppm(const char *path, const void *frame, size_t len,
                        uint32_t width, uint32_t height)
{
    const uint8_t *rgb = (const uint8_t *)frame;
    const uint64_t required = (uint64_t)width * height * 3ull;
    FILE *fp;
    int saved_errno;
    int ret = -1;

    if (!path || !*path || !rgb || !required || required > len ||
        required > SIZE_MAX) {
        errno = EINVAL;
        return -1;
    }

    fp = fopen(path, "wb");
    if (!fp)
        return -1;
    if (fprintf(fp, "P6\n%u %u\n255\n", width, height) >= 0 &&
        fwrite(rgb, 1, (size_t)required, fp) == (size_t)required &&
        fflush(fp) == 0)
        ret = 0;

    saved_errno = errno;
    if (fclose(fp) != 0 && ret == 0) {
        saved_errno = errno;
        ret = -1;
    }
    if (ret < 0)
        errno = saved_errno ? saved_errno : EIO;
    return ret;
}

static void display_destroy(struct display_state *d)
{
    if (d->edpy != EGL_NO_DISPLAY) {
        if (d->context != EGL_NO_CONTEXT)
            eglMakeCurrent(d->edpy, d->surface, d->surface, d->context);
        if (d->texture)
            glDeleteTextures(1, &d->texture);
        if (d->overlay_program)
            glDeleteProgram(d->overlay_program);
        if (d->program)
            glDeleteProgram(d->program);
        eglMakeCurrent(d->edpy, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
        if (d->context != EGL_NO_CONTEXT)
            eglDestroyContext(d->edpy, d->context);
        if (d->surface != EGL_NO_SURFACE)
            eglDestroySurface(d->edpy, d->surface);
        eglTerminate(d->edpy);
    }
    if (d->xdpy) {
        if (d->win)
            XDestroyWindow(d->xdpy, d->win);
        XCloseDisplay(d->xdpy);
    }
}

static void print_csr(struct camera_context *cam)
{
    struct camera_csr_snapshot s;

    if (camera_read_csr_snapshot(cam, &s) < 0)
        return;
    fprintf(stderr,
            "csr ctrl=0x%08x status=0x%08x prod=%u cons=%u fill=%u "
            "line=%u height=%u bytes=%u seq=%u drops=%u "
            "build=0x%08x/ver=0x%08x dup=%u hls_stall=%u axi_stall=%u "
            "cmlk_sync=%u/%u/0x%02x align_loss=%u resync=%u isp=0x%x black=%u "
            "wb=%.4f/%.4f/%.4f "
            "fval=%u lval=%u dval=%u words=%u bad=%u last_line=%u reason=0x%08x\n",
            s.control, s.status, s.producer_idx, s.consumer_idx, s.ring_fill,
            s.width, s.height, s.frame_bytes, s.frame_seq, s.drop_count,
            s.build_id, s.build_version, s.duplicate_commit_count,
            s.hls_stall_cycles, s.axi_stall_cycles,
            s.cmlk_sync_mismatch_cycles, s.cmlk_sync_mismatch_events,
            s.cmlk_sync_mismatch_mask & 0x7u,
            s.cmlk_align_loss_count,
            s.hls_resync_count, s.isp_enable, s.isp_black_level,
            q12_to_double((int32_t)s.isp_wb_gain[0]),
            q12_to_double((int32_t)s.isp_wb_gain[1]),
            q12_to_double((int32_t)s.isp_wb_gain[2]),
            s.diag.fval_edges, s.diag.lval_edges, s.diag.dval_edges,
            s.diag.pix_valid_words, s.diag.bad_frames,
            s.diag.last_line_bytes, s.diag.drop_reason);
}

int main(int argc, char **argv)
{
    const char *ctrl_path = CAMERA_DEFAULT_CTRL_DEVICE;
    const char *queue_path = CAMERA_DEFAULT_QUEUE_DEVICE;
    const char *dump_frame_path = NULL;
    const char *shm_name = NULL;
    const char *detect_shm_name = NULL;
    const char *boxes_shm_name = NULL;
    uint64_t dump_frame_at = 0;
    uint64_t csr_base = CAMERA_DEFAULT_CSR_BASE;
    uint64_t ring_base = 0;
    uint32_t qid = CAMERA_DEFAULT_QID;
    uint32_t width = VIEWER_DEFAULT_WIDTH;
    uint32_t height = VIEWER_DEFAULT_HEIGHT;
    uint32_t buffers = VIEWER_DEFAULT_BUFFERS;
    uint32_t dma_queues = VIEWER_DEFAULT_DMA_QUEUES;
    uint32_t detect_width = 640u;
    uint32_t detect_height = 384u;
    struct camera_isp_config isp_config = {
        .enable_mask = 0x0fu,
        .black_level = 0u,
        .wb_gain = { CAMERA_ISP_Q12_ONE, CAMERA_ISP_Q12_ONE,
                     CAMERA_ISP_Q12_ONE },
        .ccm = { CAMERA_ISP_Q12_ONE, 0, 0,
                 0, CAMERA_ISP_Q12_ONE, 0,
                 0, 0, CAMERA_ISP_Q12_ONE }
    };
    double fps = VIEWER_DEFAULT_FPS;
    double detect_publish_fps = 0.0;
    int detect_on_demand = 0;
    uint32_t detect_last_request = 0;
    double duration = 0.0;
    int swap_interval = 1;
    int remote_start = 1;
    int no_display = 0;
    int raw_bayer = 0;
    int latest_frame = 1;
    int debug_csr = 0;
    int dump_frame_done = 0;
    int gl_finish = 1;
    uint64_t raw_frame_bytes64, rgb_line_bytes64, frame_bytes64, stride64, ring_end;
    uint32_t raw_frame_bytes, rgb_line_bytes, frame_bytes, frame_stride;
    uint32_t detect_stride = 0, detect_frame_bytes = 0;
    uint32_t detect_content_x = 0, detect_content_y = 0;
    uint32_t detect_content_width = 0, detect_content_height = 0;
    struct camera_open_opts opts;
    struct camera_context *cam = NULL;
    struct camera_frame_info first_info;
    struct display_state display;
    struct frame_shm_publisher shm_pub = { .fd = -1 };
    /* Analysis/web snapshots are a separate latest-frame branch. The ordered
     * GPU queue, host DMA, DDR sender and frame ownership are never throttled. */
    double analysis_shm_fps = 0.0;
    uint64_t analysis_next_ns = 0;
    const char *analysis_rate = getenv("CAMERA_ANALYSIS_SHM_FPS");
    if (analysis_rate && *analysis_rate) {
        char *end = NULL;
        analysis_shm_fps = strtod(analysis_rate, &end);
        if (!end || *end || !isfinite(analysis_shm_fps) ||
            analysis_shm_fps < 0 || analysis_shm_fps > 140) {
            fprintf(stderr, "Error: CAMERA_ANALYSIS_SHM_FPS must be 0..140\n");
            return EXIT_FAILURE;
        }
    }
    fprintf(stderr, "ANALYSIS_SHM fps=%.1f (0=unlimited); ordered video/DMA unchanged\n",
            analysis_shm_fps);

    struct vdq display_queue = { .fd = -1, .lock_fd = -1 };
    const char *display_queue_path = getenv("VIDEO_DISPLAY_FIFO");
    struct frame_shm_publisher detect_shm_pub = { .fd = -1 };
    struct host_yolo_sidecar host_yolo = { .fd = -1 };
    int host_yolo_mode = 0;
    struct box_shm_reader box_reader = { .fd = -1 };
    uint64_t frames = 0, report_frames = 0;
    uint64_t vdq_cost_ns=0, analysis_cost_ns=0, detector_cost_ns=0;
    uint64_t detect_published_total = 0, detect_report_published = 0;
    uint64_t detect_throttled_total = 0, detect_report_throttled = 0;
    uint64_t host_skipped_total = 0, ddr_skipped_total = 0, preview_skipped_total = 0;
    uint64_t started, reported, deadline = 0;
    uint32_t detect_last_frame_seq = 0;
    double detect_publish_credit = 0.0;
    int detect_last_frame_valid = 0;
    int display_ready = 0;
    int ret = 1;

    memset(&display, 0, sizeof(display));
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--size") && i + 1 < argc) {
            if (parse_size(argv[++i], &width, &height)) goto bad_args;
        } else if (!strcmp(argv[i], "--fps") && i + 1 < argc) {
            if (parse_double(argv[++i], &fps, 0)) goto bad_args;
        } else if (!strcmp(argv[i], "--duration") && i + 1 < argc) {
            if (parse_double(argv[++i], &duration, 1)) goto bad_args;
        } else if (!strcmp(argv[i], "--ctrl") && i + 1 < argc) {
            ctrl_path = argv[++i];
        } else if (!strcmp(argv[i], "--device") && i + 1 < argc) {
            queue_path = argv[++i];
        } else if (!strcmp(argv[i], "--csr-base") && i + 1 < argc) {
            if (parse_u64(argv[++i], &csr_base)) goto bad_args;
        } else if (!strcmp(argv[i], "--ring-base") && i + 1 < argc) {
            if (parse_u64(argv[++i], &ring_base)) goto bad_args;
        } else if (!strcmp(argv[i], "--qid") && i + 1 < argc) {
            if (parse_u32(argv[++i], &qid)) goto bad_args;
        } else if (!strcmp(argv[i], "--buffers") && i + 1 < argc) {
            if (parse_u32(argv[++i], &buffers)) goto bad_args;
        } else if (!strcmp(argv[i], "--dma-queues") && i + 1 < argc) {
            if (parse_u32(argv[++i], &dma_queues)) goto bad_args;
        } else if (!strcmp(argv[i], "--swap-interval") && i + 1 < argc) {
            uint32_t v;
            if (parse_u32(argv[++i], &v) || v > 4) goto bad_args;
            swap_interval = (int)v;
        } else if (!strcmp(argv[i], "--isp-enable") && i + 1 < argc) {
            if (parse_u32(argv[++i], &isp_config.enable_mask) ||
                isp_config.enable_mask > 0x0fu) goto bad_args;
        } else if (!strcmp(argv[i], "--black-level") && i + 1 < argc) {
            if (parse_u32(argv[++i], &isp_config.black_level) ||
                isp_config.black_level > 255u) goto bad_args;
        } else if (!strcmp(argv[i], "--wb-r") && i + 1 < argc) {
            if (parse_uq12(argv[++i], &isp_config.wb_gain[0])) goto bad_args;
        } else if (!strcmp(argv[i], "--wb-g") && i + 1 < argc) {
            if (parse_uq12(argv[++i], &isp_config.wb_gain[1])) goto bad_args;
        } else if (!strcmp(argv[i], "--wb-b") && i + 1 < argc) {
            if (parse_uq12(argv[++i], &isp_config.wb_gain[2])) goto bad_args;
        } else if (!strcmp(argv[i], "--ccm") && i + 1 < argc) {
            if (parse_ccm_q12(argv[++i], isp_config.ccm)) goto bad_args;
        } else if (!strcmp(argv[i], "--shm-name") && i + 1 < argc) {
            shm_name = argv[++i];
            if (!frame_shm_name_valid(shm_name)) goto bad_args;
        } else if (!strcmp(argv[i], "--detect-shm-name") && i + 1 < argc) {
            detect_shm_name = argv[++i];
            if (!frame_shm_name_valid(detect_shm_name)) goto bad_args;
        } else if (!strcmp(argv[i], "--detect-size") && i + 1 < argc) {
            const char *value = argv[++i];
            if (strchr(value, 'x') || strchr(value, 'X')) {
                if (parse_size(value, &detect_width, &detect_height))
                    goto bad_args;
            } else {
                if (parse_u32(value, &detect_width)) goto bad_args;
                detect_height = detect_width;
            }
        } else if (!strcmp(argv[i], "--detect-publish-fps") &&
                   i + 1 < argc) {
            if (parse_double(argv[++i], &detect_publish_fps, 1))
                goto bad_args;
        } else if (!strcmp(argv[i], "--detect-on-demand")) {
            detect_on_demand = 1;
        } else if (!strcmp(argv[i], "--boxes-shm-name") && i + 1 < argc) {
            boxes_shm_name = argv[++i];
            if (!frame_shm_name_valid(boxes_shm_name)) goto bad_args;
        } else if (!strcmp(argv[i], "--dump-frame") && i + 1 < argc) {
            dump_frame_path = argv[++i];
        } else if (!strcmp(argv[i], "--dump-frame-at") && i + 1 < argc) {
            if (parse_u64(argv[++i], &dump_frame_at)) goto bad_args;
        } else if (!strcmp(argv[i], "--gl-finish")) {
            gl_finish = 1;
        } else if (!strcmp(argv[i], "--no-gl-finish")) {
            gl_finish = 0;
        } else if (!strcmp(argv[i], "--raw-bayer")) {
            raw_bayer = 1;
        } else if (!strcmp(argv[i], "--no-display")) {
            no_display = 1;
        } else if (!strcmp(argv[i], "--latest-frame") ||
                   !strcmp(argv[i], "--low-latency")) {
            latest_frame = 1;
        } else if (!strcmp(argv[i], "--fifo-display")) {
            latest_frame = 0;
        } else if (!strcmp(argv[i], "--no-remote-start")) {
            remote_start = 0;
        } else if (!strcmp(argv[i], "--debug-csr")) {
            debug_csr = 1;
        } else if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h")) {
            usage(argv[0]);
            return 0;
        } else {
            goto bad_args;
        }
    }

    if (dump_frame_at && !dump_frame_path) {
        fprintf(stderr, "--dump-frame-at requires --dump-frame FILE\n");
        return 2;
    }
    if ((shm_name && detect_shm_name && !strcmp(shm_name, detect_shm_name)) ||
        (shm_name && boxes_shm_name && !strcmp(shm_name, boxes_shm_name)) ||
        (detect_shm_name && boxes_shm_name &&
         !strcmp(detect_shm_name, boxes_shm_name))) {
        fprintf(stderr,
                "frame, detector and boxes shared-memory names must be different\n");
        return 2;
    }
    if (!width || !height || width > VIEWER_MAX_WIDTH ||
        height > VIEWER_MAX_HEIGHT || (width & 7u) ||
        buffers < 2 || buffers > 256 ||
        dma_queues < 1 || dma_queues > CAMERA_MAX_DMA_QUEUES ||
        buffers < dma_queues ||
        detect_width < 32u || detect_width > 2048u ||
        detect_height < 32u || detect_height > 2048u ||
        (detect_width & 31u) || (detect_height & 31u) ||
        detect_publish_fps < 0.0 || detect_publish_fps > 1000.0) {
        fprintf(stderr,
                "invalid geometry/pool: require width=8..%u divisible by 8, "
                "height=1..%u, buffers 2..256 and >= dma-queues 1..8; "
                "detect-size width/height=32..2048 and divisible by 32; "
                "detect-publish-fps=0..1000\n",
                VIEWER_MAX_WIDTH, VIEWER_MAX_HEIGHT);
        return 2;
    }
    if (raw_bayer && (!no_display || detect_shm_name || boxes_shm_name || dump_frame_path || (height & 1u))) {
        fprintf(stderr, "RAW requires --no-display, even height, no detector/boxes/RGB dump; use RDMA desktop for preview\n");
        return 2;
    }
    raw_frame_bytes64 = (uint64_t)width * height;
    rgb_line_bytes64 = (uint64_t)width * (raw_bayer ? 1ull : 3ull);
    frame_bytes64 = rgb_line_bytes64 * height;
    /* Preserve conservative RGB-sized DDR slots in both modes. */
    stride64 = (raw_frame_bytes64 * 3ull + 4095ull) & ~4095ull;
    ring_end = ring_base + stride64 * VIEWER_RING_SLOTS;
    if ((ring_base & 63ull) || raw_frame_bytes64 > UINT32_MAX ||
        rgb_line_bytes64 > UINT32_MAX || frame_bytes64 > UINT32_MAX || stride64 > UINT32_MAX ||
        ring_base > UINT32_MAX || ring_end > (1ull << 32) || ring_end < ring_base) {
        fprintf(stderr, "ring base must be 64-byte aligned and the RGB888 ring must fit the FPGA 32-bit DDR address space\n");
        return 2;
    }
    raw_frame_bytes = (uint32_t)raw_frame_bytes64;
    rgb_line_bytes = (uint32_t)rgb_line_bytes64;
    frame_bytes = (uint32_t)frame_bytes64;
    frame_stride = (uint32_t)stride64;
    if (detect_shm_name) {
        uint64_t detect_bytes64 =
            (uint64_t)detect_width * detect_height * 3ull;

        if (detect_bytes64 > UINT32_MAX) {
            fprintf(stderr, "detector shared-memory image is too large\n");
            return 2;
        }
        detect_stride = detect_width * 3u;
        detect_frame_bytes = (uint32_t)detect_bytes64;
        if ((uint64_t)width * detect_height >=
            (uint64_t)height * detect_width) {
            detect_content_width = detect_width;
            detect_content_height = (uint32_t)
                (((uint64_t)height * detect_width + width / 2u) / width);
            if (!detect_content_height)
                detect_content_height = 1u;
            if (detect_content_height > detect_height)
                detect_content_height = detect_height;
            detect_content_y = (detect_height - detect_content_height) / 2u;
        } else {
            detect_content_height = detect_height;
            detect_content_width = (uint32_t)
                (((uint64_t)width * detect_height + height / 2u) / height);
            if (!detect_content_width)
                detect_content_width = 1u;
            if (detect_content_width > detect_width)
                detect_content_width = detect_width;
            detect_content_x = (detect_width - detect_content_width) / 2u;
        }
    }

    signal(SIGINT, stop_handler);
    signal(SIGTERM, stop_handler);
    fprintf(stderr, "SOURCE_PARK capability=1\n");
    if (!no_display) {
        if (display_init(&display, width, height, swap_interval) < 0)
            goto out;
        display_ready = 1;
    }

    memset(&opts, 0, sizeof(opts));
    opts.ctrl_path = ctrl_path;
    opts.queue_path = queue_path;
    opts.csr_base = csr_base;
    opts.ring_base = ring_base;
    opts.ring_base_set = 1;
    opts.frame_stride = frame_stride;
    opts.frame_stride_set = 1;
    opts.ring_slots = VIEWER_RING_SLOTS;
    opts.qid = qid;
    opts.qid_set = 1;
    opts.dma_queue_count = dma_queues;
    /* Frame-ready camera IRQ remains enabled below. Direct mode uses the
     * driver's existing coherent-WB short poll instead of jiffy DMA-IRQ fallback. */
    opts.dma_frame_irq = getenv("CAMERA_DDR_RING") ? -1 : 1;
    opts.driver_frame_irq_dma = 0;
    opts.irq_enable = 1u;
    opts.control_flags = 0; /* FVAL/LVAL gate; tested Hik 80-bit mode omits DVAL. */
    opts.bytes_per_clock = CAMERA_DEFAULT_BYTES_PER_CLOCK;
    opts.tap_order = CAMERA_DEFAULT_TAP_ORDER;
    opts.tap_order_set = 1;
    opts.processing_mode = raw_bayer ? CAMERA_PROCESS_RAW : CAMERA_PROCESS_BAYERRG8_RGB888;
    opts.expected_line_bytes = width;
    opts.expected_height = height;
    opts.expected_frame_bytes = raw_frame_bytes;
    opts.enable_capture = 0;

    fprintf(stderr,
            "FPGA capture pipeline: %ux%u raw_bytes=%u output_line=%u "
            "output_bytes=%u stride=%u fps=%.3f ring=0x%08llx slots=%u "
            "qid=%u queues=%u buffers=%u isp=0x%x black=%u "
            "wb=%.4f/%.4f/%.4f ccm=[%.4f,%.4f,%.4f;%.4f,%.4f,%.4f;%.4f,%.4f,%.4f] "
            "mode=%s latency=%s\n",
            width, height, raw_frame_bytes, rgb_line_bytes, frame_bytes, frame_stride, fps,
            (unsigned long long)ring_base, VIEWER_RING_SLOTS,
            qid, dma_queues, buffers, isp_config.enable_mask, isp_config.black_level,
            q12_to_double(isp_config.wb_gain[0]),
            q12_to_double(isp_config.wb_gain[1]),
            q12_to_double(isp_config.wb_gain[2]),
            q12_to_double(isp_config.ccm[0]), q12_to_double(isp_config.ccm[1]),
            q12_to_double(isp_config.ccm[2]), q12_to_double(isp_config.ccm[3]),
            q12_to_double(isp_config.ccm[4]), q12_to_double(isp_config.ccm[5]),
            q12_to_double(isp_config.ccm[6]), q12_to_double(isp_config.ccm[7]),
            q12_to_double(isp_config.ccm[8]),
            no_display ? "capture-only" : "display",
            latest_frame ? "latest-complete" : "fifo");

    if (camera_open(&cam, &opts) < 0)
        goto out;
    if (camera_set_output_mode(cam, raw_bayer) < 0) {
        fprintf(stderr, "output mode setup failed: %s\n", camera_last_error(cam));
        goto out;
    }
    if (camera_set_isp_config(cam, &isp_config) < 0) {
        fprintf(stderr, "ISP setup failed: %s\n", camera_last_error(cam));
        goto out;
    }
    if (remote_start &&
        camera_set_remote_acquisition_control(cam, "w AcquisitionStart 1",
                                              "w AcquisitionStop 1", 5000) < 0) {
        fprintf(stderr, "camera start/stop setup failed: %s\n", camera_last_error(cam));
        goto out;
    }
    if (camera_dma_open_pool(cam, buffers, frame_bytes, dma_queues) < 0) {
        fprintf(stderr, "host DMA pool setup failed: %s\n", camera_last_error(cam));
        goto out;
    }
    if (camera_dma_set_latest_mode(cam, latest_frame) < 0) {
        fprintf(stderr, "latest-frame DMA setup failed: %s\n", camera_last_error(cam));
        goto out;
    }
    if (shm_name) {
        if (frame_shm_init(&shm_pub, shm_name, width, height,
                           rgb_line_bytes, frame_bytes,
                           raw_bayer ? FRAME_SHM_PIXEL_BAYER : FRAME_SHM_PIXEL_RGB888,
                           width, height, 0, 0, width, height, 0) < 0) {
            fprintf(stderr, "shared-memory publisher '%s' setup failed: %s\n",
                    shm_name, strerror(errno));
            goto out;
        }
        fprintf(stderr,
                "shared-memory frame publisher: name=%s path=/dev/shm/%s "
                "bytes=%zu buffers=%u\n",
                shm_name, shm_name + 1, shm_pub.mapping_bytes,
                FRAME_SHM_BUFFERS);
    }
    if (detect_shm_name) {
        if (frame_shm_init(&detect_shm_pub, detect_shm_name,
                           detect_width, detect_height,
                           detect_stride, detect_frame_bytes,
                           FRAME_SHM_PIXEL_RGB888,
                           width, height,
                           detect_content_x, detect_content_y,
                           detect_content_width, detect_content_height,
                           detect_on_demand) < 0) {
            fprintf(stderr,
                    "detector shared-memory publisher '%s' setup failed: %s\n",
                    detect_shm_name, strerror(errno));
            goto out;
        }
        fprintf(stderr,
                "detector RGB888 publisher: name=%s input=%ux%u "
                "content=%u,%u/%ux%u source=%ux%u bytes=%zu max_fps=%.3f "
                "notify=futex on_demand=%d\n",
                detect_shm_name, detect_width, detect_height,
                detect_content_x, detect_content_y,
                detect_content_width, detect_content_height,
                width, height, detect_shm_pub.mapping_bytes,
                detect_publish_fps, detect_on_demand);
    }
    if (detect_shm_pub.header && detect_on_demand && !raw_bayer &&
        getenv("CAMERA_DDR_RING")) {
        const char *yolo_input_path = getenv("CAMERA_YOLO_INPUT_PATH");
        if (yolo_input_path && strcmp(yolo_input_path, "preview") == 0) {
            fprintf(stderr, "HOST_YOLO enabled=0 source=ISP-PCIe-preview "
                    "reason=single-host-video-path\n");
        } else {
            host_yolo_mode = host_yolo_start(&host_yolo, ctrl_path, (uint32_t)csr_base,
                                             width, height, frame_bytes, &detect_shm_pub);
            if (host_yolo_mode < 0) {
                fprintf(stderr, "Error: dual-path YOLO host DMA setup failed: %s\n",
                        strerror(errno));
                goto out;
            }
        }
    }
    if (boxes_shm_name && !no_display) {
        if (box_shm_init(&box_reader, boxes_shm_name, width, height) < 0) {
            fprintf(stderr, "YOLO overlay reader '%s' setup failed: %s\n",
                    boxes_shm_name, strerror(errno));
            goto out;
        }
        fprintf(stderr,
                "waiting non-blockingly for YOLO overlay: name=%s\n",
                boxes_shm_name);
    }

    if (display_queue_path && *display_queue_path &&
        vdq_create(&display_queue, display_queue_path, width, height,
                   raw_bayer ? FRAME_SHM_PIXEL_BAYER : FRAME_SHM_PIXEL_RGB888,
                   VDQ_SLOTS)) {
        fprintf(stderr, "Error: ordered display queue setup failed: %s\n", strerror(errno));
        goto out;
    }
    /* Allocate and pre-fault every host publisher before admitting camera
     * frames. Full-resolution display storage can take longer than the DDR
     * ring's 32-frame budget; doing this after arm fills DDR before reap. */
    if (camera_dma_start_strict(cam, 0, rgb_line_bytes, height, frame_bytes,
                                CAMERA_DEFAULT_TIMEOUT_MS, &first_info) < 0) {
        fprintf(stderr, "strict RGB888 capture start failed: %s\n", camera_last_error(cam));
        if (debug_csr)
            print_csr(cam);
        goto out;
    }
    started = reported = monotonic_ns();
    if (duration > 0.0)
        deadline = started + (uint64_t)(duration * 1000000000.0);
    while (running && (!deadline || monotonic_ns() < deadline)) {
        struct camera_frame_info info;
        void *rgb = NULL;
        size_t len = 0;
        uint32_t host_skipped = 0;
        uint32_t ddr_skipped = 0;
        uint64_t now;

        if ((latest_frame ?
             camera_dma_reap_latest_frame(cam, CAMERA_DEFAULT_TIMEOUT_MS,
                                          &info, &rgb, &len,
                                          &host_skipped, &ddr_skipped) :
             camera_dma_reap_frame(cam, CAMERA_DEFAULT_TIMEOUT_MS,
                                   &info, &rgb, &len)) < 0) {
            /* A source gate during ordered stop is not a DMA fault. Keep the
             * pool and display alive until the sender has drained ownership. */
            if (running && errno == ETIMEDOUT && display_queue.h &&
                (video_source_park_requested(getenv("CAMERA_DDR_RING")) ||
                 camera_capture_is_enabled(cam) == 0))
                continue;
            if (errno == ECANCELED || (!running && errno == ETIMEDOUT)) {
                fprintf(stderr, "Info: capture stopped after DDR sender drain\n");
                break;
            }
            fprintf(stderr, "DMA frame wait failed: %s\n", camera_last_error(cam));
            goto out;
        }
        host_skipped_total += host_skipped;
        ddr_skipped_total += ddr_skipped;
        if (!rgb || len != frame_bytes || info.frame_bytes != frame_bytes ||
            info.width != rgb_line_bytes || info.height != height) {
            fprintf(stderr,
                    "reject frame: ptr=%p len=%zu metadata=%ux%u/%u expected=%ux%u/%u\n",
                    rgb, len, info.width, info.height, info.frame_bytes,
                    rgb_line_bytes, height, frame_bytes);
            (void)camera_dma_release_frame(cam, &info);
            goto out;
        }

        if (dump_frame_path && !dump_frame_done &&
            frames >= dump_frame_at) {
            if (dump_rgb_ppm(dump_frame_path, rgb, len, width, height) < 0) {
                fprintf(stderr, "save pre-OpenGL DMA frame '%s' failed: %s\n",
                        dump_frame_path, strerror(errno));
                (void)camera_dma_release_frame(cam, &info);
                goto out;
            }
            fprintf(stderr,
                    "saved pre-OpenGL DMA frame: %s after=%llu seq=%u slot=%u bytes=%zu\n",
                    dump_frame_path, (unsigned long long)frames,
                    info.frame_seq, info.slot, len);
            dump_frame_done = 1;
        }

        const void *host_pixels = rgb;
        uint64_t stage_begin=monotonic_ns();
        /* Display may miss a frame when the monitor/GPU is slower than
         * acquisition. Never stall the camera, DDR writer or RDMA for it. */
        int preview_published = 0;
        if (display_queue.h) {
            if (!vdq_publish(&display_queue, rgb, info.frame_seq,
                    info.ready_ns ? info.ready_ns : monotonic_ns(), NULL, NULL, 0))
                preview_published = 1;
            else if (errno == EAGAIN)
                preview_skipped_total++;
            else {
                fprintf(stderr, "Error: display queue invalid: %s\n", strerror(errno));
                (void)camera_dma_release_frame(cam, &info);
                goto out;
            }
        }
        vdq_cost_ns += monotonic_ns()-stage_begin;
        if (preview_published) {
            /* The only producer is this thread. Its just-published cached slot
             * cannot be reused before this iteration completes, even if GL has
             * already acknowledged it. Read DMA memory once, then fan out. */
            uint64_t serial=atomic_load_explicit(&display_queue.h->produced,memory_order_acquire);
            host_pixels=display_queue.data+
                (size_t)((serial-1)%display_queue.h->slots)*display_queue.h->slot_bytes+VDQ_META;
        }

        /* Copy while the DMA buffer is CHECKED_OUT; it becomes reusable below. */
        if (shm_pub.header || detect_shm_pub.header) {
            uint64_t publish_ns = info.ready_ns ? info.ready_ns : monotonic_ns();

            uint64_t analysis_now = monotonic_ns();
            if (shm_pub.header && (analysis_shm_fps <= 0.0 ||
                                  analysis_now >= analysis_next_ns)) {
                stage_begin=monotonic_ns();
                frame_shm_publish(&shm_pub, host_pixels, len, info.frame_seq,
                                  publish_ns);
                analysis_cost_ns+=monotonic_ns()-stage_begin;
                analysis_next_ns = analysis_now +
                    (analysis_shm_fps > 0.0 ? (uint64_t)(1e9/analysis_shm_fps) : 0);
            }
            if (detect_shm_pub.header && !host_yolo_mode) {
                int publish_detector = 0;

                uint32_t request = detect_on_demand ?
                    atomic_load_explicit(&detect_shm_pub.header->request_seq,
                                         memory_order_acquire) : 0;
                if (detect_on_demand) {
                    publish_detector = request != detect_last_request;
                } else if (detect_publish_fps <= 0.0 ||
                           detect_publish_fps >= fps) {
                    publish_detector = 1;
                } else if (!detect_last_frame_valid) {
                    publish_detector = 1;
                } else {
                    uint32_t sequence_delta =
                        info.frame_seq - detect_last_frame_seq;

                    detect_publish_credit +=
                        (double)sequence_delta * detect_publish_fps / fps;
                    if (detect_publish_credit >= 1.0) {
                        publish_detector = 1;
                        detect_publish_credit -= floor(detect_publish_credit);
                    }
                }
                detect_last_frame_seq = info.frame_seq;
                detect_last_frame_valid = 1;
                if (publish_detector) {
                    stage_begin=monotonic_ns();
                    frame_shm_publish_letterbox_rgb(
                        &detect_shm_pub, (const uint8_t *)host_pixels,
                        width, height, rgb_line_bytes,
                        info.frame_seq, publish_ns);
                    detector_cost_ns+=monotonic_ns()-stage_begin;
                    if (detect_on_demand) {
                        detect_last_request = request;
                        atomic_store_explicit(&detect_shm_pub.header->served_request_seq,
                                              request, memory_order_release);
                        frame_shm_notify(detect_shm_pub.header);
                    }
                    detect_published_total++;
                    detect_report_published++;
                } else {
                    detect_throttled_total++;
                    detect_report_throttled++;
                }
            }
        }

        if (!no_display) {
            display_events(&display);
            if (box_reader.name)
                box_shm_refresh(&box_reader, monotonic_ns());
            if (display_frame(&display, rgb, width, height, gl_finish,
                              box_reader.name ? &box_reader : NULL) < 0) {
                (void)camera_dma_release_frame(cam, &info);
                goto out;
            }
        }
        /* With gl_finish enabled, the texture upload and draw have completed
         * before this DMA slot is returned to the capture pool. Disabling it
         * is intended only for controlled performance comparisons. */
        if (camera_dma_release_frame(cam, &info) < 0) {
            fprintf(stderr, "DMA buffer release failed: %s\n", camera_last_error(cam));
            goto out;
        }

        frames++;
        report_frames++;
        now = monotonic_ns();
        if (now - reported >= 1000000000ull) {
            double instant = (double)report_frames * 1e9 / (double)(now - reported);
            double average = (double)frames * 1e9 / (double)(now - started);
            fprintf(stderr,
                    "frames=%llu fps=%.2f avg=%.2f target=%.2f seq=%u drops=%u "
                    "format=%s mode=%s latency=%s "
                    "skip_host=%llu skip_ddr=%llu preview_skip=%llu\n",
                    (unsigned long long)frames, instant, average, fps,
                    info.frame_seq, info.drop_count,
                    raw_bayer ? "BayerRG8-RAW" : "RGB888-FPGA",
                    no_display ? "capture-only" : "display",
                    latest_frame ? "latest-complete" : "fifo",
                    (unsigned long long)host_skipped_total,
                    (unsigned long long)ddr_skipped_total,
                    (unsigned long long)preview_skipped_total);
            fprintf(stderr,"HOST_STAGES display_copy_ms=%.3f analysis_ms=%.3f detector_ms=%.3f samples=%llu\n",vdq_cost_ns/1e6/report_frames,analysis_cost_ns/1e6/report_frames,detector_cost_ns/1e6/report_frames,(unsigned long long)report_frames);
            vdq_cost_ns=analysis_cost_ns=detector_cost_ns=0;
            if (debug_csr)
                print_csr(cam);
            if (detect_shm_pub.header) {
                if (host_yolo_mode) {
                    uint64_t total = atomic_load_explicit(&host_yolo.published, memory_order_relaxed);
                    detect_report_published = total - detect_published_total;
                    detect_published_total = total;
                }
                double detector_pub_fps =
                    (double)detect_report_published * 1e9 /
                    (double)(now - reported);
                fprintf(stderr,
                        "detector_pub fps=%.2f interval=%llu/%llu "
                        "total=%llu throttled=%llu input=%ux%u\n",
                        detector_pub_fps,
                        (unsigned long long)detect_report_published,
                        (unsigned long long)detect_report_throttled,
                        (unsigned long long)detect_published_total,
                        (unsigned long long)detect_throttled_total,
                        detect_width, detect_height);
            }
            if (box_reader.name) {
                double box_age_ms = box_reader.timestamp_ns ?
                    (double)(now - box_reader.timestamp_ns) / 1000000.0 : -1.0;
                fprintf(stderr,
                        "overlay boxes=%u camera_seq=%llu age_ms=%.2f "
                        "yolo_fps=%.2f connected=%u\n",
                        box_reader.count,
                        (unsigned long long)box_reader.frame_seq,
                        box_age_ms, box_reader.detect_fps,
                        box_reader.header ? 1u : 0u);
            }
            if (display_ready)
                display_set_status_title(
                    &display, instant,
                    box_reader.name ? &box_reader : NULL);
            reported = now;
            report_frames = 0;
            detect_report_published = 0;
            detect_report_throttled = 0;
        }
    }
    ret = 0;

out:
    vdq_finish(&display_queue, 2000);
    host_yolo_stop(&host_yolo);
    box_shm_destroy(&box_reader);
    frame_shm_destroy(&detect_shm_pub);
    frame_shm_destroy(&shm_pub);
    if (cam)
        camera_close(cam);
    if (display_ready)
        display_destroy(&display);
    return ret;

bad_args:
    usage(argv[0]);
    return 2;
}
