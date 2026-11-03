#ifndef VIDEO_HOST_DIRECT_H
#define VIDEO_HOST_DIRECT_H
#include <stdint.h>
#include <stddef.h>
#define VIDEO_HOST_BUILD_MIN 0x0003002dU
#define VIDEO_HOST_CONTROL 0xa00U
#define VIDEO_HOST_SLOTS 0xa04U
#define VIDEO_HOST_CONSUMER 0xa0cU
#define VIDEO_HOST_STATUS 0xa10U
#define VIDEO_HOST_PRODUCER 0xa14U
#define VIDEO_HOST_STALL 0xa18U
#define VIDEO_HOST_ERROR 0xa1cU
#define VIDEO_HOST_DEMAND 0xa24U
#define VIDEO_HOST_ADDRESSES 0xa80U
#define VIDEO_DDR_FRAME_IDS 0xd00U
#define VIDEO_DROP_HISTOGRAM 0xc00U
#define VIDEO_HOST_MAGIC 0x31485349U
struct video_host_footer {
    uint32_t magic,version;
    uint64_t frame_id;
    uint32_t bytes,width,height,flags,ddr_index,format;
    uint8_t reserved[16];
    uint64_t token;
};
_Static_assert(sizeof(struct video_host_footer)==64,"direct-host footer ABI");
_Static_assert(offsetof(struct video_host_footer,token)==56,"direct-host footer publication fence");
static inline int video_host_footer_valid(const struct video_host_footer *f,
    uint64_t token,uint32_t w,uint32_t h,uint32_t format,uint32_t bytes) {
    return f->token==token && f->magic==VIDEO_HOST_MAGIC && f->version==1 &&
        f->width==w && f->height==h && f->format==format && f->bytes==bytes &&
        (f->flags&~3U)==0 && f->frame_id<=UINT32_MAX;
}
/* flags bit 0 proves complete host pixels; bit 1 proves DDR acceptance.
 * A full DDR ring can reject a NEW frame without corrupting either the
 * complete host copy or any committed DDR slot. Never manufacture a DDR
 * reference for that host-only frame. A matching committed frame ID, a
 * future DDR index, or any hard capture error is still a fatal disagreement.
 */
static inline int video_host_footer_ring_full(const struct video_host_footer *f,
    uint32_t reason, uint32_t hard_errors, uint32_t producer,
    uint32_t committed_frame_id) {
    return f->flags == 1U && reason == 16U && hard_errors == 0U &&
        f->ddr_index <= producer &&
        (f->ddr_index == producer || committed_frame_id != (uint32_t)f->frame_id);
}
#endif
