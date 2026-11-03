#ifndef VIDEO_DDR_H
#define VIDEO_DDR_H
/* Session-local ownership tokens; payload bytes remain in FPGA DDR.
 * Only capture owns the hardware consumer CSR. Neither completion alone frees a slot. */
#include <stdint.h>
#include "video_integrity.h"
#include <stddef.h>
#include <stdatomic.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/file.h>
#include <time.h>
_Static_assert(ATOMIC_LLONG_LOCK_FREE==2 && ATOMIC_INT_LOCK_FREE==2,"shared DDR tokens require lock-free atomics");
#define DDR_RING_MAGIC 0x31524444U
#define DDR_RING_VERSION 1U
#define DDR_RING_SLOTS 32U
#define DDR_DEVICE_BASE UINT64_C(0xa350000000000000)
enum {DDR_SENDER_WAIT=0,DDR_SENDER_ACTIVE=1,DDR_SENDER_DRAINED=2,DDR_SENDER_FAULT=3};
struct ddr_frame {
 _Atomic uint64_t token;
 uint64_t frame_id,ready_ns,addr;
 uint32_t bytes,width,height,format;
 _Atomic uint64_t dma_done,rdma_done;
 uint32_t crc;
 uint8_t pad[60];
};
_Static_assert(sizeof(struct ddr_frame)==128,"DDR descriptor size");
struct ddr_ring {
 _Atomic uint32_t magic;
 uint32_t version,slots,width,height,format,bytes,start_idx;
 _Atomic uint32_t sender_state,owner_stop;
 _Atomic uint64_t published,released;
 uint32_t crc_mode; /* Immutable before magic is published; zero is legacy/full. */
 uint32_t host_path; /* 0: legacy DDR D2H, 1: independent ISP->host writer. */
 uint64_t box_addr; /* Reserved after the video ring; never overlaps video slots. */
 uint8_t pad[56];
 struct ddr_frame frames[DDR_RING_SLOTS];
};
_Static_assert(offsetof(struct ddr_ring,frames)==128,"DDR ring header size");
static inline uint64_t ddr_ns(void) {
 struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);
 return (uint64_t)t.tv_sec*1000000000ULL+t.tv_nsec;
}
static inline struct ddr_ring *ddr_map(const char *path,int create,int *fdout) {
 int fd=open(path,(create?(O_RDWR|O_CREAT|O_EXCL):O_RDWR)|O_CLOEXEC|O_NOFOLLOW,0600);
 if(fd<0)return NULL;
 struct stat st;
 if((create&&(flock(fd,LOCK_EX|LOCK_NB)||ftruncate(fd,sizeof(struct ddr_ring))))||
    fstat(fd,&st)||!S_ISREG(st.st_mode)||st.st_size!=(off_t)sizeof(struct ddr_ring)) {
  int saved=errno?errno:EPROTO;close(fd);errno=saved;return NULL;
 }
 void *p=mmap(NULL,sizeof(struct ddr_ring),PROT_READ|PROT_WRITE,MAP_SHARED,fd,0);
 if(p==MAP_FAILED){close(fd);return NULL;}
 *fdout=fd;return p;
}
static inline int ddr_valid(const struct ddr_ring *r) {
 return atomic_load_explicit(&r->magic,memory_order_acquire)==DDR_RING_MAGIC &&
 r->version==DDR_RING_VERSION && r->slots==DDR_RING_SLOTS && r->crc_mode<=VIDEO_CRC_TRANSPORT && r->host_path<=1 && r->width && r->height &&
 (r->format==0x33424752U||r->format==0x42474752U) &&
 (uint64_t)r->width*r->height*(r->format==0x33424752U?3:1)==r->bytes && r->bytes<=16U*1024U*1024U;
}
static inline void ddr_init_paths(struct ddr_ring *r,uint32_t w,uint32_t h,uint32_t fmt,uint32_t bytes,uint32_t idx,uint32_t mode,uint32_t host_path,uint64_t box_addr) {
 memset(r,0,sizeof(*r));r->version=DDR_RING_VERSION;r->slots=DDR_RING_SLOTS;
 r->width=w;r->height=h;r->format=fmt;r->bytes=bytes;r->start_idx=idx;r->crc_mode=mode;r->host_path=host_path;r->box_addr=box_addr;
 atomic_store(&r->published,idx);atomic_store(&r->released,idx);
 atomic_store_explicit(&r->magic,DDR_RING_MAGIC,memory_order_release);
}
static inline void ddr_init_mode(struct ddr_ring *r,uint32_t w,uint32_t h,uint32_t fmt,uint32_t bytes,uint32_t idx,uint32_t mode) {
 ddr_init_paths(r,w,h,fmt,bytes,idx,mode,0,0);
}
static inline void ddr_init(struct ddr_ring *r,uint32_t w,uint32_t h,uint32_t fmt,uint32_t bytes,uint32_t idx) {
 ddr_init_mode(r,w,h,fmt,bytes,idx,VIDEO_CRC_FULL);
}
static inline struct ddr_frame *ddr_frame_at(struct ddr_ring *r,uint32_t idx) {
 return &r->frames[idx%DDR_RING_SLOTS];
}
static inline int ddr_publish(struct ddr_ring *r,uint32_t idx,uint64_t frame_id,uint64_t stamp,uint64_t addr) {
 struct ddr_frame *f=ddr_frame_at(r,idx);
 uint64_t old=atomic_load_explicit(&f->token,memory_order_acquire);
 /* Full CRC still requires a host-direct reference. In transport mode only
  * the RDMA reader owns the DDR slot; a slow preview cannot retain it. */
 int needs_reference=!r->host_path || r->crc_mode==VIDEO_CRC_FULL;
 if(idx==UINT32_MAX || (old && ((needs_reference &&
    atomic_load_explicit(&f->dma_done,memory_order_acquire)!=old) ||
    atomic_load_explicit(&f->rdma_done,memory_order_acquire)!=old))){errno=EBUSY;return -1;}
 f->frame_id=frame_id;f->ready_ns=stamp;f->addr=addr;
 f->bytes=r->bytes;f->width=r->width;f->height=r->height;f->format=r->format;f->crc=0;
 atomic_store(&f->dma_done,0);atomic_store(&f->rdma_done,0);
 atomic_store_explicit(&f->token,(uint64_t)idx+1,memory_order_release);
 atomic_store_explicit(&r->published,(uint64_t)idx+1,memory_order_release);
 return 0;
}
static inline uint32_t ddr_release_ready(struct ddr_ring *r,uint32_t cursor) {
 while((uint64_t)cursor<atomic_load_explicit(&r->published,memory_order_acquire)) {
  struct ddr_frame *f=ddr_frame_at(r,cursor);uint64_t token=(uint64_t)cursor+1;
  if(atomic_load_explicit(&f->token,memory_order_acquire)!=token ||
     ((!r->host_path || r->crc_mode==VIDEO_CRC_FULL) &&
      atomic_load_explicit(&f->dma_done,memory_order_acquire)!=token) ||
     atomic_load_explicit(&f->rdma_done,memory_order_acquire)!=token)break;
  ++cursor;
 }
 atomic_store_explicit(&r->released,cursor,memory_order_release);return cursor;
}
#endif
