#ifndef VIDEO_DISPLAY_H
#define VIDEO_DISPLAY_H
/* Ordered, single-producer/single-consumer host display queue.
 * Pixels remain immutable until successful GPU submission is acknowledged.
 * This is separate from the latest-frame analysis/YOLO mailbox and all DMA MRs.
 * Queue full is explicit backpressure; unpresented frames are never overwritten. */
#include <stdint.h>
#include <stddef.h>
#include <stdatomic.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <time.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/file.h>
#include "video_boxes.h"
#define VDQ_MAGIC UINT32_C(0x31514456)
#define VDQ_PAGE 4096U
#define VDQ_SLOTS 32U
#define VDQ_META (64U+VIDEO_BOX_BYTES)
struct vdq_header {
 uint32_t magic,version,header_bytes,pixel_format,width,height,stride_bytes,frame_bytes,slots,slot_bytes;
 uint8_t pad0[24];
 _Atomic uint64_t produced,full_waits,high_water,producer_ns;
 _Atomic uint32_t closed,error;
 uint8_t pad1[24];
 _Atomic uint64_t consumed,consumer_ns,present_ns,frame_seq,source_gaps,max_render_ns,total_render_ns;
 _Atomic uint32_t consumer_pid;
 uint32_t reserved;
};
_Static_assert(offsetof(struct vdq_header,produced)==64,"VDQ producer cache line");
_Static_assert(offsetof(struct vdq_header,consumed)==128,"VDQ consumer cache line");
_Static_assert(sizeof(struct vdq_header)==192,"VDQ header ABI");
_Static_assert(ATOMIC_LLONG_LOCK_FREE==2,"Process-shared counters must be lock free");
struct vdq_slot {uint64_t serial,frame_seq,timestamp_ns,pad[5];struct video_boxes boxes;};
_Static_assert(sizeof(struct vdq_slot)==VDQ_META,"VDQ slot ABI");
struct vdq {
 int fd,lock_fd,owner;
 size_t length;
 struct vdq_header *h;
 uint8_t *data;
 dev_t device;ino_t inode;
};
static inline uint64_t vdq_ns(void) {
 struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);
 return (uint64_t)t.tv_sec*1000000000ULL+(uint64_t)t.tv_nsec;
}
static inline void vdq_clear(struct vdq *q){memset(q,0,sizeof(*q));q->fd=q->lock_fd=-1;}
static inline int vdq_format(uint32_t fmt,uint32_t *bpp) {
 *bpp=fmt==UINT32_C(0x42474752)?1U:(fmt==UINT32_C(0x33424752)||fmt==UINT32_C(0x33524742))?3U:0U;
 return *bpp!=0;
}
static inline int vdq_geometry(uint32_t w,uint32_t h,uint32_t fmt,uint32_t *bytes) {
 uint32_t bpp;if(!vdq_format(fmt,&bpp)||!w||!h||w>4096||h>4096||
 (bpp==1&&((w|h)&1))||(uint64_t)w*h*bpp>16U*1024U*1024U)return 0;
 *bytes=w*h*bpp;return 1;
}
static inline void vdq_unmap(struct vdq *q) {
 if(q->h) {
  if(!q->owner)atomic_store_explicit(&q->h->consumer_pid,0,memory_order_release);
  munmap(q->h,q->length);
 }
 if(q->fd>=0)close(q->fd);
 if(q->lock_fd>=0)close(q->lock_fd);
 vdq_clear(q);
}
static inline int vdq_permissions(int fd,const char *path) {
 char parent[4096];struct stat st;
 const char *slash=strrchr(path,'/');size_t n=slash?(size_t)(slash-path):0;
 if(n>=sizeof(parent)){errno=ENAMETOOLONG;return -1;}
 if(slash){if(!n)n=1;memcpy(parent,path,n);parent[n]=0;}else strcpy(parent,".");
 if(stat(parent,&st))return -1;
 if(geteuid()==0&&fchown(fd,st.st_uid,st.st_gid))return -1;
 return fchmod(fd,0600);
}
static inline int vdq_create(struct vdq *q,const char *path,uint32_t w,uint32_t h,uint32_t fmt,unsigned slots) {
 uint32_t bytes,bpp;char lock[4096],tmp[4096];struct stat st;
 vdq_clear(q);
 if(!path||!*path||!vdq_geometry(w,h,fmt,&bytes)||slots<2||slots>64){errno=EINVAL;return -1;}
 if(snprintf(lock,sizeof(lock),"%s.producer.lock",path)>=(int)sizeof(lock)||
    snprintf(tmp,sizeof(tmp),"%s.new.%ld",path,(long)getpid())>=(int)sizeof(tmp)){errno=ENAMETOOLONG;return -1;}
 q->lock_fd=open(lock,O_RDWR|O_CREAT|O_NOFOLLOW|O_CLOEXEC,0600);
 if(q->lock_fd<0)return -1;
 if(vdq_permissions(q->lock_fd,path)||flock(q->lock_fd,LOCK_EX|LOCK_NB))goto bad;
 q->fd=open(tmp,O_RDWR|O_CREAT|O_EXCL|O_NOFOLLOW|O_CLOEXEC,0600);
 if(q->fd<0)goto bad;
 uint32_t slot_bytes=(VDQ_META+bytes+63U)&~63U;
 q->length=VDQ_PAGE+(size_t)slot_bytes*slots;
 if(vdq_permissions(q->fd,path)||ftruncate(q->fd,(off_t)q->length))goto bad_tmp;
 int alloc_error=posix_fallocate(q->fd,0,(off_t)q->length);
 if(alloc_error){errno=alloc_error;goto bad_tmp;}
 q->h=mmap(NULL,q->length,PROT_READ|PROT_WRITE,MAP_SHARED,q->fd,0);
 if(q->h==MAP_FAILED){q->h=NULL;goto bad_tmp;}
 memset(q->h,0,VDQ_PAGE);vdq_format(fmt,&bpp);
 q->h->magic=VDQ_MAGIC;q->h->version=1;q->h->header_bytes=VDQ_PAGE;q->h->pixel_format=fmt;
 q->h->width=w;q->h->height=h;q->h->stride_bytes=w*bpp;q->h->frame_bytes=bytes;
 q->h->slots=slots;q->h->slot_bytes=slot_bytes;
 atomic_store(&q->h->producer_ns,vdq_ns());q->data=(uint8_t*)q->h+VDQ_PAGE;q->owner=1;
 if(fstat(q->fd,&st)||rename(tmp,path))goto bad_tmp;
 q->device=st.st_dev;q->inode=st.st_ino;return 0;
bad_tmp:
 {int saved=errno;unlink(tmp);vdq_unmap(q);errno=saved;return -1;}
bad:
 {int saved=errno;vdq_unmap(q);errno=saved;return -1;}
}
static inline int vdq_open(struct vdq *q,const char *path) {
 struct stat st;uint32_t bytes;vdq_clear(q);
 q->fd=open(path,O_RDWR|O_NOFOLLOW|O_CLOEXEC);
 if(q->fd<0)return -1;
 if(flock(q->fd,LOCK_EX|LOCK_NB))goto bad;
 if(fstat(q->fd,&st)||st.st_size<VDQ_PAGE||st.st_size>VDQ_PAGE+64LL*(16U*1024U*1024U+VDQ_META+64))goto bad;
 q->length=(size_t)st.st_size;q->h=mmap(NULL,q->length,PROT_READ|PROT_WRITE,MAP_SHARED,q->fd,0);
 if(q->h==MAP_FAILED){q->h=NULL;goto bad;}
 struct vdq_header *h=q->h;
 if(h->magic!=VDQ_MAGIC||h->version!=1||h->header_bytes!=VDQ_PAGE||
 !vdq_geometry(h->width,h->height,h->pixel_format,&bytes)||bytes!=h->frame_bytes||
 h->slots<2||h->slots>64||h->slot_bytes!=((VDQ_META+bytes+63U)&~63U)||
 q->length!=VDQ_PAGE+(size_t)h->slots*h->slot_bytes){errno=EPROTO;goto bad;}
 uint32_t bpp;vdq_format(h->pixel_format,&bpp);
 if(h->stride_bytes!=h->width*bpp){errno=EPROTO;goto bad;}
 q->data=(uint8_t*)q->h+VDQ_PAGE;q->device=st.st_dev;q->inode=st.st_ino;
 atomic_store_explicit(&h->consumer_ns,vdq_ns(),memory_order_release);
 atomic_store_explicit(&h->consumer_pid,(uint32_t)getpid(),memory_order_release);return 0;
bad:
 {int saved=errno;/* Do not write into an invalid mapping. */
 if(q->h)munmap(q->h,q->length);
 if(q->fd>=0)close(q->fd);
 vdq_clear(q);errno=saved;return -1;}
}
/* timeout_ms limits an unrecoverable backlog; never evict an unread frame.
 * Caller retains ownership of the source pixels on every return path. */
static inline int vdq_publish(struct vdq *q,const void *pixels,uint64_t seq,uint64_t stamp,
 const struct video_boxes *boxes,const volatile sig_atomic_t *stop,unsigned timeout_ms) {
 if(!q->h||!q->owner||!pixels){errno=EINVAL;return -1;}
 struct vdq_header *h=q->h;
 uint64_t w=atomic_load_explicit(&h->produced,memory_order_relaxed),r;
 uint64_t deadline=vdq_ns()+(uint64_t)timeout_ms*1000000ULL;int waited=0;
 for(;;) {
  r=atomic_load_explicit(&h->consumed,memory_order_acquire);
  if(r>w||w-r>h->slots){errno=EPROTO;atomic_store(&h->error,1);return -1;}
  if(w-r<h->slots)break;
  if(!waited){atomic_fetch_add(&h->full_waits,1);waited=1;}
  if(stop&&*stop){errno=ECANCELED;return -1;}
  if(vdq_ns()>=deadline){if(!timeout_ms){errno=EAGAIN;return -1;}errno=ETIMEDOUT;atomic_store(&h->error,2);return -1;}
  atomic_store_explicit(&h->producer_ns,vdq_ns(),memory_order_release);usleep(50);
 }
 struct vdq_slot *s=(struct vdq_slot*)(q->data+(size_t)(w%h->slots)*h->slot_bytes);
 s->serial=w+1;s->frame_seq=seq;s->timestamp_ns=stamp;
 if(boxes)memcpy(&s->boxes,boxes,sizeof(*boxes));else memset(&s->boxes,0,sizeof(s->boxes));
 memcpy((uint8_t*)s+VDQ_META,pixels,h->frame_bytes);
 uint64_t depth=w+1-r,high=atomic_load(&h->high_water);
 if(depth>high)atomic_store(&h->high_water,depth);
 atomic_store_explicit(&h->producer_ns,vdq_ns(),memory_order_release);
 atomic_store_explicit(&h->produced,w+1,memory_order_release);return 0;
}
/* Keep the local preview current without changing capture or RDMA ownership.
 * Only the consumer calls this, before touching a slot; the producer cannot
 * overwrite the newest unread slot until it is acknowledged. */
static inline int vdq_skip_to_latest(struct vdq *q,unsigned max_pending) {
 struct vdq_header *h=q->h;if(!h||q->owner){errno=EINVAL;return -1;}
 uint64_t r=atomic_load_explicit(&h->consumed,memory_order_relaxed);
 uint64_t w=atomic_load_explicit(&h->produced,memory_order_acquire);
 if(r>w||w-r>h->slots){errno=EPROTO;return -1;}
 if(w-r<=max_pending||w==r)return 0;
 atomic_store_explicit(&h->consumed,w-1,memory_order_release);
 return (int)(w-r-1);
}
static inline int vdq_peek(struct vdq *q,struct vdq_slot **slot) {
 struct vdq_header *h=q->h;if(!h){errno=EINVAL;return -1;}
 uint64_t r=atomic_load_explicit(&h->consumed,memory_order_relaxed);
 uint64_t w=atomic_load_explicit(&h->produced,memory_order_acquire);
 atomic_store_explicit(&h->consumer_ns,vdq_ns(),memory_order_release);
 if(r>w||w-r>h->slots){errno=EPROTO;return -1;}
 if(r==w)return 0;
 *slot=(struct vdq_slot*)(q->data+(size_t)(r%h->slots)*h->slot_bytes);
 if((*slot)->serial!=r+1){errno=EPROTO;return -1;}return 1;
}
static inline int vdq_ack(struct vdq *q,uint64_t serial,uint64_t render_ns) {
 struct vdq_header *h=q->h;
 uint64_t r=atomic_load_explicit(&h->consumed,memory_order_relaxed);
 if(serial!=r+1||serial>atomic_load_explicit(&h->produced,memory_order_acquire)){errno=EPROTO;return -1;}
 struct vdq_slot *s=(struct vdq_slot*)(q->data+(size_t)(r%h->slots)*h->slot_bytes);
 uint64_t old_seq=atomic_load(&h->frame_seq);
 if(r){uint32_t delta=(uint32_t)s->frame_seq-(uint32_t)old_seq;
  if(delta>1&&delta<UINT32_C(0x80000000))atomic_fetch_add(&h->source_gaps,delta-1);}
 atomic_store(&h->frame_seq,s->frame_seq);atomic_store(&h->present_ns,vdq_ns());
 atomic_fetch_add(&h->total_render_ns,render_ns);
 if(render_ns>atomic_load(&h->max_render_ns))atomic_store(&h->max_render_ns,render_ns);
 atomic_store_explicit(&h->consumed,serial,memory_order_release);return 0;
}
static inline void vdq_finish(struct vdq *q,unsigned drain_ms) {
 if(!q->h)return;
 if(q->owner) {
  atomic_store_explicit(&q->h->closed,1,memory_order_release);
  uint64_t until=vdq_ns()+(uint64_t)drain_ms*1000000ULL;
  while(atomic_load(&q->h->consumed)<atomic_load(&q->h->produced)&&vdq_ns()<until&&
        atomic_load(&q->h->consumer_pid))usleep(100);
  fprintf(stderr,"DISPLAY_DRAIN produced=%llu presented=%llu pending=%llu source_gaps=%llu full_waits=%llu high_water=%llu error=%u\n",
   (unsigned long long)atomic_load(&q->h->produced),(unsigned long long)atomic_load(&q->h->consumed),
   (unsigned long long)(atomic_load(&q->h->produced)-atomic_load(&q->h->consumed)),
   (unsigned long long)atomic_load(&q->h->source_gaps),(unsigned long long)atomic_load(&q->h->full_waits),
   (unsigned long long)atomic_load(&q->h->high_water),atomic_load(&q->h->error));
 }
 vdq_unmap(q);
}
#endif
