#ifndef VIDEO_COMMON_H
#define VIDEO_COMMON_H
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include "rn_mlx5_protocol.h"
#include <stdatomic.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/file.h>
#include <signal.h>
#include <zlib.h>
#define VIDEO_MAGIC UINT32_C(0x56494431)
#define VIDEO_VERSION 3U
#define VIDEO_WINDOW 4U
#define VIDEO_MAX_BYTES (16U*1024U*1024U)
#define VIDEO_CHUNK (256U*1024U)
#define VIDEO_FRAME 1U
#define VIDEO_END 2U
#define CSHM_MAGIC UINT32_C(0x4d485343)
#define CSHM_RGB UINT32_C(0x33424752)
#define CSHM_BGR UINT32_C(0x33524742)
#define CSHM_BAYER UINT32_C(0x42474752)
#define CSHM_PAGE 4096U
#include "video_boxes.h"
struct video_header {
 uint32_t magic,version,type,format;
 uint32_t width,height,stride,bytes;
 uint64_t frame_id,publish_seq,capture_ns;
 uint32_t crc,reserved;
};
struct video_ack {uint32_t magic,status;uint64_t frame_id;};
_Static_assert(sizeof(struct video_header)==64,"wire header size");
_Static_assert(sizeof(struct video_ack)==16,"wire ACK size");
struct cshm_header {
 uint32_t magic,version,header_bytes,pixel_format,width,height,stride_bytes,frame_bytes,buffer_count,reserved0;
 _Atomic uint64_t publish_seq;
 _Atomic uint32_t active_index;
 uint32_t reserved1;
 _Atomic uint64_t slot_guard[2];
 uint64_t frame_seq[2],timestamp_ns[2];
 uint32_t source_width,source_height,content_x,content_y,content_width,content_height;
 _Atomic uint32_t notify_seq;
 uint32_t reserved2;
};
_Static_assert(offsetof(struct cshm_header,publish_seq)==40,"camera SHM sequence ABI");
_Static_assert(offsetof(struct cshm_header,slot_guard)==56,"camera SHM guard ABI");
_Static_assert(offsetof(struct cshm_header,frame_seq)==72,"camera SHM frame ABI");
_Static_assert(offsetof(struct cshm_header,notify_seq)==128,"camera SHM notify ABI");
struct video_shm {int fd;size_t length;struct cshm_header *h;uint8_t *data;const char *name;int owner;int regular;};
static inline uint64_t video_ns(void) {struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);return (uint64_t)t.tv_sec*1000000000ULL+t.tv_nsec;}
static inline uint32_t video_bpp(uint32_t fmt) {return fmt==CSHM_BAYER?1U:(fmt==CSHM_RGB||fmt==CSHM_BGR)?3U:0U;}
static inline int video_geometry(uint32_t w,uint32_t h,uint32_t stride,uint32_t bytes,uint32_t fmt) {
 return w && h && w<=4096 && h<=4096 && video_bpp(fmt) && (fmt!=CSHM_BAYER || (!(w&1) && !(h&1))) &&
        stride==(uint64_t)w*video_bpp(fmt) && bytes==(uint64_t)stride*h && bytes<=VIDEO_MAX_BYTES;
}
static inline int video_shm_name(const char *name) {return name && name[0]=='/' && name[1] && !strchr(name+1,'/');}
static inline void video_shm_close(struct video_shm *s) {
 if(s->owner && s->name) {if(s->regular)unlink(s->name);else shm_unlink(s->name);}
 if(s->h)munmap(s->h,s->length);
 if(s->fd>=0)close(s->fd);
 s->h=NULL;s->fd=-1;s->owner=0;
}
static inline int video_map_read_open(struct video_shm *s,const char *name,int regular) {
 struct stat st;
 memset(s,0,sizeof(*s));s->fd=-1;
 if(!name || (regular ? strncmp(name,"./",2)!=0 : !video_shm_name(name))){errno=EINVAL;return -1;}
 s->regular=regular;
 s->fd=regular?open(name,O_RDONLY|O_NOFOLLOW):shm_open(name,O_RDONLY,0);if(s->fd<0)return -1;
 if(fstat(s->fd,&st)||st.st_size<(off_t)CSHM_PAGE||st.st_size>(off_t)(CSHM_PAGE+2ULL*VIDEO_MAX_BYTES))goto bad;
 s->length=(size_t)st.st_size;
 s->h=mmap(NULL,s->length,PROT_READ,MAP_SHARED,s->fd,0);
 if(s->h==MAP_FAILED){s->h=NULL;goto bad;}
 struct cshm_header *h=s->h;
 if(h->magic!=CSHM_MAGIC||h->version!=3||h->header_bytes!=CSHM_PAGE||h->buffer_count!=2||
    !video_geometry(h->width,h->height,h->stride_bytes,h->frame_bytes,h->pixel_format)||
    s->length!=CSHM_PAGE+2ULL*h->frame_bytes)goto bad;
 s->data=(uint8_t*)s->h+CSHM_PAGE;return 0;
bad: video_shm_close(s);errno=EPROTO;return -1;
}
/* Copy a whole immutable snapshot; never hold a camera DMA buffer over RDMA.
 * Return 1=new stable snapshot, 0=old/busy, -1=invalid format. */
static inline int video_shm_snapshot(struct video_shm *s,uint64_t previous,uint8_t *out,size_t capacity,struct video_header *m) {
 struct cshm_header *h=s->h;
 if(!h||h->frame_bytes>capacity)return -1;
 uint64_t pub=atomic_load_explicit(&h->publish_seq,memory_order_acquire);
 uint32_t slot=atomic_load_explicit(&h->active_index,memory_order_acquire);
 if(!pub||pub==previous||slot>1||slot!=(pub&1))return 0;
 uint64_t guard=atomic_load_explicit(&h->slot_guard[slot],memory_order_acquire);
 if((guard&1)||guard!=((pub+1)/2)*2)return 0;
 uint64_t seq=h->frame_seq[slot],stamp=h->timestamp_ns[slot];
 memcpy(out,s->data+(size_t)slot*h->frame_bytes,h->frame_bytes);
 atomic_thread_fence(memory_order_seq_cst);
 if(guard!=atomic_load_explicit(&h->slot_guard[slot],memory_order_acquire)||
    pub!=atomic_load_explicit(&h->publish_seq,memory_order_acquire))return 0;
 memset(m,0,sizeof(*m));m->magic=htonl(VIDEO_MAGIC);m->version=htonl(VIDEO_VERSION);m->type=htonl(VIDEO_FRAME);
 m->format=htonl(h->pixel_format);m->width=htonl(h->width);m->height=htonl(h->height);
 m->stride=htonl(h->stride_bytes);m->bytes=htonl(h->frame_bytes);
 m->frame_id=htobe64(seq);m->publish_seq=htobe64(pub);m->capture_ns=htobe64(stamp);
 return 1;
}
static inline int video_map_write_open(struct video_shm *s,const char *name,uint32_t w,uint32_t h,uint32_t fmt,int regular) {
 memset(s,0,sizeof(*s));s->fd=-1;
 if(!name || (regular ? strncmp(name,"./",2)!=0 : !video_shm_name(name))||!video_geometry(w,h,w*video_bpp(fmt),w*h*video_bpp(fmt),fmt)){errno=EINVAL;return -1;}
 s->regular=regular;
 s->fd=regular?open(name,O_RDWR|O_CREAT|O_EXCL|O_NOFOLLOW,0644):shm_open(name,O_RDWR|O_CREAT|O_EXCL,0644);if(s->fd<0)return -1;
 if(flock(s->fd,LOCK_EX|LOCK_NB)){close(s->fd);s->fd=-1;return -1;}
 s->name=name;s->owner=1;s->length=CSHM_PAGE+2ULL*w*h*video_bpp(fmt);
 if(ftruncate(s->fd,(off_t)s->length)||fchmod(s->fd,0644))goto bad;
 s->h=mmap(NULL,s->length,PROT_READ|PROT_WRITE,MAP_SHARED,s->fd,0);
 if(s->h==MAP_FAILED){s->h=NULL;goto bad;}
 memset(s->h,0,s->length);s->data=(uint8_t*)s->h+CSHM_PAGE;
 struct cshm_header *p=s->h;p->magic=CSHM_MAGIC;p->version=3;p->header_bytes=CSHM_PAGE;p->pixel_format=fmt;
 p->width=w;p->height=h;p->stride_bytes=w*video_bpp(fmt);p->frame_bytes=w*h*video_bpp(fmt);p->buffer_count=2;
 p->source_width=w;p->source_height=h;p->content_width=w;p->content_height=h;
 return 0;
bad:video_shm_close(s);return -1;
}
static inline int video_shm_read_open(struct video_shm *s,const char *name) {return video_map_read_open(s,name,0);}
static inline int video_shm_write_open(struct video_shm *s,const char *name,uint32_t w,uint32_t h,uint32_t fmt) {return video_map_write_open(s,name,w,h,fmt,0);}
static inline void video_shm_publish_boxes(struct video_shm *s,const uint8_t *data,uint64_t seq,uint64_t stamp,const struct video_boxes *boxes) {
 struct cshm_header *h=s->h;uint32_t slot=(atomic_load(&h->active_index)^1)&1;
 uint64_t guard=atomic_load(&h->slot_guard[slot]);
 atomic_store_explicit(&h->slot_guard[slot],guard+1,memory_order_seq_cst);
 atomic_thread_fence(memory_order_seq_cst);
 memcpy(s->data+(size_t)slot*h->frame_bytes,data,h->frame_bytes);
 struct video_boxes *dst=(struct video_boxes*)((uint8_t*)h+VIDEO_BOX_SLOT_OFFSET+slot*VIDEO_BOX_BYTES);
 if(boxes)memcpy(dst,boxes,VIDEO_BOX_BYTES);else memset(dst,0,VIDEO_BOX_BYTES);
 h->frame_seq[slot]=seq;h->timestamp_ns[slot]=stamp;
 atomic_store_explicit(&h->slot_guard[slot],guard+2,memory_order_release);
 atomic_store_explicit(&h->active_index,slot,memory_order_release);
 atomic_fetch_add_explicit(&h->publish_seq,1,memory_order_release);
 atomic_fetch_add_explicit(&h->notify_seq,1,memory_order_release);
}
static inline void video_shm_publish(struct video_shm *s,const uint8_t *data,uint64_t seq,uint64_t stamp) {video_shm_publish_boxes(s,data,seq,stamp,NULL);}
static inline int video_check_frame(const struct video_header *m,uint32_t expected_bytes) {
 return ntohl(m->magic)==VIDEO_MAGIC && ntohl(m->version)==VIDEO_VERSION && ntohl(m->type)==VIDEO_FRAME &&
        (uint64_t)ntohl(m->bytes)+ntohl(m->reserved)==expected_bytes && (ntohl(m->reserved)==0 || ntohl(m->reserved)==VIDEO_BOX_BYTES) &&
        video_geometry(ntohl(m->width),ntohl(m->height),ntohl(m->stride),ntohl(m->bytes),ntohl(m->format));
}
static inline int video_ack_send(int fd,uint64_t id,uint32_t status) {
 struct video_ack a={htonl(VIDEO_MAGIC),htonl(status),htobe64(id)};return rn_mlx5_send_all(fd,&a,sizeof(a));
}
static inline int video_ack_recv(int fd,uint64_t id) {
 struct video_ack a;
 if(rn_mlx5_recv_all(fd,&a,sizeof(a))||ntohl(a.magic)!=VIDEO_MAGIC||be64toh(a.frame_id)!=id)return -1;
 return ntohl(a.status)==0?0:ntohl(a.status)==2?2:-1;
}
static inline int video_check_conn(const struct rn_mlx5_wire_conn *c) {
 return ntohl(c->magic_be)==VIDEO_MAGIC && ntohl(c->version_be)==VIDEO_VERSION &&
        ntohl(c->qpn_be)>0 && !(ntohl(c->qpn_be)&0xff000000U) && !(ntohl(c->psn_be)&0xff000000U);
}
static inline void video_init_conn(struct rn_mlx5_wire_conn *c,uint32_t qp,uint32_t psn,uint32_t key,uint64_t addr,uint32_t bytes,const uint8_t *gid) {
 rn_mlx5_init_conn(c,qp,psn,key,addr,bytes,gid);c->magic_be=htonl(VIDEO_MAGIC);c->version_be=htonl(VIDEO_VERSION);
}
#endif
