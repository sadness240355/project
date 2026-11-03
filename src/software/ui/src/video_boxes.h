#ifndef VIDEO_BOXES_H
#define VIDEO_BOXES_H
#include <math.h>
/* Both supported endpoints are little-endian. No native pointers in this ABI. */
#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error Video boxes ABI currently requires little-endian hosts
#endif
#define VIDEO_BOX_BYTES 1024U
#define VIDEO_BOX_MAX 20U
#define VIDEO_BOX_MAGIC 0x31425859U
#define VIDEO_BOX_AGE_NS 250000000ULL
#define VIDEO_BOX_SLOT_OFFSET 512U
struct video_box {float x1,y1,x2,y2,confidence;int32_t class_id,track_id,flags;};
struct video_boxes {
 uint32_t magic,version,count,width,height,valid;
 uint64_t frame_id,capture_ns,publish_seq;
 uint32_t reserved[4];
 struct video_box boxes[VIDEO_BOX_MAX];
 uint8_t padding[320];
};
_Static_assert(sizeof(struct video_box)==32,"box record ABI");
_Static_assert(sizeof(struct video_boxes)==VIDEO_BOX_BYTES,"boxes packet ABI");
static inline int video_boxes_valid(const struct video_boxes *b,uint32_t w,uint32_t h) {
 if(!b->valid)return b->magic==0 && b->count==0;
 if(b->valid!=1||b->magic!=VIDEO_BOX_MAGIC||b->version!=1||b->count>VIDEO_BOX_MAX||b->width!=w||b->height!=h)return 0;
 for(unsigned i=0;i<b->count;i++) {
  const struct video_box *p=b->boxes+i;
  if(!isfinite(p->x1)||!isfinite(p->y1)||!isfinite(p->x2)||!isfinite(p->y2)||!isfinite(p->confidence)||
    p->x1<0||p->y1<0||p->x2>w||p->y2>h||p->x1>=p->x2||p->y1>=p->y2||p->confidence<0||p->confidence>1)return 0;
 }
 return 1;
}
static inline int video_boxes_fresh(const struct video_boxes *b,uint64_t seq,uint64_t stamp,uint64_t extra_age) {
 return b->valid && stamp>=b->capture_ns && stamp-b->capture_ns<=VIDEO_BOX_AGE_NS &&
  extra_age<=VIDEO_BOX_AGE_NS-(stamp-b->capture_ns) && (uint32_t)((uint32_t)seq-(uint32_t)b->frame_id)<0x80000000U;
}
struct video_box_reader {int fd;const uint8_t *mem;size_t size;uint64_t checked;ino_t inode;dev_t dev;char path[256];};
/* The detector mailbox can be replaced while the RDMA QP remains alive.
 * Deduplicate within each source mapping, but keep the on-wire sequence
 * monotonic for the receiver across detector process restarts. */
struct video_box_sequence {
 uint64_t source_last,frame_last,wire_last;
 ino_t inode;dev_t dev;int mapped;
};
static inline int video_box_sequence_next(struct video_box_sequence *s,
 ino_t inode,dev_t dev,uint64_t source_seq,uint64_t frame_id,uint64_t *wire_seq) {
 if(!source_seq)return 0;
 if(!s->mapped || s->inode!=inode || s->dev!=dev) {
  s->mapped=1;s->inode=inode;s->dev=dev;s->source_last=0;s->frame_last=0;
 } else if(source_seq<=s->source_last) {
  /* Also recover if an inode was reused after a very fast unlink/create. */
  if(source_seq>=s->source_last || frame_id<=s->frame_last)return 0;
 }
 s->source_last=source_seq;s->frame_last=frame_id;
 *wire_seq=++s->wire_last;
 return 1;
}
static inline void video_boxes_close(struct video_box_reader *r) {
 if(r->mem){munmap((void*)r->mem,r->size);close(r->fd);}memset(r,0,sizeof(*r));
}
static inline uint32_t video_box_u32(const uint8_t *p,size_t n){uint32_t v;memcpy(&v,p+n,4);return v;}
static inline uint64_t video_box_u64(const uint8_t *p,size_t n){uint64_t v;memcpy(&v,p+n,8);return v;}
static inline void video_boxes_latest(struct video_box_reader *r,const char *path,uint64_t now,
 uint32_t w,uint32_t h,struct video_boxes *out) {
 memset(out,0,sizeof(*out));
 if(!path||!*path){video_boxes_close(r);return;}
 if(strcmp(r->path,path)){video_boxes_close(r);snprintf(r->path,sizeof(r->path),"%s",path);}
 if(!r->checked || now-r->checked>=100000000ULL) {
  struct stat st;r->checked=now;
  if(stat(path,&st)||!r->mem||r->inode!=st.st_ino||r->dev!=st.st_dev) {
   if(r->mem){munmap((void*)r->mem,r->size);close(r->fd);r->mem=NULL;}
   int fd=open(path,O_RDONLY|O_NOFOLLOW);
   if(fd<0)return;
   if(fstat(fd,&st)||st.st_size!=4096+2*256*32){close(fd);return;}
   void *mem=mmap(NULL,st.st_size,PROT_READ,MAP_SHARED,fd,0);
   if(mem==MAP_FAILED){close(fd);return;}
   r->fd=fd;r->mem=mem;r->size=st.st_size;r->inode=st.st_ino;r->dev=st.st_dev;
  }
 }
 const uint8_t *m=r->mem;if(!m)return;
 if(video_box_u32(m,0)!=0x584f4259||video_box_u32(m,4)!=1||video_box_u32(m,8)!=4096||video_box_u32(m,12)!=32||
  video_box_u32(m,16)!=256||video_box_u32(m,20)!=2||video_box_u32(m,24)!=w||video_box_u32(m,28)!=h)return;
 uint64_t pub=__atomic_load_n((const uint64_t*)(m+32),__ATOMIC_ACQUIRE);
 unsigned slot=__atomic_load_n((const uint32_t*)(m+40),__ATOMIC_ACQUIRE);
 if(!pub||slot>1)return;
 uint64_t guard=__atomic_load_n((const uint64_t*)(m+48+8*slot),__ATOMIC_ACQUIRE);
 if(guard&1)return;
 struct video_boxes b={0};b.magic=VIDEO_BOX_MAGIC;b.version=b.valid=1;b.width=w;b.height=h;b.publish_seq=pub;
 b.frame_id=video_box_u64(m,64+8*slot);b.capture_ns=video_box_u64(m,80+8*slot);b.count=video_box_u32(m,96+4*slot);
 if(b.count>256)return;
 if(b.count>VIDEO_BOX_MAX)b.count=VIDEO_BOX_MAX;
 memcpy(b.boxes,m+4096+slot*256*32,b.count*sizeof(struct video_box));
 __atomic_thread_fence(__ATOMIC_SEQ_CST);
 if(guard!=__atomic_load_n((const uint64_t*)(m+48+8*slot),__ATOMIC_ACQUIRE)||pub!=__atomic_load_n((const uint64_t*)(m+32),__ATOMIC_ACQUIRE))return;
 if(video_boxes_valid(&b,w,h))*out=b;
}
static inline void video_boxes_snapshot(struct video_box_reader *r,const char *path,uint64_t now,
 uint64_t seq,uint64_t stamp,uint32_t w,uint32_t h,struct video_boxes *out) {
 video_boxes_latest(r,path,now,w,h,out);
 if(!video_boxes_fresh(out,seq,stamp,0))memset(out,0,sizeof(*out));
}
#endif
