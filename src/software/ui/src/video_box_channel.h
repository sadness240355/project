#ifndef VIDEO_BOX_CHANNEL_H
#define VIDEO_BOX_CHANNEL_H
#include <poll.h>
#include <pthread.h>
#define BOX_CHANNEL_MAGIC 0x58425244U
#define BOX_CHANNEL_QPID 4U
/* Parent QPN binds the independent TCP/QP to this live video session.
 * Metadata contains no pixels/boxes: all 1024 box bytes use RDMA. */
struct box_hello {
 uint32_t magic,parent_qpn,width,height;
 struct rn_mlx5_wire_conn conn;
};
struct box_notice { uint32_t magic,crc;uint64_t publish_seq; };
_Static_assert(sizeof(struct box_hello)==80,"box hello ABI");
_Static_assert(sizeof(struct box_notice)==16,"box notice ABI");
static inline void box_socket_options(int fd) {
 int one=1;struct timeval tv={.tv_sec=5};
 setsockopt(fd,IPPROTO_TCP,TCP_NODELAY,&one,sizeof(one));
 setsockopt(fd,SOL_SOCKET,SO_RCVTIMEO,&tv,sizeof(tv));
 setsockopt(fd,SOL_SOCKET,SO_SNDTIMEO,&tv,sizeof(tv));
}
struct box_mailbox {int fd;uint8_t *mem;};
static inline int box_mailbox_open(struct box_mailbox *m,const char *path,uint32_t w,uint32_t h) {
 m->fd=open(path,O_RDWR|O_CREAT|O_NOFOLLOW,0644);if(m->fd<0)return -1;
 struct stat st;
 if(fstat(m->fd,&st)||!S_ISREG(st.st_mode)||flock(m->fd,LOCK_EX|LOCK_NB)||ftruncate(m->fd,20480)) {
  close(m->fd);m->fd=-1;return -1;
 }
 m->mem=mmap(NULL,20480,PROT_READ|PROT_WRITE,MAP_SHARED,m->fd,0);
 if(m->mem==MAP_FAILED){m->mem=NULL;close(m->fd);m->fd=-1;return -1;}
 /* Reader sees a new session, no stale boxes from the prior camera stream. */
 __atomic_store_n((uint64_t*)(m->mem+32),0,__ATOMIC_RELEASE);
 uint32_t words[8]={0x584f4259,1,4096,32,256,2,w,h};
 memcpy(m->mem,words,sizeof(words));memset(m->mem+40,0,20480-40);return 0;
}
static inline void box_mailbox_publish(struct box_mailbox *m,const struct video_boxes *b) {
 uint64_t pub=b->publish_seq;unsigned slot=pub&1U;uint8_t *p=m->mem;
 uint64_t *guard=(uint64_t*)(p+48+8*slot);
 uint64_t even=(__atomic_load_n(guard,__ATOMIC_RELAXED)+2)&~UINT64_C(1);
 __atomic_store_n(guard,even-1,__ATOMIC_SEQ_CST);
 memcpy(p+4096+slot*8192,b->boxes,b->count*sizeof(struct video_box));
 memcpy(p+64+slot*8,&b->frame_id,8);memcpy(p+80+slot*8,&b->capture_ns,8);
 memcpy(p+96+slot*4,&b->count,4);
 __atomic_store_n(guard,even,__ATOMIC_RELEASE);
 __atomic_store_n((uint32_t*)(p+40),slot,__ATOMIC_RELEASE);
 __atomic_store_n((uint64_t*)(p+32),pub,__ATOMIC_RELEASE);
}
static inline void box_mailbox_close(struct box_mailbox *m) {
 if(m->mem)munmap(m->mem,20480);
 if(m->fd>=0)close(m->fd);
 m->mem=NULL;m->fd=-1;
}
#endif
