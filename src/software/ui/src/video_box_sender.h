#ifndef VIDEO_BOX_SENDER_H
#define VIDEO_BOX_SENDER_H
#include "video_box_channel.h"
#include "../capture_reference/edev_sgdma_driver.h"
struct box_sender {
 struct rdma_dev_t *dev;struct rdma_buff_t *payload,*credit;struct rdma_pd_t *pd;
 struct ddr_ring *ddr;const char *path;
 int fd;uint32_t width,height,rkey;uint64_t addr;
 const char *dma_path;
 int stage_fd;void *stage_mem;uint32_t stage_buf;
 pthread_mutex_t *rdma_submit_lock;

 _Atomic int stop,failed;pthread_t thread;int started;
};

/* Only the new ISP->host path frees MM q71 from camera D2H use.
 * H2D completion precedes ERNIC's DDR read; returned credit precedes reuse. */
static void box_stage_close(struct box_sender *c) {
 if(c->stage_mem) {
  if(c->stage_buf)(void)ioctl(c->stage_fd,EDEV_IOC_UNREGISTER_BUFFER,&c->stage_buf);
  free(c->stage_mem);c->stage_mem=NULL;
 }
 if(c->stage_fd>=0)close(c->stage_fd);
 c->stage_fd=-1;
}
static int box_stage_open(struct box_sender *c) {
 c->stage_fd=-1;
 if(!c->ddr->host_path)return 0;
 if(!c->dma_path || !c->ddr->box_addr || c->ddr->box_addr>UINT64_C(0xffffffff)-4096) {
  errno=EINVAL;return -1;
 }
 c->stage_fd=open(c->dma_path,O_RDWR|O_CLOEXEC);
 if(c->stage_fd<0)return -1;
 if(posix_memalign(&c->stage_mem,4096,4096)) {
  c->stage_mem=NULL;box_stage_close(c);errno=ENOMEM;return -1;
 }
 memset(c->stage_mem,0,4096);
 struct edev_ioc_user_buffer u={0};
 u.user_va=(uintptr_t)c->stage_mem;u.size=4096;u.dir=EDEV_DMA_DIR_H2D;
 if(ioctl(c->stage_fd,EDEV_IOC_REGISTER_BUFFER,&u)<0) {
  box_stage_close(c);return -1;
 }
 c->stage_buf=u.buf_id;
 struct edev_ioc_queue_cfg cfg={0};
 cfg.qid=EDEV_MM_QID_BASE+EDEV_MM_QUEUE_COUNT-1;
 cfg.dir_mask=EDEV_QUEUE_DIR_H2D;cfg.ring_size=EDEV_DEFAULT_RING_SIZE;cfg.flags=0;
 if(ioctl(c->stage_fd,EDEV_IOC_QUEUE_CONFIG,&cfg)<0) {
  box_stage_close(c);return -1;
 }
 return 0;
}
static int box_stage_write(struct box_sender *c,const struct video_boxes *b) {
 memcpy(c->stage_mem,b,sizeof(*b));
 struct edev_ioc_dma_submit op={0};
 op.qid=EDEV_MM_QID_BASE+EDEV_MM_QUEUE_COUNT-1;
 op.buf_id=c->stage_buf;op.dir=EDEV_DMA_DIR_H2D;
 op.flags=EDEV_SUBMIT_F_NO_IRQ;op.dev_addr=c->ddr->box_addr;op.len=sizeof(*b);
 if(ioctl(c->stage_fd,EDEV_IOC_SUBMIT_BUFFER,&op)<0 ||
    op.status || op.len!=sizeof(*b))return -1;
 return 0;
}

static void *box_send_worker(void *ptr) {
 struct box_sender *c=ptr;struct video_box_reader reader={0};
 struct video_box_sequence sequence={0};
 uint64_t last=0,wrid=0,report=video_ns(),sent=0;
 if(box_stage_open(c)<0) {
  fprintf(stderr,"FAIL: boxes DDR staging init: %s\n",strerror(errno));
  atomic_store(&c->failed,1);atomic_store(&c->ddr->sender_state,DDR_SENDER_FAULT);
  return NULL;
 }

 while(!atomic_load(&c->stop)) {
  struct video_boxes b;uint64_t now=video_ns();
  video_boxes_latest(&reader,c->path,now,c->width,c->height,&b);
  if(!b.valid||now<b.capture_ns||now-b.capture_ns>VIDEO_BOX_AGE_NS) {usleep(500);continue;}
  uint64_t wire_seq;
  if(!video_box_sequence_next(&sequence,reader.inode,reader.dev,
                              b.publish_seq,b.frame_id,&wire_seq)) {usleep(500);continue;}
  b.publish_seq=wire_seq;
  memcpy(c->payload->buffer,&b,sizeof(b));
  uint64_t boxes_source=c->payload->dma_addr;
  if(c->ddr->host_path) {
   if(box_stage_write(c,&b)<0) {
    atomic_store(&c->failed,1);atomic_store(&c->ddr->sender_state,DDR_SENDER_FAULT);
    fprintf(stderr,"FAIL: boxes H2D DMA uncertain; safety hold PID %ld. Reset FPGA before killing.\n",(long)getpid());
    for(;;)pause();
   }
   boxes_source=DDR_DEVICE_BASE|c->ddr->box_addr;
  }

  struct box_notice n={htonl(BOX_CHANNEL_MAGIC),htonl(video_crc32(&b,sizeof(b))),htobe64(b.publish_seq)};
  uint64_t commit=sent+1;
  memcpy((uint8_t*)c->payload->buffer+1024,&n,sizeof(n));
  memcpy((uint8_t*)c->payload->buffer+1088,&commit,8);
  /* One bounded hardware transaction at a time across the video and box QPs.
   * Keep H2D staging and TCP credit waits outside this shared RDMA gate. */
  pthread_mutex_lock(c->rdma_submit_lock);
  uint32_t slot=c->dev->qps_ptr[BOX_CHANNEL_QPID]->sq_pidb%RN_INTEROP_QDEPTH;
  create_a_wqe(c->dev,BOX_CHANNEL_QPID,(uint16_t)++wrid,slot,boxes_source,
               sizeof(b),RNIC_OP_WRITE,c->addr,c->rkey,0,0,0,0,0);
  create_a_wqe(c->dev,BOX_CHANNEL_QPID,(uint16_t)++wrid,(slot+1)%RN_INTEROP_QDEPTH,c->payload->dma_addr+1024,
               sizeof(n),RNIC_OP_WRITE,c->addr+1024,c->rkey,0,0,0,0,0);
  create_a_wqe(c->dev,BOX_CHANNEL_QPID,(uint16_t)++wrid,(slot+2)%RN_INTEROP_QDEPTH,c->payload->dma_addr+1088,
               8,RNIC_OP_WRITE,c->addr+1088,c->rkey,0,0,0,0,0);
  if(rn_rdma_post_write_batch_checked(c->dev,BOX_CHANNEL_QPID,3,5000)) {
   atomic_store(&c->failed,1);atomic_store(&c->ddr->sender_state,DDR_SENDER_FAULT);
   dump_registers(c->dev,1,BOX_CHANNEL_QPID);
   fprintf(stderr,"FAIL: boxes DMA outstanding; safety hold PID %ld. Reset FPGA before killing; keep driver/pool alive.\n",(long)getpid());
   /* Main joins this worker before freeing any DMA memory. */
   for(;;)pause();
  }
  pthread_mutex_unlock(c->rdma_submit_lock);
  if(video_ack_recv(c->fd,sent+1)!=0) {
   atomic_store(&c->failed,1);atomic_store(&c->ddr->sender_state,DDR_SENDER_FAULT);
   fprintf(stderr,"FAIL: boxes ownership ACK missing; safety hold PID %ld. Reset FPGA before killing.\n",(long)getpid());
   for(;;)pause();
  }
  last=b.publish_seq;++sent;
  if(video_ns()-report>=1000000000ULL) {
   fprintf(stderr,"BOXES sent=%" PRIu64 " publish=%" PRIu64 " frame_seq=%" PRIu64 " count=%u source=%s qp=%u\n",sent,last,b.frame_id,b.count,c->ddr->host_path?"host_H2D_DDR_rdma":"host_rdma",BOX_CHANNEL_QPID);report=video_ns();
  }
 }
 if(!atomic_load(&c->failed)) {
  struct box_notice end={htonl(BOX_CHANNEL_MAGIC),0,0};
  if(rn_mlx5_send_all(c->fd,&end,sizeof(end))||video_ack_recv(c->fd,0)!=0)atomic_store(&c->failed,1);
 }
 fprintf(stderr,"BOXES sent=%" PRIu64 " publish=%" PRIu64 " final=1 transport=rdma_write_tcp_credit\n",sent,last);
 box_stage_close(c);video_boxes_close(&reader);return NULL;
}
#endif
