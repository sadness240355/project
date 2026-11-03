/* Isolated FPGA-DDR color-bar source for DMA/RDMA throughput measurements. */
#include "video_common.h"
#include "video_ddr.h"
#include "video_display.h"
#include "../capture_reference/edev_sgdma_driver.h"
#include "video_crc.h"
#include <getopt.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <inttypes.h>
#include <signal.h>
#include <stdatomic.h>

#define CHUNK (1024U * 1024U)
#define LANES 8
static volatile sig_atomic_t stopping;
static _Atomic int dma_fault;
static _Atomic uint64_t dma_bytes, dma_ops;
static const uint8_t *reference;
struct lane { int fd; void *buffer; uint32_t id, qid; pthread_t thread; int started; };
static struct lane lanes[LANES];
static void stop_signal(int sig) { (void)sig; stopping=1; }
static void safety_hold(const char *reason) {
 fprintf(stderr,"FAIL: %s; DMA completion uncertain; safety hold PID %ld. Reset FPGA before killing.\n",
         reason,(long)getpid());
 atomic_store(&dma_fault,1);
 for(;;)pause();
}
static int lane_open(struct lane *l,unsigned index) {
 struct edev_ioc_user_buffer b={0};
 struct edev_ioc_queue_cfg q={0};
 l->qid=EDEV_MM_QID_BASE+index;
 l->fd=open("/dev/edev0",O_RDWR|O_CLOEXEC);
 if(l->fd<0)return -1;
 if(posix_memalign(&l->buffer,4096,CHUNK)) {errno=ENOMEM;return -1;}
 memset(l->buffer,0,CHUNK);
 b.user_va=(uintptr_t)l->buffer;b.size=CHUNK;b.dir=EDEV_DMA_DIR_D2H;
 if(ioctl(l->fd,EDEV_IOC_REGISTER_BUFFER,&b)<0)return -1;
 l->id=b.buf_id;
 q.qid=l->qid;q.dir_mask=EDEV_QUEUE_DIR_D2H;q.ring_size=EDEV_DEFAULT_RING_SIZE;
 return ioctl(l->fd,EDEV_IOC_QUEUE_CONFIG,&q);
}
static void lane_close(struct lane *l) {
 if(l->fd>=0 && l->id)(void)ioctl(l->fd,EDEV_IOC_UNREGISTER_BUFFER,&l->id);
 if(l->fd>=0)close(l->fd);
 free(l->buffer);
}
static void *lane_run(void *data) {
 struct lane *l=data;
 uint32_t count=0;
 while(!stopping && !atomic_load(&dma_fault)) {
  struct edev_ioc_dma_submit op={0};
  op.qid=l->qid;op.buf_id=l->id;op.dir=EDEV_DMA_DIR_D2H;
  op.flags=EDEV_SUBMIT_F_NO_IRQ;op.len=CHUNK;op.dev_addr=0;
  if(ioctl(l->fd,EDEV_IOC_SUBMIT_BUFFER,&op)<0 || op.status || op.len!=CHUNK)
   safety_hold("color-bar D2H");
  if((count++ & 1023U)==0 && memcmp(l->buffer,reference,CHUNK))
   safety_hold("color-bar DMA content mismatch");
  atomic_fetch_add(&dma_bytes,CHUNK);atomic_fetch_add(&dma_ops,1);
 }
 return NULL;
}
static int stage_or_read(int fd,uint8_t *dst,const uint8_t *src,uint32_t bytes,int write_mode) {
 for(uint32_t pos=0;pos<bytes;) {
  size_t n=bytes-pos;if(n>CHUNK)n=CHUNK;
  ssize_t done=write_mode?pwrite(fd,src+pos,n,pos):pread(fd,dst+pos,n,pos);
  if(done!=(ssize_t)n)return -1;
  pos+=(uint32_t)n;
 }
 return 0;
}
static int prepare_stage_queue(void) {
 int fd=open("/dev/edev0",O_RDWR|O_CLOEXEC);
 if(fd<0)return -1;
 struct edev_ioc_queue_cfg q={0};
 q.qid=EDEV_MM_QID_BASE;q.dir_mask=EDEV_QUEUE_DIR_BOTH;
 q.ring_size=EDEV_DEFAULT_RING_SIZE;
 int result=ioctl(fd,EDEV_IOC_QUEUE_CONFIG,&q);
 int saved=errno;close(fd);errno=saved;
 return result;
}
static void bars(uint8_t *dst,uint32_t w,uint32_t h) {
 static const uint8_t colors[8][3]={{255,255,255},{255,255,0},{0,255,255},{0,255,0},
   {255,0,255},{255,0,0},{0,0,255},{0,0,0}};
 for(uint32_t y=0;y<h;y++)for(uint32_t x=0;x<w;x++) {
  const uint8_t *c=colors[(uint64_t)x*8/w];
  uint8_t *p=dst+((size_t)y*w+x)*3;p[0]=c[0];p[1]=c[1];p[2]=c[2];
 }
}
int main(int argc,char **argv) {
 const char *ring_path=NULL,*preview="./runtime/display_memory/latest_frames.bin";
 uint32_t w=1920,h=1080;int use_dma=0,use_rdma=0,preview_only=0,ring_fd=-1,fd=-1,result=1;
 uint64_t published=0,released=0,last_report=0,last_bytes=0,last_preview=0,preview_seq=1;
 struct ddr_ring *ring=NULL;struct video_shm output={.fd=-1};struct vdq display;vdq_clear(&display);
 static const struct option opts[]={{"ddr-ring",1,0,'r'},{"size",1,0,'s'},
   {"dma-stress",0,0,'d'},{"rdma-stress",0,0,'n'},{"preview-only",0,0,'v'},
   {"output-file",1,0,'o'},{0,0,0,0}};
 int choice;
 while((choice=getopt_long(argc,argv,"r:s:dnvo:",opts,NULL))!=-1) {
  switch(choice) {
   case 'r':ring_path=optarg;break;
   case 's':if(sscanf(optarg,"%ux%u",&w,&h)!=2)return 2;break;
   case 'd':use_dma=1;break;
   case 'n':use_rdma=1;break;
   case 'v':preview_only=1;break;
   case 'o':preview=optarg;break;
   default:return 2;
  }
 }
 if((!preview_only && (!ring_path || (!use_dma&&!use_rdma))) ||
    (preview_only && (use_dma || use_rdma)) || !w || !h ||
    (uint64_t)w*h*3>VIDEO_MAX_BYTES)return 2;
 uint32_t bytes=w*h*3;
 for(unsigned i=0;i<LANES;i++)lanes[i].fd=-1;
 signal(SIGINT,stop_signal);signal(SIGTERM,stop_signal);signal(SIGHUP,stop_signal);
 uint8_t *pattern=NULL,*readback=NULL;
 if(posix_memalign((void**)&pattern,4096,bytes) || posix_memalign((void**)&readback,4096,bytes))goto out;
 reference=pattern;bars(pattern,w,h);
 if(preview_only)memcpy(readback,pattern,bytes);
 else {
  if(prepare_stage_queue()) {perror("configure q64 for H2D/D2H");goto out;}
  fd=open("/dev/edev0_q64",O_RDWR|O_CLOEXEC);
  if(fd<0) {perror("open pattern DDR queue");goto out;}
  if(stage_or_read(fd,NULL,pattern,bytes,1) || stage_or_read(fd,readback,NULL,bytes,0)) {
   perror("color-bar DDR initialization");
   safety_hold("color-bar DDR initialization");
  }
  if(memcmp(pattern,readback,bytes)) {fprintf(stderr,"FAIL: color-bar DDR readback mismatch\n");goto out;}
  close(fd);fd=-1;
 }
 uint32_t crc=video_crc32(readback,bytes);
 fprintf(stderr,"PATTERN staged=%d verified=%d reference=%d bytes=%u width=%u height=%u crc=%08x\n",
   !preview_only,!preview_only,preview_only,bytes,w,h,crc);
 if(video_map_write_open(&output,preview,w,h,CSHM_RGB,1)) {perror("pattern preview");goto out;}
 video_shm_publish(&output,readback,1,video_ns());
 last_preview=video_ns();
 const char *fifo=getenv("VIDEO_DISPLAY_FIFO");
 if(fifo && *fifo) {
  if(vdq_create(&display,fifo,w,h,CSHM_RGB,VDQ_SLOTS)) {perror("pattern display queue");goto out;}
  if(vdq_publish(&display,readback,1,video_ns(),NULL,&stopping,1000)) {perror("pattern display");goto out;}
 }
 fprintf(stderr,"VIDEO frames=1 seq=1 bytes=%u dma_bytes_total=0 dma_ops=0 fps=0 source=pressure_pattern\n",bytes);
 if(use_rdma) {
  ring=ddr_map(ring_path,1,&ring_fd);
  if(!ring) {perror("pattern DDR ring");goto out;}
  ddr_init_mode(ring,w,h,CSHM_RGB,bytes,0,VIDEO_CRC_TRANSPORT);
  fprintf(stderr,"DDR_RING ready=1 slots=%u bytes=%u path=%s\n",DDR_RING_SLOTS,bytes,ring_path);
 }
 if(use_dma) {
  for(unsigned i=0;i<LANES;i++)if(lane_open(&lanes[i],i)) {perror("pattern DMA lane setup");goto out;}
  for(unsigned i=0;i<LANES;i++) {
   if(pthread_create(&lanes[i].thread,NULL,lane_run,&lanes[i])) {
    fprintf(stderr,"FAIL: pattern DMA thread start\n");stopping=1;goto out;
   }
   lanes[i].started=1;
  }
 }
 last_report=video_ns();
 while(!stopping && !atomic_load(&dma_fault)) {
  if(ring) {
   uint32_t state=atomic_load_explicit(&ring->sender_state,memory_order_acquire);
   if(state==DDR_SENDER_FAULT)safety_hold("pattern sender fault");
   if(state==DDR_SENDER_DRAINED)break;
   if(state==DDR_SENDER_ACTIVE) {
    released=ddr_release_ready(ring,(uint32_t)released);
    while(published-released<DDR_RING_SLOTS && !stopping) {
     uint32_t index=(uint32_t)published;
     if(ddr_publish(ring,index,published+2,video_ns(),0))safety_hold("pattern DDR slot");
     struct ddr_frame *frame=ddr_frame_at(ring,index);
     frame->crc=crc;
     atomic_store_explicit(&frame->dma_done,published+1,memory_order_release);
     published++;
    }
   }
  }
  uint64_t now=video_ns();
  if(now-last_preview>=100000000ULL) {
   ++preview_seq;
   video_shm_publish(&output,readback,preview_seq,now);
   if(display.h && vdq_publish(&display,readback,preview_seq,now,NULL,&stopping,0) &&
      errno!=EAGAIN && errno!=ECANCELED) {
    fprintf(stderr,"FAIL: color-bar display queue: %s\n",strerror(errno));
    break;
   }
   last_preview=now;
  }
  if(now-last_report>=250000000ULL) {
   uint64_t count=atomic_load(&dma_bytes);
   fprintf(stderr,"VIDEO frames=1 seq=1 bytes=%u dma_bytes_total=%" PRIu64
     " dma_ops=%" PRIu64 " dma_MiB_s=%.3f fps=0 source=pressure_pattern rdma_published=%" PRIu64 "\n",
     bytes,count,atomic_load(&dma_ops),(double)(count-last_bytes)/1048576.0/((now-last_report)/1e9),published);
   last_report=now;last_bytes=count;
  }
  usleep(50);
 }
 result=atomic_load(&dma_fault)?1:0;
out:
 stopping=1;
 for(unsigned i=0;i<LANES;i++)if(lanes[i].started)pthread_join(lanes[i].thread,NULL);
 if(atomic_load(&dma_fault))safety_hold("pattern DMA worker fault");
 for(unsigned i=0;i<LANES;i++)lane_close(&lanes[i]);
 if(ring) {
  if(atomic_load(&ring->sender_state)==DDR_SENDER_FAULT)safety_hold("pattern sender fault at stop");
  atomic_store_explicit(&ring->owner_stop,1,memory_order_release);
  munmap(ring,sizeof(*ring));close(ring_fd);
 }
 fprintf(stderr,"VIDEO frames=1 seq=1 bytes=%u dma_bytes_total=%" PRIu64 " dma_ops=%" PRIu64
   " fps=0 source=pressure_pattern\n",bytes,atomic_load(&dma_bytes),atomic_load(&dma_ops));
 vdq_finish(&display,1000);video_shm_close(&output);
 if(fd>=0)close(fd);
 free(readback);
 free(pattern);
 if(!result)fprintf(stderr,"PASS: pattern source dma_bytes=%" PRIu64 " rdma_published=%" PRIu64 "\n",
   atomic_load(&dma_bytes),published);
 return result;
}
