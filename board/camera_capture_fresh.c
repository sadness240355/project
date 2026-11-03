#define _GNU_SOURCE
#include "edev_camera_io.h"
#include "video_crc.h"
#include "video_ddr.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#define W 2400u
#define H 2048u
#define BYTES (W*H*3u)
static uint64_t ns(void){struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);return (uint64_t)t.tv_sec*1000000000ull+t.tv_nsec;}
static void hold(const char *why){fprintf(stderr,"SAFETY_HOLD PID=%ld %s; retain DMA memory\n",(long)getpid(),why);fflush(stderr);for(;;)pause();}
static uint32_t rd(volatile unsigned char *p,unsigned a){return *(volatile uint32_t*)(p+0x14000+a);}
static void wr(volatile unsigned char *p,unsigned a,uint32_t v){*(volatile uint32_t*)(p+0x14000+a)=v;__sync_synchronize();(void)rd(p,a);}
static unsigned stop_and_drain(struct camera_context *cam){
 if(camera_remote_acquisition_stop(cam))hold("camera AcquisitionStop not acknowledged");
 int fd=open("/sys/bus/pci/devices/0005:01:00.0/resource2",O_RDWR);if(fd<0)hold("stop BAR access");
 volatile unsigned char *p=mmap(NULL,0x15000,PROT_READ|PROT_WRITE,MAP_SHARED,fd,0);if(p==MAP_FAILED)hold("stop BAR map");
 unsigned drained_frames=0;uint32_t control=rd(p,0);wr(p,0,control|0x10);
 uint64_t deadline=ns()+5000000000ull,quiet=ns();uint32_t previous=rd(p,0x24);int abort_requested=0;
 while(ns()<deadline){
  uint32_t hp=rd(p,0xa14),hc=rd(p,0xa0c),dp=rd(p,0x24),dc=rd(p,0x20),status=rd(p,0xa10);
  if(rd(p,0xa1c))hold("host writer error during drain");
  if(dp!=previous){previous=dp;quiet=ns();}
  if(hp!=hc){
   struct camera_frame_info info;void *data=NULL;size_t len=0;
   if(camera_dma_reap_frame(cam,20,&info,&data,&len)==0){if(camera_dma_release_frame(cam,&info))hold("drain frame release");drained_frames++;}
   else if(errno!=ETIMEDOUT&&errno!=EAGAIN)hold("drain frame reap");
   continue;
  }
  if((rd(p,0)&0x20)&&hp==hc&&dp==dc&&ns()-quiet>40000000ull){
   if(status&1){
    wr(p,0,rd(p,0)&~0x31u);usleep(2000);if(!(rd(p,0xa10)&1))hold("writer no longer idle after capture disable");
    wr(p,0xa00,0);usleep(2000);if(rd(p,0xa00)||!(rd(p,0xa10)&1))hold("writer disable verification");
    wr(p,0xa28,0);fprintf(stderr,"CLEAN_STOP gated=1 host_idle=1 ddr_empty=1 drained_after_window=%u tail_abort=%d\n",drained_frames,abort_requested);
    munmap((void*)p,0x15000);close(fd);return drained_frames;
   }
   if(!abort_requested&&(rd(p,0xa20)&15)==1){wr(p,0xa28,1);abort_requested=1;}
  }
  usleep(1000);
 }
 hold("camera input gated but ownership/hardware did not drain");return 0;
}
int main(int argc,char**argv){
 if(argc!=4&&argc!=5){fprintf(stderr,"usage: %s SECONDS FRAME_PPM FRAMES_CSV [DDR_RING]\n",argv[0]);return 2;}
 unsigned seconds=strtoul(argv[1],0,10);if(!seconds||seconds>120)return 2;
 setenv("CAMERA_HOST_PATH","direct",1);const char *ring_path=argc==5?argv[4]:NULL;
 if(ring_path){setenv("CAMERA_DDR_RING",ring_path,1);setenv("VIDEO_CRC_MODE","full",1);}else unsetenv("CAMERA_DDR_RING");
 struct camera_open_opts opts={.ctrl_path="/dev/edev0",.queue_path="/dev/edev0_q64",.csr_base=0x14000,.ring_base=0x80000000ull,.ring_base_set=1,.frame_stride=16777216,.frame_stride_set=1,.ring_slots=32,.qid=64,.qid_set=1,.dma_queue_count=4,.dma_frame_irq=-1,.driver_frame_irq_dma=0,.irq_enable=1,.bytes_per_clock=10,.tap_order=0x76543210,.tap_order_set=1,.processing_mode=CAMERA_PROCESS_BAYERRG8_RGB888,.expected_line_bytes=W,.expected_height=H,.expected_frame_bytes=W*H};
 struct camera_context*cam=NULL;unsigned char*snapshot=calloc(1,BYTES);if(!snapshot)return 2;
 FILE*csv=fopen(argv[3],"w");if(!csv)return 2;
 fprintf(csv,"host_index,source_frame_seq,ready_ns,observed_ns,bytes,crc32\n");
 if(camera_open(&cam,&opts)){fprintf(stderr,"camera_open failed errno=%d\n",errno);return 2;}
 if(camera_set_output_mode(cam,0)||camera_set_remote_acquisition_control(cam,"w AcquisitionStart 1","w AcquisitionStop 1",5000)){fprintf(stderr,"camera setup: %s\n",camera_last_error(cam));camera_close(cam);return 3;}
 char response[512];
 for(unsigned i=0;i<4;i++){
  const char*commands[]={"r Width","r Height","r PixelFormat","r AcquisitionFrameRate"};
  int rc=camera_serial_send_command(cam,commands[i],5000,response,sizeof response);
  fprintf(stderr,"CAMERA_QUERY command=%s rc=%d response=%s\n",commands[i],rc,response);
  const unsigned expected[]={W,H,0x01080009u,1075141824u};
  const char *markers[]={"get Width:","get Height:","get PixelFormat:","get AcquisitionFrameRate:"};
  char *value=strstr(response,markers[i]);
  if(rc||!value||strtoul(value+strlen(markers[i]),NULL,0)!=expected[i]){
   fprintf(stderr,"CAMERA_PREFLIGHT_MISMATCH command=%s expected=%u; no DMA pool armed\n",commands[i],expected[i]);camera_close(cam);fclose(csv);free(snapshot);return 4;
  }
 }
 struct camera_isp_config isp={.enable_mask=15,.black_level=0,.wb_gain={7782,7782,7987},.ccm={4096,0,0,0,4096,0,0,0,4096}};
 if(camera_set_isp_config(cam,&isp)||camera_dma_open_pool(cam,16,BYTES,4)||camera_dma_set_latest_mode(cam,0)){fprintf(stderr,"pool/ISP: %s\n",camera_last_error(cam));camera_close(cam);return 3;}
 struct camera_frame_info first;
 if(camera_dma_start_strict(cam,0,W*3,H,BYTES,5000,&first)){fprintf(stderr,"strict start: %s\n",camera_last_error(cam));camera_close(cam);return 3;}
 struct ddr_ring *ring=NULL;int ring_fd=-1;
 if(ring_path){ring=ddr_map(ring_path,0,&ring_fd);if(!ring||!ddr_valid(ring))hold("DDR monitor map");}
 struct camera_frame_info warm;void *warm_data=NULL;size_t warm_len=0;
 if(camera_dma_reap_frame(cam,2000,&warm,&warm_data,&warm_len)||warm_len!=BYTES)hold("warmup frame");
 memcpy(snapshot,warm_data,BYTES);
 if(ring){struct ddr_frame *d=&ring->frames[warm.slot];if(d->frame_id!=warm.frame_seq||d->crc!=video_crc32(warm_data,warm_len))hold("warmup host/reference CRC");}
 if(camera_dma_release_frame(cam,&warm))hold("warmup release");
 uint64_t frames=0,start=ns(),deadline=start+(uint64_t)seconds*1000000000ull,end=start;
 fprintf(stderr,"MEASUREMENT_STARTED monotonic_ns=%llu duration_seconds=%u dual=%d\n",(unsigned long long)start,seconds,ring!=NULL);

 uint32_t previous=0;int have=0;uint64_t gaps=0;
 while(end<deadline){
  struct camera_frame_info info;void*data=NULL;size_t len=0;
  if(camera_dma_reap_frame(cam,2000,&info,&data,&len)){fprintf(stderr,"reap: %s errno=%d\n",camera_last_error(cam),errno);camera_close(cam);return 3;}
  if(len!=BYTES||info.width!=W*3||info.height!=H){fprintf(stderr,"geometry mismatch width=%u height=%u bytes=%zu\n",info.width,info.height,len);camera_dma_release_frame(cam,&info);camera_close(cam);return 3;}
  uint32_t checksum;
  if(ring){struct ddr_frame *d=&ring->frames[info.slot];if(d->frame_id!=info.frame_seq)hold("reference frame-ID mismatch");checksum=d->crc;}else checksum=video_crc32(data,len);frames++;
  if(have){uint32_t delta=info.frame_seq-previous;if(delta!=1)gaps+=delta?delta-1:1;}
  previous=info.frame_seq;have=1;
  fprintf(csv,"%llu,%u,%llu,%llu,%zu,%08x\n",(unsigned long long)frames,info.frame_seq,(unsigned long long)info.ready_ns,(unsigned long long)ns(),len,checksum);
  if(camera_dma_release_frame(cam,&info)){fprintf(stderr,"release: %s\n",camera_last_error(cam));camera_close(cam);return 3;}
  end=ns();
 }
 struct camera_csr_snapshot before_stop;memset(&before_stop,0,sizeof before_stop);camera_read_csr_snapshot(cam,&before_stop);
 unsigned drained_frames=stop_and_drain(cam);camera_close(cam);fclose(csv);
 if(ring){uint64_t until=ns()+3000000000ull;while(atomic_load(&ring->sender_state)!=DDR_SENDER_DRAINED&&ns()<until)usleep(1000);fprintf(stderr,"DDR_FINAL published=%llu released=%llu sender_state=%u\n",(unsigned long long)atomic_load(&ring->published),(unsigned long long)atomic_load(&ring->released),atomic_load(&ring->sender_state));munmap(ring,sizeof(*ring));close(ring_fd);}
 fprintf(stderr,"DRAINED_AFTER_MEASUREMENT=%u\n",drained_frames);
 FILE*f=fopen(argv[2],"wb");if(!f)return 2;fprintf(f,"P6\n%u %u\n255\n",W,H);if(fwrite(snapshot,1,BYTES,f)!=BYTES)return 2;fclose(f);free(snapshot);
 printf("{\"scope\":\"FRESH_CAMERA_DIRECT_HOST_CAPTURE_CHECK_PAIRED_SENDER_LOGS_FOR_DUAL_BRANCH_PROOF\",\"width\":%u,\"height\":%u,\"frames\":%llu,\"elapsed_ns\":%llu,\"host_delivered_FPS\":%.6f,\"source_sequence_gaps\":%llu,\"hardware_drop_count\":%u,\"hardware_status\":%u,\"build_version\":%u,\"CRC_backend\":\"%s\",\"CRC_note\":\"per-frame fingerprints, not independent scene correctness reference\"}\n",W,H,(unsigned long long)frames,(unsigned long long)(end-start),(double)frames*1e9/(end-start),(unsigned long long)gaps,before_stop.drop_count,before_stop.status,before_stop.build_version,video_crc_backend());
 return 0;
}
