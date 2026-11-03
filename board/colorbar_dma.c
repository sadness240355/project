#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <errno.h>
#include <sys/ioctl.h>
#include "edev_sgdma_driver.h"
#define FRAME_BYTES (2400u*2048u*3u)
static uint64_t ns(void){struct timespec t;clock_gettime(CLOCK_MONOTONIC_RAW,&t);return (uint64_t)t.tv_sec*1000000000ull+t.tv_nsec;}
static uint32_t table[256];
static uint32_t crc(const unsigned char*p,size_t n){uint32_t c=~0u;while(n--)c=table[(c^*p++)&255]^(c>>8);return ~c;}
static void hold(const char*s){fprintf(stderr,"SAFETY_HOLD %s errno=%d; DMA buffers retained until controlled reset\n",s,errno);fflush(stderr);for(;;)sleep(60);}
static void op(int fd,unsigned id,int dir){
 struct edev_ioc_dma_submit a={.qid=66,.buf_id=id,.dir=dir,.len=FRAME_BYTES,.dev_addr=0x80000000ull};
 if(ioctl(fd,EDEV_IOC_SUBMIT_BUFFER,&a)||a.status)hold("submit");
 struct edev_ioc_dma_cmpl c={.qid=66,.timeout_ms=5000};
 if(ioctl(fd,EDEV_IOC_REAP_CMPL,&c)||c.status||c.error_code||c.bytes_done!=FRAME_BYTES||c.op_id!=a.op_id)hold("completion");
}
int main(int argc,char**argv){
 if(argc!=4){fprintf(stderr,"usage: %s COLORBAR_REFERENCE SECONDS FINAL_READBACK\n",argv[0]);return 2;}
 unsigned seconds=strtoul(argv[2],0,10);if(seconds<1||seconds>120)return 2;
 for(unsigned i=0;i<256;i++){uint32_t c=i;for(int j=0;j<8;j++)c=(c>>1)^((c&1)?0xedb88320u:0);table[i]=c;}
 unsigned char*src=NULL,*dst=NULL;
 if(posix_memalign((void**)&src,4096,FRAME_BYTES)||posix_memalign((void**)&dst,4096,FRAME_BYTES))return 2;
 FILE*f=fopen(argv[1],"rb");if(!f||fread(src,1,FRAME_BYTES,f)!=FRAME_BYTES||fgetc(f)!=EOF)return 2;fclose(f);memset(dst,0,FRAME_BYTES);
 int fd=open("/dev/edev0_q66",O_RDWR);if(fd<0){perror("open");return 2;}
 struct edev_ioc_hw_caps caps={0};if(ioctl(fd,EDEV_IOC_GET_CAPS,&caps))return 2;
 struct edev_ioc_queue_cfg q={.qid=66,.dir_mask=EDEV_QUEUE_DIR_BOTH,.ring_size=caps.ring_size};if(ioctl(fd,EDEV_IOC_QUEUE_CONFIG,&q))return 2;
 struct edev_ioc_user_buffer a={.user_va=(uintptr_t)src,.size=FRAME_BYTES,.dir=EDEV_DMA_DIR_H2D},b={.user_va=(uintptr_t)dst,.size=FRAME_BYTES,.dir=EDEV_DMA_DIR_D2H};
 if(ioctl(fd,EDEV_IOC_REGISTER_BUFFER,&a)||ioctl(fd,EDEV_IOC_REGISTER_BUFFER,&b))return 2;
 op(fd,a.buf_id,0);op(fd,b.buf_id,1);
 if(memcmp(src,dst,FRAME_BYTES)){fprintf(stderr,"initial readback mismatch\n");return 3;}
 uint32_t expected=crc(src,FRAME_BYTES);uint64_t frames=0,samples=0,start=ns(),deadline=start+(uint64_t)seconds*1000000000ull,next=start+1000000000ull,end=start;
 while(end<deadline){
  op(fd,b.buf_id,1);frames++;end=ns();
  if(end>=next){uint32_t c=crc(dst,FRAME_BYTES);samples++;fprintf(stderr,"CRC_SAMPLE frame=%llu crc32=%08x\n",(unsigned long long)frames,c);if(c!=expected)return 3;next+=1000000000ull;end=ns();}
 }
 uint64_t elapsed=end-start,bytes=frames*(uint64_t)FRAME_BYTES;
 if(memcmp(src,dst,FRAME_BYTES)){fprintf(stderr,"final readback mismatch\n");return 3;}
 f=fopen(argv[3],"wb");if(!f||fwrite(dst,1,FRAME_BYTES,f)!=FRAME_BYTES)return 2;fclose(f);
 printf("{\"scope\":\"STATIC_COLORBAR_C2H_CONTINUOUS_WALL_CLOCK_WITH_PERIODIC_CRC\",\"frame_bytes\":%u,\"completed_frames\":%llu,\"completed_bytes\":%llu,\"elapsed_ns\":%llu,\"GBps\":%.9f,\"Gbit_s\":%.9f,\"CRC_samples\":%llu,\"CRC_errors\":0,\"reference_crc32\":\"%08x\",\"initial_and_final_full_byte_comparison\":true}\n",FRAME_BYTES,(unsigned long long)frames,(unsigned long long)bytes,(unsigned long long)elapsed,(double)bytes/elapsed,(double)bytes/elapsed*8,(unsigned long long)samples,expected);
 if(ioctl(fd,EDEV_IOC_UNREGISTER_BUFFER,&a.buf_id)||ioctl(fd,EDEV_IOC_UNREGISTER_BUFFER,&b.buf_id))hold("deregister");
 close(fd);free(src);free(dst);return 0;
}
