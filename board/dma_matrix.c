#define _GNU_SOURCE
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <time.h>
#include <pthread.h>
#include "../source/edev_sgdma_driver.h"
static pthread_barrier_t gate;
struct worker {int q,fd,dir,iters;size_t len;uint64_t base,bytes,ns,start_ns,end_ns;void *src,*dst;uint32_t src_id,dst_id;int verified;};
static uint64_t now_ns(void){struct timespec t;clock_gettime(CLOCK_MONOTONIC_RAW,&t);return (uint64_t)t.tv_sec*1000000000ull+t.tv_nsec;}
static unsigned char pattern(uint64_t i){return (17*i+(i>>8)+0x5a)&255;}
static void hold(const char *why){fprintf(stderr,"SAFETY_HOLD: %s errno=%d; buffers retained; do not kill until device drained/reset by operator\n",why,errno);fflush(stderr);for(;;)sleep(60);}
static void transfer(struct worker *w,int dir,uint32_t id){
 struct edev_ioc_dma_submit s={.qid=w->q,.buf_id=id,.dir=dir,.len=w->len,.dev_addr=w->base};
 if(ioctl(w->fd,EDEV_IOC_SUBMIT_BUFFER,&s)<0 || s.status)hold("SUBMIT failed/uncertain");
 struct edev_ioc_dma_cmpl c={.qid=w->q,.timeout_ms=5000};
 if(ioctl(w->fd,EDEV_IOC_REAP_CMPL,&c)<0 || c.status || c.error_code || c.op_id!=s.op_id || c.bytes_done!=w->len)hold("completion mismatch");
}
static void register_mem(struct worker*w,void*p,int dir,uint32_t *id){
 struct edev_ioc_user_buffer b={.user_va=(uintptr_t)p,.size=w->len,.dir=dir};
 if(ioctl(w->fd,EDEV_IOC_REGISTER_BUFFER,&b)<0){perror("REGISTER_BUFFER");exit(2);}*id=b.buf_id;
}
static void *run(void *arg){struct worker*w=arg;char dev[80];snprintf(dev,sizeof dev,"/dev/edev0_q%d",w->q);
 w->fd=open(dev,O_RDWR);if(w->fd<0){perror(dev);exit(2);}
 struct edev_ioc_hw_caps caps={0};
 if(ioctl(w->fd,EDEV_IOC_GET_CAPS,&caps)<0){perror("GET_CAPS");exit(2);}
 fprintf(stderr,"CONFIG queue=%d ring_size=%u hw_mm_outstanding=%u\n",w->q,caps.ring_size,caps.hw_mm_outstanding);
 struct edev_ioc_queue_cfg cfg={.qid=w->q,.dir_mask=EDEV_QUEUE_DIR_BOTH,.ring_size=caps.ring_size};
 if(ioctl(w->fd,EDEV_IOC_QUEUE_CONFIG,&cfg)<0){perror("QUEUE_CONFIG");exit(2);}
 if(posix_memalign(&w->src,4096,w->len)||posix_memalign(&w->dst,4096,w->len))exit(2);
 for(size_t i=0;i<w->len;i++)((unsigned char*)w->src)[i]=pattern(i);
 memset(w->dst,0,w->len);register_mem(w,w->src,EDEV_DMA_DIR_H2D,&w->src_id);register_mem(w,w->dst,EDEV_DMA_DIR_D2H,&w->dst_id);
 transfer(w,0,w->src_id);transfer(w,1,w->dst_id);
 if(memcmp(w->src,w->dst,w->len)){fprintf(stderr,"precheck mismatch queue=%d\n",w->q);exit(3);}
 pthread_barrier_wait(&gate);
 uint64_t t=now_ns();w->start_ns=t;
 for(int i=0;i<w->iters;i++){transfer(w,w->dir,w->dir ? w->dst_id:w->src_id);w->bytes+=w->len;}
 w->end_ns=now_ns();w->ns=w->end_ns-t;
 transfer(w,1,w->dst_id);w->verified=memcmp(w->src,w->dst,w->len)==0;
 if(!w->verified){fprintf(stderr,"postcheck mismatch queue=%d\n",w->q);exit(3);}
 ioctl(w->fd,EDEV_IOC_UNREGISTER_BUFFER,&w->src_id);ioctl(w->fd,EDEV_IOC_UNREGISTER_BUFFER,&w->dst_id);
 free(w->src);free(w->dst);close(w->fd);return NULL;
}
int main(int argc,char**argv){
 if(argc==2 && !strcmp(argv[1],"--selftest")){for(uint64_t i=0;i<1024;i++)putchar(pattern(i));return 0;}
 if(argc!=7 || strcmp(argv[6],"--execute")){fprintf(stderr,"usage: %s DDR_BASE LENGTH QUEUES h2c|c2h ITERATIONS --execute\nRequires an idle board and explicitly reserved DDR region.\n",argv[0]);return 2;}
 uint64_t base=strtoull(argv[1],0,0);size_t len=strtoull(argv[2],0,0);int n=atoi(argv[3]),iters=atoi(argv[5]);
 int dir=!strcmp(argv[4],"c2h");if(strcmp(argv[4],"h2c")&&strcmp(argv[4],"c2h"))return 2;
 if(n<1||n>4||len<4096||len>(1u<<20)||iters<1||base%4096||base+((uint64_t)n<<21)>0x100000000ull)return 2;
 struct worker w[4]={0};pthread_t threads[4];pthread_barrier_init(&gate,0,n+1);
 for(int i=0;i<n;i++){w[i]=(struct worker){.q=64+i,.dir=dir,.iters=iters,.len=len,.base=base+((uint64_t)i<<21)};if(pthread_create(&threads[i],0,run,&w[i]))exit(2);}
 pthread_barrier_wait(&gate);
 /* Aggregate elapsed ends at the latest worker completion; post-checks are excluded. */
 for(int i=0;i<n;i++)pthread_join(threads[i],0);
 uint64_t bytes=0,max_ns=0,min_start=UINT64_MAX,max_end=0;printf("direction,queue,length,iterations,completed_bytes,elapsed_ns,GBps,full_buffer_verified,start_ns,end_ns\n");
 for(int i=0;i<n;i++){bytes+=w[i].bytes;if(w[i].start_ns<min_start)min_start=w[i].start_ns;if(w[i].end_ns>max_end)max_end=w[i].end_ns;printf("%s,%d,%zu,%d,%llu,%llu,%.9f,%d,%llu,%llu\n",argv[4],w[i].q,len,iters,(unsigned long long)w[i].bytes,(unsigned long long)w[i].ns,(double)w[i].bytes/w[i].ns,w[i].verified,(unsigned long long)w[i].start_ns,(unsigned long long)w[i].end_ns);}
 max_ns=max_end-min_start;printf("AGGREGATE,%d,%zu,%d,%llu,%llu,%.9f,1,%llu,%llu\n",n,len,iters,(unsigned long long)bytes,(unsigned long long)max_ns,(double)bytes/max_ns,(unsigned long long)min_start,(unsigned long long)max_end);
 return 0;
}
