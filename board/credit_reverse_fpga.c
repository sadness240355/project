#define _GNU_SOURCE
#include "rdma_api.h"
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <signal.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include "edev_sgdma_driver.h"
static uint64_t ns(void){struct timespec t;clock_gettime(CLOCK_MONOTONIC_RAW,&t);return (uint64_t)t.tv_sec*1000000000ull+t.tv_nsec;}
static unsigned char pattern(uint64_t i){return (17*i+(i>>8)+0x5a)&255;}
static void hold(const char*s){fprintf(stderr,"SAFETY_HOLD %s errno=%d; coherent memory retained; drain/reset before stopping process\n",s,errno);fflush(stderr);for(;;)sleep(60);}
static void ddr_init(unsigned length) {
 size_t bytes=(length+4095u)&~4095u;
 unsigned char*src=NULL,*dst=NULL;
 if(posix_memalign((void**)&src,4096,bytes)||posix_memalign((void**)&dst,4096,bytes))exit(2);
 for(size_t i=0;i<bytes;i++)src[i]=pattern(i);
 memset(dst,0,bytes);
 int qfd=open("/dev/edev0_q71",O_RDWR);if(qfd<0){perror("DDR queue");exit(2);}
 struct edev_ioc_hw_caps caps={0};if(ioctl(qfd,EDEV_IOC_GET_CAPS,&caps))exit(2);
 struct edev_ioc_queue_cfg cfg={.qid=71,.dir_mask=EDEV_QUEUE_DIR_BOTH,.ring_size=caps.ring_size};
 if(ioctl(qfd,EDEV_IOC_QUEUE_CONFIG,&cfg))exit(2);
 struct edev_ioc_user_buffer a={.user_va=(uintptr_t)src,.size=bytes,.dir=EDEV_DMA_DIR_H2D};
 struct edev_ioc_user_buffer b={.user_va=(uintptr_t)dst,.size=bytes,.dir=EDEV_DMA_DIR_D2H};
 if(ioctl(qfd,EDEV_IOC_REGISTER_BUFFER,&a)||ioctl(qfd,EDEV_IOC_REGISTER_BUFFER,&b))exit(2);
 for(unsigned off=0;off<bytes;off+=1048576u) {
  unsigned len=bytes-off;if(len>1048576u)len=1048576u;
  for(int dir=0;dir<2;dir++) {
   struct edev_ioc_dma_submit sub={.qid=71,.buf_id=dir?b.buf_id:a.buf_id,.dir=dir,.len=len,.dev_addr=0x80000000ull+off,.buf_offset=off};
   if(ioctl(qfd,EDEV_IOC_SUBMIT_BUFFER,&sub)||sub.status)hold("DDR preparation submit uncertain");
   struct edev_ioc_dma_cmpl c={.qid=71,.timeout_ms=5000};
   if(ioctl(qfd,EDEV_IOC_REAP_CMPL,&c)||c.status||c.error_code||c.bytes_done!=len||c.op_id!=sub.op_id)hold("DDR preparation completion uncertain");
  }
 }
 if(memcmp(src,dst,bytes)){fprintf(stderr,"DDR initialization full-readback mismatch\n");exit(3);}
 fprintf(stderr,"DDR_PREPARATION_VERIFIED base=0x80000000 bytes=%zu\n",bytes);
 if(ioctl(qfd,EDEV_IOC_UNREGISTER_BUFFER,&a.buf_id)||ioctl(qfd,EDEV_IOC_UNREGISTER_BUFFER,&b.buf_id))hold("DDR buffer deregistration");
 close(qfd);free(src);free(dst);
}
int main(int argc,char**argv){
 if(argc!=8||strcmp(argv[7],"--execute")){fprintf(stderr,"usage: %s RECEIVER_CONTROL_IPV4 PORT LENGTH BATCH host|ddr REPEATS --execute\nExclusive disabled LRDMA and idle applications required.\n",argv[0]);return 2;}
 unsigned port=strtoul(argv[2],0,10),length=strtoul(argv[3],0,0),batch=strtoul(argv[4],0,0);
 if(port<1024||port>65535||length<128||length>8u*1024*1024||(batch!=1&&batch!=50))return 2;
 unsigned repeats=strtoul(argv[6],0,10);if(!repeats||repeats>1000)return 2;
 int ddr=!strcmp(argv[5],"ddr");if(!ddr&&strcmp(argv[5],"host"))return 2;
 if(ddr)ddr_init(length);
 int fd=socket(AF_INET,SOCK_STREAM,0);struct sockaddr_in addr={.sin_family=AF_INET,.sin_port=htons(port)};
 if(fd<0||inet_pton(AF_INET,argv[1],&addr.sin_addr)!=1){perror("TCP address");return 2;}
 int connected=0;
 for(unsigned attempt=0;attempt<100;attempt++) {
  if(connect(fd,(void*)&addr,sizeof addr)==0){connected=1;break;}
  int saved=errno;close(fd);
  if(saved!=ECONNREFUSED&&saved!=EINTR){errno=saved;perror("TCP connect");return 2;}
  usleep(100000);fd=socket(AF_INET,SOCK_STREAM,0);if(fd<0)return 2;
 }
 if(!connected){perror("TCP connect retries exhausted");close(fd);return 2;}
 int nodelay=1;setsockopt(fd,IPPROTO_TCP,TCP_NODELAY,&nodelay,sizeof nodelay);
 FILE*rx=fdopen(dup(fd),"r"),*tx=fdopen(fd,"w");if(!rx||!tx)return 2;setvbuf(tx,NULL,_IONBF,0);
 char line[256],mac[80];unsigned remote_qpn,remote_psn,rkey;uint64_t remote_addr;size_t capacity;
 if(!fgets(line,sizeof line,rx)||sscanf(line,"META %u %u %u %" SCNu64 " %zu %79s",&remote_qpn,&remote_psn,&rkey,&remote_addr,&capacity,mac)!=6||!rkey||!remote_addr||length>capacity/batch){fprintf(stderr,"metadata invalid\n");return 2;}
 int resource_fd=-1;
 // 16 MiB coherent pool: source + retry data + SQ/CQ/RQ and error queues.
 struct rn_dev_t*rn=create_rn_dev_dma("/sys/bus/pci/devices/0005:01:00.0/resource2",&resource_fd,8,8,"/dev/edev0");
 struct rdma_dev_t*dev=create_rdma_dev(rn);
 if(rn_rdma_prepare_qp(dev,3)){perror("prepare QP");destroy_rn_dev(rn);return 2;}
 struct rdma_buff_t*db=allocate_rdma_buffer(rn,4096,HOST_MEM);
 struct rdma_buff_t*retry=allocate_rdma_buffer(rn,256*4096,HOST_MEM);
 struct rdma_buff_t*iperr=allocate_rdma_buffer(rn,8192,HOST_MEM);
 struct rdma_buff_t*err=allocate_rdma_buffer(rn,65536,HOST_MEM);
 struct rdma_buff_t*resp=allocate_rdma_buffer(rn,65536,HOST_MEM);
 struct rdma_buff_t*payload=allocate_rdma_buffer(rn,ddr?4096:length,HOST_MEM);
 if(!ddr)for(unsigned i=0;i<length;i++)((unsigned char*)payload->buffer)[i]=pattern(i);
 struct rdma_pd_t*pd=allocate_rdma_pd(dev,0);
 struct rdma_buff_t*credit=allocate_rdma_buffer(rn,4096,HOST_MEM);memset(credit->buffer,0,4096);rdma_register_memory_region(dev,pd,0x100,credit);
 struct mac_addr_t src=get_mac_addr_from_str_ip(fd,"192.168.1.10"),dst=convert_mac_addr_str_to_uint(mac);
 sigset_t block;sigemptyset(&block);sigaddset(&block,SIGPIPE);sigaddset(&block,SIGINT);sigaddset(&block,SIGTERM);sigaddset(&block,SIGHUP);sigprocmask(SIG_BLOCK,&block,NULL);
 open_rdma_dev(dev,src,0xc0a8010a,0x12b7,256,4096,retry->dma_addr,8192,iperr->dma_addr,256,256,err->dma_addr,(4096u<<16)|16,resp->dma_addr);
 allocate_rdma_qp(dev,3,remote_qpn,pd,db->dma_addr+3*64,db->dma_addr+8*64+3*64,64,HOST_MEM,&dst,0xc0a80114,0xffff,8);
 config_last_rq_psn(dev,3,(remote_psn-1)&0xffffff);config_sq_psn(dev,3,0x12345);
 if(getenv("CREDIT_CFG_SERIALIZED")){
  unsigned a[]={0x200004,0x200008,0x20000c,0x200010,0x200014,0x200018,0x20001c,0x3802b0,0x380204,0x380250,0x380254,0x380260,0x380248,0x380244,0x380200,0x300010,0x300014,0x300070,0x300000};
  uint32_t v[]={ (uint32_t)(uintptr_t)credit->buffer,(uint32_t)((uint64_t)(uintptr_t)credit->buffer>>32),(uint32_t)credit->dma_addr,(uint32_t)(credit->dma_addr>>32),0x100,4096,2,0};
  for(unsigned k=0;k<sizeof(a)/sizeof(a[0]);k++){
   uint32_t value=k<8?v[k]:read32_data(dev->axil_ctl,a[k]);
   write32_data(dev->axil_ctl,a[k],value);(void)read32_data(dev->axil_ctl,0x300000);usleep(1000);
   fprintf(stderr,"SERIALIZED_CFG address=0x%x value=0x%x\n",a[k],value);
  }
 }
 fprintf(stderr,"EXPECTED_CREDIT va=0x%016" PRIx64 " iova=0x%016" PRIx64 " rkey=0x100 length=8\n",(uint64_t)(uintptr_t)credit->buffer,credit->dma_addr);
 fprintf(tx,"REVERSE %" PRIu64 " %u\n",(uint64_t)(uintptr_t)credit->buffer,0x100u);
 if(!fgets(line,sizeof line,rx)||strcmp(line,"REVERSE_PASS\n")){rn_dma_rmb();fprintf(stderr,"FAILED_CREDIT_OBSERVED value=0x%016" PRIx64 "\n",*(volatile uint64_t*)credit->buffer);hold("reverse credit CQ failed");}
 rn_dma_rmb();if(*(volatile uint64_t*)credit->buffer!=1)hold("reverse credit CPU value mismatch");
 fprintf(stderr,"REVERSE_CREDIT_PASS value=1 full_8_bytes=1\n");
 fprintf(tx,"QUIT\n");if(!fgets(line,sizeof line,rx)||strcmp(line,"BYE\n"))hold("receiver teardown");
 // The peer has destroyed its QP after BYE; retire the FPGA QP while the
 // coherent pool remains mapped. Do not leave an enabled responder behind.
 write32_data(dev->axil_ctl,0x300000,read32_data(dev->axil_ctl,0x300000)&~1u);
 if(read32_data(dev->axil_ctl,0x300000)&1u)hold("RDMA disable did not settle");
 if(rn_rdma_prepare_qp(dev,3))hold("successful credit QP retirement");
 usleep(1000000);rn_dma_rmb();
 if(*(volatile uint64_t*)credit->buffer!=1)hold("credit changed during teardown");
 fprintf(stderr,"REVERSE_CREDIT_QP_RETIRED_RDMA_DISABLED_BEFORE_POOL_RELEASE\n");
 destroy_rn_dev(rn);close(resource_fd);fclose(rx);fclose(tx);return 0;
}
