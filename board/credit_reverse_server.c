#include <time.h>
#define _GNU_SOURCE
#include <infiniband/verbs.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <inttypes.h>
// Fresh control protocol; TCP carries metadata/verification commands, never payload.
// Fixed peer configuration is deliberately explicit and must match sender.
static unsigned char pattern(uint64_t i){return (17*i+(i>>8)+0x5a)&255;}
static uint32_t crc_table[256];
static unsigned char colorbar(uint64_t i){static const unsigned char rgb[8][3]={{255,255,255},{255,255,0},{0,255,255},{0,255,0},{255,0,255},{255,0,0},{0,0,255},{0,0,0}};return rgb[((i/3)%2400)*8/2400][i%3];}
static void fail(const char*s){perror(s);exit(2);}
int main(int argc,char**argv){
 if(argc!=3){fprintf(stderr,"usage: %s TCP_PORT MAX_REGISTERED_BYTES\n",argv[0]);return 2;}
 for(unsigned i=0;i<256;i++){uint32_t c=i;for(int j=0;j<8;j++)c=(c>>1)^((c&1)?0xedb88320u:0);crc_table[i]=c;}
 unsigned port=strtoul(argv[1],0,10);size_t capacity=strtoull(argv[2],0,0);
 if(port<1024||port>65535||!capacity||capacity>512ull*1024*1024)return 2;
 int n;struct ibv_device **list=ibv_get_device_list(&n);struct ibv_context*ctx=NULL;
 if(!list)fail("device list");
 for(int i=0;i<n;i++)if(!strcmp(ibv_get_device_name(list[i]),"mlx5_0"))ctx=ibv_open_device(list[i]);
 if(!ctx)fail("mlx5_0 open");
 struct ibv_port_attr pa;if(ibv_query_port(ctx,1,&pa)||pa.state!=IBV_PORT_ACTIVE||pa.active_mtu<IBV_MTU_4096)fail("active port/4096 MTU required");
 union ibv_gid gid;int gid_index=-1;unsigned char ip[4];inet_pton(AF_INET,"192.168.1.20",ip);
 for(int i=0;i<pa.gid_tbl_len;i++){
  char path[256],type[80]={0};snprintf(path,sizeof path,"/sys/class/infiniband/mlx5_0/ports/1/gid_attrs/types/%d",i);
  FILE*f=fopen(path,"r");if(f){if(!fgets(type,sizeof type,f))type[0]=0;fclose(f);}
  if(!ibv_query_gid(ctx,1,i,&gid)&&strstr(type,"RoCE v2")&&!memcmp(gid.raw+12,ip,4)) {gid_index=i;break;}
 }
 if(gid_index<0){fprintf(stderr,"No RoCE v2 IPv4 GID for 192.168.1.20\n");return 2;}
 struct ibv_pd*pd=ibv_alloc_pd(ctx);if(!pd)fail("alloc_pd");
 void*mem=NULL;if(posix_memalign(&mem,4096,capacity))return 2;memset(mem,0xa5,capacity);
 struct ibv_mr*mr=ibv_reg_mr(pd,mem,capacity,IBV_ACCESS_LOCAL_WRITE|IBV_ACCESS_REMOTE_WRITE);if(!mr)fail("reg_mr");
 struct ibv_cq*cq=ibv_create_cq(ctx,128,NULL,NULL,0);if(!cq)fail("create_cq");
 struct ibv_qp_init_attr qi={.send_cq=cq,.recv_cq=cq,.qp_type=IBV_QPT_RC,.cap={.max_send_wr=64,.max_recv_wr=64,.max_send_sge=1,.max_recv_sge=1}};
 struct ibv_qp*qp=ibv_create_qp(pd,&qi);if(!qp)fail("create_qp");
 struct ibv_qp_attr qa={.qp_state=IBV_QPS_INIT,.pkey_index=0,.port_num=1,.qp_access_flags=IBV_ACCESS_LOCAL_WRITE|IBV_ACCESS_REMOTE_WRITE};
 if(ibv_modify_qp(qp,&qa,IBV_QP_STATE|IBV_QP_PKEY_INDEX|IBV_QP_PORT|IBV_QP_ACCESS_FLAGS))fail("QP INIT");
 memset(&qa,0,sizeof qa);qa.qp_state=IBV_QPS_RTR;qa.path_mtu=IBV_MTU_4096;qa.dest_qp_num=3;qa.rq_psn=0x12345;qa.max_dest_rd_atomic=1;qa.min_rnr_timer=12;
 qa.ah_attr.is_global=1;qa.ah_attr.port_num=1;qa.ah_attr.grh.sgid_index=gid_index;qa.ah_attr.grh.hop_limit=64;
 qa.ah_attr.grh.dgid.raw[10]=255;qa.ah_attr.grh.dgid.raw[11]=255;inet_pton(AF_INET,"192.168.1.10",qa.ah_attr.grh.dgid.raw+12);
 if(ibv_modify_qp(qp,&qa,IBV_QP_STATE|IBV_QP_AV|IBV_QP_PATH_MTU|IBV_QP_DEST_QPN|IBV_QP_RQ_PSN|IBV_QP_MAX_DEST_RD_ATOMIC|IBV_QP_MIN_RNR_TIMER))fail("QP RTR");
 memset(&qa,0,sizeof qa);qa.qp_state=IBV_QPS_RTS;qa.sq_psn=0x23456;qa.timeout=14;qa.retry_cnt=7;qa.rnr_retry=7;qa.max_rd_atomic=1;
 if(ibv_modify_qp(qp,&qa,IBV_QP_STATE|IBV_QP_SQ_PSN|IBV_QP_TIMEOUT|IBV_QP_RETRY_CNT|IBV_QP_RNR_RETRY|IBV_QP_MAX_QP_RD_ATOMIC))fail("QP RTS");
 char mac[80];FILE*mf=fopen("/sys/class/net/enp1s0f0np0/address","r");if(!mf||!fgets(mac,sizeof mac,mf))fail("MAC");fclose(mf);mac[strcspn(mac,"\r\n")]=0;
 int ls=socket(AF_INET,SOCK_STREAM,0),one=1;if(ls<0)fail("socket");setsockopt(ls,SOL_SOCKET,SO_REUSEADDR,&one,sizeof one);
 struct sockaddr_in addr={.sin_family=AF_INET,.sin_port=htons(port)};inet_pton(AF_INET,"10.23.66.188",&addr.sin_addr);
 if(bind(ls,(void*)&addr,sizeof addr)||listen(ls,1))fail("bind/listen");
 printf("RECEIVER_READY port=%u capacity=%zu gid_index=%d mtu=4096\n",port,capacity,gid_index);fflush(stdout);
 int fd=accept(ls,NULL,NULL);if(fd<0)fail("accept");close(ls);
 setsockopt(fd,IPPROTO_TCP,TCP_NODELAY,&one,sizeof one);
 FILE*io=fdopen(fd,"w"),*input=fdopen(dup(fd),"r");if(!io||!input)fail("fdopen");setvbuf(io,NULL,_IONBF,0);
 fprintf(io,"META %u %u %u %" PRIu64 " %zu %s\n",qp->qp_num,0x23456,mr->rkey,(uint64_t)(uintptr_t)mem,capacity,mac);
 char line[256];size_t len=0;unsigned batch=0;int prepared=0,rc=0,orderly_quit=0,colorbar_mode=0;
 while(fgets(line,sizeof line,input)){
  char kind[16]="byte";
  uint64_t reverse_addr=0;unsigned reverse_key=0;
  if(sscanf(line,"REVERSE %" SCNu64 " %u",&reverse_addr,&reverse_key)==2){
   if(!reverse_addr||!reverse_key){rc=3;break;}
   *(uint64_t*)mem=1;__sync_synchronize();
   struct ibv_sge sg={.addr=(uintptr_t)mem,.length=8,.lkey=mr->lkey};
   struct ibv_send_wr wr={.wr_id=1,.sg_list=&sg,.num_sge=1,.opcode=IBV_WR_RDMA_WRITE,.send_flags=IBV_SEND_SIGNALED,.wr.rdma={.remote_addr=reverse_addr,.rkey=reverse_key}},*badwr=NULL;
   if(ibv_post_send(qp,&wr,&badwr)){fprintf(io,"REVERSE_FAIL post\n");rc=3;break;}
   struct ibv_wc wc;int got=0;struct timespec ts;clock_gettime(CLOCK_MONOTONIC,&ts);time_t deadline=ts.tv_sec+5;
   while(!got){got=ibv_poll_cq(cq,1,&wc);clock_gettime(CLOCK_MONOTONIC,&ts);if(got<0||ts.tv_sec>deadline){fprintf(io,"REVERSE_FAIL timeout\n");rc=3;break;}usleep(10);}
   if(rc)break;
   printf("REVERSE_CQ status=%s wrid=%" PRIu64 "\n",ibv_wc_status_str(wc.status),wc.wr_id);fflush(stdout);
   if(wc.status!=IBV_WC_SUCCESS||wc.wr_id!=1){fprintf(io,"REVERSE_FAIL completion\n");rc=3;break;}
   fprintf(io,"REVERSE_PASS\n");
  } else if(sscanf(line,"PREP %zu %u %15s",&len,&batch,kind)>=2){
   colorbar_mode=!strcmp(kind,"colorbar");
   if(colorbar_mode&&(len!=2400u*2048u*3u||batch!=1)){rc=3;fprintf(io,"ERROR colorbar geometry\n");break;}
   if(!len||!batch||batch>50||len>capacity/batch){fprintf(io,"ERROR range\n");rc=3;break;}
   memset(mem,0xa5,len*batch);__sync_synchronize();prepared=1;fprintf(io,"READY\n");
  } else if(!strcmp(line,"CHECK\n")&&prepared){
   __sync_synchronize();uint64_t bad=0,first=UINT64_MAX;uint32_t checksum=~0u;
   for(unsigned j=0;j<batch;j++)for(size_t i=0;i<len;i++){unsigned char actual=((volatile unsigned char*)mem)[j*len+i];if(actual!=(colorbar_mode?colorbar(i):pattern(i))){if(!bad)first=j*len+i;bad++;}if(colorbar_mode)checksum=crc_table[(checksum^actual)&255]^(checksum>>8);}
   checksum=~checksum;
   printf("VERIFY length=%zu batch=%u checked=%zu mismatches=%" PRIu64 " first=%" PRIu64 " crc32=%08x\n",len,batch,len*batch,bad,first,checksum);fflush(stdout);
   fprintf(io,"%s %zu %" PRIu64 " %08x\n",bad?"FAIL":"PASS",len*batch,bad,checksum);prepared=colorbar_mode;if(bad){rc=3;break;}
  } else if(!strcmp(line,"QUIT\n")){fprintf(io,"BYE\n");orderly_quit=1;break;}
  else {fprintf(io,"ERROR command\n");rc=3;break;}
 }
 if(!orderly_quit&&rc==0)rc=4;
 // Move the RC QP to ERR before deregistering remotely writable memory.
 memset(&qa,0,sizeof qa);qa.qp_state=IBV_QPS_ERR;ibv_modify_qp(qp,&qa,IBV_QP_STATE);
 fclose(input);fclose(io);ibv_destroy_qp(qp);ibv_destroy_cq(cq);ibv_dereg_mr(mr);free(mem);ibv_dealloc_pd(pd);ibv_close_device(ctx);ibv_free_device_list(list);return rc;
}
