#ifndef VIDEO_BOX_RECEIVER_H
#define VIDEO_BOX_RECEIVER_H
#include "video_box_channel.h"
struct box_receiver {
 struct ibv_cq *cq;struct ibv_qp *qp;struct ibv_mr *mr;
 struct video_boxes *buffer;struct box_mailbox output;struct video_credit_writer credit;
 int listen_fd,fd;uint32_t width,height;pthread_t thread;int started;
 _Atomic int stop,failed;_Atomic uint64_t count,last_frame,last_publish;
};
static void *box_receive_worker(void *ptr) {
 struct box_receiver *c=ptr;uint64_t last=0,report=video_ns();
 while(!atomic_load(&c->stop)) {
  struct box_notice n;
  uint64_t serial=__atomic_load_n((uint64_t*)((uint8_t*)c->buffer+1088),__ATOMIC_ACQUIRE);
  uint64_t expected=atomic_load(&c->count)+1;
  if(serial==expected) {
   memcpy(&n,(uint8_t*)c->buffer+1024,sizeof(n));
  } else {
   if(serial>expected)goto fail;
   struct pollfd p={c->fd,POLLIN,0};int ready=poll(&p,1,0);
   if(ready<0&&errno==EINTR)continue;
   if(ready==0){usleep(50);continue;}
   if(ready<0||rn_mlx5_recv_all(c->fd,&n,sizeof(n))||be64toh(n.publish_seq))goto fail;
  }
  if(ntohl(n.magic)!=BOX_CHANNEL_MAGIC)goto fail;
  uint64_t pub=be64toh(n.publish_seq);
  if(!pub){(void)video_ack_send(c->fd,0,0);return NULL;}
  __sync_synchronize();
  struct video_boxes b;memcpy(&b,c->buffer,sizeof(b));
  if(pub<=last||pub!=b.publish_seq||!video_boxes_valid(&b,c->width,c->height)||!b.valid||
     video_crc32(&b,sizeof(b))!=ntohl(n.crc)) {
   fprintf(stderr,"FAIL: independent boxes CRC/sequence/geometry\n");(void)video_ack_send(c->fd,pub,1);goto fail;
  }
  box_mailbox_publish(&c->output,&b);last=pub;
  atomic_fetch_add(&c->count,1);atomic_store(&c->last_frame,b.frame_id);atomic_store(&c->last_publish,pub);
  if(video_credit_send(&c->credit,atomic_load(&c->count),0))goto fail;
  if(video_ns()-report>=1000000000ULL) {
   fprintf(stderr,"BOXES received=%" PRIu64 " publish=%" PRIu64 " frame_seq=%" PRIu64 " count=%u transport=rdma\n",
    atomic_load(&c->count),pub,b.frame_id,b.count);report=video_ns();
  }
 }
 return NULL;
fail:
 if(!atomic_load(&c->stop)){fprintf(stderr,"FAIL: independent boxes channel disconnected\n");atomic_store(&c->failed,1);}
 return NULL;
}
static int box_receiver_start(struct box_receiver *c,struct ibv_context *context,struct ibv_pd *pd,
 const struct sockaddr_in *address,uint8_t port,uint8_t gid_index,const union ibv_gid *gid,
 uint32_t parent_qpn,int parent_fd,const char *output) {
 int one=1;c->listen_fd=c->fd=c->output.fd=-1;
 c->listen_fd=socket(AF_INET,SOCK_STREAM,0);if(c->listen_fd<0)return -1;
 setsockopt(c->listen_fd,SOL_SOCKET,SO_REUSEADDR,&one,sizeof(one));
 struct sockaddr_in a=*address;a.sin_port=htons(ntohs(a.sin_port)+1);
 if(bind(c->listen_fd,(struct sockaddr*)&a,sizeof(a))||listen(c->listen_fd,1))return -1;
 struct pollfd p={c->listen_fd,POLLIN,0};
 if(poll(&p,1,15000)<=0)return -1;
 struct sockaddr_in remote,parent;socklen_t len=sizeof(remote),plen=sizeof(parent);
 c->fd=accept(c->listen_fd,(struct sockaddr*)&remote,&len);
 if(c->fd<0||getpeername(parent_fd,(struct sockaddr*)&parent,&plen)||
    remote.sin_addr.s_addr!=parent.sin_addr.s_addr)return -1;
 box_socket_options(c->fd);
 struct box_hello h;
 if(rn_mlx5_recv_all(c->fd,&h,sizeof(h))||ntohl(h.magic)!=BOX_CHANNEL_MAGIC||
    ntohl(h.parent_qpn)!=parent_qpn||!video_check_conn(&h.conn)||video_conn_mode(&h.conn)!=VIDEO_NOTIFY_MODE||
    ntohl(h.conn.payload_size_be)!=VIDEO_BOX_BYTES||ntohl(h.conn.batch_be)!=1||
    ntohl(h.conn.qpn_be)!=BOX_CHANNEL_QPID||!ntohl(h.width)||!ntohl(h.height)||
    ntohl(h.width)>8192||ntohl(h.height)>8192)return -1;
 c->width=ntohl(h.width);c->height=ntohl(h.height);
 c->cq=ibv_create_cq(context,4,NULL,NULL,0);if(!c->cq)return -1;
 struct ibv_qp_init_attr attr={.send_cq=c->cq,.recv_cq=c->cq,.qp_type=IBV_QPT_RC,
  .cap={.max_send_wr=1,.max_recv_wr=1,.max_send_sge=1,.max_recv_sge=1}};
 c->qp=ibv_create_qp(pd,&attr);if(!c->qp||move_qp_to_init(c->qp,port))return -1;
 if(posix_memalign((void**)&c->buffer,4096,4096))return -1;
 memset(c->buffer,0,4096);c->mr=ibv_reg_mr(pd,c->buffer,4096,IBV_ACCESS_LOCAL_WRITE|IBV_ACCESS_REMOTE_WRITE);
 if(!c->mr||move_qp_to_rtr(c->qp,port,gid_index,&h.conn)||move_qp_to_rts(c->qp,0x13579))return -1;
 if(video_credit_open(&c->credit,pd,c->qp,c->cq,&h.conn))return -1;
 if(box_mailbox_open(&c->output,output,c->width,c->height))return -1;
 struct rn_mlx5_wire_conn local;
 video_init_conn(&local,c->qp->qp_num,0x13579,c->mr->rkey,(uintptr_t)c->buffer,VIDEO_BOX_BYTES,gid->raw);
 local.batch_be=htonl(1);video_set_conn_mode(&local,VIDEO_NOTIFY_MODE);
 if(rn_mlx5_send_all(c->fd,&local,sizeof(local)))return -1;
 if(pthread_create(&c->thread,NULL,box_receive_worker,c))return -1;
 c->started=1;fprintf(stderr,"Info: independent boxes RDMA QP ready, mailbox=%s\n",output);return 0;
}
static void box_receiver_close(struct box_receiver *c) {
 atomic_store(&c->stop,1);
 if(c->fd>=0)shutdown(c->fd,SHUT_RDWR);
 if(c->started)pthread_join(c->thread,NULL);
 if(c->qp)ibv_destroy_qp(c->qp);
 video_credit_close(&c->credit);
 if(c->mr)ibv_dereg_mr(c->mr);
 if(c->cq)ibv_destroy_cq(c->cq);
 free(c->buffer);box_mailbox_close(&c->output);
 if(c->fd>=0)close(c->fd);
 if(c->listen_fd>=0)close(c->listen_fd);
}
#endif
