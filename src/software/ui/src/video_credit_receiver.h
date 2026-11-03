#ifndef VIDEO_CREDIT_RECEIVER_H
#define VIDEO_CREDIT_RECEIVER_H
/* Every returned credit is a signalled, completed RC RDMA Write. No per-frame
 * TCP RTT or management-LAN retransmission enters the pixel pipeline. */
struct video_credit_writer {
 uint64_t *value,remote_addr;uint32_t rkey;struct ibv_mr *mr;struct ibv_qp *qp;struct ibv_cq *cq;
};
static int video_credit_open(struct video_credit_writer *c,struct ibv_pd *pd,
 struct ibv_qp *qp,struct ibv_cq *cq,const struct rn_mlx5_wire_conn *peer) {
 c->remote_addr=be64toh(peer->addr_be);c->rkey=ntohl(peer->rkey_be);c->qp=qp;c->cq=cq;
 if(!c->remote_addr||!c->rkey||posix_memalign((void**)&c->value,4096,4096))return -1;
 *c->value=0;c->mr=ibv_reg_mr(pd,c->value,4096,IBV_ACCESS_LOCAL_WRITE);return c->mr?0:-1;
}
static int video_credit_send(struct video_credit_writer *c,uint64_t count,unsigned status) {
 if(count>VIDEO_CREDIT_MASK||status>2)return -1;
 *c->value=count|((uint64_t)status<<56);__sync_synchronize();
 struct ibv_sge sge={.addr=(uintptr_t)c->value,.length=8,.lkey=c->mr->lkey};
 struct ibv_send_wr wr={.wr_id=count,.sg_list=&sge,.num_sge=1,.opcode=IBV_WR_RDMA_WRITE,
  .send_flags=IBV_SEND_SIGNALED,.wr.rdma={.remote_addr=c->remote_addr,.rkey=c->rkey}},*bad=NULL;
 if(ibv_post_send(c->qp,&wr,&bad))return -1;
 uint64_t deadline=video_ns()+5000000000ULL;
 for(;;) {
  struct ibv_wc wc;int n=ibv_poll_cq(c->cq,1,&wc);
  if(n<0)return -1;
  if(n>0) {
   if(wc.status!=IBV_WC_SUCCESS||wc.wr_id!=count) {
    fprintf(stderr,"FAIL: RDMA credit completion status=%s wrid=%" PRIu64 "\n",ibv_wc_status_str(wc.status),wc.wr_id);return -1;
   }
   return 0;
  }
  if(video_ns()>deadline){fprintf(stderr,"FAIL: RDMA credit timeout\n");return -1;}
  usleep(10);
 }
}
static void video_credit_close(struct video_credit_writer *c) {
 if(c->mr)ibv_dereg_mr(c->mr);
 free(c->value);c->mr=NULL;c->value=NULL;
}
#endif
