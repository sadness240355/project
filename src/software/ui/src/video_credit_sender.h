#ifndef VIDEO_CREDIT_SENDER_H
#define VIDEO_CREDIT_SENDER_H
static inline int video_credit_wait(struct rdma_buff_t *buffer,uint64_t expected,uint64_t maximum) {
 uint64_t deadline=video_ns()+5000000000ULL;
 for(;;) {
  uint64_t word=*(volatile uint64_t*)buffer->buffer;rn_dma_rmb();
  uint64_t count=word&VIDEO_CREDIT_MASK;unsigned status=word>>56;
  if(status>2||count>maximum){fprintf(stderr,"FAIL: invalid RDMA credit\n");return -1;}
  if(count>=expected)return status==1?-1:(int)status;
  if(video_ns()>deadline){fprintf(stderr,"FAIL: RDMA credit timeout expected=%" PRIu64 " got=%" PRIu64 "\n",expected,count);return -1;}
  usleep(20);
 }
}
#endif
