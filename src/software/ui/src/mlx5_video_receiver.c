#include "video_common.h"
#include "video_display.h"
#include "video_crc.h"
#include "video_notify.h"
#include "video_session_wait.h"

#include <getopt.h>
#include <infiniband/verbs.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <netinet/tcp.h>

#define RN_MLX5_DEFAULT_MAX_PAYLOAD VIDEO_MAX_BYTES

static volatile sig_atomic_t video_stop;
static void stop_video(int s) {(void)s;video_stop=1;}

static void usage(const char* program) {
  fprintf(stderr,
          "Usage: %s --gid-index INDEX [options]\n"
          "  --boxes-channel    Independent RDMA boxes QP (control tcp-port+1)\n"
          "  --boxes-file PATH  Local YBOX mailbox (default ./received_boxes.bin)\n"
          "  --device NAME       RDMA device (default mlx5_0)\n"
          "  --ib-port PORT      RDMA physical port (default 1)\n"
          "  --gid-index INDEX   RoCE v2 GID index for the server IPv4 address\n"
          "  --listen-ip ADDR    TCP/RoCE server IPv4 (default 192.168.1.20)\n"
          "  --netdev NAME       Mellanox port (default enp1s0f0np0)\n"
          "  --control-ip ADDR   Metadata TCP bind IP (RDMA GID remains --listen-ip)\n"
          "  --tcp-port PORT     Metadata TCP port (default 11112)\n"
          "  --max-size BYTES    Maximum frame bytes (default 16777216)\n"
          "  --output-file PATH  Local mmap mailbox (default ./received_frames.bin)\n"
          "  --save-last FILE    Save last verified frame as PPM on exit\n"
          "  --psn PSN           Local 24-bit starting PSN (default 0xabd)\n"
          "  --pressure-preview-ms N  Sample verified color bars every N ms\n",
          program);
}

static int parse_u32(const char* text, uint32_t* value) {
  char* end = NULL;
  unsigned long parsed;

  errno = 0;
  parsed = strtoul(text, &end, 0);
  if((errno != 0) || (end == text) || (*end != '\0') ||
     (parsed > UINT32_MAX)) {
    return -1;
  }
  *value = (uint32_t) parsed;
  return 0;
}

static struct ibv_context* open_named_device(const char* name) {
  struct ibv_device** devices;
  struct ibv_context* context = NULL;
  int count = 0;

  devices = ibv_get_device_list(&count);
  if(devices == NULL) {
    return NULL;
  }
  for(int i=0; i<count; i++) {
    if(strcmp(ibv_get_device_name(devices[i]), name) == 0) {
      context = ibv_open_device(devices[i]);
      break;
    }
  }
  ibv_free_device_list(devices);
  return context;
}

static int move_qp_to_init(struct ibv_qp* qp, uint8_t ib_port) {
  struct ibv_qp_attr attr;

  memset(&attr, 0, sizeof(attr));
  attr.qp_state = IBV_QPS_INIT;
  attr.pkey_index = 0;
  attr.port_num = ib_port;
  attr.qp_access_flags = IBV_ACCESS_LOCAL_WRITE |
                         IBV_ACCESS_REMOTE_WRITE |
                         IBV_ACCESS_REMOTE_READ;
  return ibv_modify_qp(qp, &attr,
                       IBV_QP_STATE | IBV_QP_PKEY_INDEX |
                       IBV_QP_PORT | IBV_QP_ACCESS_FLAGS);
}

static int move_qp_to_rtr(struct ibv_qp* qp, uint8_t ib_port,
                          uint8_t gid_index,
                          const struct rn_mlx5_wire_conn* peer) {
  struct ibv_qp_attr attr;

  memset(&attr, 0, sizeof(attr));
  attr.qp_state = IBV_QPS_RTR;
  attr.path_mtu = IBV_MTU_4096;
  attr.dest_qp_num = ntohl(peer->qpn_be);
  attr.rq_psn = ntohl(peer->psn_be);
  attr.max_dest_rd_atomic = 1;
  attr.min_rnr_timer = 12;
  attr.ah_attr.is_global = 1;
  attr.ah_attr.port_num = ib_port;
  memcpy(&attr.ah_attr.grh.dgid, peer->gid, sizeof(peer->gid));
  attr.ah_attr.grh.sgid_index = gid_index;
  attr.ah_attr.grh.hop_limit = 64;

  return ibv_modify_qp(qp, &attr,
                       IBV_QP_STATE | IBV_QP_AV | IBV_QP_PATH_MTU |
                       IBV_QP_DEST_QPN | IBV_QP_RQ_PSN |
                       IBV_QP_MAX_DEST_RD_ATOMIC | IBV_QP_MIN_RNR_TIMER);
}

static int move_qp_to_rts(struct ibv_qp* qp, uint32_t psn) {
  struct ibv_qp_attr attr;

  memset(&attr, 0, sizeof(attr));
  attr.qp_state = IBV_QPS_RTS;
  attr.timeout = 14;
  attr.retry_cnt = 7;
  attr.rnr_retry = 7;
  attr.sq_psn = psn;
  attr.max_rd_atomic = 1;
  return ibv_modify_qp(qp, &attr,
                       IBV_QP_STATE | IBV_QP_TIMEOUT | IBV_QP_RETRY_CNT |
                       IBV_QP_RNR_RETRY | IBV_QP_SQ_PSN |
                       IBV_QP_MAX_QP_RD_ATOMIC);
}

#include "video_credit_receiver.h"
#include "video_box_receiver.h"

int main(int argc, char** argv) {
  static const struct option options[] = {
    {"boxes-channel", no_argument, NULL, 'B'},
    {"boxes-file", required_argument, NULL, 'F'},
    {"control-ip", required_argument, NULL, 'C'},
    {"device", required_argument, NULL, 'd'},
    {"ib-port", required_argument, NULL, 'p'},
    {"gid-index", required_argument, NULL, 'g'},
    {"listen-ip", required_argument, NULL, 'a'},
    {"tcp-port", required_argument, NULL, 't'},
    {"max-size", required_argument, NULL, 'z'},
    {"psn", required_argument, NULL, 'n'},
    {"netdev", required_argument, NULL, 'e'},
    {"output-file", required_argument, NULL, 'f'},
    {"save-last", required_argument, NULL, 'o'},
    {"pressure-preview-ms", required_argument, NULL, 'P'},
    {"help", no_argument, NULL, 'h'},
    {NULL, 0, NULL, 0}
  };
  const char *control_ip=NULL,*boxes_file="./received_boxes.bin";
  int boxes_channel=0;struct box_receiver boxes_rx={.listen_fd=-1,.fd=-1,.output={.fd=-1}};
  int crc_mode=video_crc_mode_env();
  if(crc_mode<0){fprintf(stderr,"FAIL: invalid VIDEO_CRC_MODE\n");return 2;}
  uint64_t crc_checked=0;
  uint64_t prof_wait_ns=0,prof_crc_ns=0,prof_display_ns=0,prof_preview_ns=0,prof_credit_ns=0;
  uint32_t prof_samples=0;
  uint32_t report_done=0,remote_stride=0;int direct=0;
  uint32_t pressure_preview_ms=0;
  uint64_t last_preview_ns=0;
  struct video_credit_writer credit={0};
  struct in_addr control_addr;
  const char* device_name = "mlx5_0";
  const char* listen_ip_text = "192.168.1.20";
  const char* netdev = "enp1s0f0np0";
  uint32_t count, batch, done = 0;
  const char *shm_name="./received_frames.bin",*save_last=NULL;
  struct video_shm output={.fd=-1};
  struct vdq display_queue={.fd=-1,.lock_fd=-1};
  const char *display_queue_path=getenv("VIDEO_DISPLAY_FIFO");
  struct video_header frame;
  uint64_t last_pub=0,last_frame_id=0,crc_errors=0;
  uint32_t width=0,height=0,format=0;
  uint64_t start_ns=video_ns(),report_ns=start_ns;
  struct sigaction action;memset(&action,0,sizeof(action));action.sa_handler=stop_video;
  sigaction(SIGINT,&action,NULL);sigaction(SIGTERM,&action,NULL);sigaction(SIGHUP,&action,NULL);
  uint64_t region_size;
  uint32_t ib_port_value = 1;
  uint32_t gid_index_value = UINT32_MAX;
  uint32_t tcp_port = 11112;
  uint32_t max_payload = RN_MLX5_DEFAULT_MAX_PAYLOAD;
  uint32_t local_psn = 0xabd;
  struct in_addr listen_in_addr;
  struct sockaddr_in listen_addr;
  struct ibv_context* context = NULL;
  struct ibv_pd* pd = NULL;
  struct ibv_cq* cq = NULL;
  struct ibv_qp* qp = NULL;
  struct ibv_mr* mr = NULL;
  struct ibv_port_attr port_attr;
  union ibv_gid local_gid;
  struct ibv_qp_init_attr qp_init_attr;
  struct rn_mlx5_wire_conn local_conn;
  struct rn_mlx5_wire_conn peer_conn;
  uint8_t expected_gid[16];
  uint8_t* buffer = NULL;
  uint32_t payload_size = 0;
  int listen_fd = -1;
  int peer_fd = -1;
  int reuse = 1;
  int option;
  int result = 1;

  while((option = getopt_long(argc, argv, "C:d:p:g:a:t:z:n:e:f:o:P:h",
                              options, NULL)) != -1) {
    switch(option) {
      case 'B': boxes_channel=1;break;
      case 'F': boxes_file=optarg;break;
      case 'C': control_ip=optarg;break;
      case 'd': device_name = optarg; break;
      case 'f': shm_name=optarg;break;
      case 'o': save_last=optarg;break;
      case 'P':
        if(parse_u32(optarg,&pressure_preview_ms) ||
           pressure_preview_ms<50 || pressure_preview_ms>2000)return 2;
        break;
      case 'p':
        if(parse_u32(optarg, &ib_port_value) != 0) { usage(argv[0]); return 2; }
        break;
      case 'g':
        if(parse_u32(optarg, &gid_index_value) != 0) { usage(argv[0]); return 2; }
        break;
      case 'a': listen_ip_text = optarg; break;
      case 'e': netdev = optarg; break;
      case 't':
        if(parse_u32(optarg, &tcp_port) != 0) { usage(argv[0]); return 2; }
        break;
      case 'z':
        if(parse_u32(optarg, &max_payload) != 0) { usage(argv[0]); return 2; }
        break;
      case 'n':
        if(parse_u32(optarg, &local_psn) != 0) { usage(argv[0]); return 2; }
        break;
      case 'h': usage(argv[0]); return 0;
      default: usage(argv[0]); return 2;
    }
  }

  if((control_ip && inet_pton(AF_INET,control_ip,&control_addr)!=1) || (ib_port_value == 0) || (ib_port_value > UINT8_MAX) ||
     (gid_index_value == UINT32_MAX) || (gid_index_value > UINT8_MAX) ||
     (tcp_port == 0) || (tcp_port > UINT16_MAX-(boxes_channel?1U:0U)) ||
     (max_payload == 0) || (max_payload>VIDEO_MAX_BYTES) || strncmp(shm_name,"./",2)!=0 || strncmp(boxes_file,"./",2)!=0 ||
     ((local_psn & UINT32_C(0xff000000)) != 0) ||
     (inet_pton(AF_INET, listen_ip_text, &listen_in_addr) != 1)) {
    usage(argv[0]);
    return 2;
  }

  context = open_named_device(device_name);
  if(context == NULL) {
    fprintf(stderr, "Error: cannot open RDMA device %s\n", device_name);
    goto out;
  }
  if(ibv_query_port(context, (uint8_t) ib_port_value, &port_attr) != 0) {
    perror("ibv_query_port");
    goto out;
  }
  if((port_attr.state != IBV_PORT_ACTIVE) ||
     (port_attr.link_layer != IBV_LINK_LAYER_ETHERNET) ||
     (port_attr.active_mtu != IBV_MTU_4096)) {
    fprintf(stderr,
            "Error: mlx5 port must be ACTIVE, Ethernet, active_mtu 4096 "
            "(state=%d link_layer=%d active_mtu=%d)\n",
            port_attr.state, port_attr.link_layer, port_attr.active_mtu);
    goto out;
  }
  if(ibv_query_gid(context, (uint8_t) ib_port_value,
                   (int) gid_index_value, &local_gid) != 0) {
    perror("ibv_query_gid");
    goto out;
  }
  rn_mlx5_ipv4_gid(listen_in_addr, expected_gid);
  if(memcmp(local_gid.raw, expected_gid, sizeof(expected_gid)) != 0) {
    char actual_gid[INET6_ADDRSTRLEN];
    inet_ntop(AF_INET6, local_gid.raw, actual_gid, sizeof(actual_gid));
    fprintf(stderr,
            "Error: GID index %u is %s, not the IPv4 GID for %s; "
            "select its RoCE v2 index from show_gids\n",
            gid_index_value, actual_gid, listen_ip_text);
    goto out;
  }

  // IPv4 GIDs can have both v1 and v2 entries: address equality is insufficient.
  char gid_path[512], gid_text[128];
  const char* attrs[] = {"types", "ndevs"};
  const char* values[] = {"RoCE v2", netdev};
  for(unsigned int k=0; k<2; k++) {
    snprintf(gid_path, sizeof(gid_path), "/sys/class/infiniband/%s/ports/%u/gid_attrs/%s/%u",
             device_name, ib_port_value, attrs[k], gid_index_value);
    FILE* f = fopen(gid_path, "r");
    if(f == NULL) { perror(gid_path); goto out; }
    char* got = fgets(gid_text, sizeof(gid_text), f);
    fclose(f);
    if(got == NULL) goto out;
    gid_text[strcspn(gid_text, "\r\n")] = 0;
    if(strcmp(gid_text, values[k])) {
      fprintf(stderr, "Error: GID %s is '%s', expected '%s'\n", attrs[k], gid_text, values[k]);
      goto out;
    }
  }

  pd = ibv_alloc_pd(context);
  cq = ibv_create_cq(context, 4, NULL, NULL, 0);
  if((pd == NULL) || (cq == NULL)) {
    fprintf(stderr, "Error: failed to allocate mlx5 PD/CQ\n");
    goto out;
  }

  memset(&qp_init_attr, 0, sizeof(qp_init_attr));
  qp_init_attr.send_cq = cq;
  qp_init_attr.recv_cq = cq;
  qp_init_attr.qp_type = IBV_QPT_RC;
  qp_init_attr.cap.max_send_wr = 1;
  qp_init_attr.cap.max_recv_wr = 1;
  qp_init_attr.cap.max_send_sge = 1;
  qp_init_attr.cap.max_recv_sge = 1;
  qp = ibv_create_qp(pd, &qp_init_attr);
  if((qp == NULL) || (move_qp_to_init(qp, (uint8_t) ib_port_value) != 0)) {
    fprintf(stderr, "Error: failed to create/initialize mlx5 RC QP\n");
    goto out;
  }

  if(posix_memalign((void**) &buffer, (size_t) sysconf(_SC_PAGESIZE),
                    (size_t)video_region_stride(max_payload,1)*VIDEO_WINDOW) != 0) {
    fprintf(stderr, "Error: failed to allocate server buffer\n");
    goto out;
  }
  memset(buffer, 0, (size_t)video_region_stride(max_payload,1)*VIDEO_WINDOW);
  mr = ibv_reg_mr(pd, buffer, (size_t)video_region_stride(max_payload,1)*VIDEO_WINDOW,
                  IBV_ACCESS_LOCAL_WRITE | IBV_ACCESS_REMOTE_WRITE |
                  IBV_ACCESS_REMOTE_READ);
  if(mr == NULL) {
    perror("ibv_reg_mr");
    goto out;
  }

  listen_fd = socket(AF_INET, SOCK_STREAM, 0);
  if(listen_fd < 0) {
    perror("socket");
    goto out;
  }
  if(!control_ip && setsockopt(listen_fd, SOL_SOCKET, SO_BINDTODEVICE, netdev, strlen(netdev)+1)) {
    perror("bind Mellanox netdev"); goto out;
  }
  setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
  memset(&listen_addr, 0, sizeof(listen_addr));
  listen_addr.sin_family = AF_INET;
  listen_addr.sin_port = htons((uint16_t) tcp_port);
  listen_addr.sin_addr = control_ip ? control_addr : listen_in_addr;
  if((bind(listen_fd, (struct sockaddr*) &listen_addr,
           sizeof(listen_addr)) != 0) ||
     (listen(listen_fd, 1) != 0)) {
    perror("bind/listen");
    goto out;
  }

  fprintf(stderr,
          "Info: listening on %s:%u, device=%s port=%u gid_index=%u\n",
          control_ip ? control_ip : listen_ip_text, tcp_port, device_name, ib_port_value,
          gid_index_value);
  peer_fd = accept(listen_fd, NULL, NULL);
  if(peer_fd < 0) {
    perror("accept");
    goto out;
  }
  struct timeval timeout = {.tv_sec=30};
  setsockopt(peer_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  setsockopt(peer_fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
  setsockopt(peer_fd, IPPROTO_TCP, TCP_NODELAY, &reuse, sizeof(reuse));
  if((rn_mlx5_recv_all(peer_fd, &peer_conn, sizeof(peer_conn)) != 0) ||
     !video_check_conn(&peer_conn)) {
    fprintf(stderr, "Error: invalid ERNIC connection metadata\n");
    goto out;
  }
  payload_size = ntohl(peer_conn.payload_size_be);
  count = ntohl(peer_conn.count_be);
  batch = ntohl(peer_conn.batch_be);
  direct=video_conn_mode(&peer_conn)==VIDEO_NOTIFY_MODE ||
         video_conn_mode(&peer_conn)==VIDEO_NOTIFY_MODE_PATTERN;
  if((video_conn_mode(&peer_conn)==VIDEO_NOTIFY_MODE_PATTERN)!=(pressure_preview_ms!=0)) {
    fprintf(stderr,"FAIL: pattern pressure mode mismatch\n");goto out;
  }
  if(video_conn_mode(&peer_conn) && !direct){fprintf(stderr,"FAIL: unknown notification mode\n");goto out;}
  if(!video_crc_agreed(&peer_conn,crc_mode,direct)) {
    fprintf(stderr,"FAIL: CRC mode mismatch or unsupported transport mode (local=%s peer=%u)\n",
            video_crc_mode_name(crc_mode),video_conn_crc(&peer_conn));goto out;
  }
  remote_stride=video_region_stride(payload_size,direct);
  region_size = (uint64_t)remote_stride*batch;
  if(batch != VIDEO_WINDOW ||
     payload_size == 0 || payload_size > RN_MLX5_DEFAULT_MAX_PAYLOAD || region_size > (uint64_t)video_region_stride(max_payload,1)*VIDEO_WINDOW) {
    fprintf(stderr, "Error: requested payload %u exceeds maximum %u\n",
            payload_size, max_payload);
    goto out;
  }

  if((move_qp_to_rtr(qp, (uint8_t) ib_port_value,
                     (uint8_t) gid_index_value, &peer_conn) != 0) ||
     (move_qp_to_rts(qp, local_psn) != 0)) {
    fprintf(stderr, "Error: failed to move mlx5 RC QP to RTS: %s\n",
            strerror(errno));
    goto out;
  }

  video_init_conn(&local_conn, qp->qp_num, local_psn, mr->rkey,
                    (uint64_t) (uintptr_t) buffer, payload_size,
                    local_gid.raw);
  if(direct && video_credit_open(&credit,pd,qp,cq,&peer_conn)){perror("credit MR");goto out;}
  video_set_conn_mode(&local_conn,direct?
    (pressure_preview_ms?VIDEO_NOTIFY_MODE_PATTERN:VIDEO_NOTIFY_MODE):0);
  video_set_conn_crc(&local_conn,(uint32_t)crc_mode);
  fprintf(stderr,"INTEGRITY crc_mode=%s software_crc=%s\n",video_crc_mode_name(crc_mode),
          crc_mode==VIDEO_CRC_FULL?"every_frame":"diagnostic_disabled");
  fprintf(stderr,"CRC_BACKEND backend=%s\n",video_crc_backend());
  local_conn.count_be = htonl(count);
  local_conn.batch_be = htonl(batch);
  if(rn_mlx5_send_all(peer_fd, &local_conn, sizeof(local_conn)) != 0) {
    fprintf(stderr, "Error: failed to send mlx5 connection metadata\n");
    goto out;
  }

  fprintf(stderr,
          "Info: QP RTS, local QPN=0x%x PSN=0x%x rkey=0x%x "
          "addr=0x%016" PRIx64 "; waiting for ERNIC RDMA Write\n",
          qp->qp_num, local_psn, mr->rkey, (uint64_t) (uintptr_t) buffer);
  if(boxes_channel && box_receiver_start(&boxes_rx,context,pd,&listen_addr,ib_port_value,gid_index_value,
       &local_gid,qp->qp_num,peer_fd,boxes_file)) {
    perror("independent boxes RDMA initialization");goto out;
  }
  start_ns=video_ns();report_ns=start_ns;
  for(;;) {
    uint64_t prof_begin=video_ns();
    if(direct) {
      const uint8_t *slot=buffer+(size_t)(done%VIDEO_WINDOW)*remote_stride;
      int wait_result=video_wait_rdma_frame(slot,payload_size,(uint64_t)done+1,peer_fd,
                                           count==0,30000000000ULL,&video_stop,&frame);
      if(wait_result>0) {result=0;break;}
      if(wait_result<0)goto out;
    } else if(rn_mlx5_recv_all(peer_fd,&frame,sizeof(frame))) {
      fprintf(stderr,"FAIL: video control disconnected/timed out; complete frames=%u\n",done);goto out;
    }
    prof_wait_ns+=video_ns()-prof_begin;
    if(ntohl(frame.magic)==VIDEO_MAGIC && ntohl(frame.version)==VIDEO_VERSION && ntohl(frame.type)==VIDEO_END) {
      if(count && done!=count) {fprintf(stderr,"FAIL: early END\n");goto out;}
      (void)video_ack_send(peer_fd,0,0);break;
    }
    if((count && done>=count)||!video_check_frame(&frame,payload_size)||be64toh(frame.publish_seq)<=last_pub) {
      fprintf(stderr,"FAIL: invalid/stale frame header\n");(void)video_ack_send(peer_fd,be64toh(frame.frame_id),1);goto out;
    }
    if(!output.h) {
      width=ntohl(frame.width);height=ntohl(frame.height);format=ntohl(frame.format);
      if(video_map_write_open(&output,shm_name,width,height,format,1)) {perror("output mmap file");goto out;}
      if(display_queue_path && *display_queue_path &&
         vdq_create(&display_queue,display_queue_path,width,height,format,VDQ_SLOTS)) {
        perror("ordered display queue setup");goto out;
      }
    }
    if(boxes_channel && (atomic_load(&boxes_rx.failed)||width!=boxes_rx.width||height!=boxes_rx.height)) {fprintf(stderr,"FAIL: box channel failure/geometry mismatch\n");goto out;}
    if(width!=ntohl(frame.width)||height!=ntohl(frame.height)||format!=ntohl(frame.format)) {fprintf(stderr,"FAIL: geometry change\n");goto out;}
    __sync_synchronize();
    const uint8_t *frame_pixels=buffer+(size_t)(done%VIDEO_WINDOW)*remote_stride;
    uint64_t prof_crc_begin=video_ns();
    uint64_t received_ns=video_ns();
    int preview_due=!pressure_preview_ms || !last_preview_ns ||
      received_ns-last_preview_ns>=(uint64_t)pressure_preview_ms*1000000ULL;
    if(crc_mode==VIDEO_CRC_FULL || (pressure_preview_ms && preview_due)) {
      uint32_t crc=(uint32_t)video_crc32(frame_pixels,payload_size);
      crc_checked++;
      if(crc!=ntohl(frame.crc)) {
        crc_errors++;fprintf(stderr,"FAIL: video CRC frame=%" PRIu64 " got=%08x expected=%08x\n",be64toh(frame.frame_id),crc,ntohl(frame.crc));
        (void)video_ack_send(peer_fd,be64toh(frame.frame_id),1);goto out;
      }
    }
    prof_crc_ns+=video_ns()-prof_crc_begin;
    // Copy before ownership ACK: the sender may overwrite the registered MR
    // after ACK, but the display reads separate seqlock-protected host slots.
    struct video_boxes boxes={0};
    if(ntohl(frame.reserved)==VIDEO_BOX_BYTES)memcpy(&boxes,frame_pixels+ntohl(frame.bytes),sizeof(boxes));
    if(!video_boxes_valid(&boxes,width,height)){fprintf(stderr,"FAIL: invalid YOLO metadata\n");(void)video_ack_send(peer_fd,be64toh(frame.frame_id),1);goto out;}
    uint64_t prof_display_begin=video_ns();
    if(preview_due && display_queue.h && vdq_publish(&display_queue,frame_pixels,be64toh(frame.frame_id),
       be64toh(frame.capture_ns),&boxes,NULL,1000)) {
      fprintf(stderr,"FAIL: ordered display queue full/invalid: %s; no unread frame overwritten\n",strerror(errno));goto out;
    }
    uint64_t prof_preview_begin=video_ns();
    prof_display_ns+=prof_preview_begin-prof_display_begin;
    if(preview_due) {
      video_shm_publish_boxes(&output,frame_pixels,be64toh(frame.frame_id),be64toh(frame.capture_ns),&boxes);
      last_preview_ns=received_ns;
    }
    uint64_t prof_credit_begin=video_ns();
    prof_preview_ns+=prof_credit_begin-prof_preview_begin;
    last_pub=be64toh(frame.publish_seq);last_frame_id=be64toh(frame.frame_id);done++;
    if(direct?video_credit_send(&credit,done,video_stop?2:0):video_ack_send(peer_fd,last_frame_id,video_stop?2:0))goto out;
    prof_credit_ns+=video_ns()-prof_credit_begin;++prof_samples;
    if(done==1||video_ns()-report_ns>=1000000000ULL) {
      double sec=(video_ns()-start_ns)/1e9;
      double interval=(video_ns()-report_ns)/1e9;
      fprintf(stderr,"VIDEO crc_mode=%s crc_checked=%" PRIu64 " received=%u seq=%" PRIu64 " crc_errors=%" PRIu64 " geometry=%ux%u bytes=%u fps=%.2f avg=%.2f boxes_channel=%d notification=%s payload_MiB_s=%.3f mailbox=%s boxes_valid=%u boxes_count=%u boxes_seq=%" PRIu64 " boxes_publish=%" PRIu64 "\n",video_crc_mode_name(crc_mode),crc_checked,done,last_frame_id,crc_errors,width,height,payload_size,(done-report_done)/interval,done/sec,boxes_channel,direct?"rdma":"tcp",(double)done*payload_size/1048576/sec,shm_name,boxes.valid,boxes.count,boxes.frame_id,boxes.publish_seq);
      fprintf(stderr,"RX_STAGES samples=%u receive_wait_ms=%.3f crc_ms=%.3f display_publish_ms=%.3f preview_publish_ms=%.3f credit_ms=%.3f\n",
              prof_samples,prof_wait_ns/1e6/prof_samples,prof_crc_ns/1e6/prof_samples,
              prof_display_ns/1e6/prof_samples,prof_preview_ns/1e6/prof_samples,prof_credit_ns/1e6/prof_samples);
      prof_wait_ns=prof_crc_ns=prof_display_ns=prof_preview_ns=prof_credit_ns=0;prof_samples=0;
      report_ns=video_ns();report_done=done;
    }
    // Continue acknowledging already submitted slots until END.
  }
  result=0;

out:
  vdq_finish(&display_queue,2000);
  box_receiver_close(&boxes_rx);
  if(save_last && done && output.h) {
    FILE *f=fopen(save_last,"wb");
    if(!f) {perror("save-last");result=1;}
    else {
      fprintf(f,"%s\n%u %u\n255\n",format==CSHM_BAYER?"P5":"P6",width,height);
      uint32_t slot=atomic_load(&output.h->active_index);
      uint32_t image_bytes=output.h->frame_bytes;
      const uint8_t *pixels=output.data+(size_t)slot*image_bytes;
      if(format==CSHM_RGB || format==CSHM_BAYER) {if(fwrite(pixels,1,image_bytes,f)!=image_bytes)result=1;}
      else for(uint32_t i=0;i<image_bytes;i+=3) {uint8_t rgb[3]={pixels[i+2],pixels[i+1],pixels[i]};if(fwrite(rgb,1,3,f)!=3){result=1;break;}}
      if(fclose(f))result=1;
    }
  }
  video_shm_close(&output);
  if(peer_fd >= 0) close(peer_fd);
  if(listen_fd >= 0) close(listen_fd);
  if(qp != NULL) ibv_destroy_qp(qp);
  video_credit_close(&credit);
  if(mr != NULL) ibv_dereg_mr(mr);
  if(cq != NULL) ibv_destroy_cq(cq);
  if(pd != NULL) ibv_dealloc_pd(pd);
  if(context != NULL) ibv_close_device(context);
  free(buffer);
  if(!result) {
  if(crc_mode==VIDEO_CRC_TRANSPORT)
    fprintf(stderr,"PASS: received=%u seq=%" PRIu64 " crc_mode=transport crc_checked=0 boxes_received=%" PRIu64 "\n",done,last_frame_id,atomic_load(&boxes_rx.count));
  else
  fprintf(stderr,"PASS: received and CRC-verified %u video frames; last_seq=%" PRIu64 " crc_errors=%" PRIu64 " boxes_received=%" PRIu64 "\n",done,last_frame_id,crc_errors,atomic_load(&boxes_rx.count));
  }
  return result;
}
