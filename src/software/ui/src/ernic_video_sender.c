#include "reconic.h"
#include "rdma_api.h"
#include "video_common.h"
#include "video_source_park.h"
#include "video_crc.h"
#include "video_ddr.h"

#include <getopt.h>
#include <signal.h>
#include <netinet/tcp.h>
#include <time.h>

#define RN_INTEROP_NUM_QP 8U
#define RN_INTEROP_QPID 3U
#define RN_INTEROP_QDEPTH 64U
#define RN_INTEROP_PKEY UINT32_C(0x0000ffff)
#define RN_INTEROP_LOCAL_RKEY UINT32_C(0x00000008)
#define RN_INTEROP_MAX_PAYLOAD (256U * 1024U)

/* Bound both retry occupancy and the peer's receive burst. The measured
 * Mellanox port has 262016 bytes of shared RX buffering; a 3 x 256 KiB
 * burst overflowed it (126 physical RX discards). This CMAC wrapper does
 * not connect stat_rx_pause_req to TX admission, so do not rely on PAUSE.
 * Camera video and pattern pressure slide four 56 KiB WQEs through the
 * same 224 KiB in-flight cap, waiting for strict CQ/DB/WRID completion
 * before releasing each frame. Leave headroom for headers and boxes QP.
 * Pixels still come directly from FPGA DDR, never via a host copy. */
#define VIDEO_DATA_BUFFER_COUNT 256U
#define VIDEO_DATA_BUFFER_BYTES 4096U
#define VIDEO_DDR_CHUNK (224U * 1024U)
#define VIDEO_DDR_BATCH_WQES 1U
/* Refill the safe 224 KiB peer RX window as individual WQEs complete. */
#define VIDEO_PRESSURE_CHUNK (56U * 1024U)
#define VIDEO_PRESSURE_WINDOW_WQES 4U
_Static_assert(VIDEO_PRESSURE_CHUNK * VIDEO_PRESSURE_WINDOW_WQES <= VIDEO_DDR_CHUNK,
               "pressure in-flight window exceeds known-safe burst");
_Static_assert(VIDEO_DDR_CHUNK % VIDEO_DATA_BUFFER_BYTES == 0, "DDR chunk/PMTU alignment");
_Static_assert(VIDEO_DDR_BATCH_WQES * VIDEO_DDR_CHUNK <= 224U * 1024U,
               "DDR burst exceeds conservative peer RX headroom");
_Static_assert(VIDEO_DDR_BATCH_WQES * (VIDEO_DDR_CHUNK / VIDEO_DATA_BUFFER_BYTES)
               < VIDEO_DATA_BUFFER_COUNT, "video burst exceeds retry ring");

#include "video_notify.h"
#include "video_credit_sender.h"
#include "video_box_sender.h"

static volatile sig_atomic_t video_stop;
static void stop_video(int signo) {(void)signo;video_stop=1;}
static void pressure_window_hold(struct rdma_dev_t *dev,struct ddr_ring *ddr,
                                 uint32_t qpid,int sockfd,const char *stage) {
  int saved=errno;
  fprintf(stderr,"FAIL: sliding RDMA %s: %s\n",stage,strerror(saved));
  atomic_store(&ddr->sender_state,DDR_SENDER_FAULT);
  dump_registers(dev,1,qpid);
  sigset_t hold;sigemptyset(&hold);sigaddset(&hold,SIGINT);
  sigaddset(&hold,SIGTERM);sigaddset(&hold,SIGHUP);
  sigprocmask(SIG_BLOCK,&hold,NULL);
  close(sockfd);
  fprintf(stderr,"FAIL: DMA outstanding; safety hold PID %ld. Reset FPGA before killing; keep driver/pool alive.\n",(long)getpid());
  for(;;)pause();
}
static void usage(const char *p) {
 fprintf(stderr,"Usage: %s --resource2 PATH [options]\n"
 "  --ddr-ring PATH     Read pixels directly from the FPGA DDR ownership ring\n"
 "  --boxes-file PATH   Alternative detector mailbox file (validation)\n"
 "  --boxes-shm NAME    Optional session-specific YOLO YBOX mailbox\n"
 "  --shm NAME          Full-resolution camera CSHM v3 (default /fpga_video_input)\n"
 "  --frames N          Frames to send, 0=until stopped (default 0)\n"
 "  --dma-dev PATH      Coherent DMA device (default /dev/edev0)\n"
 "  --netdev NAME       ERNIC interface (default enP5p1s0)\n"
 "  --src-ip ADDR       ERNIC IP (default 192.168.1.10)\n"
 "  --dst-ip ADDR       Mellanox IP (default 192.168.1.20)\n"
 "  --dst-mac MAC       Mellanox MAC (required)\n"
 "  --control-ip ADDR   Optional metadata TCP destination; pixels remain RDMA\n"
 "  --tcp-port N        Control port (default 11112)\n"
 "  --idle-timeout N    Camera/SHM wait seconds (default 10)\n"
 "  --debug             Library debug logging\n",p);
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

int main(int argc, char** argv) {
  static const struct option options[] = {
    {"boxes-file", required_argument, NULL, 'F'},
    {"ddr-ring", required_argument, NULL, 'D'},
    {"control-ip", required_argument, NULL, 'C'},
    {"resource2", required_argument, NULL, 'p'},
    {"dma-dev", required_argument, NULL, 'd'},
    {"dma-check", no_argument, NULL, 'k'},
    {"src-ip", required_argument, NULL, 'r'},
    {"dst-ip", required_argument, NULL, 'i'},
    {"dst-mac", required_argument, NULL, 'm'},
    {"tcp-port", required_argument, NULL, 't'},
    {"udp-src-port", required_argument, NULL, 'u'},
    {"shm", required_argument, NULL, 'f'},
    {"boxes-shm", required_argument, NULL, 'B'},
    {"idle-timeout", required_argument, NULL, 'I'},
    {"frames", required_argument, NULL, 'c'},
    {"netdev", required_argument, NULL, 'e'},
    {"qpn", required_argument, NULL, 'q'},
    {"psn", required_argument, NULL, 'n'},
    {"debug", no_argument, NULL, 'g'},
    {"help", no_argument, NULL, 'h'},
    {NULL, 0, NULL, 0}
  };
  const char* resource2 = NULL;
  int crc_mode=video_crc_mode_env();
  int pressure_pattern=getenv("VIDEO_PRESSURE_PATTERN") &&
                       !strcmp(getenv("VIDEO_PRESSURE_PATTERN"),"1");
  if(crc_mode<0){fprintf(stderr,"FAIL: invalid VIDEO_CRC_MODE\n");return 2;}
  const char *ddr_path=NULL;
  struct ddr_ring *ddr=NULL;
  struct ddr_frame *ddr_current=NULL;
  int ddr_fd=-1;
  struct box_sender box_tx={.fd=-1};
  pthread_mutex_t rdma_submit_lock=PTHREAD_MUTEX_INITIALIZER;
  struct rn_mlx5_wire_conn box_remote={0};
  uint32_t ddr_index=0;
  const char *control_ip=NULL;
  struct in_addr control_addr;
  const char* dma_device = "/dev/edev0";
  int dma_check = 0;
  sigset_t dma_signals;
  sigset_t old_signals;
  const char* src_ip_text = "192.168.1.10";
  const char* dst_ip_text = "192.168.1.20";
  const char* dst_mac_text = NULL;
  const char* netdev = "enP5p1s0";
  uint32_t count = 0;
  const char *shm_name="/fpga_video_input",*boxes_name=NULL;
  char boxes_path[256]={0};struct video_box_reader boxes_reader={0};
  uint32_t image_size=0;
  uint32_t idle_timeout=10;
  struct video_shm input={.fd=-1};
  struct video_header frame;
  uint8_t *snapshot=NULL;
  uint64_t last_pub=0, skipped=0, total_writes=0;
  double copy_seconds=0, rdma_seconds=0, control_seconds=0, crc_seconds=0;
  int lockfd=-1;
  struct sigaction action;memset(&action,0,sizeof(action));action.sa_handler=stop_video;
  sigaction(SIGINT,&action,NULL);sigaction(SIGTERM,&action,NULL);sigaction(SIGHUP,&action,NULL);
  uint32_t done = 0, submitted=0, pending=0;
  uint64_t pending_ids[VIDEO_WINDOW]={0};
  int nodelay = 1;
  struct timespec started, finished;
  uint32_t tcp_port = 11112;
  uint32_t udp_src_port = 49152;
  uint32_t payload_size = 262144;
  uint32_t qpid = RN_INTEROP_QPID;
  uint32_t local_psn = 0xabd;
  struct in_addr src_in_addr;
  struct in_addr dst_in_addr;
  struct sockaddr_in local_addr;
  struct sockaddr_in server_addr;
  struct timeval socket_timeout = {.tv_sec = 30, .tv_usec = 0};
  struct rn_mlx5_wire_conn local_conn;
  struct rn_mlx5_wire_conn remote_conn;
  uint8_t local_gid[16];
  
  uint8_t validation_status = 1;
  struct mac_addr_t src_mac;
  struct mac_addr_t dst_mac;
  struct rn_dev_t* rn_dev;
  struct rdma_dev_t* rdma_dev;
  struct rdma_pd_t* rdma_pd;
  struct rdma_buff_t* cidb_buffer;
  struct rdma_buff_t* data_buf;
  struct rdma_buff_t* ipkterr_buf;
  struct rdma_buff_t* err_buf;
  struct rdma_buff_t* resp_err_buf;
  struct rdma_buff_t* payload_buf;
  struct rdma_buff_t *credit_buf=NULL,*meta_buf=NULL;
  uint32_t remote_stride=0;
  uint64_t cq_cidb_addr;
  uint64_t rq_cidb_addr;
  uint32_t remote_qpn;
  uint32_t remote_psn;
  uint32_t remote_rkey;
  uint64_t remote_addr;
  int resource2_fd = -1;
  int sockfd;
  int option;
  int rc;

  while((option = getopt_long(argc, argv, "C:p:d:kr:i:m:t:u:f:B:I:q:n:c:e:gh",
                              options, NULL)) != -1) {
    switch(option) {
      case 'C': control_ip=optarg; break;
      case 'p': resource2 = optarg; break;
      case 'd': dma_device = optarg; break;
      case 'k': dma_check = 1; break;
      case 'r': src_ip_text = optarg; break;
      case 'i': dst_ip_text = optarg; break;
      case 'm': dst_mac_text = optarg; break;
      case 'e': netdev = optarg; break;
      case 'c': if(parse_u32(optarg, &count)) return 2; break;
      case 'f': shm_name=optarg; break;
      case 'B': boxes_name=optarg;break;
      case 'F': if(strlen(optarg)>=sizeof(boxes_path))return 2;snprintf(boxes_path,sizeof(boxes_path),"%s",optarg);break;
      case 'I': if(parse_u32(optarg,&idle_timeout)||!idle_timeout) return 2; break;
      case 't':
        if(parse_u32(optarg, &tcp_port) != 0) { usage(argv[0]); return 2; }
        break;
      case 'u':
        if(parse_u32(optarg, &udp_src_port) != 0) { usage(argv[0]); return 2; }
        break;

      case 'q':
        if(parse_u32(optarg, &qpid) != 0) { usage(argv[0]); return 2; }
        break;
      case 'n':
        if(parse_u32(optarg, &local_psn) != 0) { usage(argv[0]); return 2; }
        break;
      case 'D': ddr_path=optarg; break;
      case 'g': debug = 1; break;
      case 'h': usage(argv[0]); return 0;
      default: usage(argv[0]); return 2;
    }
  }

  if((control_ip && inet_pton(AF_INET,control_ip,&control_addr)!=1) || (resource2 == NULL) || (!dma_check && dst_mac_text == NULL) ||
     !video_shm_name(shm_name) || (tcp_port == 0) || (tcp_port > UINT16_MAX-(ddr_path?1U:0U)) ||
     (udp_src_port == 0) || (udp_src_port > UINT16_MAX) ||
     (payload_size == 0) || (payload_size > RN_INTEROP_MAX_PAYLOAD) ||
     (qpid == 0) || (qpid >= RN_INTEROP_NUM_QP) || (ddr_path && qpid==BOX_CHANNEL_QPID) ||
     ((local_psn & UINT32_C(0xff000000)) != 0) ||
     (inet_pton(AF_INET, src_ip_text, &src_in_addr) != 1) ||
     (inet_pton(AF_INET, dst_ip_text, &dst_in_addr) != 1)) {
    usage(argv[0]);
    return 2;
  }

  lockfd=open("/run/lock/ernic_bandwidth_client.lock",O_CREAT|O_RDWR|O_NOFOLLOW,0600);
  if(lockfd<0 || flock(lockfd,LOCK_EX|LOCK_NB)) {perror("another RDMA sender owns device lock");return 1;}

  if(dma_check) {
    struct rn_dma_pool pool;
    if(rn_dma_pool_open(&pool, dma_device, resource2, 2U * 1024U * 1024U) != 0) {
      fprintf(stderr, "FAIL: DMA allocation/mmap on %s: %s\n",
              dma_device, strerror(errno));
      return 1;
    }
    fprintf(stderr, "PASS: coherent DMA allocation/mmap: device=%s CPU VA=%p "
            "DMA/IOVA=0x%016" PRIx64 " size=%zu; no WQE submitted\n",
            dma_device, pool.cpu, pool.dma_addr, pool.size);
    rn_dma_pool_close(&pool);
    return 0;
  }

  if(crc_mode==VIDEO_CRC_TRANSPORT && !ddr_path) {
    fprintf(stderr,"FAIL: transport CRC mode requires direct DDR\n");return 2;
  }
  uint64_t wait_begin=video_ns();
  if(ddr_path) {
    while(!(ddr=ddr_map(ddr_path,0,&ddr_fd))) {
      if(video_stop || video_ns()-wait_begin>(uint64_t)idle_timeout*1000000000ULL) {
        fprintf(stderr,"FAIL: DDR descriptor ring unavailable: %s\n",ddr_path);return 1;
      }
      usleep(1000);
    }
    if(!ddr_valid(ddr) || ddr->crc_mode!=(uint32_t)crc_mode || atomic_load(&ddr->owner_stop) ||
       atomic_load(&ddr->sender_state)!=DDR_SENDER_WAIT) {
      fprintf(stderr,"FAIL: DDR session invalid or already owned\n");return 1;
    }
    ddr_index=ddr->start_idx;image_size=payload_size=ddr->bytes;
    fprintf(stderr,"Camera input: %ux%u bytes=%u source=FPGA_DDR direct; no host pixel bounce\n",
            ddr->width,ddr->height,image_size);
  } else {
    while(video_shm_read_open(&input,shm_name)) {
      if(video_stop || video_ns()-wait_begin>(uint64_t)idle_timeout*1000000000ULL) {
        fprintf(stderr,"FAIL: camera SHM %s unavailable/invalid\n",shm_name);return 1;
      }
      usleep(10000);
    }
    image_size=input.h->frame_bytes;payload_size=image_size+VIDEO_BOX_BYTES;
    if(payload_size>VIDEO_MAX_BYTES){fprintf(stderr,"Image plus boxes exceeds RDMA limit\n");return 2;}
    snapshot=malloc(payload_size);
    if(!snapshot){perror("frame snapshot allocation");return 1;}
  }
  if(boxes_name && boxes_path[0]){fprintf(stderr,"Choose boxes-shm or boxes-file\n");return 2;}
  if(boxes_name) {
    if(!video_shm_name(boxes_name)||strlen(boxes_name)>200){fprintf(stderr,"Invalid boxes SHM name\n");return 2;}
    snprintf(boxes_path,sizeof(boxes_path),"/dev/shm%s",boxes_name);
  }
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if(sockfd < 0) {
    perror("socket");
    return 1;
  }
  if(!control_ip && setsockopt(sockfd, SOL_SOCKET, SO_BINDTODEVICE, netdev, strlen(netdev)+1)) {
    perror("bind Jetson netdev"); close(sockfd); return 1;
  }
  setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
  setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &socket_timeout,
             sizeof(socket_timeout));
  setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, &socket_timeout,
             sizeof(socket_timeout));

  memset(&local_addr, 0, sizeof(local_addr));
  local_addr.sin_family = AF_INET;
  local_addr.sin_addr.s_addr = control_ip ? htonl(INADDR_ANY) : src_in_addr.s_addr;
  if(bind(sockfd, (struct sockaddr*) &local_addr, sizeof(local_addr)) != 0) {
    perror("bind source IP");
    close(sockfd);
    return 1;
  }

  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons((uint16_t) tcp_port);
  server_addr.sin_addr = control_ip ? control_addr : dst_in_addr;
  if(connect(sockfd, (struct sockaddr*) &server_addr, sizeof(server_addr)) != 0) {
    perror("connect Mellanox metadata server");
    close(sockfd);
    return 1;
  }

  rn_mlx5_ipv4_gid(src_in_addr, local_gid);
  src_mac = get_mac_addr_from_str_ip(sockfd, (char*) src_ip_text);
  dst_mac = convert_mac_addr_str_to_uint((char*) dst_mac_text);

  rn_dev = create_rn_dev_dma((char*) resource2, &resource2_fd, 1,
                             RN_INTEROP_NUM_QP, dma_device);
  rdma_dev = create_rdma_dev(rn_dev);

  cidb_buffer = allocate_rdma_buffer(rn_dev, 4096, HOST_MEM);
  cq_cidb_addr = cidb_buffer->dma_addr;
  rq_cidb_addr = cidb_buffer->dma_addr + RN_INTEROP_NUM_QP*64U;
  data_buf = allocate_rdma_buffer(rn_dev, VIDEO_DATA_BUFFER_COUNT * VIDEO_DATA_BUFFER_BYTES, HOST_MEM);
  ipkterr_buf = allocate_rdma_buffer(rn_dev, 8192, HOST_MEM);
  err_buf = allocate_rdma_buffer(rn_dev, 256U * 256U, HOST_MEM);
  resp_err_buf = allocate_rdma_buffer(rn_dev, 16U * 4096U, HOST_MEM);

  /* Do not let Ctrl-C, terminal loss or a broken control socket free a DMA
   * pool that ERNIC may still be accessing. Unblock after normal teardown.
   * On failure keep the process and pool alive until hardware is reset. */
  sigemptyset(&dma_signals);
  sigaddset(&dma_signals, SIGPIPE);
  if(sigprocmask(SIG_BLOCK, &dma_signals, &old_signals) != 0) {
    perror("block signals before enabling DMA");
    destroy_rn_dev(rn_dev);
    close(resource2_fd);
    close(sockfd);
    return 1;
  }

  /* A previous process may have left sequence and completion state behind
   * even though ERNIC was disabled. Re-arm recovery and clear the selected
   * QP before installing any new DMA addresses. */
  if(rn_rdma_prepare_qp(rdma_dev, qpid) != 0 || (ddr && rn_rdma_prepare_qp(rdma_dev,BOX_CHANNEL_QPID)!=0)) {
    fprintf(stderr, "FAIL: could not prepare QP %u: %s\n",
            qpid, strerror(errno));
    free(cidb_buffer);
    free(data_buf);
    free(ipkterr_buf);
    free(err_buf);
    free(resp_err_buf);
    destroy_rn_dev(rn_dev);
    close(resource2_fd);
    close(sockfd);
    sigprocmask(SIG_SETMASK, &old_signals, NULL);
    return 1;
  }

  rdma_pd = allocate_rdma_pd(rdma_dev, 0);
  if(ddr) {
    credit_buf=allocate_rdma_buffer(rn_dev,4096,HOST_MEM);
    meta_buf=allocate_rdma_buffer(rn_dev,4096,HOST_MEM);
    memset(credit_buf->buffer,0,4096);memset(meta_buf->buffer,0,4096);
    rdma_register_memory_region(rdma_dev,rdma_pd,RN_INTEROP_LOCAL_RKEY,credit_buf);
  }
  video_init_conn(&local_conn, qpid, local_psn, ddr?RN_INTEROP_LOCAL_RKEY:0,
                    ddr?(uintptr_t)credit_buf->buffer:0,payload_size,local_gid);
  if(pressure_pattern && (!ddr || crc_mode!=VIDEO_CRC_TRANSPORT)) {
    fprintf(stderr,"FAIL: pattern pressure requires DDR and transport CRC mode\n");return 2;
  }
  video_set_conn_mode(&local_conn,ddr?(pressure_pattern?VIDEO_NOTIFY_MODE_PATTERN:VIDEO_NOTIFY_MODE):0);
  video_set_conn_crc(&local_conn,(uint32_t)crc_mode);
  fprintf(stderr,"INTEGRITY crc_mode=%s software_crc=%s backend=%s\n",video_crc_mode_name(crc_mode),
          crc_mode==VIDEO_CRC_FULL?"every_frame":"diagnostic_disabled",video_crc_backend());
  remote_stride=video_region_stride(payload_size,ddr!=NULL);
  local_conn.count_be = htonl(count);
  local_conn.batch_be = htonl(VIDEO_WINDOW);
  if((rn_mlx5_send_all(sockfd, &local_conn, sizeof(local_conn)) != 0) ||
     (rn_mlx5_recv_all(sockfd, &remote_conn, sizeof(remote_conn)) != 0) ||
     !video_check_conn(&remote_conn)) {
    fprintf(stderr, "Error: Mellanox connection metadata exchange failed\n");
    close(sockfd);
    return 1;
  }

  remote_qpn = ntohl(remote_conn.qpn_be);
  remote_psn = ntohl(remote_conn.psn_be);
  remote_rkey = ntohl(remote_conn.rkey_be);
  remote_addr = be64toh(remote_conn.addr_be);
  if(!video_crc_agreed(&remote_conn,crc_mode,ddr!=NULL) ||
     (video_conn_mode(&remote_conn)!=(ddr?(pressure_pattern?VIDEO_NOTIFY_MODE_PATTERN:VIDEO_NOTIFY_MODE):0)) ||
     (ntohl(remote_conn.payload_size_be) != payload_size) ||
     ntohl(remote_conn.count_be) != count || ntohl(remote_conn.batch_be) != VIDEO_WINDOW ||
     (remote_rkey == 0) || (remote_addr == 0) || remote_addr>UINT64_MAX-(uint64_t)remote_stride*VIDEO_WINDOW) {
    fprintf(stderr, "Error: invalid Mellanox memory-region metadata\n");
    close(sockfd);
    return 1;
  }

  fprintf(stderr,
          "Info: Mellanox QPN=0x%x PSN=0x%x rkey=0x%x addr=0x%016" PRIx64
          " size=%u\n",
          remote_qpn, remote_psn, remote_rkey, remote_addr, payload_size);

  if(ddr) {
    struct sockaddr_in address=server_addr;address.sin_port=htons(tcp_port+1);
    uint64_t deadline=video_ns()+15000000000ULL;
    for(;;) {
      box_tx.fd=socket(AF_INET,SOCK_STREAM,0);
      if(box_tx.fd<0)break;
      box_socket_options(box_tx.fd);
      if(!control_ip && setsockopt(box_tx.fd,SOL_SOCKET,SO_BINDTODEVICE,netdev,strlen(netdev)+1))break;
      if(bind(box_tx.fd,(struct sockaddr*)&local_addr,sizeof(local_addr)))break;
      if(!connect(box_tx.fd,(struct sockaddr*)&address,sizeof(address)))break;
      close(box_tx.fd);box_tx.fd=-1;
      if(video_stop||video_ns()>deadline)break;
      usleep(10000);
    }
    box_tx.pd=allocate_rdma_pd(rdma_dev,1);
    box_tx.credit=allocate_rdma_buffer(rn_dev,4096,HOST_MEM);
    memset(box_tx.credit->buffer,0,4096);
    rdma_register_memory_region(rdma_dev,box_tx.pd,0x108,box_tx.credit);
    struct box_hello hello={.magic=htonl(BOX_CHANNEL_MAGIC),.parent_qpn=htonl(remote_qpn),
      .width=htonl(ddr->width),.height=htonl(ddr->height)};
    video_init_conn(&hello.conn,BOX_CHANNEL_QPID,0x13579,0x108,(uintptr_t)box_tx.credit->buffer,VIDEO_BOX_BYTES,local_gid);
    video_set_conn_mode(&hello.conn,VIDEO_NOTIFY_MODE);
    hello.conn.batch_be=htonl(1);
    if(box_tx.fd<0 || rn_mlx5_send_all(box_tx.fd,&hello,sizeof(hello)) ||
       rn_mlx5_recv_all(box_tx.fd,&box_remote,sizeof(box_remote)) || !video_check_conn(&box_remote) || video_conn_mode(&box_remote)!=VIDEO_NOTIFY_MODE ||
       ntohl(box_remote.payload_size_be)!=VIDEO_BOX_BYTES || ntohl(box_remote.batch_be)!=1 ||
       !ntohl(box_remote.rkey_be)||!be64toh(box_remote.addr_be)||
       be64toh(box_remote.addr_be)>UINT64_MAX-VIDEO_BOX_BYTES) {
      fprintf(stderr,"FAIL: independent boxes QP handshake\n");atomic_store(&ddr->sender_state,DDR_SENDER_FAULT);
      if(box_tx.fd>=0)close(box_tx.fd);
      close(sockfd);return 1;
    }
  }

  open_rdma_dev(rdma_dev, src_mac, ntohl(src_in_addr.s_addr), udp_src_port,
                VIDEO_DATA_BUFFER_COUNT, VIDEO_DATA_BUFFER_BYTES, data_buf->dma_addr,
                8192, ipkterr_buf->dma_addr,
                256, 256, err_buf->dma_addr,
                (UINT32_C(4096) << 16) | 16U, resp_err_buf->dma_addr);

  allocate_rdma_qp(rdma_dev, qpid, remote_qpn, rdma_pd,
                   cq_cidb_addr+qpid*64U, rq_cidb_addr+qpid*64U, RN_INTEROP_QDEPTH,
                   HOST_MEM, &dst_mac, ntohl(dst_in_addr.s_addr),
                   RN_INTEROP_PKEY, RN_INTEROP_LOCAL_RKEY);
  config_last_rq_psn(rdma_dev, qpid, (remote_psn - 1U) & 0x00ffffffU);
  config_sq_psn(rdma_dev, qpid, local_psn);

  payload_buf = ddr ? NULL : allocate_rdma_buffer(rn_dev, VIDEO_CHUNK, HOST_MEM);
  if(ddr) {
    /* Each QP owns its own registered PD and completion/credit cache lines. */
    allocate_rdma_qp(rdma_dev,BOX_CHANNEL_QPID,ntohl(box_remote.qpn_be),box_tx.pd,
      cq_cidb_addr+BOX_CHANNEL_QPID*64U,rq_cidb_addr+BOX_CHANNEL_QPID*64U,RN_INTEROP_QDEPTH,HOST_MEM,&dst_mac,ntohl(dst_in_addr.s_addr),
      RN_INTEROP_PKEY,RN_INTEROP_LOCAL_RKEY);
    config_last_rq_psn(rdma_dev,BOX_CHANNEL_QPID,(ntohl(box_remote.psn_be)-1U)&0xffffffU);
    config_sq_psn(rdma_dev,BOX_CHANNEL_QPID,0x13579);
    box_tx.dev=rdma_dev;box_tx.ddr=ddr;box_tx.path=boxes_path;box_tx.dma_path=dma_device;
    box_tx.rdma_submit_lock=&rdma_submit_lock;
    box_tx.width=ddr->width;box_tx.height=ddr->height;
    box_tx.addr=be64toh(box_remote.addr_be);box_tx.rkey=ntohl(box_remote.rkey_be);
    box_tx.payload=allocate_rdma_buffer(rn_dev,4096,HOST_MEM);
    if(pthread_create(&box_tx.thread,NULL,box_send_worker,&box_tx)) {
      fprintf(stderr,"FAIL: boxes worker creation\n");atomic_store(&box_tx.failed,1);video_stop=1;
    } else box_tx.started=1;
    atomic_store_explicit(&ddr->sender_state,video_stop?DDR_SENDER_FAULT:DDR_SENDER_ACTIVE,memory_order_release);
  }
  if(ddr) fprintf(stderr, "RDMA_WINDOW mode=%s chunk=%u window_wqes=%u max_inflight_bytes=%u retry_slots=%u retry_slot_bytes=%u\n",
                  pressure_pattern?"sliding_pressure":"sliding_video",
                  VIDEO_PRESSURE_CHUNK,
                  VIDEO_PRESSURE_WINDOW_WQES,
                  VIDEO_PRESSURE_CHUNK*VIDEO_PRESSURE_WINDOW_WQES,
                  VIDEO_DATA_BUFFER_COUNT, VIDEO_DATA_BUFFER_BYTES);
  clock_gettime(CLOCK_MONOTONIC, &started);
  rc=0;validation_status=0;
  uint64_t last_frame_time=video_ns(), report_time=last_frame_time;
  uint32_t report_done=0,latency_count=0;
  uint64_t prof_credit_ns=0,prof_source_ns=0,prof_commit_ns=0,prof_wqes=0;
  double prof_last_rdma_seconds=0;
  uint64_t ready_to_rdma_ns=0,ready_to_notice_ns=0,latency_peak_ns=0,reference_wait_ns=0;
  while(!video_stop && !atomic_load(&box_tx.failed) && (!count || submitted<count)) {
    if(pending==VIDEO_WINDOW) {
      /* The native LRDMA engine is WRITE-only. Credits return on the already
       * negotiated control socket after the receiver copies and checks each
       * frame; pixel payload and ordered commit remain RDMA WRITEs. */
      uint64_t wait_t=video_ns();int a=video_ack_recv(sockfd,pending_ids[done%VIDEO_WINDOW]);
      uint64_t waited_ns=video_ns()-wait_t;
      control_seconds+=waited_ns/1e9;prof_credit_ns+=waited_ns;
      if(a<0){fprintf(stderr,"FAIL: frame credit/CRC ACK rejected\n");validation_status=1;break;}
      done++;pending--;if(a==2){video_stop=1;break;}
    }
    uint64_t t=video_ns();
    int got=0;
    if(ddr) {
      ddr_current=ddr_frame_at(ddr,ddr_index);
      if(atomic_load_explicit(&ddr_current->token,memory_order_acquire)==(uint64_t)ddr_index+1) {
        if(ddr_current->addr>UINT32_MAX || ddr_current->addr+payload_size>UINT64_C(0x100000000) ||
           ddr_current->bytes!=payload_size || ddr_current->width!=ddr->width ||
           ddr_current->height!=ddr->height || ddr_current->format!=ddr->format) {
          fprintf(stderr,"FAIL: invalid DDR source descriptor\n");validation_status=1;break;
        }
        memset(&frame,0,sizeof(frame));frame.magic=htonl(VIDEO_MAGIC);frame.version=htonl(VIDEO_VERSION);
        frame.type=htonl(VIDEO_FRAME);frame.format=htonl(ddr->format);frame.width=htonl(ddr->width);
        frame.height=htonl(ddr->height);frame.stride=htonl(ddr->bytes/ddr->height);frame.bytes=htonl(ddr->bytes);
        frame.frame_id=htobe64(ddr_current->frame_id);frame.capture_ns=htobe64(ddr_current->ready_ns);
        frame.publish_seq=htobe64((uint64_t)ddr_index+1);got=1;
      } else if(atomic_load_explicit(&ddr->owner_stop,memory_order_acquire)) {
        fprintf(stderr,"FAIL: DDR capture owner stopped\n");validation_status=1;break;
      }
    } else {
      got=video_shm_snapshot(&input,last_pub,snapshot,payload_size,&frame);
      copy_seconds+=(video_ns()-t)/1e9;
    }
    if(got<0) {fprintf(stderr,"FAIL: camera SHM geometry changed\n");validation_status=1;break;}
    if(!got) {
      if(video_ns()-last_frame_time>(uint64_t)idle_timeout*1000000000ULL) {
        if(ddr && video_source_park_requested(ddr_path)) {
          /* An intentional park keeps both sessions and all credit DMA alive.
           * This does not mask CQ failures or release any DDR ownership. */
          fprintf(stderr,"TX_IDLE waiting_source=1 source_parked=1 session_alive=1\n");
          last_frame_time=video_ns();
        } else {
          fprintf(stderr,"FAIL: camera stopped publishing fresh frames\n");validation_status=1;break;
        }
      }
      usleep(ddr?100:1000);prof_source_ns+=video_ns()-t;continue;
    }
    uint64_t pub=be64toh(frame.publish_seq);
    if(last_pub && pub>last_pub+1)skipped+=pub-last_pub-1;
    if(last_pub && pub<=last_pub) {fprintf(stderr,"FAIL: camera publisher restarted; reconnect required\n");validation_status=1;break;}
    if(video_ns()-be64toh(frame.capture_ns)>(uint64_t)idle_timeout*1000000000ULL) {fprintf(stderr,"FAIL: stale camera timestamp\n");validation_status=1;break;}
    last_pub=pub;last_frame_time=video_ns();
    if(!ddr) {
      struct video_boxes boxes;
      video_boxes_snapshot(&boxes_reader,boxes_path,video_ns(),be64toh(frame.frame_id),
                           be64toh(frame.capture_ns),input.h->width,input.h->height,&boxes);
      memcpy(snapshot+image_size,&boxes,VIDEO_BOX_BYTES);frame.reserved=htonl(VIDEO_BOX_BYTES);
      t=video_ns();frame.crc=htonl((uint32_t)video_crc32(snapshot,payload_size));crc_seconds+=(video_ns()-t)/1e9;
    }
    if(ddr) {
      struct rn_rdma_write_window window;
      pthread_mutex_lock(&rdma_submit_lock);
      uint64_t post_t=video_ns();
      if(rn_rdma_write_window_open(&window,rdma_dev,qpid,VIDEO_PRESSURE_WINDOW_WQES))
        pressure_window_hold(rdma_dev,ddr,qpid,sockfd,"open");
      for(uint32_t offset=0;offset<payload_size;) {
        if(window.pending==window.max_pending &&
           rn_rdma_write_window_wait_one(&window,5000U))
          pressure_window_hold(rdma_dev,ddr,qpid,sockfd,"reap");
        uint32_t length=payload_size-offset;
        if(length>VIDEO_PRESSURE_CHUNK)length=VIDEO_PRESSURE_CHUNK;
        uint32_t slot=(uint32_t)rdma_dev->qps_ptr[qpid]->sq_pidb % RN_INTEROP_QDEPTH;
        create_a_wqe(rdma_dev,qpid,(uint16_t)(total_writes+1U),slot,
                     DDR_DEVICE_BASE | (ddr_current->addr+offset),length,RNIC_OP_WRITE,
                     remote_addr+(uint64_t)(submitted%VIDEO_WINDOW)*remote_stride+offset,
                     remote_rkey,0,0,0,0,0);
        if(rn_rdma_write_window_post(&window))
          pressure_window_hold(rdma_dev,ddr,qpid,sockfd,"post");
        offset+=length;total_writes++;prof_wqes++;
      }
      if(rn_rdma_write_window_drain(&window,5000U))
        pressure_window_hold(rdma_dev,ddr,qpid,sockfd,"drain");
      pthread_mutex_unlock(&rdma_submit_lock);
      rdma_seconds+=(video_ns()-post_t)/1e9;
    } else {
    for(uint32_t offset=0;offset<payload_size;) {
      uint32_t batch=0,slot=(uint32_t)rdma_dev->qps_ptr[qpid]->sq_pidb % RN_INTEROP_QDEPTH;
      uint64_t post_t=video_ns();
      do {
        uint32_t length=payload_size-offset;
        uint32_t limit=ddr?VIDEO_DDR_CHUNK:VIDEO_CHUNK;
        if(length>limit)length=limit;
        uint64_t local_addr;
        if(ddr) local_addr=DDR_DEVICE_BASE | (ddr_current->addr+offset);
        else {
          t=video_ns();memcpy(payload_buf->buffer,snapshot+offset,length);copy_seconds+=(video_ns()-t)/1e9;
          local_addr=payload_buf->dma_addr;
        }
        create_a_wqe(rdma_dev,qpid,(uint16_t)(total_writes+batch+1U),(slot+batch)%RN_INTEROP_QDEPTH,
                     local_addr,length,RNIC_OP_WRITE,
                     remote_addr+(uint64_t)(submitted%VIDEO_WINDOW)*remote_stride+offset,
                     remote_rkey,0,0,0,0,0);
        offset+=length;batch++;
      } while(ddr && batch<VIDEO_DDR_BATCH_WQES && offset<payload_size);
      rc=rn_rdma_post_write_batch_checked(rdma_dev,qpid,batch,5000U);
      rdma_seconds+=(video_ns()-post_t)/1e9;
      if(rc) {
        /* Diagnostics only: retain every descriptor and coherent buffer. */
        struct rdma_qp_t *failed_qp=rdma_dev->qps_ptr[qpid];
        rn_dma_rmb();
        for(uint32_t i=0;i<batch;i++) {
          uint32_t index=(slot+i)%RN_INTEROP_QDEPTH;
          const struct rdma_wqe_t *w=((const struct rdma_wqe_t*)failed_qp->sq->buffer)+index;
          uint32_t value=((volatile uint32_t*)failed_qp->cq->buffer)[index];
          fprintf(stderr,"BATCH_FAULT slot=%u posted=%u wrid=%u cqe=0x%08x local=0x%08x%08x remote=0x%08x%08x bytes=%u\n",
                  index,batch,w->wrid,value,w->laddr_high,w->laddr_low,
                  w->remote_offset_high,w->remote_offset_low,w->length);
        }
        if(ddr)atomic_store(&ddr->sender_state,DDR_SENDER_FAULT);
        dump_registers(rdma_dev,1,qpid);
        sigset_t hold;sigemptyset(&hold);sigaddset(&hold,SIGINT);sigaddset(&hold,SIGTERM);sigaddset(&hold,SIGHUP);
        sigprocmask(SIG_BLOCK,&hold,NULL);
        close(sockfd);
        fprintf(stderr,"FAIL: DMA outstanding; safety hold PID %ld. Reset FPGA before killing; keep driver/pool alive.\n",(long)getpid());
        for(;;)pause();
      }
      total_writes+=batch;prof_wqes+=batch;
    }
    }
    if(ddr) {
      uint64_t rdma_at=video_ns();
      ready_to_rdma_ns+=rdma_at-ddr_current->ready_ns;
      /* Full CRC waits for the independent ISP->host reference. Transport
       * mode releases the DDR slot after RDMA alone; local preview does not
       * hold it. dma_done is the reference-ready token in direct mode. */
      uint64_t deadline=video_ns()+5000000000ULL;
      while(crc_mode==VIDEO_CRC_FULL &&
            atomic_load_explicit(&ddr_current->dma_done,memory_order_acquire)!=(uint64_t)ddr_index+1) {
        if(atomic_load(&ddr->owner_stop)||video_ns()>deadline) {
          fprintf(stderr,"FAIL: host-direct CRC reference did not complete; DDR slot retained\n");
          validation_status=1;break;
        }
        usleep(50);
      }
      if(validation_status)break;
      uint64_t notice_at=video_ns(),age=notice_at-ddr_current->ready_ns;
      ready_to_notice_ns+=age;reference_wait_ns+=notice_at-rdma_at;
      if(age>latency_peak_ns)latency_peak_ns=age;
      ++latency_count;
      frame.crc=(crc_mode==VIDEO_CRC_FULL || pressure_pattern)?htonl(ddr_current->crc):0;
      atomic_store_explicit(&ddr_current->rdma_done,(uint64_t)ddr_index+1,memory_order_release);
      ddr_index++;
    }
    // Direct DDR mode publishes metadata and an ordered commit via RDMA;
    // receiver credits return over the control socket after ownership release.
    t=video_ns();
    if(ddr) {
      memcpy(meta_buf->buffer,&frame,sizeof(frame));
      uint64_t serial=(uint64_t)submitted+1;memcpy((uint8_t*)meta_buf->buffer+64,&serial,8);
      pthread_mutex_lock(&rdma_submit_lock);
      uint32_t slot=rdma_dev->qps_ptr[qpid]->sq_pidb%RN_INTEROP_QDEPTH;
      uint64_t dest=remote_addr+(uint64_t)(submitted%VIDEO_WINDOW)*remote_stride+video_meta_offset(payload_size);
      create_a_wqe(rdma_dev,qpid,(uint16_t)(total_writes+1),slot,meta_buf->dma_addr,64,RNIC_OP_WRITE,dest,remote_rkey,0,0,0,0,0);
      create_a_wqe(rdma_dev,qpid,(uint16_t)(total_writes+2),(slot+1)%RN_INTEROP_QDEPTH,meta_buf->dma_addr+64,8,RNIC_OP_WRITE,dest+64,remote_rkey,0,0,0,0,0);
      if(rn_rdma_post_write_batch_checked(rdma_dev,qpid,2,5000)) {
        atomic_store(&ddr->sender_state,DDR_SENDER_FAULT);
        fprintf(stderr,"FAIL: RDMA commit DMA outstanding; safety hold PID %ld. Reset FPGA before killing.\n",(long)getpid());
        for(;;)pause();
      }
      pthread_mutex_unlock(&rdma_submit_lock);
      total_writes+=2;
    } else if(rn_mlx5_send_all(sockfd,&frame,sizeof(frame))) {
      fprintf(stderr,"FAIL: frame descriptor send failed\n");validation_status=1;break;
    }
    uint64_t commit_ns=video_ns()-t;
    control_seconds+=commit_ns/1e9;prof_commit_ns+=commit_ns;
    pending_ids[submitted%VIDEO_WINDOW]=be64toh(frame.frame_id);
    submitted++;pending++;
    if(done==1 || video_ns()-report_time>=(pressure_pattern?250000000ULL:1000000000ULL)) {
      double secs=(video_ns()-((uint64_t)started.tv_sec*1000000000ULL+started.tv_nsec))/1e9;
      double interval=(video_ns()-report_time)/1e9;
      double fps=interval>0?(done-report_done)/interval:0;
      fprintf(stderr,"VIDEO frames=%u submitted=%u frame_seq=%" PRIu64 " bytes=%u source_skipped=%" PRIu64 " fps=%.2f avg=%.2f payload_MiB_s=%.3f rdma_active_s=%.3f copy_s=%.3f crc_s=%.3f control_s=%.3f source=%s\n",
              done,submitted,be64toh(frame.frame_id),payload_size,skipped,fps,done/secs,(double)done*payload_size/1048576/secs,rdma_seconds,copy_seconds,crc_seconds,control_seconds,ddr?"fpga_ddr":"host_shm");
      if(ddr && latency_count) {
        fprintf(stderr,"TX_STAGES samples=%u payload_rdma_ms=%.3f commit_rdma_ms=%.3f reference_wait_ms=%.3f credit_wait_ms=%.3f source_wait_ms=%.3f payload_wqes=%" PRIu64 "\n",
          latency_count,(rdma_seconds-prof_last_rdma_seconds)*1e3/latency_count,prof_commit_ns/1e6/latency_count,
          reference_wait_ns/1e6/latency_count,prof_credit_ns/1e6/latency_count,prof_source_ns/1e6/latency_count,prof_wqes);
        fprintf(stderr,"DDR_LATENCY samples=%u ready_to_rdma_ms=%.3f ready_to_notice_ms=%.3f peak_ms=%.3f reference_wait_ms=%.3f published=%" PRIu64 " released=%" PRIu64 "\n",
          latency_count,ready_to_rdma_ns/1e6/latency_count,ready_to_notice_ns/1e6/latency_count,
          latency_peak_ns/1e6,reference_wait_ns/1e6/latency_count,atomic_load(&ddr->published),atomic_load(&ddr->released));
        latency_count=0;ready_to_rdma_ns=ready_to_notice_ns=latency_peak_ns=reference_wait_ns=0;
      }
      prof_last_rdma_seconds=rdma_seconds;
      prof_credit_ns=prof_source_ns=prof_commit_ns=prof_wqes=0;
      report_done=done;report_time=video_ns();
    }
  }
  /* Stop/join box DMA before video END allows the receiver to destroy either MR.
   * A box DMA timeout holds this join indefinitely, preserving the coherent pool. */
  if(box_tx.started) {atomic_store(&box_tx.stop,1);pthread_join(box_tx.thread,NULL);}
  if(atomic_load(&box_tx.failed))validation_status=1;
  // Drain all per-slot ownership ACKs before END or normal teardown.
  while(!validation_status && pending) {
    uint64_t t=video_ns();int a=video_ack_recv(sockfd,pending_ids[done%VIDEO_WINDOW]);
    control_seconds+=(video_ns()-t)/1e9;
    if(a<0){validation_status=1;fprintf(stderr,"FAIL: final frame ACK rejected\n");break;}
    done++;pending--;if(a==2)video_stop=1;
  }
  // No DMA remains outstanding here. END permits receiver teardown; don't
  // send END when the receiver already requested stop and may have closed.
  if(!validation_status && !video_stop) {
    struct video_header end={0};end.magic=htonl(VIDEO_MAGIC);end.version=htonl(VIDEO_VERSION);end.type=htonl(VIDEO_END);
    if(rn_mlx5_send_all(sockfd,&end,sizeof(end)) || video_ack_recv(sockfd,0)<0)validation_status=1;
  } else if(!validation_status && video_stop) {
    struct video_header end={0};end.magic=htonl(VIDEO_MAGIC);end.version=htonl(VIDEO_VERSION);end.type=htonl(VIDEO_END);
    (void)rn_mlx5_send_all(sockfd,&end,sizeof(end));
  }
  if(ddr && validation_status && pending) {
    atomic_store(&ddr->sender_state,DDR_SENDER_FAULT);
    fprintf(stderr,"FAIL: unacknowledged DDR frame; safety hold PID %ld. Reset FPGA before killing.\n",(long)getpid());
    for(;;)pause();
  }
  clock_gettime(CLOCK_MONOTONIC,&finished);
  double seconds=finished.tv_sec-started.tv_sec+(finished.tv_nsec-started.tv_nsec)/1e9;
  fprintf(stderr,"%s: video frames=%u bytes_per_frame=%u source_skipped=%" PRIu64 " seconds=%.6f end_to_end_MiB_s=%.6f rdma_active_MiB_s=%.6f (active intervals exclude inter-chunk gaps; not wire line rate)\n",
          validation_status?"FAIL":"PASS",done,payload_size,skipped,seconds,
          seconds>0?(double)done*payload_size/1048576/seconds:0,
          rdma_seconds>0?(double)done*payload_size/1048576/rdma_seconds:0);
  if(ddr) {
    atomic_store_explicit(&ddr->sender_state,validation_status?DDR_SENDER_FAULT:DDR_SENDER_DRAINED,memory_order_release);
    munmap(ddr,sizeof(*ddr));close(ddr_fd);
  }
  video_boxes_close(&boxes_reader);free(snapshot);video_shm_close(&input);

  free(cidb_buffer);
  free(data_buf);
  free(ipkterr_buf);
  free(err_buf);
  free(resp_err_buf);
  free(payload_buf);
  free(credit_buf);free(meta_buf);
  free(box_tx.payload);free(box_tx.credit);
  if(box_tx.fd>=0)close(box_tx.fd);
  close(sockfd);
  if(rc == 0) {
    destroy_rn_dev(rn_dev);
  }
  close(resource2_fd);
  sigprocmask(SIG_SETMASK, &old_signals, NULL);
  close(lockfd);
  return (rc == 0 && validation_status == 0) ? 0 : 1;
}
