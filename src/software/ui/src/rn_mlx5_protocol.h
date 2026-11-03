#ifndef RN_MLX5_PROTOCOL_H
#define RN_MLX5_PROTOCOL_H

#include <arpa/inet.h>
#include <endian.h>
#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <sys/socket.h>

#define RN_MLX5_MAGIC UINT32_C(0x524e4d58)
#define RN_MLX5_VERSION UINT32_C(3)

struct rn_mlx5_wire_conn {
  uint64_t addr_be;
  uint32_t magic_be;
  uint32_t version_be;
  uint32_t qpn_be;
  uint32_t psn_be;
  uint32_t rkey_be;
  uint32_t payload_size_be;
  uint8_t gid[16];
  uint32_t count_be;
  uint32_t batch_be;
  uint8_t reserved[8];
};

_Static_assert(sizeof(struct rn_mlx5_wire_conn) == 64,
               "RDMA connection record must be 64 bytes");

static inline uint32_t rn_mlx5_slot_stride(uint32_t size) {
  return (size + 63U) & ~63U;
}

static inline void rn_mlx5_init_conn(struct rn_mlx5_wire_conn* conn,
                                     uint32_t qpn, uint32_t psn,
                                     uint32_t rkey, uint64_t addr,
                                     uint32_t payload_size,
                                     const uint8_t gid[16]) {
  memset(conn, 0, sizeof(*conn));
  conn->addr_be = htobe64(addr);
  conn->magic_be = htonl(RN_MLX5_MAGIC);
  conn->version_be = htonl(RN_MLX5_VERSION);
  conn->qpn_be = htonl(qpn);
  conn->psn_be = htonl(psn);
  conn->rkey_be = htonl(rkey);
  conn->payload_size_be = htonl(payload_size);
  if(gid != NULL) {
    memcpy(conn->gid, gid, sizeof(conn->gid));
  }
}

static inline int rn_mlx5_check_conn(const struct rn_mlx5_wire_conn* conn) {
  return (ntohl(conn->magic_be) == RN_MLX5_MAGIC) &&
         (ntohl(conn->version_be) == RN_MLX5_VERSION) &&
         ((ntohl(conn->qpn_be) & UINT32_C(0xff000000)) == 0) &&
         ((ntohl(conn->psn_be) & UINT32_C(0xff000000)) == 0);
}

static inline void rn_mlx5_ipv4_gid(struct in_addr addr, uint8_t gid[16]) {
  memset(gid, 0, 16);
  gid[10] = 0xff;
  gid[11] = 0xff;
  memcpy(&gid[12], &addr.s_addr, sizeof(addr.s_addr));
}

static inline int rn_mlx5_send_all(int fd, const void* buffer, size_t length) {
  const uint8_t* next = (const uint8_t*) buffer;

  while(length != 0) {
    ssize_t rc = send(fd, next, length, MSG_NOSIGNAL);
    if(rc > 0) {
      next += rc;
      length -= (size_t) rc;
    } else if((rc < 0) && (errno == EINTR)) {
      continue;
    } else {
      return -1;
    }
  }
  return 0;
}

static inline int rn_mlx5_recv_all(int fd, void* buffer, size_t length) {
  uint8_t* next = (uint8_t*) buffer;

  while(length != 0) {
    ssize_t rc = recv(fd, next, length, 0);
    if(rc > 0) {
      next += rc;
      length -= (size_t) rc;
    } else if((rc < 0) && (errno == EINTR)) {
      continue;
    } else {
      return -1;
    }
  }
  return 0;
}

/* Hash position and transfer sequence: catches swapped 256B/4KiB blocks
 * and stale data from an earlier write. Deterministic on both CPU widths. */
static inline uint8_t rn_mlx5_pattern_byte_seq(size_t index, uint32_t seq) {
  uint32_t x = (uint32_t)index ^ (seq * UINT32_C(0x9e3779b9));
  x ^= x >> 16; x *= UINT32_C(0x7feb352d);
  x ^= x >> 15; x *= UINT32_C(0x846ca68b); x ^= x >> 16;
  return (uint8_t)x;
}
static inline uint8_t rn_mlx5_pattern_byte(size_t index) {
  return rn_mlx5_pattern_byte_seq(index, 0);
}

#endif
