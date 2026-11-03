#ifndef VIDEO_NOTIFY_H
#define VIDEO_NOTIFY_H
#include "video_integrity.h"
#define VIDEO_NOTIFY_MODE 0x44445231U
#define VIDEO_NOTIFY_MODE_PATTERN 0x50544231U
#define VIDEO_CREDIT_MASK UINT64_C(0x00ffffffffffffff)
static inline uint32_t video_meta_offset(uint32_t bytes){return (bytes+63U)&~63U;}
static inline uint32_t video_region_stride(uint32_t bytes,int direct){return direct?video_meta_offset(bytes)+128U:bytes;}
static inline uint32_t video_conn_mode(const struct rn_mlx5_wire_conn *c) {
 uint32_t v;memcpy(&v,c->reserved,4);return ntohl(v);
}
static inline void video_set_conn_mode(struct rn_mlx5_wire_conn *c,uint32_t mode) {
 uint32_t v=htonl(mode);memcpy(c->reserved,&v,4);
}
/* reserved[0:4] is notification mode; [4:8] explicitly negotiates CRC.
 * A legacy response is zero/full, so transport cannot silently downgrade. */
static inline uint32_t video_conn_crc(const struct rn_mlx5_wire_conn *c) {
 uint32_t v;memcpy(&v,c->reserved+4,4);return ntohl(v);
}
static inline void video_set_conn_crc(struct rn_mlx5_wire_conn *c,uint32_t mode) {
 uint32_t v=htonl(mode);memcpy(c->reserved+4,&v,4);
}
static inline int video_crc_agreed(const struct rn_mlx5_wire_conn *c,int mode,int direct) {
 return mode>=0 && mode<=VIDEO_CRC_TRANSPORT && video_conn_crc(c)==(uint32_t)mode &&
        (mode==VIDEO_CRC_FULL || direct);
}
#endif
