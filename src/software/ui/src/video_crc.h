#ifndef VIDEO_CRC_H
#define VIDEO_CRC_H
#include <stddef.h>
#include <stdint.h>
const char *video_crc_backend(void);
uint32_t video_crc32(const void *data,size_t bytes);
#endif
