#ifndef VIDEO_INTEGRITY_H
#define VIDEO_INTEGRITY_H
#include <stdlib.h>
#include <string.h>
enum { VIDEO_CRC_FULL=0, VIDEO_CRC_TRANSPORT=1 };
/* Full is backward compatible. Unknown modes fail before any transfer. */
static inline int video_crc_mode_env(void) {
 const char *s=getenv("VIDEO_CRC_MODE");
 if(!s || !*s || !strcmp(s,"full"))return VIDEO_CRC_FULL;
 if(!strcmp(s,"transport"))return VIDEO_CRC_TRANSPORT;
 return -1;
}
static inline const char *video_crc_mode_name(int mode) {
 return mode==VIDEO_CRC_TRANSPORT?"transport":"full";
}
#endif
