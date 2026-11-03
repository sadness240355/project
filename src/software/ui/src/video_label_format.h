#ifndef VIDEO_LABEL_FORMAT_H
#define VIDEO_LABEL_FORMAT_H
#include "video_label_names.h"
#define VIDEO_LABEL_CHARS 64
static inline void video_box_label(const struct video_box *b,char out[VIDEO_LABEL_CHARS]) {
 const char *name=video_class_name(b->class_id);char fallback[32],identity[24];
 if(!name){snprintf(fallback,sizeof(fallback),"class:%d",b->class_id);name=fallback;}
 if(b->track_id>=0)snprintf(identity,sizeof(identity),"%d",b->track_id);
 else snprintf(identity,sizeof(identity),"--");
 snprintf(out,VIDEO_LABEL_CHARS,"%s ID:%s %.2f",name,identity,b->confidence);
}
#endif
