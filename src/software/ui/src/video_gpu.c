/* Native display only. No device access, DMA ownership changes or image filters.
 * An acknowledged FIFO decouples GPU uploads from producer slot reuse.
 * The renderer owns its X11/EGL context on one thread. Tk handles controls only. */
#define _GNU_SOURCE
#include "video_common.h"
#include "video_display.h"
#include <pthread.h>
#include <EGL/egl.h>
#include <GLES2/gl2.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/extensions/shape.h>
#include <math.h>

struct gpu {
 pthread_t thread; pthread_mutex_t lock;
 int stop,paused,cw,ch,probe,ready,show_boxes,boxes_valid,boxes_count,swap_interval;
 char boxes_path[256];uint32_t boxes_frame_age;double boxes_age_ms;
 double limit,zoom,panx,pany;
 char path[4096],error[256],renderer[128];
 unsigned long parent;
 uint64_t count,skipped,pub,seq,epoch,last_ns,queue_depth,queue_high_water,queue_waits;
 unsigned width,height,format;
 unsigned char rgb[3];
 double fps;
};
static void error_set(struct gpu *g,const char *s) {
 pthread_mutex_lock(&g->lock);snprintf(g->error,sizeof(g->error),"%s",s);pthread_mutex_unlock(&g->lock);
}
static GLuint shader(GLenum type,const char *code) {
 GLuint s=glCreateShader(type);glShaderSource(s,1,&code,NULL);glCompileShader(s);
 GLint ok;glGetShaderiv(s,GL_COMPILE_STATUS,&ok);
 if(!ok){char log[1024];glGetShaderInfoLog(s,sizeof(log),NULL,log);fprintf(stderr,"GPU shader: %s\n",log);glDeleteShader(s);return 0;}return s;
}
static const char *vs="attribute vec2 pos; varying vec2 uv; void main(){gl_Position=vec4(pos,0.,1.);uv=vec2((pos.x+1.)*.5,(1.-pos.y)*.5);}";
static const char *fs=
 "precision highp float;varying vec2 uv;uniform sampler2D tex;uniform vec2 sz;uniform int fmt;"
 "float s(vec2 p){p=abs(p);p=min(p,2.*(sz-1.)-p);return texture2D(tex,(p+.5)/sz).r;}"
 "void main(){if(fmt!=2){vec2 p=floor(uv*sz);vec2 t=vec2((p.x*3.+.5)/(sz.x*3.),(p.y+.5)/sz.y);"
 "float dx=1./(sz.x*3.);vec3 c=vec3(texture2D(tex,t).r,texture2D(tex,t+vec2(dx,0)).r,texture2D(tex,t+vec2(2.*dx,0)).r);"
 "gl_FragColor=vec4(fmt==1?c.bgr:c,1.);return;}"
 "vec2 p=floor(uv*sz);vec2 a=mod(p,2.);float c=s(p);"
 "float x=(s(p+vec2(-1,0))+s(p+vec2(1,0)))*.5;"
 "float y=(s(p+vec2(0,-1))+s(p+vec2(0,1)))*.5;"
 "float d=(s(p+vec2(-1,-1))+s(p+vec2(1,-1))+s(p+vec2(-1,1))+s(p+vec2(1,1)))*.25;"
 "vec3 rgb;if(a.y<.5){if(a.x<.5)rgb=vec3(c,(x+y)*.5,d);else rgb=vec3(x,c,y);}"
 "else{if(a.x<.5)rgb=vec3(y,c,x);else rgb=vec3(d,(x+y)*.5,c);}gl_FragColor=vec4(rgb,1.);}";
static void box_bar(int x,int y,int w,int h,int cw,int ch) {
 int x2=x+w,y2=y+h;if(x<0)x=0;if(y<0)y=0;if(x2>cw)x2=cw;if(y2>ch)y2=ch;
 if(x2>x&&y2>y){glScissor(x,ch-y2,x2-x,y2-y);glClear(GL_COLOR_BUFFER_BIT);}
}
#include "video_gpu_labels.h"
static int open_map(const char *path,struct vdq *s,struct stat *st) {
 char lock[4096];
 if(snprintf(lock,sizeof(lock),"%s.producer.lock",path)>=(int)sizeof(lock))return -1;
 if(vdq_open(s,path))return -1;
 int fd=open(lock,O_RDONLY|O_NOFOLLOW|O_CLOEXEC);
 int active=0;
 if(fd>=0){
  if(flock(fd,LOCK_EX|LOCK_NB)<0&&(errno==EWOULDBLOCK||errno==EAGAIN))active=1;
  else flock(fd,LOCK_UN);
  close(fd);
 }
 if(!active||fstat(s->fd,st)){vdq_unmap(s);errno=ESTALE;return -1;}return 0;
}
static void *render(void *arg) {
 struct gpu *g=arg;Display *x=NULL;EGLDisplay ed=EGL_NO_DISPLAY;EGLSurface surf=EGL_NO_SURFACE;
 EGLContext ctx=EGL_NO_CONTEXT;Window win=0;Colormap cmap=0;
 GLuint prog=0,tex=0,vbo=0,vert=0,frag=0;
 struct gpu_labels labels={0};
 struct video_box_reader boxes_reader={0};struct video_boxes boxes={0};
 uint64_t image_stamp=0,image_seq=0,image_arrived=0;int last_boxes_visible=0;
 struct vdq map={.fd=-1,.lock_fd=-1};struct stat identity={0};
 x=XOpenDisplay(NULL);if(!x){error_set(g,"XOpenDisplay failed");goto done;}
 ed=eglGetDisplay((EGLNativeDisplayType)x);
 if(ed==EGL_NO_DISPLAY||!eglInitialize(ed,NULL,NULL)||!eglBindAPI(EGL_OPENGL_ES_API)){error_set(g,"EGL initialization failed");goto done;}
 EGLint attrs[]={EGL_SURFACE_TYPE,EGL_WINDOW_BIT,EGL_RENDERABLE_TYPE,EGL_OPENGL_ES2_BIT,EGL_RED_SIZE,8,EGL_GREEN_SIZE,8,EGL_BLUE_SIZE,8,EGL_NONE};
 EGLConfig config;EGLint n,visual;
 if(!eglChooseConfig(ed,attrs,&config,1,&n)||!n||!eglGetConfigAttrib(ed,config,EGL_NATIVE_VISUAL_ID,&visual)){error_set(g,"EGL config unavailable");goto done;}
 XVisualInfo vi={.visualid=(VisualID)visual};int nvi;XVisualInfo *info=XGetVisualInfo(x,VisualIDMask,&vi,&nvi);
 if(!info){error_set(g,"X11 EGL visual unavailable");goto done;}
 cmap=XCreateColormap(x,g->parent,info->visual,AllocNone);
 XSetWindowAttributes wa={.colormap=cmap,.border_pixel=0,.background_pixel=0};
 win=XCreateWindow(x,g->parent,0,0,1,1,0,info->depth,InputOutput,info->visual,CWColormap|CWBorderPixel|CWBackPixel,&wa);XFree(info);
 /* Mouse input remains with the Tk canvas for pixel selection and panning. */
 int ev,er;if(XShapeQueryExtension(x,&ev,&er))XShapeCombineRectangles(x,win,ShapeInput,0,0,NULL,0,ShapeSet,Unsorted);
 surf=eglCreateWindowSurface(ed,config,(EGLNativeWindowType)win,NULL);
 EGLint ca[]={EGL_CONTEXT_CLIENT_VERSION,2,EGL_NONE};ctx=eglCreateContext(ed,config,EGL_NO_CONTEXT,ca);
 if(surf==EGL_NO_SURFACE||ctx==EGL_NO_CONTEXT||!eglMakeCurrent(ed,surf,surf,ctx)){error_set(g,"EGL window/context failed");goto done;}
 /* Opt-in sync: mixed-refresh X11 desktops can cap EGL to the slow screen. */
 const char *vsync=getenv("VIDEO_DISPLAY_VSYNC");
 g->swap_interval=vsync&&!strcmp(vsync,"1")?1:0;
 if(!eglSwapInterval(ed,g->swap_interval)){error_set(g,"EGL refresh synchronization failed");goto done;}
 vert=shader(GL_VERTEX_SHADER,vs);frag=shader(GL_FRAGMENT_SHADER,fs);
 if(!vert||!frag){error_set(g,"GPU shader compile failed");goto done;}
 prog=glCreateProgram();glAttachShader(prog,vert);glAttachShader(prog,frag);glBindAttribLocation(prog,0,"pos");glLinkProgram(prog);
 GLint ok;glGetProgramiv(prog,GL_LINK_STATUS,&ok);if(!ok){error_set(g,"GPU shader link failed");goto done;}
 if(gpu_labels_init(&labels)){error_set(g,"GPU label atlas initialization failed");goto done;}
 glUseProgram(prog);glGenTextures(1,&tex);glBindTexture(GL_TEXTURE_2D,tex);
 glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_NEAREST);glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
 glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP_TO_EDGE);glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP_TO_EDGE);
 glPixelStorei(GL_UNPACK_ALIGNMENT,1);glPixelStorei(GL_PACK_ALIGNMENT,1);
 const GLfloat quad[]={-1,-1,1,-1,-1,1,1,1};
 glGenBuffers(1,&vbo);glBindBuffer(GL_ARRAY_BUFFER,vbo);glBufferData(GL_ARRAY_BUFFER,sizeof(quad),quad,GL_STATIC_DRAW);
 glVertexAttribPointer(0,2,GL_FLOAT,GL_FALSE,0,0);glEnableVertexAttribArray(0);
 pthread_mutex_lock(&g->lock);g->ready=1;snprintf(g->renderer,sizeof(g->renderer),"%s",glGetString(GL_RENDERER));pthread_mutex_unlock(&g->lock);
 uint64_t last=0,check_at=0,draw_at=0,rate_at=video_ns(),rate_count=0,epoch=0;unsigned tw=0,th=0,tf=0;
 int oldw=0,oldh=0,mapped=0;double oldz=-1,oldx=-1,oldy=-1;
 for(;;) {
  int stop,pause,cw,ch,probe,show_boxes;char boxes_path[256];double limit,z,px,py;
  pthread_mutex_lock(&g->lock);stop=g->stop;show_boxes=g->show_boxes;snprintf(boxes_path,sizeof(boxes_path),"%s",g->boxes_path);pause=g->paused;cw=g->cw;ch=g->ch;probe=g->probe;limit=g->limit;z=g->zoom;px=g->panx;py=g->pany;pthread_mutex_unlock(&g->lock);
  if(stop)break;
  uint64_t now=video_ns();
  if(now-check_at>50000000ULL){
   struct stat st;check_at=now;
   if(stat(g->path,&st)||!map.h||st.st_ino!=identity.st_ino||st.st_dev!=identity.st_dev||st.st_size!=identity.st_size){
    {
    /* Display-only backlog belongs to the old session: switch immediately. */
    vdq_unmap(&map);last=0;
    if(!open_map(g->path,&map,&identity)){
     ++epoch;tw=th=tf=0;memset(&boxes,0,sizeof(boxes));image_arrived=0;
     pthread_mutex_lock(&g->lock);g->count=g->skipped=0;g->epoch=epoch;g->fps=0;g->last_ns=0;pthread_mutex_unlock(&g->lock);
     rate_at=now;rate_count=0;
    }
    }
   }
  }
  pthread_mutex_lock(&g->lock);uint64_t last_ns=g->last_ns;pthread_mutex_unlock(&g->lock);
  int visible=!pause&&map.h&&last_ns&&now-last_ns<2000000000ULL&&cw>1&&ch>1;
  if(!visible&&mapped){XUnmapWindow(x,win);XFlush(x);mapped=0;}
  if(pause||!map.h){usleep(2000);continue;}
  if(cw<1)cw=1;
  if(ch<1)ch=1;
  int changed=cw!=oldw||ch!=oldh||z!=oldz||px!=oldx||py!=oldy;
  if(cw!=oldw||ch!=oldh) {
   XMoveResizeWindow(x,win,0,0,cw,ch);
   /* EGL may use a different X connection. Flush this connection on resize
    * so a maximized Tk canvas cannot leave its GPU child at the old size. */
   XFlush(x);
  }
  oldw=cw;oldh=ch;oldz=z;oldx=px;oldy=py;
  struct vdq_header *h=map.h;struct vdq_slot *frame_slot=NULL;
  if(vdq_skip_to_latest(&map,4)<0){error_set(g,"Invalid display queue backlog");break;}
  int available=vdq_peek(&map,&frame_slot);
  if(available<0){error_set(g,"Invalid ordered display queue");break;}
  uint64_t pub=available?frame_slot->serial:0,seq=0;
  int fresh=available>0; /* Every published frame is consumed in order. */
  /* An application rate cap cannot discard frames; it only slows this queue. */
  if(fresh&&limit>0&&now-draw_at<(uint64_t)(1e9/limit))fresh=0;
  if(fresh){
   seq=frame_slot->frame_seq;uint64_t stamp=frame_slot->timestamp_ns;
   const uint8_t *pixels=(const uint8_t*)frame_slot+VDQ_META;
   struct video_boxes frame_boxes=frame_slot->boxes;
   image_stamp=stamp;image_seq=seq;image_arrived=now;
   if(show_boxes && boxes_path[0])video_boxes_snapshot(&boxes_reader,boxes_path,now,seq,stamp,h->width,h->height,&frame_boxes);
   if(video_boxes_valid(&frame_boxes,h->width,h->height))boxes=frame_boxes;else memset(&boxes,0,sizeof(boxes));
   /* Client pixels remain immutable through upload and presentation submission.
    * Avoid the old full-frame seqlock copy/retry path. The producer may reuse
    * this slot only after vdq_ack below, never while GL is reading it. */
   GLenum fmt=GL_LUMINANCE;unsigned texw=h->width*video_bpp(h->pixel_format);
   if(tw!=h->width||th!=h->height||tf!=h->pixel_format){
    tw=h->width;th=h->height;tf=h->pixel_format;glTexImage2D(GL_TEXTURE_2D,0,fmt,texw,th,0,fmt,GL_UNSIGNED_BYTE,pixels);
   }else glTexSubImage2D(GL_TEXTURE_2D,0,0,0,texw,th,fmt,GL_UNSIGNED_BYTE,pixels);
  }
  if(show_boxes && image_arrived && boxes_path[0]) {
   struct video_boxes latest;
   video_boxes_snapshot(&boxes_reader,boxes_path,now,image_seq,image_stamp,tw,th,&latest);
   if(latest.publish_seq!=boxes.publish_seq || latest.valid!=boxes.valid) {boxes=latest;changed=1;}
  }
  int boxes_visible=show_boxes&&image_arrived&&video_boxes_fresh(&boxes,image_seq,image_stamp,now-image_arrived);
  if(boxes_visible!=last_boxes_visible)changed=1;
  last_boxes_visible=boxes_visible;
  pthread_mutex_lock(&g->lock);g->boxes_valid=boxes_visible;g->boxes_count=boxes_visible?(int)boxes.count:0;
  g->boxes_frame_age=boxes_visible?(uint32_t)((uint32_t)image_seq-(uint32_t)boxes.frame_id):0;
  g->boxes_age_ms=boxes_visible?(image_stamp-boxes.capture_ns+now-image_arrived)/1e6:0;pthread_mutex_unlock(&g->lock);
  if((fresh||changed)&&tw&&(fresh||visible)){
   if(!mapped){XMapRaised(x,win);XFlush(x);mapped=1;}
   glViewport(0,0,cw,ch);glClearColor(.027,.055,.098,1);glClear(GL_COLOR_BUFFER_BIT);
   double scale=z>0?z:fmin((double)cw/tw,(double)ch/th);
   int rw=(int)round(tw*scale),rh=(int)round(th*scale);
   int ox=rw<cw?(cw-rw)/2:-(int)px,oy=rh<ch?(ch-rh)/2:-(int)py;
   glViewport(ox,ch-oy-rh,rw,rh);
   glUniform2f(glGetUniformLocation(prog,"sz"),tw,th);
   glUniform1i(glGetUniformLocation(prog,"fmt"),tf==CSHM_BAYER?2:tf==CSHM_BGR?1:0);
   glDrawArrays(GL_TRIANGLE_STRIP,0,4);
   if(boxes_visible && boxes.count){
    glEnable(GL_SCISSOR_TEST);glClearColor(image_seq==boxes.frame_id?.2f:1.f,.85f,.1f,1.f);
    for(unsigned i=0;i<boxes.count;i++){
     const struct video_box *b=boxes.boxes+i;
     int x1=ox+(int)round(b->x1*scale),y1=oy+(int)round(b->y1*scale);
     int x2=ox+(int)round(b->x2*scale),y2=oy+(int)round(b->y2*scale);
     box_bar(x1,y1,x2-x1,2,cw,ch);box_bar(x1,y2-2,x2-x1,2,cw,ch);
     box_bar(x1,y1,2,y2-y1,cw,ch);box_bar(x2-2,y1,2,y2-y1,cw,ch);
    }
    glDisable(GL_SCISSOR_TEST);
    gpu_labels_draw(&labels,&boxes,scale,ox,oy,cw,ch,image_seq==boxes.frame_id);
    /* Restore the image bindings before the next packed-byte upload. */
    glUseProgram(prog);glBindTexture(GL_TEXTURE_2D,tex);glBindBuffer(GL_ARRAY_BUFFER,vbo);
    glVertexAttribPointer(0,2,GL_FLOAT,GL_FALSE,0,0);
   }
   unsigned char rgb[3]={0};
   if(probe)glReadPixels(cw/2,ch/2,1,1,GL_RGB,GL_UNSIGNED_BYTE,rgb);
   if(glGetError()!=GL_NO_ERROR||!eglSwapBuffers(ed,surf)){error_set(g,"GPU upload/present failed");break;}
   if(fresh){
    if(last&&pub<=last){error_set(g,"Display publication sequence regressed");break;}
    if(vdq_ack(&map,pub,video_ns()-now)){error_set(g,"Ordered display acknowledgement failed");break;}
    pthread_mutex_lock(&g->lock);
    g->count=pub;g->skipped=atomic_load(&h->source_gaps);
    g->queue_depth=atomic_load(&h->produced)-atomic_load(&h->consumed);
    g->queue_high_water=atomic_load(&h->high_water);g->queue_waits=atomic_load(&h->full_waits);
    g->pub=pub;g->seq=seq;g->width=tw;g->height=th;g->format=tf;g->last_ns=video_ns();
    if(probe)memcpy(g->rgb,rgb,3);
    ++rate_count;
    if(now-rate_at>=500000000ULL){g->fps=rate_count*1e9/(now-rate_at);rate_at=now;rate_count=0;}
    pthread_mutex_unlock(&g->lock);last=pub;draw_at=now;
   }
  }
  if(!fresh)usleep(100);
 }
done:
 video_boxes_close(&boxes_reader);vdq_unmap(&map);
 if(ctx!=EGL_NO_CONTEXT){gpu_labels_destroy(&labels);glDeleteBuffers(1,&vbo);glDeleteTextures(1,&tex);if(prog)glDeleteProgram(prog);if(vert)glDeleteShader(vert);if(frag)glDeleteShader(frag);}
 if(ed!=EGL_NO_DISPLAY){eglMakeCurrent(ed,EGL_NO_SURFACE,EGL_NO_SURFACE,EGL_NO_CONTEXT);if(surf!=EGL_NO_SURFACE)eglDestroySurface(ed,surf);if(ctx!=EGL_NO_CONTEXT)eglDestroyContext(ed,ctx);eglTerminate(ed);}
 if(x){if(win)XDestroyWindow(x,win);if(cmap)XFreeColormap(x,cmap);XCloseDisplay(x);}
 return NULL;
}
void *video_gpu_start(unsigned long parent,const char *path){
 if(!path||strlen(path)>=4096)return NULL;
 struct gpu *g=calloc(1,sizeof(*g));if(!g)return NULL;
 g->parent=parent;g->cw=g->ch=1;snprintf(g->path,sizeof(g->path),"%s",path);pthread_mutex_init(&g->lock,NULL);
 if(pthread_create(&g->thread,NULL,render,g)){pthread_mutex_destroy(&g->lock);free(g);return NULL;}return g;
}
void video_gpu_config(void *ptr,int w,int h,int pause,double limit,double zoom,double px,double py){
 struct gpu *g=ptr;pthread_mutex_lock(&g->lock);g->cw=w;g->ch=h;g->paused=pause;g->limit=limit;g->zoom=zoom;g->panx=px;g->pany=py;pthread_mutex_unlock(&g->lock);
}
void video_gpu_boxes(void *ptr,const char *path,int enabled){
 struct gpu *g=ptr;pthread_mutex_lock(&g->lock);g->show_boxes=enabled;snprintf(g->boxes_path,sizeof(g->boxes_path),"%s",path?path:"");pthread_mutex_unlock(&g->lock);
}
void video_gpu_probe(void *ptr,int enabled){struct gpu *g=ptr;pthread_mutex_lock(&g->lock);g->probe=enabled;pthread_mutex_unlock(&g->lock);}
void video_gpu_status(void *ptr,char *out,size_t n){
 struct gpu *g=ptr;pthread_mutex_lock(&g->lock);
 snprintf(out,n,"{\"swap_interval\":%d,\"queue_depth\":%llu,\"queue_high_water\":%llu,\"queue_waits\":%llu,\"ordered\":true,\"boxes_valid\":%d,\"boxes_count\":%d,\"boxes_frame_age\":%u,\"boxes_age_ms\":%.3f,\"ready\":%d,\"count\":%llu,\"skipped\":%llu,\"pub\":%llu,\"seq\":%llu,\"epoch\":%llu,\"width\":%u,\"height\":%u,\"format\":%u,\"fps\":%.3f,\"age\":%.4f,\"probe\":[%u,%u,%u],\"renderer\":\"%s\",\"error\":\"%s\"}",
 g->swap_interval,(unsigned long long)g->queue_depth,(unsigned long long)g->queue_high_water,(unsigned long long)g->queue_waits,g->boxes_valid,g->boxes_count,g->boxes_frame_age,g->boxes_age_ms,g->ready,(unsigned long long)g->count,(unsigned long long)g->skipped,(unsigned long long)g->pub,(unsigned long long)g->seq,(unsigned long long)g->epoch,g->width,g->height,g->format,g->last_ns&&video_ns()-g->last_ns<2000000000ULL?g->fps:0.,
 g->last_ns?(video_ns()-g->last_ns)/1e9:999.,g->rgb[0],g->rgb[1],g->rgb[2],g->renderer,g->error);pthread_mutex_unlock(&g->lock);
}
void video_gpu_stop(void *ptr){if(!ptr)return;struct gpu *g=ptr;pthread_mutex_lock(&g->lock);g->stop=1;pthread_mutex_unlock(&g->lock);pthread_join(g->thread,NULL);pthread_mutex_destroy(&g->lock);free(g);}
