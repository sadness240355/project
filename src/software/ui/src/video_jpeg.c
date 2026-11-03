#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <setjmp.h>
#include <stdatomic.h>
void video_memory_fence(void){atomic_thread_fence(memory_order_seq_cst);}
uint32_t video_memory_request_next(void *word){return atomic_fetch_add_explicit((_Atomic uint32_t *)word,1u,memory_order_release)+1u;}
#include <jpeglib.h>
struct video_jpeg_error {struct jpeg_error_mgr base;jmp_buf jump;};
struct encoder {struct jpeg_compress_struct c;struct video_jpeg_error e;unsigned char *out;unsigned long size;};
static void fail(j_common_ptr c){longjmp(((struct video_jpeg_error*)c->err)->jump,1);}
int video_jpeg_encode(const void *data,size_t bytes,unsigned w,unsigned h,unsigned fmt,int quality,void **output,unsigned long *size){
 if(!data||!output||!size||!w||!h||w>4096||h>4096||bytes!=(size_t)w*h*3||bytes>16*1024*1024||quality<1||quality>95||(fmt!=0x33424752U&&fmt!=0x33524742U))return -1;
 *output=NULL;*size=0;
 struct encoder *s=calloc(1,sizeof(*s));if(!s)return -1;
 s->c.err=jpeg_std_error(&s->e.base);s->e.base.error_exit=fail;
 if(setjmp(s->e.jump)){jpeg_destroy_compress(&s->c);free(s->out);free(s);return -1;}
 jpeg_create_compress(&s->c);jpeg_mem_dest(&s->c,&s->out,&s->size);
 s->c.image_width=w;s->c.image_height=h;s->c.input_components=3;
 s->c.in_color_space=fmt==0x33424752U?JCS_EXT_RGB:JCS_EXT_BGR;
 jpeg_set_defaults(&s->c);jpeg_set_quality(&s->c,quality,TRUE);s->c.dct_method=JDCT_IFAST;
 jpeg_start_compress(&s->c,TRUE);
 while(s->c.next_scanline<h){JSAMPROW row=(JSAMPROW)((const uint8_t*)data+(size_t)s->c.next_scanline*w*3);jpeg_write_scanlines(&s->c,&row,1);}
 jpeg_finish_compress(&s->c);*output=s->out;*size=s->size;jpeg_destroy_compress(&s->c);free(s);return 0;
}
void video_jpeg_free(void *p){free(p);}

/* Bilinear RGGB preview only. Raw bytes remain the RDMA/CRC payload. */
static unsigned bayer_avg(const uint8_t *p,int w,int h,int x,int y,const int *dx,const int *dy,int n){
 unsigned sum=0,count=0;
 for(int i=0;i<n;i++){int xx=x+dx[i],yy=y+dy[i];if(xx>=0&&yy>=0&&xx<w&&yy<h){sum+=p[(size_t)yy*w+xx];count++;}}
 return count?(sum+count/2)/count:0;
}
int video_bayer_rg8_to_rgb(const uint8_t *p,size_t bytes,unsigned w,unsigned h,uint8_t *out,size_t capacity){
 if(!p||!out||w<2||h<2||w>4096||h>4096||(w&1)||(h&1)||bytes!=(size_t)w*h||capacity<(size_t)w*h*3)return -1;
 const int crossx[4]={-1,1,0,0},crossy[4]={0,0,-1,1},diagx[4]={-1,1,-1,1},diagy[4]={-1,-1,1,1};
 const int hx[2]={-1,1},hy[2]={0,0},vx[2]={0,0},vy[2]={-1,1};
 for(unsigned y=0;y<h;y++)for(unsigned x=0;x<w;x++){
  size_t at=(size_t)y*w+x;unsigned r,g,b;
  if(!(y&1)&&!(x&1)){r=p[at];g=bayer_avg(p,w,h,x,y,crossx,crossy,4);b=bayer_avg(p,w,h,x,y,diagx,diagy,4);}
  else if((y&1)&&(x&1)){b=p[at];g=bayer_avg(p,w,h,x,y,crossx,crossy,4);r=bayer_avg(p,w,h,x,y,diagx,diagy,4);}
  else {g=p[at];r=bayer_avg(p,w,h,x,y,(y&1)?vx:hx,(y&1)?vy:hy,2);b=bayer_avg(p,w,h,x,y,(y&1)?hx:vx,(y&1)?hy:vy,2);}
  out[at*3]=r;out[at*3+1]=g;out[at*3+2]=b;
 }
 return 0;
}
