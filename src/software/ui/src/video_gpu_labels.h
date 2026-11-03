#ifndef VIDEO_GPU_LABELS_H
#define VIDEO_GPU_LABELS_H
#include "video_label_format.h"
#include "video_label_atlas.h"
#define LABEL_MAX_FLOATS (VIDEO_BOX_MAX * VIDEO_LABEL_CHARS * 6 * 4)
struct gpu_labels {
 GLuint program,texture,buffer;
 GLint tint,textured;
 GLfloat glyphs[LABEL_MAX_FLOATS],backs[VIDEO_BOX_MAX*6*4];
};
static void gpu_labels_destroy(struct gpu_labels *l) {
 glDeleteTextures(1,&l->texture);glDeleteBuffers(1,&l->buffer);
 if(l->program)glDeleteProgram(l->program);
}
static int gpu_labels_init(struct gpu_labels *l) {
 const char *v="attribute vec2 pos;attribute vec2 coord;varying vec2 uv;void main(){gl_Position=vec4(pos,0.,1.);uv=coord;}";
 const char *f="precision mediump float;varying vec2 uv;uniform sampler2D atlas;uniform vec4 tint;uniform float textured;"
               "void main(){gl_FragColor=vec4(tint.rgb,tint.a*mix(1.,texture2D(atlas,uv).r,textured));}";
 GLuint vs=shader(GL_VERTEX_SHADER,v),fs=shader(GL_FRAGMENT_SHADER,f);
 if(!vs||!fs){if(vs)glDeleteShader(vs);if(fs)glDeleteShader(fs);return -1;}
 l->program=glCreateProgram();glAttachShader(l->program,vs);glAttachShader(l->program,fs);
 glBindAttribLocation(l->program,0,"pos");glBindAttribLocation(l->program,1,"coord");glLinkProgram(l->program);
 glDeleteShader(vs);glDeleteShader(fs);GLint ok;glGetProgramiv(l->program,GL_LINK_STATUS,&ok);if(!ok)return -1;
 l->tint=glGetUniformLocation(l->program,"tint");l->textured=glGetUniformLocation(l->program,"textured");
 glUseProgram(l->program);glUniform1i(glGetUniformLocation(l->program,"atlas"),0);
 glGenTextures(1,&l->texture);glBindTexture(GL_TEXTURE_2D,l->texture);
 glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
 glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP_TO_EDGE);glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP_TO_EDGE);
 glPixelStorei(GL_UNPACK_ALIGNMENT,1);glTexImage2D(GL_TEXTURE_2D,0,GL_LUMINANCE,LABEL_ATLAS_W,LABEL_ATLAS_H,0,GL_LUMINANCE,GL_UNSIGNED_BYTE,label_atlas);
 glGenBuffers(1,&l->buffer);glBindBuffer(GL_ARRAY_BUFFER,l->buffer);glBufferData(GL_ARRAY_BUFFER,sizeof(l->glyphs),NULL,GL_DYNAMIC_DRAW);
 return glGetError()==GL_NO_ERROR?0:-1;
}
static size_t label_quad(GLfloat *v,size_t n,float x,float y,float w,float h,
 float u,float t,float uw,float th,int cw,int ch) {
 float x0=2*x/cw-1,x1=2*(x+w)/cw-1,y0=1-2*y/ch,y1=1-2*(y+h)/ch;
 const GLfloat q[]={x0,y0,u,t,x0,y1,u,t+th,x1,y0,u+uw,t,
                    x1,y0,u+uw,t,x0,y1,u,t+th,x1,y1,u+uw,t+th};
 memcpy(v+n,q,sizeof(q));return n+24;
}
static void gpu_labels_draw(struct gpu_labels *l,const struct video_boxes *boxes,
 double scale,int ox,int oy,int cw,int ch,int exact) {
 size_t ng=0,nb=0;
 float unit=(float)fmax(1.,fmin(2.25,fmin(cw/1280.,ch/720.))),s=.75f*unit,pad=3*unit;
 for(unsigned i=0;i<boxes->count;i++){
  const struct video_box *b=boxes->boxes+i;
  float x1=ox+b->x1*scale,y1=oy+b->y1*scale,x2=ox+b->x2*scale,y2=oy+b->y2*scale;
  if(x2<=0||y2<=0||x1>=cw||y1>=ch)continue;
  char label[VIDEO_LABEL_CHARS];video_box_label(b,label);
  float text_width=0;
  for(const unsigned char *p=(const unsigned char*)label;*p;p++)text_width+=label_advance[(*p>=32&&*p<127?*p:'?')-32]*s;
  char *second=NULL;float first_width=text_width,second_width=0;
  if(text_width+2*pad>fmaxf(x2-x1,140*unit)) {
   char *split=strstr(label," ID:");
   if(split) {
    *split=0;second=split+1;first_width=0;
    for(const unsigned char *p=(const unsigned char*)label;*p;p++)first_width+=label_advance[(*p>=32&&*p<127?*p:'?')-32]*s;
    for(const unsigned char *p=(const unsigned char*)second;*p;p++)second_width+=label_advance[(*p>=32&&*p<127?*p:'?')-32]*s;
   }
  }
  float width=fmaxf(first_width,second_width)+2*pad,height=36*s*(second?2:1)+2*pad;
  float x=fmaxf(0,fminf(x1,cw-width)),y=y1>=height?y1-height:y1+2;
  y=fmaxf(0,fminf(y,ch-height));
  nb=label_quad(l->backs,nb,x,y,width,height,0,0,0,0,cw,ch);
  for(unsigned line=0;line<(second?2U:1U);line++) {
   float pen=x+pad;
   for(const unsigned char *p=(const unsigned char*)(line?second:label);*p;p++){
    unsigned a=(*p>=32&&*p<127?*p:'?')-32;
    if(*p!=' ')ng=label_quad(l->glyphs,ng,pen,y+pad+line*36*s,32*s,36*s,
     (a%16)*32.f/LABEL_ATLAS_W,(a/16)*36.f/LABEL_ATLAS_H,32.f/LABEL_ATLAS_W,36.f/LABEL_ATLAS_H,cw,ch);
    pen+=label_advance[a]*s;
   }
  }
 }
 if(!ng)return;
 glViewport(0,0,cw,ch);glUseProgram(l->program);glActiveTexture(GL_TEXTURE0);glBindTexture(GL_TEXTURE_2D,l->texture);
 glBindBuffer(GL_ARRAY_BUFFER,l->buffer);glEnableVertexAttribArray(0);glEnableVertexAttribArray(1);
 glVertexAttribPointer(0,2,GL_FLOAT,GL_FALSE,4*sizeof(GLfloat),(void*)0);
 glVertexAttribPointer(1,2,GL_FLOAT,GL_FALSE,4*sizeof(GLfloat),(void*)(2*sizeof(GLfloat)));
 glEnable(GL_BLEND);glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
 glUniform1f(l->textured,0);glUniform4f(l->tint,.025f,.04f,.06f,.9f);
 glBufferSubData(GL_ARRAY_BUFFER,0,nb*sizeof(GLfloat),l->backs);glDrawArrays(GL_TRIANGLES,0,(GLsizei)(nb/4));
 glUniform1f(l->textured,1);glUniform4f(l->tint,exact?.2f:1.f,.85f,.1f,1.f);
 glBufferSubData(GL_ARRAY_BUFFER,0,ng*sizeof(GLfloat),l->glyphs);glDrawArrays(GL_TRIANGLES,0,(GLsizei)(ng/4));
 glDisable(GL_BLEND);glDisableVertexAttribArray(1);
}
#endif
