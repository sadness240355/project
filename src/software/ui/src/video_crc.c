#include "video_crc.h"
#include <string.h>
#include <zlib.h>
#include <dlfcn.h>
static uint32_t (*fast_crc)(uint32_t,const void*,size_t);
__attribute__((constructor)) static void init_crc(void) {
 void *h=dlopen("libdeflate.so.0",RTLD_NOW|RTLD_LOCAL);
 if(h)fast_crc=(uint32_t (*)(uint32_t,const void*,size_t))dlsym(h,"libdeflate_crc32");
}

#if defined(__aarch64__)
#include <sys/auxv.h>
#include <asm/hwcap.h>
#include <arm_acle.h>
#define CRC_TARGET __attribute__((target("+crc")))
#define CRC_WORD(c,v) __crc32d(c,v)
#define CRC_BYTE(c,v) __crc32b(c,v)
#elif defined(VIDEO_CRC_TEST_ARM)
/* Exercise the exact split/combine algorithm with a portable instruction
 * oracle. This checks bytes/tails, not native ARM performance. */
#define CRC_TARGET
static uint32_t crc_word_oracle(uint32_t c,uint64_t v) {
 unsigned char b[8];for(unsigned i=0;i<8;i++)b[i]=(unsigned char)(v>>(8*i));
 return ~((uint32_t)crc32(~c,b,8));
}
static uint32_t crc_byte_oracle(uint32_t c,unsigned char v) {
 return ~((uint32_t)crc32(~c,&v,1));
}
#define CRC_WORD(c,v) crc_word_oracle(c,v)
#define CRC_BYTE(c,v) crc_byte_oracle(c,v)
#endif

#if defined(__aarch64__) || defined(VIDEO_CRC_TEST_ARM)
CRC_TARGET
static uint32_t arm_crc(const unsigned char *p,size_t n) {
 uint32_t c=~0U;
 if(n>=4096) {
  /* Four independent contiguous CRC chains hide instruction dependency
   * latency. Combine finalized IEEE CRCs in byte order, including the tail.
   * No frame copy, heap allocation or extra DDR read is introduced. */
  size_t span=(n/32)*8;
  const unsigned char *p0=p,*p1=p+span,*p2=p+2*span,*p3=p+3*span;
  uint32_t c0=~0U,c1=~0U,c2=~0U,c3=~0U;
  for(size_t i=0;i<span;i+=8) {
   uint64_t v0,v1,v2,v3;
   memcpy(&v0,p0+i,8);memcpy(&v1,p1+i,8);
   memcpy(&v2,p2+i,8);memcpy(&v3,p3+i,8);
   c0=CRC_WORD(c0,v0);c1=CRC_WORD(c1,v1);
   c2=CRC_WORD(c2,v2);c3=CRC_WORD(c3,v3);
  }
  size_t tail=n-4*span;
  const unsigned char *last=p+4*span;
  for(size_t i=0;i<tail;i++)c3=CRC_BYTE(c3,last[i]);
  uint32_t combined=(uint32_t)crc32_combine(~c0,~c1,(z_off_t)span);
  combined=(uint32_t)crc32_combine(combined,~c2,(z_off_t)span);
  return (uint32_t)crc32_combine(combined,~c3,(z_off_t)(span+tail));
 }
 while(n>=8){uint64_t v;memcpy(&v,p,8);c=CRC_WORD(c,v);p+=8;n-=8;}
 while(n--){c=CRC_BYTE(c,*p++);}
 return ~c;
}
#endif

#ifdef VIDEO_CRC_TEST_ARM
uint32_t video_crc32_test_arm(const void *data,size_t bytes) {
 return arm_crc(data,bytes);
}
#endif

const char *video_crc_backend(void) {
 if(fast_crc)return "libdeflate";
#if defined(__aarch64__)
 if(getauxval(AT_HWCAP)&HWCAP_CRC32)return "arm_crc32_4way";
#endif
 return "zlib";
}
uint32_t video_crc32(const void *data,size_t bytes) {
 /* libdeflate performs its own CPU feature dispatch; do not bypass its
  * vector/folding implementation merely because scalar ARM CRC exists. */
 if(fast_crc)return fast_crc(0,data,bytes);
#if defined(__aarch64__)
 if(getauxval(AT_HWCAP)&HWCAP_CRC32)return arm_crc(data,bytes);
#endif
 return (uint32_t)crc32(0,data,bytes);
}
