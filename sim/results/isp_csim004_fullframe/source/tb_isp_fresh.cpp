#include "bayer_rg8_to_rgba8888.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>
// Independent scalar image oracle. No DUT helper, old testbench or old output used.
static int clamp8(int x) { return std::max(0,std::min(255,x)); }
static int gamma8(int x) {
    double u=x/255.0;
    return (int)std::lround(255.0*(u<=0.0031308?12.92*u:1.055*std::pow(u,1.0/2.4)-0.055));
}
static int frame(int w,int h,int mask,int seed) {
    hls::stream<bayer8_axis_t> in;
    hls::stream<rgb8_axis_t> out;
    std::vector<int> raw(w*h), pre(w*h);
    int gains[3]={6144,4096,3072};
    for(int y=0;y<h;y++) for(int x=0;x<w;x++) {
        int p=y*w+x, v=(x*29+y*47+seed*31+(x*y*3))&255;
        raw[p]=v;
        if(mask&1) v=std::max(0,v-17);
        int color=(!(y&1)&&!(x&1))?0:((y&1)&&(x&1))?2:1;
        if(mask&2) v=clamp8((v*gains[color]+2048)/4096);
        pre[p]=v;
    }
    for(int p=0;p<w*h;p+=8) {
        bayer8_axis_t b; b.data=0;
        for(int k=0;k<8;k++) b.data.range(k*8+7,k*8)=raw[p+k];
        b.keep=255;b.strb=255;b.user=(p==0);b.last=(p+8==w*h);b.id=0;b.dest=0;in.write(b);
    }
    // A non-identity CCM exercises signed coefficients and saturation.
    int ccm[9]={5120,-1024,0,0,4096,0,0,-1024,5120};
    bayer_rg8_to_rgba8888(in,out,w,h,mask,17,gains[0],gains[1],gains[2],
        ccm[0],ccm[1],ccm[2],ccm[3],ccm[4],ccm[5],ccm[6],ccm[7],ccm[8]);
    auto at=[&](int y,int x){return pre[std::max(0,std::min(h-1,y))*w+std::max(0,std::min(w-1,x))];};
    int beats=w*h/8;
    for(int t=0;t<beats;t++) {
        if(out.empty()){std::fprintf(stderr,"missing beat %d\n",t);return 1;}
        rgb8_axis_t b=out.read();
        if(b.user!=(t==0)||b.last!=(t==beats-1)||b.keep!=0xffffff||b.strb!=0xffffff||b.id!=0||b.dest!=0) {
            std::fprintf(stderr,"sideband mismatch beat=%d\n",t);return 1;
        }
        for(int lane=0;lane<8;lane++) {
            int p=t*8+lane,y=p/w,x=p%w;
            int v=at(y,x),n=at(y-1,x),s=at(y+1,x),l=at(y,x-1),r=at(y,x+1);
            int cross=(n+s+l+r+2)/4;
            int diag=(at(y-1,x-1)+at(y-1,x+1)+at(y+1,x-1)+at(y+1,x+1)+2)/4;
            int rgb[3];
            if(!(y&1)&&!(x&1)){rgb[0]=v;rgb[1]=cross;rgb[2]=diag;}
            else if((y&1)&&(x&1)){rgb[0]=diag;rgb[1]=cross;rgb[2]=v;}
            else if(!(y&1)){rgb[0]=(l+r+1)/2;rgb[1]=v;rgb[2]=(n+s+1)/2;}
            else {rgb[0]=(n+s+1)/2;rgb[1]=v;rgb[2]=(l+r+1)/2;}
            int expected[3];
            for(int c=0;c<3;c++) {
                int value=rgb[c];
                if(mask&4) {int acc=0;for(int k=0;k<3;k++)acc+=ccm[c*3+k]*rgb[k];value=clamp8(acc<=0?0:(acc+2048)/4096);}
                expected[c]=(mask&8)?gamma8(value):value;
                unsigned got=b.data.range(lane*24+c*8+7,lane*24+c*8).to_uint();
                if(got!=(unsigned)expected[c]) {
                    std::fprintf(stderr,"pixel mismatch w=%d h=%d mask=%d seed=%d x=%d y=%d channel=%d got=%u expected=%d\n",w,h,mask,seed,x,y,c,got,expected[c]);return 1;
                }
            }
        }
    }
    if(!out.empty()||!in.empty()){std::fprintf(stderr,"extra beats\n");return 1;}
    std::printf("ISP_CASE_PASS,w=%d,h=%d,mask=%d,seed=%d,pixels=%d\n",w,h,mask,seed,w*h);
    return 0;
}
int main() {
    int count=0;
    for(int w: {8,16,40}) for(int h:{1,2,5,9}) for(int mask:{0,1,2,4,8,15}) {
        if(frame(w,h,mask,++count))return 1;
    }
    // Repeated invocation checks that static line memories do not leak prior-frame pixels.
    for(int mask:{15,0,15,0,15})if(frame(40,9,mask,++count))return 1;
    if(frame(2400,2048,15,++count))return 1;
    std::printf("ISP_CSIM_PASS,cases=%d,SCOPE=C_MODEL_ONLY_NOT_RTL_OR_BOARD\n",count);
    return 0;
}
