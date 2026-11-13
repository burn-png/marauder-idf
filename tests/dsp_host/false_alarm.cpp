#include <initializer_list>
#include "dsp_adv.hpp"
#include <cstdio>
#include <cmath>
#include <algorithm>
using namespace marauder;
static unsigned rs=777; static double rnd(){rs=rs*1664525u+1013904223u;return ((rs>>8)&0xFFFF)/65536.0-0.5;}
int main(){
 for(double nz:{0.25,0.5,1.0}) for(int sens:{4,5,6}) for(int prom:{8,16}) for(int type:{0,2}){
  DspConfig c=dsp_cfg_defaults();c.d_sens=sens;c.d_prom=prom;c.d_type=type;dsp_reset_state();
  int fr=0,live=0;DspResults r;
  for(int f=0;f<300;++f){int8_t iq[512];for(int k=0;k<512;++k)iq[k]=(int8_t)std::max(-127.0,std::min(127.0,round(nz*(rnd()+rnd()+rnd())*2)));
    uint8_t x[240],p[240]={};dsp_fft_frame(iq,x,p,false,c);dsp_stage_spectrum(x,c,2437);dsp_results_snapshot(&r);
    fr+=r.count>0; for(unsigned i=0;i<r.count;++i) if(r.sig[i].persistence>200)++live;}
  printf("noise %.2f sens %d prom %2d %s: frames with any track %3d/300, tracks persist>200: %d  floor=%u\n",nz,sens,prom,type?"OS":"CA",fr,live,r.frame_floor);}
}
