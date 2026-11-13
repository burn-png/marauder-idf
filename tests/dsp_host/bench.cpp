#include "dsp_adv.hpp"
#include <chrono>
#include <cstdio>
#include <cstdlib>
using namespace marauder; using clk=std::chrono::steady_clock;
static double ns(clk::time_point a,clk::time_point b){return std::chrono::duration<double,std::nano>(b-a).count();}
int main(){
  int8_t iq[512];for(int i=0;i<512;++i)iq[i]=(int8_t)((rand()%7)-3);
  DspConfig off=dsp_cfg_defaults(); off.d_on=0;
  DspConfig all=dsp_cfg_defaults(); all.window=WIN_BLACKMAN_HARRIS;all.iq_dc=all.iq_balance=all.iq_normalize=1;all.f_mode=3;all.notch1=60;all.notch2=100;all.notch3=180;all.dc_suppress=2;all.adj_reject=2;
  all.nr_strength=6;all.nr_impulse=2;all.nr_transient=2;all.nr_outlier=2;all.t_mode=4;all.d_on=1;all.v_smooth=2;all.v_decay=3;all.v_minhold=1;all.v_floor=1;all.v_range=1;all.w_suppress=2;all.w_contrast=2;all.w_smooth=2;
  const int N=3000; SpectrumFrame fr{};
  for(int pass=0;pass<2;++pass){const DspConfig&c=pass?all:off;dsp_reset_state();uint8_t x[240],p[240]={};double tf=0,ts=0,td=0;
    for(int i=0;i<N;++i){iq[(i*7)%512]=(int8_t)(rand()%9-4);auto a=clk::now();dsp_fft_frame(iq,x,p,false,c);auto b=clk::now();dsp_stage_spectrum(x,c,2437);auto d=clk::now();dsp_stage_display(x,p,false,c,fr);auto e=clk::now();tf+=ns(a,b);ts+=ns(b,d);td+=ns(d,e);}
    printf("%s: fft %.1f us  stage %.1f us  display %.1f us (host x86, relative only)\n",pass?"ALL ON":"DEFAULT",tf/N/1e3,ts/N/1e3,td/N/1e3);}
}
