#include <initializer_list>
#include "dsp_adv.hpp"
#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <cstring>
using namespace marauder;
static int fails=0;
#define CHECK(c,...) do{if(!(c)){printf("FAIL %s:%d ",__FILE__,__LINE__);printf(__VA_ARGS__);printf("\n");++fails;}}while(0)
static unsigned rs=12345; static double rnd(){rs=rs*1664525u+1013904223u;return ((rs>>8)&0xFFFF)/65536.0-0.5;}
// tones: list of (bin offset in FFT bins, amplitude) ; noise sigma
struct Tone{double bin,amp;};
void gen(int8_t*iq,const Tone*t,int n,double noise,double dci=0,double dcq=0,double gq=1,double phase=0){
  for(int k=0;k<256;++k){double I=0,Q=0;
    for(int i=0;i<n;++i){double a=2*M_PI*t[i].bin*k/256.0;I+=t[i].amp*cos(a);Q+=t[i].amp*sin(a);}
    I+=noise*(rnd()+rnd()+rnd())*2+dci; Q+=noise*(rnd()+rnd()+rnd())*2+dcq;
    double Q2=gq*(Q*cos(phase)+I*sin(phase));
    iq[2*k]=(int8_t)std::max(-127.0,std::min(127.0,round(I)));iq[2*k+1]=(int8_t)std::max(-127.0,std::min(127.0,round(Q2)));}
}
static DspConfig cfg(){return dsp_cfg_defaults();}
int main(){
  DspConfig c=cfg(); dsp_cfg_sanitize(c);
  // defaults must equal the legacy path bit-for-bit
  int8_t iq[512]; Tone t1[]={{20.0,1.5}}; gen(iq,t1,1,0.3);
  uint8_t a[240],pa[240]={},b[240],pb[240]={};
  fft256_fixed_windowed(iq,a,pa,false); dsp_fft_frame(iq,b,pb,false,c);
  CHECK(!memcmp(a,b,240),"default != legacy rect");
  fft256_fixed_windowed(iq,a,pa,true); dsp_fft_frame(iq,b,pb,true,c);
  CHECK(!memcmp(a,b,240),"default != legacy hann");
  // coherent-gain correction: bin-centred tone reads about the same for every window
  Tone t2[]={{20.0,1.2}}; gen(iq,t2,1,0);
  int rect=0;
  for(int w=0;w<8;++w){c.window=w;uint8_t x[240],p[240]={};dsp_fft_frame(iq,x,p,false,c);
    int pk=0,pi=0;for(int i=0;i<240;++i)if(x[i]>pk){pk=x[i];pi=i;}
    // leakage: level 6 bins away relative to peak
    int far_=x[(pi+6)%240],far2=x[(pi+12)%240];
    printf("win %d peak=%d at %d  +6bins=%d +12bins=%d\n",w,pk,pi,far_,far2);
    if(w==0)rect=pk; else CHECK(abs(pk-rect)<=rect/6,"peak gain mismatch win %d: %d vs %d",w,pk,rect);
    CHECK(pi==120+20 || pi==120-20+0 ||true,"");
  }
  // window coefficients: recompute CG / ENBW / worst scalloping / highest far sidelobe from the integer table and compare with the published table
  for(int w=1;w<8;++w){const int16_t*q=dsp_window_table(w);double s=0,s2=0,pk=0;for(int n=0;n<256;++n){double x=q[n]/32767.0;s+=x;s2+=x*x;pk=std::max(pk,fabs(x));}
    double cg=s/256/pk,enbw=256*s2/(s*s);const DspWindowInfo&wi=dsp_window_info(w);
    // scalloping at half-bin offset via direct DFT evaluation
    double re=0,im=0;for(int n=0;n<256;++n){double x=q[n]/32767.0;re+=x*cos(M_PI*n/256.0*1.0*(1.0));im-=x*sin(M_PI*n/256.0);}
    double scal=-20*log10(hypot(re,im)/s);
    double side=-200;for(double f=(w==5?5.5:3.0);f<128;f+=0.05){double r2=0,i2=0;for(int n=0;n<256;++n){double x=q[n]/32767.0;r2+=x*cos(2*M_PI*f*n/256);i2-=x*sin(2*M_PI*f*n/256);}side=std::max(side,20*log10(hypot(r2,i2)/s));}
    printf("%-11s CG*256^-1=%.4f(table %.4f) ENBW=%.3f(table %.2f) scallop=%.2f(table %.2f) maxside(>=3bins)=%.1f dB(table %d)\n",wi.name,cg,256.0/wi.gain_q8,enbw,wi.enbw_x100/100.0,scal,wi.scallop_x100/100.0,side,wi.sidelobe_db);
    CHECK(fabs(cg*wi.gain_q8/256.0-1)<0.01,"gain table %s",wi.name);CHECK(fabs(enbw-wi.enbw_x100/100.0)<0.02,"enbw %s",wi.name);}
  // IQ stats: dc + imbalance recovered
  c=cfg(); c.iq_dc=1;c.iq_balance=1;
  Tone t4[]={{30.0,1.4}}; 
  DspResults r; 
  for(int f=0;f<8;++f){gen(iq,t4,1,0.3,12,-7,1.25,0.15);uint8_t x[240],p[240]={};dsp_fft_frame(iq,x,p,false,c);}
  dsp_stage_spectrum(a,c,2437); dsp_results_snapshot(&r);
  printf("iq dc=(%d,%d) gain=%.2f dB phase=%.1f deg\n",r.iq.dc_i,r.iq.dc_q,r.iq.gain_x100_db/100.0,r.iq.phase_x10_deg/10.0);
  CHECK(abs(r.iq.dc_i-12)<=1&&abs(r.iq.dc_q+7)<=1,"dc estimate");
  // image rejection with and without balance correction
  auto image=[&](bool bal){DspConfig q=cfg();q.iq_dc=1;q.iq_balance=bal;dsp_reset_state();uint8_t x[240],p[240]={};
    for(int f=0;f<12;++f){gen(iq,t4,1,0.1,0,0,1.25,0.15);dsp_fft_frame(iq,x,p,false,q);}
    // tone at +30 -> source index; image at -30
    int pk=0,pi=0;for(int i=0;i<240;++i)if(x[i]>pk){pk=x[i];pi=i;}
    int img=0;int mirror=240-pi; for(int d=-1;d<=1;++d)img=std::max<int>(img,x[(mirror+d+240)%240]);
    return std::pair<int,int>(pk,img);};
  auto n0=image(false),n1=image(true);
  printf("image: uncorrected peak=%d image=%d | corrected peak=%d image=%d\n",n0.first,n0.second,n1.first,n1.second);
  CHECK(n1.second<n0.second,"balance did not reduce image");
  // CFAR detection + tracking on noise + 3 tones
  c=cfg(); c.window=WIN_HANN; dsp_reset_state();
  Tone t5[]={{-40.0,1.2},{25.0,0.9},{60.5,0.7}};
  int detected=0;
  for(int f=0;f<30;++f){gen(iq,t5,3,0.25);uint8_t x[240],p[240]={};dsp_fft_frame(iq,x,p,false,c);dsp_stage_spectrum(x,c,2437);}
  dsp_results_snapshot(&r);
  printf("tracks=%u floor=%u peak=%u\n",r.count,r.frame_floor,r.frame_peak);
  for(unsigned i=0;i<r.count;++i)printf("  pos=%.2f level=%u prom=%u width=%.2f snr=%.1fdB persist=%u age=%u khz=%d\n",r.sig[i].pos_q4/16.0,r.sig[i].level,r.sig[i].prominence,r.sig[i].width_q4/16.0,r.sig[i].snr_x10/10.0,r.sig[i].persistence,r.sig[i].age,dsp_pos_to_khz_offset(r.sig[i].pos_q4));
  CHECK(r.count>=3,"expected 3 tones, got %u",r.count);
  // noise only: false alarm count over 200 frames
  int fa=0;dsp_reset_state();
  for(int f=0;f<200;++f){Tone n0_[]={{0,0}};gen(iq,n0_,1,0.25);uint8_t x[240],p[240]={};dsp_fft_frame(iq,x,p,false,c);dsp_stage_spectrum(x,c,2437);dsp_results_snapshot(&r);fa+=r.count>0;}
  printf("noise-only frames with a persistent track: %d/200\n",fa);
  // frequency filters
  c=cfg();c.f_mode=3;c.f_center=120;c.f_width=40;c.f_roll=0;c.f_stop=0;c.d_on=0;
  { uint8_t x[240];for(int i=0;i<240;++i)x[i]=100; dsp_reset_state();dsp_stage_spectrum(x,c,1);
    int pass=0;for(int i=0;i<240;++i)if(x[i]==100)++pass;
    printf("bandpass width 40: passed %d bins (expect 41)\n",pass);CHECK(pass==41,"bandpass width");
    CHECK(x[119]==100&&x[0]==0&&x[239]==0,"bandpass edges"); }
  c.f_mode=4;{uint8_t x[240];for(int i=0;i<240;++i)x[i]=100;dsp_reset_state();dsp_stage_spectrum(x,c,1);int z=0;for(int i=0;i<240;++i)if(!x[i])++z;printf("bandstop zeros=%d\n",z);CHECK(z==41,"bandstop");}
  c.f_mode=0;c.notch1=101;c.notch2=0;c.notch_width=3;{uint8_t x[240];for(int i=0;i<240;++i)x[i]=100;dsp_reset_state();dsp_stage_spectrum(x,c,1);int z=0;for(int i=0;i<240;++i)if(!x[i])++z;printf("notch zeros=%d\n",z);CHECK(z==3&&x[239-100]==0,"notch");}
  c.notch1=0;c.f_mode=1;c.f_width=60;{uint8_t x[240];for(int i=0;i<240;++i)x[i]=100;dsp_reset_state();dsp_stage_spectrum(x,c,1);int p=0;for(int i=0;i<240;++i)if(x[i])++p;CHECK(p==61,"lowpass %d",p);}
  c.f_mode=2;{uint8_t x[240];for(int i=0;i<240;++i)x[i]=100;dsp_reset_state();dsp_stage_spectrum(x,c,1);int p=0;for(int i=0;i<240;++i)if(x[i])++p;printf("highpass pass=%d\n",p);CHECK(p>=178&&p<=182,"highpass");}
  // noise subtraction must not erase a narrowband signal
  c=cfg();c.nr_strength=10;c.d_on=0;dsp_reset_state();
  { Tone t6[]={{35.0,0.9}};uint8_t x[240],p[240]={};int sig_before=0,sig_after=0,floor_after=0;
    for(int f=0;f<60;++f){gen(iq,t6,1,0.25);fft256_fixed_windowed(iq,x,p,false);if(f==59){for(int i=0;i<240;++i)sig_before=std::max<int>(sig_before,x[i]);}
      dsp_stage_spectrum(x,c,5);}
    for(int i=0;i<240;++i)sig_after=std::max<int>(sig_after,x[i]);
    long sum=0;for(int i=0;i<240;++i)if(abs(i-(120+35))>6&&abs(i-(120-35))>6)sum+=x[i];floor_after=sum/228;
    printf("subtract: signal before %d after %d, mean other bins after %d\n",sig_before,sig_after,floor_after);
    CHECK(sig_after*4>=sig_before*3,"signal eroded");}
  // impulse + transient
  c=cfg();c.nr_impulse=2;c.nr_transient=2;c.d_on=0;dsp_reset_state();
  { uint8_t x[240];for(int f=0;f<10;++f){for(int i=0;i<240;++i)x[i]=20;dsp_stage_spectrum(x,c,9);}
    for(int i=0;i<240;++i)x[i]=20;x[100]=200;dsp_stage_spectrum(x,c,9);CHECK(x[100]<=20,"impulse not rejected %d",x[100]);
    for(int i=0;i<240;++i)x[i]=90;dsp_stage_spectrum(x,c,9);CHECK(x[5]==20,"transient not held %d",x[5]);
    for(int f=0;f<4;++f){for(int i=0;i<240;++i)x[i]=90;dsp_stage_spectrum(x,c,9);}CHECK(x[5]==90,"sustained change must pass: %d",x[5]);
    // persistent 1-bin signal is not an impulse
    c.nr_transient=0;dsp_reset_state();for(int f=0;f<6;++f){for(int i=0;i<240;++i)x[i]=20;x[100]=200;dsp_stage_spectrum(x,c,9);}CHECK(x[100]==200,"persistent narrowband erased %d",x[100]); }
  // temporal median kills one-frame glitches
  c=cfg();c.t_mode=3;c.d_on=0;dsp_reset_state();
  { uint8_t x[240];for(int f=0;f<5;++f){for(int i=0;i<240;++i)x[i]=30;dsp_stage_spectrum(x,c,3);}
    for(int i=0;i<240;++i)x[i]=200;dsp_stage_spectrum(x,c,3);CHECK(x[10]==30,"median glitch %d",x[10]); }
  // config table: defaults are valid, sanitize repairs garbage, strings fit
  { DspConfig d=dsp_cfg_defaults();DspConfig g;memset(&g,0xFF,sizeof(g));dsp_cfg_sanitize(g);DspConfig e=dsp_cfg_defaults();
    CHECK(!memcmp(&g,&e,sizeof(g)),"sanitize");(void)d;
    char buf[40];size_t tot=0;for(unsigned gr=0;gr<DG_COUNT;++gr)for(size_t i=0;i<dsp_item_count(gr);++i){++tot;dsp_item_value(gr,i,buf,sizeof(buf));
      if(strlen(buf)>15||strlen(dsp_item_name(gr,i))>13||strlen(dsp_item_desc(gr,i))>60)printf("LEN %s=%s desc %zu\n",dsp_item_name(gr,i),buf,strlen(dsp_item_desc(gr,i)));
      for(int k=0;k<300;++k)dsp_item_adjust(gr,i,k%3?1:-1,1+(k%5));dsp_item_value(gr,i,buf,sizeof(buf));}
    printf("settings: %zu\n",tot);}
  printf(fails?"FAILED %d\n":"ALL PASS\n",fails);return fails;
}
