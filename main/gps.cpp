#include "include/gps.hpp"
#include "include/nav.hpp"
#include "include/pins.hpp"
#include "driver/uart.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cmath>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {
portMUX_TYPE s_mux=portMUX_INITIALIZER_UNLOCKED;
marauder::GpsFix s_fix{};
uint64_t s_rate_window_us{}; uint32_t s_rate_count{};
uint64_t s_gst_us{}, s_gsa_us{}, s_vtg_us{};   // receiver extras are only reported while they keep arriving
constexpr uint64_t kExtraMaxAgeUs=5000000;
bool checksum_ok(const char *line){
    if(line[0]!='$')return false;
    const char *star=strchr(line,'*');
    if(!star||!star[1]||!star[2]||star[3])return false;
    if(!isxdigit((unsigned char)star[1])||!isxdigit((unsigned char)star[2]))return false;
    uint8_t c=0;for(const char*p=line+1;p<star;++p)c^=(uint8_t)*p;
    return c==(uint8_t)strtoul(star+1,nullptr,16);
}
bool parse_coordinate(const char *text,char hemisphere,bool latitude,double &out){
    if(!text[0])return false;
    char *end=nullptr;
    const double raw=strtod(text,&end);
    if(end==text||*end||!std::isfinite(raw)||raw<0)return false;
    const double degrees=floor(raw/100.0);
    const double minutes=raw-degrees*100.0;
    const double maximum=latitude?90.0:180.0;
    if(minutes>=60.0||degrees>maximum||(degrees==maximum&&minutes!=0.0))return false;
    if(latitude?(hemisphere!='N'&&hemisphere!='S'):(hemisphere!='E'&&hemisphere!='W'))return false;
    out=degrees+minutes/60.0;
    if(hemisphere=='S'||hemisphere=='W')out=-out;
    return true;
}
bool parse_decimal(const char *text,double &out){
    if(!text[0])return false;
    char *end=nullptr;
    out=strtod(text,&end);
    return end!=text&&!*end&&std::isfinite(out);
}
bool set_position(marauder::GpsFix &f,const char *lat,const char *ns,const char *lon,const char *ew){
    double latitude,longitude;
    if(!ns[0]||!ew[0]||!parse_coordinate(lat,ns[0],true,latitude)||
       !parse_coordinate(lon,ew[0],false,longitude))return false;
    f.latitude=latitude;f.longitude=longitude;return true;
}
void sentence(char *line){
    const uint64_t now=(uint64_t)esp_timer_get_time();
    if(!checksum_ok(line)){portENTER_CRITICAL(&s_mux);++s_fix.invalid_sentences;portEXIT_CRITICAL(&s_mux);return;}
    char *star=strchr(line,'*');if(star)*star=0;
    char *fields[20]{};size_t n=0;fields[n++]=line;
    for(char*p=line;*p&&n<20;++p)if(*p==','){*p=0;fields[n++]=p+1;}
    const char *type=fields[0]+(fields[0][0]=='$'?1:0);
    const size_t type_length=strlen(type);
    const char *sentence_type=type_length>=3?type+type_length-3:"";
    marauder::GpsFix parsed{};
    const bool is_gst=!strcmp(sentence_type,"GST")&&n>=9;
    const bool is_gsa=!strcmp(sentence_type,"GSA")&&n>=18;
    const bool is_vtg=!strcmp(sentence_type,"VTG")&&n>=8;
    const bool is_rmc=!strcmp(sentence_type,"RMC")&&n>=10;
    const bool is_gga=!strcmp(sentence_type,"GGA")&&n>=11;
    if(is_rmc){
        parsed.valid=fields[2][0]=='A'&&set_position(parsed,fields[3],fields[4],fields[5],fields[6]);
        parsed.has_speed=parse_decimal(fields[7],parsed.speed_mps);
        if(parsed.has_speed)parsed.speed_mps*=0.514444;
        parsed.has_course=parse_decimal(fields[8],parsed.course_deg);
        if(strlen(fields[1])>=6&&strlen(fields[9])>=6)
            snprintf(parsed.utc,sizeof(parsed.utc),"20%c%c-%c%c-%c%cT%c%c:%c%c:%c%cZ",fields[9][4],fields[9][5],fields[9][2],fields[9][3],fields[9][0],fields[9][1],fields[1][0],fields[1][1],fields[1][2],fields[1][3],fields[1][4],fields[1][5]);
    }else if(is_gga){
        const unsigned long quality=strtoul(fields[6],nullptr,10);
        const unsigned long satellites=strtoul(fields[7],nullptr,10);
        parsed.fix_quality=(uint8_t)(quality>255?255:quality);
        parsed.satellites=(uint8_t)(satellites>255?255:satellites);
        parsed.has_hdop=parse_decimal(fields[8],parsed.hdop);
        parsed.has_altitude=parse_decimal(fields[9],parsed.altitude_m);
        parsed.valid=parsed.fix_quality>0&&set_position(parsed,fields[2],fields[3],fields[4],fields[5]);
        if(strlen(fields[1])>=6)
            snprintf(parsed.utc,sizeof(parsed.utc),"%c%c:%c%c:%c%cZ",fields[1][0],fields[1][1],fields[1][2],fields[1][3],fields[1][4],fields[1][5]);
    }
    if(is_gst){
        double a=0,b=0,c=0;
        const bool ok=parse_decimal(fields[6],a)&&parse_decimal(fields[7],b)&&a>=0&&b>=0&&a<1000&&b<1000;
        const bool alt=parse_decimal(fields[8],c)&&c>=0&&c<1000;
        portENTER_CRITICAL(&s_mux);
        if(ok){s_fix.has_gst=true;s_fix.sigma_lat_m=a;s_fix.sigma_lon_m=b;s_fix.sigma_alt_m=alt?c:0;s_gst_us=now;}
        portEXIT_CRITICAL(&s_mux);
    }else if(is_gsa){
        double p=0,h=0,v=0;
        const unsigned long ft=strtoul(fields[2],nullptr,10);
        const bool ok=parse_decimal(fields[15],p)&&parse_decimal(fields[16],h)&&parse_decimal(fields[17],v);
        portENTER_CRITICAL(&s_mux);
        s_fix.fix_type=(uint8_t)(ft>3?0:ft);s_gsa_us=now;
        if(ok){s_fix.has_dop=true;s_fix.pdop=p;s_fix.vdop=v;}
        portEXIT_CRITICAL(&s_mux);
    }else if(is_vtg){
        double c=0,k=0;
        const bool okc=parse_decimal(fields[1],c)&&c>=0&&c<360.0;
        const bool oks=parse_decimal(fields[7],k)&&k>=0;
        portENTER_CRITICAL(&s_mux);
        s_fix.has_vtg=okc||oks;s_fix.vtg_course_deg=okc?c:0;s_fix.vtg_speed_mps=oks?k/3.6:0;s_vtg_us=now;
        portEXIT_CRITICAL(&s_mux);
    }
    portENTER_CRITICAL(&s_mux);
    ++s_fix.valid_sentences;s_fix.last_sentence_us=now;
    if(now-s_rate_window_us>=1000000){s_fix.update_hz=s_rate_count;s_rate_count=0;s_rate_window_us=now;}++s_rate_count;
    if(is_rmc){
        s_fix.has_speed=parsed.valid&&parsed.has_speed;
        s_fix.has_course=parsed.valid&&parsed.has_course;
        if(parsed.valid){
            s_fix.latitude=parsed.latitude;s_fix.longitude=parsed.longitude;
            s_fix.speed_mps=parsed.speed_mps;s_fix.course_deg=parsed.course_deg;
            s_fix.valid=true;s_fix.last_fix_us=now;
        }else{
            s_fix.valid=false;
        }
        if(parsed.utc[0])memcpy(s_fix.utc,parsed.utc,sizeof(s_fix.utc));
    }else if(is_gga){
        s_fix.fix_quality=parsed.fix_quality;s_fix.satellites=parsed.satellites;
        s_fix.has_hdop=parsed.has_hdop;s_fix.hdop=parsed.hdop;
        s_fix.has_altitude=parsed.has_altitude;s_fix.altitude_m=parsed.altitude_m;
        if(parsed.valid){
            s_fix.latitude=parsed.latitude;s_fix.longitude=parsed.longitude;
            s_fix.valid=true;s_fix.last_fix_us=now;
        }else{
            s_fix.valid=false;
        }
        if(parsed.utc[0]&&!strchr(s_fix.utc,'T'))memcpy(s_fix.utc,parsed.utc,sizeof(s_fix.utc));
    }
    marauder::GpsFix for_nav{};
    if(is_rmc&&parsed.valid){++s_fix.fix_seq;for_nav=s_fix;}
    portEXIT_CRITICAL(&s_mux);
    if(for_nav.valid)marauder::nav_on_fix_internal(for_nav);   // navigation service, outside the spinlock
}
}
namespace marauder {
esp_err_t gps_init(){uart_config_t cfg{};cfg.baud_rate=pins::gps_baud;cfg.data_bits=UART_DATA_8_BITS;cfg.parity=UART_PARITY_DISABLE;cfg.stop_bits=UART_STOP_BITS_1;cfg.flow_ctrl=UART_HW_FLOWCTRL_DISABLE;cfg.source_clk=UART_SCLK_DEFAULT;ESP_RETURN_ON_ERROR(uart_driver_install(UART_NUM_2,2048,0,0,nullptr,0),"gps","uart install");ESP_RETURN_ON_ERROR(uart_param_config(UART_NUM_2,&cfg),"gps","uart config");return uart_set_pin(UART_NUM_2,pins::gps_tx,pins::gps_rx,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE);}
GpsFix gps_current_fix(){GpsFix f;portENTER_CRITICAL(&s_mux);f=s_fix;const uint64_t g=s_gst_us,a=s_gsa_us,v=s_vtg_us;portEXIT_CRITICAL(&s_mux);const uint64_t now=(uint64_t)esp_timer_get_time();if(!f.last_fix_us||now-f.last_fix_us>3000000)f.valid=false;
    if(!g||now-g>kExtraMaxAgeUs){f.has_gst=false;}
    if(!a||now-a>kExtraMaxAgeUs){f.has_dop=false;f.fix_type=0;}
    if(!v||now-v>kExtraMaxAgeUs){f.has_vtg=false;}
    return f;}
void gps_task(void*){char line[128];size_t used=0;uint8_t buf[64];for(;;){int n=uart_read_bytes(UART_NUM_2,buf,sizeof(buf),pdMS_TO_TICKS(100));if(n<0){portENTER_CRITICAL(&s_mux);++s_fix.uart_errors;portEXIT_CRITICAL(&s_mux);continue;}for(int i=0;i<n;++i){char c=(char)buf[i];if(c=='\n'){line[used]=0;if(used)sentence(line);used=0;}else if(c!='\r'&&used+1<sizeof(line))line[used++]=c;else if(used+1>=sizeof(line)){used=0;portENTER_CRITICAL(&s_mux);++s_fix.invalid_sentences;portEXIT_CRITICAL(&s_mux);}}}}
}
