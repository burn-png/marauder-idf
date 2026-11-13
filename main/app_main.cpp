#include "include/pins.hpp"
#include "include/display.hpp"
#include "include/dsp.hpp"
#include "include/gps.hpp"
#include "include/storage.hpp"
#include "include/media_player.hpp"
#include "include/browser.hpp"
#include "include/map_ui.hpp"
#include "include/apprun.hpp"
#include "include/settings.hpp"
#include "include/input.hpp"
#include "include/battery.hpp"
#include "esp_system.h"
#include "include/thermal.hpp"
#include "include/dsp_adv.hpp"
#include "include/transmitter.hpp"
#include "include/spi_gate.hpp"
#include "radio_backend.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_random.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <atomic>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <algorithm>
#include <cmath>
#include <sys/stat.h>
#include <unistd.h>

namespace {
constexpr char kTag[] = "dominion";
constexpr uint16_t C_BLACK=0x0000, C_NAVY=0x0841, C_PANEL=0x18E3, C_CYAN=0x07FF;
constexpr uint16_t C_WHITE=0xFFFF, C_AMBER=0xFEA0, C_RED=0xF800, C_GREEN=0x07E0;
StaticQueue_t s_frame_queue_control;
alignas(4) uint8_t s_frame_queue_storage[sizeof(marauder::SpectrumFrame)];
QueueHandle_t s_frame_queue;
marauder::AppSettings s_settings;
portMUX_TYPE s_state_mux = portMUX_INITIALIZER_UNLOCKED;
std::atomic<bool> s_capture_requested{false};
std::atomic<bool> s_capture_active{false};
std::atomic<bool> s_replay_active{false};
std::atomic<bool> s_replay_loop{true};
std::atomic<bool> s_radio_paused{false};
std::atomic<uint8_t> s_dc_mode{1}; // 0: raw DC bin, 1: interpolate the center bin
std::atomic<bool> s_recon_active{false};
std::atomic<bool> s_looking_glass_active{false};
std::atomic<bool> s_foxhunt_active{false};
std::atomic<bool> s_rssi_detector_active{false};
std::atomic<uint32_t> s_rssi_detector_frequency{2412};
std::atomic<uint32_t> s_glass_min_frequency{2412};
std::atomic<uint32_t> s_glass_max_frequency{2484};
std::atomic<uint32_t> s_glass_current_frequency{2412};
std::atomic<uint8_t> s_glass_step_index{2};
std::atomic<uint8_t> s_glass_dwell_index{2};
std::atomic<uint8_t> s_glass_view_mode{0};
std::atomic<bool> s_glass_paused{false};
std::atomic<uint32_t> s_foxhunt_frequency{2412};
std::atomic<uint8_t> s_spectrum_filter{0};
std::atomic<bool> s_trace_autofit{true};
std::atomic<uint32_t> s_frames{0};
std::atomic<uint32_t> s_radio_capture_successes{0};
std::atomic<uint32_t> s_radio_errors{0};
std::atomic<uint32_t> s_last_capture_us{0};
std::atomic<int32_t> s_last_radio_error{0};
std::atomic<uint8_t> s_last_fft_peak{0};
std::atomic<uint16_t> s_bins_above_trace_floor{0};
std::atomic<bool> s_trace_autoscale{false};
TaskHandle_t s_radio_task_handle{};
TaskHandle_t s_gps_task_handle{};
TaskHandle_t s_ui_task_handle{};
char s_replay_name[64]{};
size_t s_replay_offset{};
char s_last_profile[64]{};
char s_status[32]="READY";
size_t s_replay_cursor{};
char s_note[256]{};size_t s_note_len{};size_t s_note_char{};size_t s_note_scroll{};bool s_note_loaded{};
size_t s_external_cursor{};
size_t s_rf_cursor{},s_rf_config_cursor{},s_gps_cursor{},s_gps_view{},s_gps_point_index{},s_gps_name_cursor{};bool s_gps_point_review{},s_gps_delete_confirm{},s_gps_name_editing{};int s_nav_dir=1;
char s_gps_track_name[16]="TRACK001";uint64_t s_gps_log_last_us{},s_gps_log_last_fix_us{};
uint8_t s_gps_coordinate_format{};bool s_gps_imperial_units{true}; // imperial by default
bool s_capture_delete_confirm{},s_profile_delete_confirm{};
bool s_tx_armed{};
size_t s_glass_setting{};
bool s_glass_editing{};
bool s_glass_range_locked{};
uint8_t s_glass_peak_bins[240]{};
size_t s_dsp_cursor{};
int s_dsp_group{-1};size_t s_dsp_menu{};bool s_dsp_clear{true};size_t s_dsp_sig_cursor{};
size_t s_foxhunt_step_index{};
uint8_t s_foxhunt_level{},s_foxhunt_peak_level{};
int s_foxhunt_peak_offset_khz{};
uint8_t s_rssi_history[200]{};
size_t s_rssi_history_count{},s_rssi_history_head{};
uint8_t s_rssi_threshold{96},s_rssi_level{};
bool s_rssi_triggered{},s_rssi_detector_enabled{true};
struct GlassQuickCache { char text[24]; bool focused,editing,valid; };
GlassQuickCache s_glass_quick_cache[10]{};
unsigned s_antenna_mode{};uint64_t s_stopwatch_elapsed_ms{},s_stopwatch_started_ms{};bool s_stopwatch_running{};char s_random_password[17]{};
int64_t s_calc_lhs{},s_calc_rhs{};int s_calc_op=-1;unsigned s_calc_key{};bool s_calc_rhs_mode{},s_calc_error{};char s_calc_result[64]="0";
int8_t s_iq[512];
uint8_t s_avg_bins[240]{};
uint8_t s_peak_bins[240]{};
bool s_avg_initialized;
void br_open();                       // SD file browser (defined before ui_task)


marauder::AppSettings settings_snapshot() {
    marauder::AppSettings copy;
    portENTER_CRITICAL(&s_state_mux); copy=s_settings; portEXIT_CRITICAL(&s_state_mux);
    return copy;
}
void settings_commit(const marauder::AppSettings &s, bool persist=true) {
    portENTER_CRITICAL(&s_state_mux); s_settings=s; portEXIT_CRITICAL(&s_state_mux);
    if (persist) {
        esp_err_t e=marauder::settings_save(s);
        if (e!=ESP_OK) ESP_LOGW(kTag,"settings save: %s",esp_err_to_name(e));
    }
}
void load_note() {
    s_note_len=marauder::storage_read_file("NOTES.TXT",0,s_note,sizeof(s_note)-1);
    s_note[s_note_len]=0;s_note_loaded=true;s_note_scroll=0;s_note_char=0;
}
void persist_note() {
    if(!marauder::storage_ready()){snprintf(s_status,sizeof(s_status),"SD UNAVAILABLE");return;}
    if(marauder::storage_write_file("NOTES.TXT",s_note,s_note_len,false))snprintf(s_status,sizeof(s_status),"NOTE SAVED");
    else snprintf(s_status,sizeof(s_status),"NOTE WRITE FAILED");
}
void make_password() {
    static constexpr char hex[]="0123456789ABCDEF";
    for(unsigned i=0;i<8;++i){uint8_t b=(uint8_t)esp_random();s_random_password[2*i]=hex[b>>4];s_random_password[2*i+1]=hex[b&15];}
    s_random_password[16]=0;
}
void gps_ui_preferences(bool save=false){nvs_handle_t h;if(nvs_open("gpsui",save?NVS_READWRITE:NVS_READONLY,&h)!=ESP_OK)return;if(save){nvs_set_u8(h,"coordfmt",s_gps_coordinate_format);nvs_set_u8(h,"imperial",s_gps_imperial_units?1:0);nvs_commit(h);}else{uint8_t v;if(nvs_get_u8(h,"coordfmt",&v)==ESP_OK&&v<=2)s_gps_coordinate_format=v;if(nvs_get_u8(h,"imperial",&v)==ESP_OK)s_gps_imperial_units=v!=0;}nvs_close(h);}
void dc_mode_load(){nvs_handle_t h;uint8_t v;if(nvs_open("sdr_ui",NVS_READONLY,&h)==ESP_OK){if(nvs_get_u8(h,"dc_mode",&v)==ESP_OK&&v<=1)s_dc_mode.store(v);nvs_close(h);}}
void dc_mode_cycle(){uint8_t v=(uint8_t)((s_dc_mode.load()+1)%2);s_dc_mode.store(v);nvs_handle_t h;if(nvs_open("sdr_ui",NVS_READWRITE,&h)==ESP_OK){nvs_set_u8(h,"dc_mode",v);nvs_commit(h);nvs_close(h);}}
constexpr uint8_t kSpectrumFilterCount=9;
void spectrum_filter_load(){nvs_handle_t h;uint8_t v;if(nvs_open("sdr_ui",NVS_READONLY,&h)==ESP_OK){if(nvs_get_u8(h,"spec_filter",&v)==ESP_OK&&v<kSpectrumFilterCount)s_spectrum_filter.store(v);if(nvs_get_u8(h,"trace_auto",&v)==ESP_OK&&v<=1)s_trace_autofit.store(v!=0);nvs_close(h);}}
void spectrum_filter_set(uint8_t value){if(value>=kSpectrumFilterCount)return;s_spectrum_filter.store(value);s_avg_initialized=false;memset(s_peak_bins,0,sizeof(s_peak_bins));nvs_handle_t h;esp_err_t e=nvs_open("sdr_ui",NVS_READWRITE,&h);if(e==ESP_OK){e=nvs_set_u8(h,"spec_filter",value);if(e==ESP_OK)e=nvs_commit(h);nvs_close(h);}if(e!=ESP_OK)ESP_LOGW(kTag,"spectrum filter save: %s",esp_err_to_name(e));}
void trace_autofit_set(bool enabled){s_trace_autofit.store(enabled,std::memory_order_relaxed);nvs_handle_t h;esp_err_t e=nvs_open("sdr_ui",NVS_READWRITE,&h);if(e==ESP_OK){e=nvs_set_u8(h,"trace_auto",enabled?1:0);if(e==ESP_OK)e=nvs_commit(h);nvs_close(h);}if(e!=ESP_OK)ESP_LOGW(kTag,"spectrum auto-fit save: %s",esp_err_to_name(e));}
const char *spectrum_filter_name(uint8_t value){static constexpr const char *names[]={"OFF","MEAN 3","TRIANGLE 5","MEDIAN 3","MEAN 5","MEAN 7","TRIANGLE 7","GAUSSIAN 5","MEDIAN 5"};return names[value<kSpectrumFilterCount?value:0];}
uint8_t smooth_spectrum_bin(const uint8_t bins[240],unsigned index,uint8_t filter) {
    if(filter==0)return bins[index];
    const unsigned radius=filter==1||filter==3?1:filter==5||filter==6?3:2;
    if(filter==3||filter==8){
        uint8_t values[5]{};const unsigned count=filter==3?3:5;
        for(unsigned j=0;j<count;++j){
            const int offset=(int)j-(int)(count/2);
            const unsigned at=(unsigned)std::clamp<int>((int)index+offset,0,239);
            values[j]=bins[at];
        }
        for(unsigned i=1;i<count;++i){const uint8_t value=values[i];unsigned j=i;while(j&&values[j-1]>value){values[j]=values[j-1];--j;}values[j]=value;}
        return values[count/2];
    }
    unsigned sum=0,weight_sum=0;
    for(int offset=-(int)radius;offset<=(int)radius;++offset){
        const unsigned at=(unsigned)std::clamp<int>((int)index+offset,0,239);
        unsigned weight=1;
        if(filter==2)weight=(unsigned)(3-abs(offset));
        else if(filter==6)weight=(unsigned)(4-abs(offset));
        else if(filter==7){static constexpr unsigned weights[]={1,4,6,4,1};weight=weights[offset+2];}
        sum+=(unsigned)bins[at]*weight;weight_sum+=weight;
    }
    return (uint8_t)(sum/weight_sum);
}
void format_coordinate(double value,bool latitude,char out[32]){const char hemi=latitude?(value<0?'S':'N'):(value<0?'W':'E');double a=fabs(value);unsigned deg=(unsigned)a;double min=(a-deg)*60.0;if(s_gps_coordinate_format==0)snprintf(out,32,"%s %+.5f",latitude?"LAT":"LON",value);else if(s_gps_coordinate_format==1)snprintf(out,32,"%s %02ud%07.4f %c",latitude?"LAT":"LON",deg,min,hemi);else{unsigned m=(unsigned)min;double sec=(min-m)*60.0;snprintf(out,32,"%s %02ud%02u'%04.1f %c",latitude?"LAT":"LON",deg,m,sec,hemi);}}
void copy_replay_name(char out[64]) {
    portENTER_CRITICAL(&s_state_mux); memcpy(out,s_replay_name,64); portEXIT_CRITICAL(&s_state_mux);
}
void set_replay(const char *name) {
    portENTER_CRITICAL(&s_state_mux);
    snprintf(s_replay_name,sizeof(s_replay_name),"%s",name?name:"");
    s_replay_offset=0;
    portEXIT_CRITICAL(&s_state_mux);
}

void radio_task(void *) {
    marauder::SpectrumFrame frame{};
    uint8_t raw_bins[240], raw_peaks[240]{};
    static constexpr uint32_t channels[]={2412,2417,2422,2427,2432,2437,2442,2447,2452,2457,2462,2467,2472,2484};
    static constexpr uint32_t glass_steps[]={4,8,16,32};
    static constexpr unsigned glass_dwells[]={1,2,4,8};
    uint32_t recon_index=0,glass_frequency=s_glass_min_frequency.load(std::memory_order_relaxed),averaged_frequency=0;
    unsigned averaged_divisor=0,glass_dwell=0,glass_step=glass_steps[2],glass_dwell_limit=glass_dwells[2];
    bool averaged_hann=false;int64_t last_gps_log_us=0;
    bool glass_was_active=false;
    for (;;) {
        const auto cfg=settings_snapshot();
        bool ok=false;
        const int64_t started=esp_timer_get_time();
        uint32_t tuned=cfg.center_mhz;
        if (s_radio_paused.load(std::memory_order_relaxed)) {
            vTaskDelay(pdMS_TO_TICKS(40));
            continue;
        }
        if(s_looking_glass_active.load(std::memory_order_relaxed)&&s_glass_paused.load(std::memory_order_relaxed)){
            vTaskDelay(pdMS_TO_TICKS(40));
            continue;
        }
        if (s_replay_active.load(std::memory_order_relaxed)) {
            char name[64]; copy_replay_name(name);
            const size_t size=marauder::storage_file_size(name);
            if (size>=512) {
                portENTER_CRITICAL(&s_state_mux); size_t offset=s_replay_offset; portEXIT_CRITICAL(&s_state_mux);
                if (offset+sizeof(s_iq)>size) {
                    if (s_replay_loop.load(std::memory_order_relaxed)) {
                        offset=0; portENTER_CRITICAL(&s_state_mux); s_replay_offset=0; portEXIT_CRITICAL(&s_state_mux);
                    } else {
                        s_replay_active.store(false,std::memory_order_relaxed);
                        vTaskDelay(pdMS_TO_TICKS(40)); continue;
                    }
                }
                ok=marauder::storage_read_file(name,offset,s_iq,sizeof(s_iq))==sizeof(s_iq);
                if (ok) { portENTER_CRITICAL(&s_state_mux); s_replay_offset=offset+sizeof(s_iq); portEXIT_CRITICAL(&s_state_mux); }
                frame.center_mhz=cfg.center_mhz;
            }
        } else {
            if(s_recon_active.load(std::memory_order_relaxed))tuned=channels[recon_index++%(sizeof(channels)/sizeof(channels[0]))];
            else if(s_foxhunt_active.load(std::memory_order_relaxed))tuned=s_foxhunt_frequency.load(std::memory_order_relaxed);
            else if(s_looking_glass_active.load(std::memory_order_relaxed)){
                const uint32_t glass_min=s_glass_min_frequency.load(std::memory_order_relaxed);
                const uint32_t glass_max=s_glass_max_frequency.load(std::memory_order_relaxed);
                if(!glass_was_active){glass_frequency=glass_min;glass_dwell=0;glass_was_active=true;}
                else glass_frequency=std::clamp(glass_frequency,glass_min,glass_max);
                const uint8_t step_index=std::min<uint8_t>(s_glass_step_index.load(std::memory_order_relaxed),3);
                const uint8_t dwell_index=std::min<uint8_t>(s_glass_dwell_index.load(std::memory_order_relaxed),3);
                if(glass_step!=glass_steps[step_index]||glass_dwell_limit!=glass_dwells[dwell_index]){
                    glass_step=glass_steps[step_index];
                    glass_dwell_limit=glass_dwells[dwell_index];
                    glass_dwell=0;
                }
                tuned=glass_frequency;
            }else if(s_rssi_detector_active.load(std::memory_order_relaxed)){glass_was_active=false;tuned=s_rssi_detector_frequency.load(std::memory_order_relaxed);}
            else glass_was_active=false;
            if(tuned!=averaged_frequency||cfg.averaging!=averaged_divisor||cfg.hann_window!=averaged_hann){
                averaged_frequency=tuned;averaged_divisor=cfg.averaging;averaged_hann=cfg.hann_window;
                s_avg_initialized=false;memset(s_peak_bins,0,sizeof(s_peak_bins));
            }
            frame.center_mhz=tuned;
            if(s_looking_glass_active.load(std::memory_order_relaxed))s_glass_current_frequency.store(tuned,std::memory_order_relaxed);
            const esp_err_t capture_error=marauder_radio_capture_iq8(tuned,256,s_iq);
            ok=capture_error==ESP_OK;
            s_last_radio_error.store(ok?0:capture_error,std::memory_order_relaxed);
            if(ok)s_radio_capture_successes.fetch_add(1,std::memory_order_relaxed);
            if(s_looking_glass_active.load(std::memory_order_relaxed)&&
               !s_glass_paused.load(std::memory_order_relaxed)&&++glass_dwell>=glass_dwell_limit){
                const uint32_t glass_min=s_glass_min_frequency.load(std::memory_order_relaxed);
                const uint32_t glass_max=s_glass_max_frequency.load(std::memory_order_relaxed);
                if(glass_frequency>=glass_max)
                    glass_frequency=glass_min;
                else
                    glass_frequency=std::min(glass_frequency+glass_step,glass_max);
                glass_dwell=0;
            }
            if (ok && s_capture_requested.load(std::memory_order_relaxed)) {
                if (!s_capture_active.load(std::memory_order_relaxed))
                    s_capture_active.store(marauder::storage_capture_begin(frame.center_mhz),std::memory_order_relaxed);
                if (s_capture_active.load(std::memory_order_relaxed) &&
                    !marauder::storage_append_iq8(frame.center_mhz,s_iq,256)) {
                    marauder::storage_capture_end(); s_capture_active.store(false,std::memory_order_relaxed);
                    s_capture_requested.store(false,std::memory_order_relaxed);
                }
                auto fix=marauder::gps_current_fix();
                const int64_t gps_now=esp_timer_get_time();
                if (s_capture_active.load(std::memory_order_relaxed) && fix.valid && gps_now-last_gps_log_us>=1000000) {
                    marauder::storage_log_position(frame.center_mhz,fix.utc,fix.latitude,fix.longitude);last_gps_log_us=gps_now;
                }
            } else if (!s_capture_requested.load(std::memory_order_relaxed) && s_capture_active.exchange(false)) {
                marauder::storage_capture_end();
            }
        }
        if (ok) {
            const marauder::DspConfig dspc=marauder::dsp_cfg_snapshot();
            static uint8_t last_dsp_window=0xff;
            if(dspc.window!=last_dsp_window){last_dsp_window=dspc.window;s_avg_initialized=false;memset(s_peak_bins,0,sizeof(s_peak_bins));memset(raw_peaks,0,sizeof(raw_peaks));}
            const int64_t dsp_t0=esp_timer_get_time();
            marauder::dsp_fft_frame(s_iq,raw_bins,raw_peaks,cfg.hann_window,dspc);
            const int64_t dsp_t1=esp_timer_get_time();
            marauder::dsp_stage_spectrum(raw_bins,dspc,frame.center_mhz);
            const int64_t dsp_t2=esp_timer_get_time();
            const uint8_t filter=s_spectrum_filter.load(std::memory_order_relaxed);
            if(filter){
                uint8_t filtered_bins[240],filtered_peaks[240];
                for(unsigned i=0;i<240;++i){
                    filtered_bins[i]=smooth_spectrum_bin(raw_bins,i,filter);
                    filtered_peaks[i]=smooth_spectrum_bin(raw_peaks,i,filter);
                }
                memcpy(raw_bins,filtered_bins,sizeof(raw_bins));
                memcpy(raw_peaks,filtered_peaks,sizeof(raw_peaks));
            }
            if(s_dc_mode.load(std::memory_order_relaxed)==1){raw_bins[120]=(uint8_t)(((unsigned)raw_bins[119]+raw_bins[121])/2);raw_peaks[120]=std::max(raw_bins[120],std::max(raw_peaks[119],raw_peaks[121]));}
            const unsigned divisor=std::max<unsigned>(1,cfg.averaging);
            for (unsigned i=0;i<240;++i) {
                if (!s_avg_initialized || divisor==1) s_avg_bins[i]=raw_bins[i];
                else s_avg_bins[i]=(uint8_t)((int)s_avg_bins[i]+((int)raw_bins[i]-(int)s_avg_bins[i])/(int)divisor);
                if (cfg.max_hold) s_peak_bins[i]=std::max(s_peak_bins[i],s_avg_bins[i]);
                else s_peak_bins[i]=raw_peaks[i];
            }
            s_avg_initialized=true;
            uint8_t fft_peak=0;
            uint16_t bins_above_floor=0;
            for(unsigned i=0;i<240;++i){
                fft_peak=std::max(fft_peak,s_avg_bins[i]);
                if(s_avg_bins[i]>cfg.trace_floor)++bins_above_floor;
            }
            s_last_fft_peak.store(fft_peak,std::memory_order_relaxed);
            s_bins_above_trace_floor.store(bins_above_floor,std::memory_order_relaxed);
            marauder::dsp_stage_display(s_avg_bins,s_peak_bins,cfg.max_hold,dspc,frame);
            marauder::dsp_perf_note((uint32_t)(dsp_t1-dsp_t0),(uint32_t)(dsp_t2-dsp_t1),(uint32_t)(esp_timer_get_time()-dsp_t2));
            xQueueOverwrite(s_frame_queue,&frame);
            s_frames.fetch_add(1,std::memory_order_relaxed);
            if(s_recon_active.load(std::memory_order_relaxed)&&marauder::storage_ready()){
                uint8_t level=0;for(unsigned i=0;i<240;++i)level=std::max(level,raw_bins[i]);
                if(level>72){char record[80];int n=snprintf(record,sizeof(record),"%lu,%lu,%u\r\n",(unsigned long)(esp_timer_get_time()/1000),(unsigned long)tuned,level);marauder::storage_write_file("RECON.CSV",record,(size_t)n,true);}
            }
        } else if (!s_replay_active.load(std::memory_order_relaxed)) {
            s_radio_errors.fetch_add(1,std::memory_order_relaxed);
        }
        s_last_capture_us.store((uint32_t)(esp_timer_get_time()-started),std::memory_order_relaxed);
        vTaskDelay(pdMS_TO_TICKS(80));
    }
}

void gps_task_entry(void *arg) { marauder::gps_task(arg); }

enum class Page : uint8_t { HOME, CATEGORY, DETAIL };
enum class Category : uint8_t { RECEIVE, TRANSMIT, RECON, CAPTURE, REPLAY, RF_REPLAY, TX_TEST, GPS, LOOKING_GLASS, PROFILES, UTILITIES, SETTINGS, EXTERNALS, DIAGNOSTICS, FOXHUNT };
enum class Detail : uint8_t { NONE, SPECTRUM, FREQ, GAIN, SCANNER, CAPTURE, CAPTURE_FILES, REPLAY, RF_REPLAY, RF_CONFIGS, TX_TEST, GPS, PROFILES, FILES, FREQ_MANAGER, IQ_TRIM, NOTEPAD, FLASH, DEBUG, STORAGE, DEVICE, SETTINGS, EXTERNALS, DIAG, ANTENNA, STOPWATCH, RANDPWD, CALCULATOR, MEDIA, FOXHUNT, DSP, RSSI_GRAPH, FOXTX, MAP_VIEW, NAV_VIEW, RID_VIEW, APPRUN };
char s_ext_names[24][64];
bool is_map_detail(Detail d);
void map_radio_hook(bool pause);
struct Item { const char *name; Detail detail; };
Page s_page=Page::HOME;Category s_category=Category::RECEIVE;Detail s_detail=Detail::NONE;
char s_header_title[32]{},s_header_right[16]{},s_footer_text[32]{};
bool s_header_force{},s_footer_force{true},s_footer_valid{};
const Item kHome[]={{"Receiver",Detail::NONE},{"Transmitter",Detail::NONE},{"IQ capture",Detail::NONE},{"Offline IQ playback",Detail::NONE},{"RF Replay compatibility",Detail::NONE},{"GPS",Detail::NONE},{"RF Profiles",Detail::NONE},{"Utilities",Detail::NONE},{"TX bench test",Detail::NONE},{"Settings",Detail::NONE},{"External Apps",Detail::NONE},{"Diagnostics",Detail::NONE}};
const Category kHomeCategories[]={Category::RECEIVE,Category::TRANSMIT,Category::CAPTURE,Category::REPLAY,Category::RF_REPLAY,Category::GPS,Category::PROFILES,Category::UTILITIES,Category::TX_TEST,Category::SETTINGS,Category::EXTERNALS,Category::DIAGNOSTICS};
static_assert(sizeof(kHomeCategories)/sizeof(*kHomeCategories)==sizeof(kHome)/sizeof(*kHome));
const Item kReceive[]={{"Live spectrum",Detail::SPECTRUM},{"Level / signal meter",Detail::NONE},{"Looking Glass sweep",Detail::NONE},{"Passive channel scan",Detail::NONE},{"Rolling RSSI detector",Detail::NONE},{"Frequency setup",Detail::FREQ},{"RX gain / sample rate",Detail::GAIN},{"DSP and trace controls",Detail::DSP},{"Back",Detail::NONE}};
const Item kTransmit[]={{"Fox beacon",Detail::FOXTX},{"Back",Detail::NONE}};
const Item kRecon[]={{"Passive channel scan",Detail::SCANNER},{"Signal level",Detail::SPECTRUM},{"Back",Detail::NONE}};
const Item kCapture[]={{"Start / stop IQ8 capture",Detail::CAPTURE},{"Browse captures",Detail::CAPTURE_FILES},{"Back",Detail::NONE}};
const Item kReplay[]={{"Offline IQ replay",Detail::REPLAY},{"Back",Detail::NONE}};
const Item kRfReplay[]={{"Inspect captures",Detail::RF_REPLAY},{"Saved RF replay configs",Detail::RF_CONFIGS},{"Back",Detail::NONE}};
const Item kTxTest[]={{"One attenuated test packet",Detail::TX_TEST},{"Back",Detail::NONE}};
const Item kGps[]={{"GPS dashboard / logging",Detail::GPS},{"Saved tracks",Detail::GPS},{"Saved locations",Detail::GPS},{"Offline map",Detail::MAP_VIEW},{"Nav / speedometer",Detail::NAV_VIEW},{"Remote ID (passive)",Detail::RID_VIEW},{"Back",Detail::NONE}};
const Item kGlass[]={{"Sweep waterfall",Detail::SPECTRUM},{"Back",Detail::NONE}};
const Item kProfiles[]={{"RF profile manager",Detail::PROFILES},{"Back",Detail::NONE}};
const Item kUtilities[]={{"Notepad",Detail::NOTEPAD},{"Calculator",Detail::CALCULATOR},{"Stopwatch",Detail::STOPWATCH},{"Antenna length",Detail::ANTENNA},{"Random password",Detail::RANDPWD},{"SD file browser",Detail::FILES},{"Frequency manager",Detail::FREQ_MANAGER},{"IQ trim",Detail::IQ_TRIM},{"Flash utility",Detail::FLASH},{"Debug tools",Detail::DEBUG},{"Storage diagnostics",Detail::STORAGE},{"Device information",Detail::DEVICE},{"Back",Detail::NONE}};
const Item kExternal[]={{"SD apps",Detail::EXTERNALS},{"Back",Detail::NONE}};
const Item kDiag[]={{"Runtime diagnostics",Detail::DIAG},{"GPS status",Detail::DEVICE},{"Back",Detail::NONE}};
const Item kFoxhunt[]={{"Start signal meter",Detail::FOXHUNT},{"Back",Detail::NONE}};
const Item *items_for(Category c,size_t &n) {
    switch(c){
        case Category::RECEIVE:n=sizeof(kReceive)/sizeof(*kReceive);return kReceive;
        case Category::TRANSMIT:n=sizeof(kTransmit)/sizeof(*kTransmit);return kTransmit;
        case Category::RECON:n=sizeof(kRecon)/sizeof(*kRecon);return kRecon;
        case Category::CAPTURE:n=sizeof(kCapture)/sizeof(*kCapture);return kCapture;
        case Category::REPLAY:n=sizeof(kReplay)/sizeof(*kReplay);return kReplay;
        case Category::RF_REPLAY:n=sizeof(kRfReplay)/sizeof(*kRfReplay);return kRfReplay;
        case Category::TX_TEST:n=sizeof(kTxTest)/sizeof(*kTxTest);return kTxTest;
        case Category::GPS:n=sizeof(kGps)/sizeof(*kGps);return kGps;
        case Category::LOOKING_GLASS:n=sizeof(kGlass)/sizeof(*kGlass);return kGlass;
        case Category::PROFILES:n=sizeof(kProfiles)/sizeof(*kProfiles);return kProfiles;
        case Category::UTILITIES:n=sizeof(kUtilities)/sizeof(*kUtilities);return kUtilities;
        case Category::EXTERNALS:n=sizeof(kExternal)/sizeof(*kExternal);return kExternal;
        case Category::FOXHUNT:n=sizeof(kFoxhunt)/sizeof(*kFoxhunt);return kFoxhunt;
        default:n=sizeof(kDiag)/sizeof(*kDiag);return kDiag;
    }
}
const char *category_name(Category c) {
    switch(c){case Category::RECEIVE:return "RECEIVE";case Category::TRANSMIT:return "TRANSMITTER";case Category::RECON:return "RECON";case Category::CAPTURE:return "CAPTURE";case Category::REPLAY:return "OFFLINE IQ PLAYBACK";case Category::RF_REPLAY:return "RF REPLAY";case Category::TX_TEST:return "TX BENCH";case Category::GPS:return "GPS";case Category::LOOKING_GLASS:return "LOOKING GLASS";case Category::PROFILES:return "RF PROFILES";case Category::UTILITIES:return "UTILITIES";case Category::SETTINGS:return "SETTINGS";case Category::EXTERNALS:return "EXTERNAL APPS";case Category::FOXHUNT:return "SIGNAL METER";default:return "DIAGNOSTICS";}
}
void draw_header(const char *title,const char *right="",bool force=false,uint16_t title_color=C_CYAN) {
    force=force||s_header_force;s_header_force=false;
    if(!force&&strcmp(s_header_title,title)==0&&strcmp(s_header_right,right)==0)return;
    if(force)marauder::display_fill_rect(0,14,240,17,C_PANEL);
    marauder::display_text(4,18,title,title_color,C_PANEL);
    if(right[0])marauder::display_text(180,18,right,C_AMBER,C_PANEL);
    if(force){char rule[31];memset(rule,0xC4,30);rule[30]=0;marauder::display_text(0,31,rule,C_CYAN,C_BLACK);}
    snprintf(s_header_title,sizeof(s_header_title),"%s",title);snprintf(s_header_right,sizeof(s_header_right),"%s",right);
}
const char *short_app_title(){
    if(s_page==Page::HOME)return "HOME";
    if(s_page==Page::CATEGORY){switch(s_category){case Category::RECEIVE:return "RX";case Category::TRANSMIT:return "TX";case Category::RECON:return "SCN";case Category::CAPTURE:return "CAP";case Category::REPLAY:return "IQ";case Category::RF_REPLAY:return "RF";case Category::TX_TEST:return "TX";case Category::GPS:return "GPS";case Category::LOOKING_GLASS:return "SWP";case Category::PROFILES:return "PRO";case Category::UTILITIES:return "UTL";case Category::SETTINGS:return "SET";case Category::EXTERNALS:return "EXT";case Category::FOXHUNT:return "LEVEL";default:return "DIA";}}
    switch(s_detail){case Detail::FOXTX:return "FOX";case Detail::MAP_VIEW:return "MAP";case Detail::APPRUN:return "APP";case Detail::NAV_VIEW:return "NAV";case Detail::RID_VIEW:return "RID";case Detail::SPECTRUM:return "SDR";case Detail::DSP:return "DSP";case Detail::FOXHUNT:return "LEVEL";case Detail::TX_TEST:return "TX";case Detail::GPS:return "GPS";case Detail::SETTINGS:return "SET";case Detail::CAPTURE:return "CAP";case Detail::CAPTURE_FILES:return "FILES";case Detail::REPLAY:return "IQ";case Detail::RF_REPLAY:case Detail::RF_CONFIGS:return "RF";case Detail::PROFILES:return "PROF";case Detail::FILES:return "FILES";case Detail::DIAG:case Detail::DEBUG:return "DIAG";default:return "UTIL";}
}
void draw_status_bar(bool force=false){
    // Every field has a fixed width and is redrawn with its background, so a shorter value
    // (BAT 100 -> 99, RAM 10% -> 9%) can never leave stale digits behind.
    static int64_t last_update_us=0,last_battery_us=0;
    static bool last_flash=false,flash_latched=false;
    static char last_title[8]{};
    const int64_t now_us=esp_timer_get_time();
    const bool slow_due=force||!last_update_us||now_us-last_update_us>=1000000;
    // Over-temperature alarm: latches at 190 F, clears below 187 F. The temperature field flashes red.
    const int temp_f=marauder::die_temp_f();
    if(temp_f>=190)flash_latched=true;else if(temp_f<187)flash_latched=false;
    const bool phase=flash_latched&&((now_us/400000)&1);
    auto draw_temp=[&](){
        char field[8];
        if(temp_f<=-1000)snprintf(field,sizeof(field),"T --F");
        else snprintf(field,sizeof(field),"T%3dF",std::min(temp_f,999));
        uint16_t fg=C_WHITE,bg=C_PANEL;
        if(flash_latched){fg=phase?C_WHITE:C_RED;bg=phase?C_RED:C_PANEL;}
        else if(temp_f>=175){fg=C_AMBER;}
        marauder::display_text(160,2,field,fg,bg);
        last_flash=phase;
    };
    if(!slow_due){
        if(flash_latched&&phase!=last_flash)draw_temp();
        return;
    }
    last_update_us=now_us;
    if(force||now_us-last_battery_us>=3000000){marauder::battery_update();last_battery_us=now_us;}
    marauder::thermal_update();
    const size_t total=heap_caps_get_total_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
    const size_t free=heap_caps_get_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
    const unsigned ram=total?((unsigned)((total-free)*100/total)):0;
    const unsigned cpu=marauder::cpu_load_pct();
    const auto fix=marauder::gps_current_fix();
    const bool sd_ok=marauder::storage_accessible();
    const int battery=marauder::battery_percent();
    char title[8],field[8];
    snprintf(title,sizeof(title),"%-3.3s",short_app_title());
    if(force)marauder::display_fill_rect(0,0,240,12,C_PANEL);
    if(force||strcmp(title,last_title)!=0){marauder::display_text(0,2,title,C_WHITE,C_PANEL);snprintf(last_title,sizeof(last_title),"%s",title);}
    snprintf(field,sizeof(field),"C%3u%%",std::min(cpu,100u));
    marauder::display_text(28,2,field,cpu>=90?C_AMBER:C_WHITE,C_PANEL);
    snprintf(field,sizeof(field),"M%3u%%",std::min(ram,100u));
    marauder::display_text(72,2,field,ram>=90?C_AMBER:C_WHITE,C_PANEL);
    if(battery<0)snprintf(field,sizeof(field),"B --%%");else snprintf(field,sizeof(field),"B%3u%%",(unsigned)std::min(battery,100));
    marauder::display_text(116,2,field,battery<0?C_WHITE:battery<=15?C_RED:battery<=30?C_AMBER:C_WHITE,C_PANEL);
    draw_temp();
    marauder::display_text(204,2,"SD",sd_ok?C_GREEN:C_RED,C_PANEL);
    marauder::display_text(224,2,"G",fix.valid?C_GREEN:C_WHITE,C_PANEL);
}
void draw_footer(const char *text) {
    if(s_footer_force){char rule[31];memset(rule,0xC4,30);rule[30]=0;marauder::display_text(0,296,rule,C_CYAN,C_BLACK);s_footer_force=false;}
    if(s_footer_valid&&strcmp(s_footer_text,text)==0)return;
    marauder::display_fill_rect(0,306,240,14,C_PANEL);
    marauder::display_text(4,309,text,C_WHITE,C_PANEL);
    snprintf(s_footer_text,sizeof(s_footer_text),"%s",text);s_footer_valid=true;
}
void draw_menu(const char *title,const Item *items,size_t count,size_t selected,size_t first,bool force) {
    static const Item *last_items=nullptr;static size_t last_selected=(size_t)-1,last_first=(size_t)-1;static char last_title[32]{};
    const bool new_menu=force||last_items!=items||strcmp(last_title,title)!=0;
    if(new_menu){marauder::display_clear(C_BLACK);s_footer_force=true;s_footer_valid=false;draw_header(title,"",true);last_items=items;last_first=(size_t)-1;snprintf(last_title,sizeof(last_title),"%s",title);}
    const size_t visible=23;
    if(selected<first)first=selected;
    if(selected>=first+visible)first=selected-visible+1;
    const bool page_shift=first!=last_first;
    const size_t begin=page_shift?first:std::min(selected,last_selected);
    const size_t end=page_shift?std::min(count,first+visible):std::max(selected,last_selected)+1;
    for(size_t i=begin;i<count&&i<first+visible&&i<end;++i){
        const int y=36+(int)(i-first)*11; const bool sel=i==selected;
        marauder::display_fill_rect(2,y-1,236,11,sel?0x21A7:C_BLACK);
        char line[31]; snprintf(line,sizeof(line),"%c %-26.26s",sel?'>':' ',items[i].name);
        marauder::display_text(5,y,line,sel?C_WHITE:C_CYAN,sel?0x21A7:C_BLACK);
    }
    if(new_menu)draw_footer("UP/DN move  CENTER open  LEFT back");
    last_selected=selected;last_first=first;
}

void draw_detail_frame(const char *status="LEFT BACK / HOLD CENTER") {
    marauder::display_clear(C_BLACK);
    s_header_force=true;s_footer_force=true;s_footer_valid=false;
    draw_footer(status);
}
void draw_line(int row,const char *text,uint16_t fg=C_WHITE) { marauder::display_text(4,34+row*11,text,fg,C_BLACK); }
void draw_value_row(int row,const char *label,const char *value,bool selected) {
    int y=36+row*11;
    char line[31];snprintf(line,sizeof(line),"%c%-13.13s %.15s",selected?'>':' ',label,value);
    marauder::display_text(0,y,line,selected?C_WHITE:C_CYAN,selected?0x21A7:C_BLACK);
}
uint8_t enhance_spectrum_contrast(uint8_t value) {
    return (uint8_t)std::sqrt((unsigned)value * 255u);
}
void project_trace(const marauder::SpectrumFrame &frame,const marauder::AppSettings &cfg,
                   uint8_t bins[240],uint8_t peaks[240],uint8_t floor_out[240]=nullptr,uint8_t min_out[240]=nullptr) {
    const unsigned span=std::min<unsigned>(16,std::max<unsigned>(1,cfg.span_mhz));
    const auto dspc=marauder::dsp_cfg_snapshot();
    static constexpr unsigned range_span[5]={32,64,96,128,192};
    uint8_t raw[240],raw_peak[240],raw_floor[240],raw_min[240];
    for(unsigned i=0;i<240;++i){
        const int centered=(int)i-120;
        const int source=std::clamp(239-(120+(centered*(int)span)/16),0,239);
        raw[i]=frame.bins[source];
        raw_peak[i]=frame.peaks[source];
        raw_floor[i]=frame.floor[source];
        raw_min[i]=frame.mins[source];
    }
    uint8_t ordered[240];
    memcpy(ordered,raw,sizeof(ordered));
    std::sort(ordered,ordered+240);
    unsigned floorv=cfg.trace_floor,ceilv=cfg.trace_ceiling;
    const bool dsp_range=dspc.v_range&&frame.have_floor;
    if(s_trace_autofit.load(std::memory_order_relaxed)||dsp_range){
        static float smoothed_floor=0,smoothed_ceiling=0;
        static uint32_t previous_center=0;
        static uint16_t previous_span=0;
        unsigned target_floor=ordered[12];
        unsigned target_ceiling=ordered[236];
        if(dsp_range){ // DSP > DISPLAY range modes: anchor the bottom on the tracked noise floor
            target_floor=frame.floor_level>1?frame.floor_level-1u:0u;
            target_ceiling=dspc.v_range==1?std::min<unsigned>(255,target_floor+range_span[dspc.v_span%5]):std::min<unsigned>(255,std::max<unsigned>(target_floor+32,ordered[239]+8u));
        }
        if(target_ceiling<target_floor+6){
            const unsigned midpoint=(target_floor+target_ceiling)/2;
            target_floor=midpoint>3?midpoint-3:0;
            target_ceiling=std::min<unsigned>(255,target_floor+6);
            target_floor=target_ceiling>=6?target_ceiling-6:0;
        }
        if(previous_center!=frame.center_mhz||previous_span!=cfg.span_mhz||smoothed_ceiling<=smoothed_floor){
            smoothed_floor=(float)target_floor;
            smoothed_ceiling=(float)target_ceiling;
            previous_center=frame.center_mhz;
            previous_span=cfg.span_mhz;
        }else{
            smoothed_floor+=(target_floor-smoothed_floor)*0.25f;
            smoothed_ceiling+=(target_ceiling-smoothed_ceiling)*0.25f;
        }
        floorv=(unsigned)smoothed_floor;
        ceilv=std::max<unsigned>(floorv+1,(unsigned)smoothed_ceiling);
    }
    s_trace_autoscale.store(s_trace_autofit.load(std::memory_order_relaxed)||dsp_range,std::memory_order_relaxed);
    for(unsigned i=0;i<240;++i){
        auto normalize=[&](uint8_t v)->uint8_t{
            const uint8_t linear=v<=floorv?0:v>=ceilv?255:(uint8_t)(((unsigned)(v-floorv)*255)/(ceilv-floorv));
            return enhance_spectrum_contrast(linear);
        };
        bins[i]=normalize(raw[i]);peaks[i]=normalize(raw_peak[i]);
        if(floor_out)floor_out[i]=normalize(raw_floor[i]);
        if(min_out)min_out[i]=normalize(raw_min[i]);
    }
}
void project_waterfall(const marauder::SpectrumFrame &frame,const marauder::AppSettings &cfg,uint8_t bins[240]) {
    const unsigned span=std::min<unsigned>(16,std::max<unsigned>(1,cfg.span_mhz));
    uint8_t raw[240],ordered[240];
    const bool interp=marauder::dsp_cfg_snapshot().w_interp&&span<16;
    for(unsigned i=0;i<240;++i){
        const int centered=(int)i-120;
        const int source=std::clamp(239-(120+(centered*(int)span)/16),0,239);
        if(interp){ // linear interpolation between neighbouring bins when zoomed in (waterfall only)
            const int q4=std::clamp(1904-centered*(int)span,0,239*16);
            const int lo=q4>>4,fr=q4&15,hi=std::min(lo+1,239);
            raw[i]=(uint8_t)(((int)frame.wf[lo]*(16-fr)+(int)frame.wf[hi]*fr)>>4);
        }else raw[i]=frame.wf[source];
    }
    memcpy(ordered,raw,sizeof(ordered));
    std::sort(ordered,ordered+240);
    const unsigned below_floor=(unsigned)(std::upper_bound(ordered,ordered+240,cfg.intensity_floor)-ordered);
    const bool low_level=below_floor>=180;
    const unsigned floorv=low_level?std::min<unsigned>(cfg.intensity_floor,ordered[24]):cfg.intensity_floor;
    const unsigned auto_ceiling=std::max<unsigned>(floorv+1,ordered[238]);
    const unsigned ceilv=low_level?std::min<unsigned>(cfg.intensity_ceiling,auto_ceiling):cfg.intensity_ceiling;
    for(unsigned i=0;i<240;++i){
        const unsigned v=raw[i];
        const uint8_t linear=v<=floorv?0:v>=ceilv?255:(uint8_t)(((v-floorv)*255)/(ceilv-floorv));
        bins[i]=enhance_spectrum_contrast(linear);
    }
}

bool input_direction(const marauder::InputEvent &e){return e.kind==marauder::InputKind::PRESS||e.kind==marauder::InputKind::REPEAT;}
marauder::InputConfig input_config(const marauder::AppSettings &c){return{c.joystick_repeat_delay_ms,c.joystick_repeat_period_ms,c.joystick_long_press_ms,c.joystick_debounce_ms,c.joystick_acceleration,c.joystick_invert_vertical,c.joystick_invert_horizontal};}

size_t list_capture_names(char names[][64],size_t cap) { return marauder::storage_list_files(".",".C8",names,cap); }
void draw_rf_replay_page(){
    char line[40];draw_header("RF REPLAY COMPATIBILITY");static char names[24][64];size_t nf=list_capture_names(names,24);
    if(!nf){draw_line(1,"NO IQ8 CAPTURES ON SD",C_AMBER);draw_line(3,"OFFLINE PLAYBACK IS SEPARATE");draw_line(5,"NO TX HARDWARE ON V7");return;}
    s_rf_cursor%=nf;const char*name=names[s_rf_cursor];snprintf(line,sizeof(line),"CAPTURE %s",name);draw_line(1,line,C_AMBER);
    char meta[80],data[512]{};snprintf(meta,sizeof(meta),"%s.META",name);size_t got=marauder::storage_read_file(meta,0,data,sizeof(data)-1);data[got]=0;
    auto field=[&](const char*k,const char*fallback)->const char*{static char values[4][48];static unsigned slot=0;slot=(slot+1)%4;char token[40];snprintf(token,sizeof(token),"%s=",k);char*p=strstr(data,token);if(!p)return fallback;p+=strlen(token);size_t z=strcspn(p,"\r\n");z=std::min(z,sizeof(values[slot])-1);memcpy(values[slot],p,z);values[slot][z]=0;return values[slot];};
    const char *freq_text=field("FREQUENCY_MHZ","unknown"),*rate_text=field("SAMPLE_RATE_HZ","unknown");unsigned long freq=strtoul(freq_text,nullptr,10);size_t bytes=marauder::storage_file_size(name);bool iq8=bytes>=512&&bytes%512==0&&strstr(data,"FORMAT=IQ8_INTERLEAVED");
    snprintf(line,sizeof(line),"FREQ %s RATE %s",freq_text,rate_text);draw_line(2,line);snprintf(line,sizeof(line),"FORMAT %s",field("FORMAT","unknown"));draw_line(3,line);snprintf(line,sizeof(line),"MOD %s",field("MODULATION","unknown"));draw_line(4,line);
    draw_line(5,iq8?"IQ8 FILE OK: OFFLINE FFT":"FORMAT UNSUPPORTED / CONVERT",iq8?C_GREEN:C_AMBER);
    const bool freq_in_request_range=freq>=MARAUDER_RADIO_FREQ_MIN_MHZ&&freq<=MARAUDER_RADIO_FREQ_MAX_MHZ;
    const bool freq_unverified=freq>MARAUDER_RADIO_FREQ_BAND_MAX_MHZ;
    draw_line(6,!freq_in_request_range?"FREQ OUT OF RX RANGE":freq_unverified?"OUT OF BAND / UNVERIFIED":"IN REQUEST RANGE",
              !freq_in_request_range||freq_unverified?C_RED:C_GREEN);
    draw_line(7,"TX RATE: NO V7 TX TARGET",C_RED);
    draw_line(8,strstr(data,"CONTINUOUS_STREAM=1")?"IQ CONTIGUOUS; TX STILL UNAVAILABLE":"SPARSE IQ: RF REPLAY UNSUPPORTED",C_RED);
    char cfgpath[80];snprintf(cfgpath,sizeof(cfgpath),"%s",name);char*ext=strrchr(cfgpath,'.');if(ext)strcpy(ext,".RRC");draw_line(9,marauder::storage_file_size(cfgpath)?"CONFIG SAVED; TRANSMIT DISABLED":"CENTER SAVE CONFIG / TX OFF",C_CYAN);
}
void draw_rf_configs_page(){char line[40];draw_header("SAVED RF REPLAY CONFIGS");static char names[24][64];size_t n=marauder::storage_list_files(".",".RRC",names,24);if(!n){draw_line(1,"NO SAVED CONFIGS",C_AMBER);draw_line(3,"SELECT A CAPTURE TO SAVE");draw_line(5,"TRANSMIT REMAINS DISABLED");return;}s_rf_config_cursor%=n;const char*name=names[s_rf_config_cursor];snprintf(line,sizeof(line),"CONFIG %s",name);draw_line(1,line,C_AMBER);char data[256]{};size_t z=marauder::storage_read_file(name,0,data,sizeof(data)-1);data[z]=0;auto show=[&](const char*key,int row){char token[32];snprintf(token,sizeof(token),"%s=",key);char*p=strstr(data,token);if(!p){snprintf(line,sizeof(line),"%s unknown",key);draw_line(row,line);return;}p+=strlen(token);size_t take=strcspn(p,"\r\n");if(take>25)take=25;char value[26];memcpy(value,p,take);value[take]=0;snprintf(line,sizeof(line),"%s %.20s",key,value);draw_line(row,line);};show("CAPTURE",3);show("MODE",4);draw_line(6,"TRANSMIT_ALLOWED=0",C_RED);draw_line(8,"UP/DOWN BROWSE; LEFT BACK");}
struct TrackSummary { uint32_t points; double start_lat,start_lon,end_lat,end_lon,distance_km; bool have_points; char start_utc[24],end_utc[24];uint32_t duration_s;bool has_duration; };
bool track_row_coords(char *row,double &lat,double &lon,char utc[24]){
    char *context=nullptr;
    char *field=strtok_r(row,",",&context);
    if(!field)return false;
    snprintf(utc,24,"%.23s",field);
    field=strtok_r(nullptr,",",&context);
    if(!field)return false;
    char *end=nullptr;
    lat=strtod(field,&end);
    if(end==field||!std::isfinite(lat)||lat < -90.0||lat > 90.0)return false;
    field=strtok_r(nullptr,",",&context);
    if(!field)return false;
    end=nullptr;
    lon=strtod(field,&end);
    return end!=field&&std::isfinite(lon)&&lon >= -180.0&&lon <= 180.0;
}
bool track_utc_seconds(const char *text,uint32_t &seconds){
    const char *time=strchr(text,'T');
    time=time?time+1:text;
    if(strlen(time)<8||time[2]!=':'||time[5]!=':')return false;
    const int hour=atoi(time),minute=atoi(time+3),second=atoi(time+6);
    if(hour>23||minute>59||second>59)return false;
    seconds=(uint32_t)(hour*3600+minute*60+second);
    return true;
}
TrackSummary summarize_track(const char *path,size_t bytes){
    ESP_LOGI(kTag,"GPS track scan start: %s (%u bytes)",path,(unsigned)bytes);
    TrackSummary summary{};
    marauder::StorageReader reader{};
    if(!marauder::storage_reader_open(&reader,path)){
        ESP_LOGW(kTag,"GPS track open failed: %s",path);
        return summary;
    }
    static char chunk[128],row[160],first[160],last[160];
    size_t used=0,bytes_read=0,next_yield=2048;
    unsigned headers=0;
    first[0]=last[0]=row[0]=0;
    auto accept_row=[&](){
        row[used]=0;
        if(!summary.points)snprintf(first,sizeof(first),"%s",row);
        snprintf(last,sizeof(last),"%s",row);
        if(summary.points<UINT32_MAX)++summary.points;
        used=0;
    };
    size_t count;
    while((count=marauder::storage_reader_read(&reader,chunk,sizeof(chunk)))>0){
        bytes_read+=count;
        for(size_t i=0;i<count;++i){
            const char c=chunk[i];
            if(headers<3){if(c=='\n')++headers;continue;}
            if(c=='\n'){if(used)accept_row();continue;}
            if(c!='\r'&&used+1<sizeof(row))row[used++]=c;
        }
        if(bytes_read>=next_yield){vTaskDelay(1);next_yield=bytes_read+2048;}
    }
    if(used)accept_row();
    marauder::storage_reader_close(&reader);
    if(!summary.points){
        ESP_LOGI(kTag,"GPS track scan complete: %s has no rows",path);
        return summary;
    }
    summary.have_points=track_row_coords(first,summary.start_lat,summary.start_lon,summary.start_utc)&&
                        track_row_coords(last,summary.end_lat,summary.end_lon,summary.end_utc);
    if(summary.have_points){
        constexpr double rad=0.017453292519943295;
        const double a=summary.start_lat*rad,b=summary.end_lat*rad;
        const double dl=(summary.end_lon-summary.start_lon)*rad;
        const double dlat=(summary.end_lat-summary.start_lat)*rad;
        const double haversine=sin(dlat/2)*sin(dlat/2)+cos(a)*cos(b)*sin(dl/2)*sin(dl/2);
        summary.distance_km=12742.0*asin(sqrt(std::min(1.0,haversine)));
        uint32_t start_seconds,end_seconds;
        if(track_utc_seconds(summary.start_utc,start_seconds)&&track_utc_seconds(summary.end_utc,end_seconds)){
            summary.duration_s=end_seconds>=start_seconds?end_seconds-start_seconds:end_seconds+86400-start_seconds;
            summary.has_duration=true;
        }
    }
    ESP_LOGI(kTag,"GPS track scan complete: %s rows=%lu coords=%u",
             path,(unsigned long)summary.points,summary.have_points?1u:0u);
    return summary;
}
bool read_track_point(const char *path,size_t index,char out[160]){
    marauder::StorageReader reader{};
    if(!marauder::storage_reader_open(&reader,path)){out[0]=0;return false;}
    size_t row=0,used=0;
    unsigned headers=0;
    size_t bytes_read=0,next_yield=4096;
    char buf[128];
    while(true){
        const size_t n=marauder::storage_reader_read(&reader,buf,sizeof(buf));
        if(!n)break;
        bytes_read+=n;
        for(size_t i=0;i<n;++i){
            const char c=buf[i];
            if(headers<3){if(c=='\n')++headers;continue;}
            if(c=='\n'){
                if(row==index){out[used]=0;marauder::storage_reader_close(&reader);return used>0;}
                ++row;used=0;continue;
            }
            if(c!='\r'&&used<159)out[used++]=c;
        }
        if(bytes_read>=next_yield){vTaskDelay(1);next_yield=bytes_read+4096;}
    }
    const bool found=row==index&&used>0;
    out[found?used:0]=0;
    marauder::storage_reader_close(&reader);
    return found;
}
void draw_track_graph(const char *path,uint32_t total,bool force=false){
    (void)path;
    (void)total;
    (void)force;
}
bool select_replay(size_t index) {
    static char names[24][64];
    const size_t n=list_capture_names(names,24);
    if(!n || index>=n) { s_replay_active.store(false); set_replay(""); return false; }
    const size_t bytes=marauder::storage_file_size(names[index]);
    if(bytes<512 || (bytes%512)!=0) { s_replay_active.store(false); set_replay(""); snprintf(s_status,sizeof(s_status),"BAD IQ8 FILE"); return false; }
    set_replay(names[index]);s_replay_active.store(false);s_radio_paused.store(false);return true;
}
bool start_replay_next() {
    static char names[24][64];
    const size_t n=list_capture_names(names,24);
    if(!n){s_replay_active.store(false);set_replay("");snprintf(s_status,sizeof(s_status),"NO IQ8 CAPTURES");return false;}
    for(size_t attempt=0;attempt<n;++attempt){
        const size_t index=(s_replay_cursor+attempt)%n;
        if(select_replay(index)){s_replay_cursor=(index+1)%n;return true;}
    }
    snprintf(s_status,sizeof(s_status),"NO VALID IQ8 FILES");return false;
}
bool trim_capture(const char *name) {
    constexpr size_t kBlock=512;
    const size_t bytes=marauder::storage_file_size(name);
    if(bytes<kBlock || bytes%kBlock){snprintf(s_status,sizeof(s_status),"INVALID IQ8 SIZE");return false;}
    int8_t block[kBlock];const size_t blocks=bytes/kBlock;size_t first=blocks,last=0;
    for(size_t b=0;b<blocks;++b){
        if(marauder::storage_read_file(name,b*kBlock,block,kBlock)!=kBlock){snprintf(s_status,sizeof(s_status),"SD READ ERROR");return false;}
        int peak=0;for(size_t i=0;i<kBlock;++i){int v=block[i];if(v<0)v=-v;if(v>peak)peak=v;}
        if(peak>3){if(first==blocks)first=b;last=b;}
    }
    if(first==blocks){snprintf(s_status,sizeof(s_status),"NO ACTIVE IQ FOUND");return false;}
    char out[32];bool found=false;
    for(unsigned id=0;id<1000;++id){snprintf(out,sizeof(out),"TR%03u.C8",id);if(!marauder::storage_file_size(out)){found=true;break;}}
    if(!found){snprintf(s_status,sizeof(s_status),"TRIM SLOTS FULL");return false;}
    bool started=false;
    for(size_t b=first;b<=last;++b){
        if(marauder::storage_read_file(name,b*kBlock,block,kBlock)!=kBlock || !marauder::storage_write_file(out,block,kBlock,started)){
            marauder::storage_delete_file(out);snprintf(s_status,sizeof(s_status),"TRIM WRITE ERROR");return false;
        }
        started=true;
    }
    char meta[96],meta_path[40];snprintf(meta,sizeof(meta),"MARAUDER_TRIM=1\nSOURCE=%s\nFIRST_BLOCK=%lu\nLAST_BLOCK=%lu\n",name,(unsigned long)first,(unsigned long)last);
    snprintf(meta_path,sizeof(meta_path),"%s.META",out);marauder::storage_write_file(meta_path,meta,strlen(meta),false);
    snprintf(s_status,sizeof(s_status),"TRIM SAVED");return true;
}
bool save_profile() {
    if(!marauder::storage_ready()){snprintf(s_status,sizeof(s_status),"SD UNAVAILABLE");return false;}
    mkdir("/sdcard/RFPROFILES",0775);
    char names[32][64];size_t n=marauder::storage_list_files("RFPROFILES",".RFP",names,32);
    bool used[32]{};
    for(size_t i=0;i<n;++i){unsigned idx=0;if(sscanf(names[i],"RF%2u.RFP",&idx)==1&&idx<32)used[idx]=true;}
    unsigned id=0;while(id<32&&used[id])++id;if(id==32){snprintf(s_status,sizeof(s_status),"PROFILE SLOTS FULL");return false;}
    char path[40],text[256];snprintf(path,sizeof(path),"RFPROFILES/RF%02u.RFP",id);
    const auto c=settings_snapshot();
    int len=snprintf(text,sizeof(text),"MARAUDER_RF_PROFILE=1\nNAME=RF%02u\nCENTER_MHZ=%lu\nSPAN_MHZ=%u\nSTEP_MHZ=%u\nFLOOR=%u\nCEILING=%u\nTRACE_FLOOR=%u\nTRACE_CEILING=%u\nPALETTE=%u\nAVERAGING=%u\nHANN=%u\nMAX_HOLD=%u\n",id,(unsigned long)c.center_mhz,c.span_mhz,c.frequency_step,c.intensity_floor,c.intensity_ceiling,c.trace_floor,c.trace_ceiling,c.waterfall_palette,c.averaging,c.hann_window,c.max_hold);
    if(len>0&&marauder::storage_write_file(path,text,(size_t)len,false)){snprintf(s_last_profile,sizeof(s_last_profile),"RF%02u.RFP",id);snprintf(s_status,sizeof(s_status),"PROFILE SAVED");return true;}
    snprintf(s_status,sizeof(s_status),"PROFILE WRITE FAILED");return false;
}
bool load_profile(const char *name) {
    char path[96],text[256];snprintf(path,sizeof(path),"RFPROFILES/%s",name);
    size_t n=marauder::storage_read_file(path,0,text,sizeof(text)-1);if(!n)return false;text[n]=0;
    if(!strstr(text,"MARAUDER_RF_PROFILE=1\n"))return false;
    auto c=settings_snapshot();unsigned center=c.center_mhz,span=c.span_mhz,step=c.frequency_step,floorv=c.intensity_floor,ceilv=c.intensity_ceiling,palette=c.waterfall_palette,avg=c.averaging,hann=c.hann_window,hold=c.max_hold,trace_floor=20,trace_ceiling=220;
    auto read=[&](const char *key,unsigned *value)->bool{char token[32];snprintf(token,sizeof(token),"%s=",key);const char *p=strstr(text,token);if(!p)return false;p+=strlen(token);char *end=nullptr;unsigned long v=strtoul(p,&end,10);if(end==p||(*end!='\n'&&*end!='\r'&&*end!='\0')||v>UINT32_MAX)return false;*value=(unsigned)v;return true;};
    if(!read("CENTER_MHZ",&center)||!read("SPAN_MHZ",&span)||!read("STEP_MHZ",&step)||!read("FLOOR",&floorv)||!read("CEILING",&ceilv)||!read("PALETTE",&palette)||!read("AVERAGING",&avg)||!read("HANN",&hann)||!read("MAX_HOLD",&hold))return false;
    read("TRACE_FLOOR",&trace_floor);read("TRACE_CEILING",&trace_ceiling);
    c.center_mhz=center;c.span_mhz=span;c.frequency_step=step;c.intensity_floor=floorv;c.intensity_ceiling=ceilv;c.trace_floor=trace_floor;c.trace_ceiling=trace_ceiling;c.waterfall_palette=palette;c.averaging=avg;c.hann_window=hann;c.max_hold=hold;
    if(center<MARAUDER_RADIO_FREQ_MIN_MHZ||center>MARAUDER_RADIO_FREQ_MAX_MHZ||span<1||span>16||step<1||step>100||floorv>=ceilv||ceilv>255||trace_floor>=trace_ceiling||trace_ceiling>255||palette>2||avg<1||avg>8||hann>1||hold>1)return false;
    settings_commit(c);return true;
}

size_t s_home_selected=0,s_category_selected=0,s_capture_cursor=0,s_profile_cursor=0;

void adjust_settings(bool persist=true) {
    auto c=settings_snapshot();
    switch(s_category_selected){
        case 0:c.center_mhz=c.center_mhz+c.frequency_step>MARAUDER_RADIO_FREQ_MAX_MHZ?MARAUDER_RADIO_FREQ_MIN_MHZ:c.center_mhz+c.frequency_step;break;
        case 1:c.span_mhz=c.span_mhz>=16?1:c.span_mhz*2;break;
        case 2:c.frequency_step=c.frequency_step>=100?1:c.frequency_step*10;break;
        case 3:c.intensity_floor=c.intensity_floor<120?c.intensity_floor+10:0;if(c.intensity_floor>=c.intensity_ceiling)c.intensity_floor=c.intensity_ceiling-1;break;
        case 4:c.intensity_ceiling=c.intensity_ceiling<=230?c.intensity_ceiling+10:255;if(c.intensity_ceiling<=c.intensity_floor)c.intensity_ceiling=255;break;
        case 5:c.waterfall_palette=(c.waterfall_palette+1)%3;break;
        case 6:c.averaging=c.averaging>=8?1:c.averaging+1;break;
        case 7:c.hann_window=!c.hann_window;break;
        case 8:c.max_hold=!c.max_hold;break;
        case 9:s_radio_paused.store(!s_radio_paused.load());c.paused=s_radio_paused.load();break;
        case 10:c.trace_floor=c.trace_floor<120?c.trace_floor+10:0;if(c.trace_floor>=c.trace_ceiling)c.trace_floor=c.trace_ceiling-1;break;
        case 11:c.trace_ceiling=c.trace_ceiling<=230?c.trace_ceiling+10:255;if(c.trace_ceiling<=c.trace_floor)c.trace_ceiling=255;break;
        case 12:save_profile();break;
        case 13:c=marauder::settings_defaults();s_radio_paused.store(false);break;
        case 14:c.joystick_repeat_delay_ms=c.joystick_repeat_delay_ms>=800?100:c.joystick_repeat_delay_ms+100;break;
        case 15:c.joystick_repeat_period_ms=c.joystick_repeat_period_ms<=60?250:c.joystick_repeat_period_ms-30;break;
        case 16:c.joystick_long_press_ms=c.joystick_long_press_ms>=1200?350:c.joystick_long_press_ms+100;break;
        case 17:c.joystick_acceleration=(c.joystick_acceleration+1)%4;break;
        case 18:c.joystick_invert_vertical=!c.joystick_invert_vertical;break;
        case 19:c.joystick_invert_horizontal=!c.joystick_invert_horizontal;break;
        case 20:c.joystick_debounce_ms=c.joystick_debounce_ms>=90?5:c.joystick_debounce_ms+5;break;
        default:return;
    }
    settings_commit(c,persist);s_avg_initialized=false;
}

void foxtx_enter();
void execute_detail_action(const Item *items,size_t count) {
    if(s_category==Category::RECEIVE){
        // Menu order: apps first, setup last. Map the menu position back to the original action numbers.
        static const uint8_t kReceiveAction[]={0,4,5,6,7,1,2,3};
        switch(s_category_selected<sizeof(kReceiveAction)?kReceiveAction[s_category_selected]:99){
            case 0:s_detail=Detail::SPECTRUM;break;
            case 1:s_detail=Detail::FREQ;break;
            case 2:s_detail=Detail::GAIN;break;
            case 3:s_dsp_cursor=0;s_dsp_group=-1;s_dsp_menu=0;s_dsp_clear=true;s_detail=Detail::DSP;break;
            case 4:s_foxhunt_frequency.store(settings_snapshot().center_mhz,std::memory_order_relaxed);s_foxhunt_level=s_foxhunt_peak_level=0;s_foxhunt_peak_offset_khz=0;s_foxhunt_step_index=0;s_recon_active.store(false);s_looking_glass_active.store(false);s_replay_active.store(false);s_radio_paused.store(false);s_foxhunt_active.store(true);s_detail=Detail::FOXHUNT;break;
            case 5:s_glass_setting=0;s_glass_editing=false;s_glass_paused.store(false,std::memory_order_relaxed);s_radio_paused.store(false,std::memory_order_relaxed);memset(s_glass_peak_bins,0,sizeof(s_glass_peak_bins));for(auto &entry:s_glass_quick_cache)entry.valid=false;s_looking_glass_active.store(true);s_detail=Detail::SPECTRUM;break;
            case 6:s_detail=Detail::SCANNER;break;
            case 7:{const auto c=settings_snapshot();s_rssi_detector_frequency.store(c.center_mhz,std::memory_order_relaxed);memset(s_rssi_history,0,sizeof(s_rssi_history));s_rssi_history_count=s_rssi_history_head=0;s_rssi_level=0;s_rssi_triggered=false;s_rssi_detector_enabled=true;s_rssi_detector_active.store(true,std::memory_order_relaxed);s_detail=Detail::RSSI_GRAPH;break;}
            default:s_page=Page::HOME;break;
        }
        if(s_detail!=Detail::NONE)s_page=Page::DETAIL;
        return;
    }
    if(s_category==Category::TRANSMIT){if(s_category_selected==0){foxtx_enter();s_detail=Detail::FOXTX;s_page=Page::DETAIL;}else s_page=Page::HOME;return;}
    if(s_category==Category::RECON){if(s_category_selected==0){s_detail=Detail::SCANNER;s_page=Page::DETAIL;}else if(s_category_selected==1){s_detail=Detail::SPECTRUM;s_page=Page::DETAIL;}else s_page=Page::HOME;return;}
    if(s_category==Category::CAPTURE){if(s_category_selected==0){bool n=!s_capture_requested.load();s_capture_requested.store(n);if(!n&&s_capture_active.exchange(false))marauder::storage_capture_end();snprintf(s_status,sizeof(s_status),n?"CAPTURE START":"CAPTURE STOP");}else if(s_category_selected==1){s_detail=Detail::CAPTURE_FILES;s_page=Page::DETAIL;}else s_page=Page::HOME;return;}
    if(s_category==Category::REPLAY){if(s_category_selected==0){s_replay_cursor=0;select_replay(0);s_detail=Detail::REPLAY;s_page=Page::DETAIL;}else s_page=Page::HOME;return;}
    if(s_category==Category::RF_REPLAY){if(s_category_selected==0){s_rf_cursor=0;s_detail=Detail::RF_REPLAY;s_page=Page::DETAIL;}else if(s_category_selected==1){s_rf_config_cursor=0;s_detail=Detail::RF_CONFIGS;s_page=Page::DETAIL;}else s_page=Page::HOME;return;}
    if(s_category==Category::TX_TEST){if(s_category_selected==0){s_tx_armed=false;s_detail=Detail::TX_TEST;s_page=Page::DETAIL;}else s_page=Page::HOME;return;}
    if(s_category==Category::GPS){if(s_category_selected<3){if(s_category_selected>0&&!marauder::storage_accessible())marauder::storage_retry_mount();s_gps_view=s_category_selected;s_gps_cursor=0;s_detail=Detail::GPS;s_page=Page::DETAIL;}else if(s_category_selected<6){marauder::map_ui_set_radio_hook(map_radio_hook);s_detail=s_category_selected==3?Detail::MAP_VIEW:s_category_selected==4?Detail::NAV_VIEW:Detail::RID_VIEW;marauder::map_ui_open(s_category_selected==3?marauder::MapScreen::MAP:s_category_selected==4?marauder::MapScreen::NAV:marauder::MapScreen::RID,s_gps_imperial_units);s_page=Page::DETAIL;}else s_page=Page::HOME;return;}
    if(s_category==Category::LOOKING_GLASS){if(s_category_selected==0){s_glass_setting=0;s_glass_editing=false;s_glass_paused.store(false,std::memory_order_relaxed);s_radio_paused.store(false,std::memory_order_relaxed);memset(s_glass_peak_bins,0,sizeof(s_glass_peak_bins));for(auto &entry:s_glass_quick_cache)entry.valid=false;s_looking_glass_active.store(true);s_detail=Detail::SPECTRUM;s_page=Page::DETAIL;}else s_page=Page::HOME;return;}
    if(s_category==Category::PROFILES){if(s_category_selected==0){s_detail=Detail::PROFILES;s_page=Page::DETAIL;}else s_page=Page::HOME;return;}
    if(s_category==Category::UTILITIES){static const Detail d[]={Detail::NOTEPAD,Detail::CALCULATOR,Detail::STOPWATCH,Detail::ANTENNA,Detail::RANDPWD,Detail::FILES,Detail::FREQ_MANAGER,Detail::IQ_TRIM,Detail::FLASH,Detail::DEBUG,Detail::STORAGE,Detail::DEVICE,Detail::NONE};s_detail=d[s_category_selected];if(s_detail==Detail::NOTEPAD)load_note();if(s_detail==Detail::ANTENNA)s_antenna_mode=0;if(s_detail==Detail::STOPWATCH){s_stopwatch_elapsed_ms=0;s_stopwatch_running=false;}if(s_detail==Detail::RANDPWD)make_password();if(s_detail==Detail::CALCULATOR){s_calc_lhs=s_calc_rhs=0;s_calc_op=-1;s_calc_key=0;s_calc_rhs_mode=s_calc_error=false;strcpy(s_calc_result,"0");}if(s_detail==Detail::FILES)br_open();if(s_detail==Detail::NONE)s_page=Page::HOME;else s_page=Page::DETAIL;return;}
    if(s_category==Category::EXTERNALS){if(s_category_selected==0){s_detail=Detail::EXTERNALS;s_page=Page::DETAIL;}else s_page=Page::HOME;return;}
    if(s_category==Category::DIAGNOSTICS){if(s_category_selected==0){s_detail=Detail::DIAG;s_page=Page::DETAIL;}else if(s_category_selected==1){s_detail=Detail::DEVICE;s_page=Page::DETAIL;}else s_page=Page::HOME;}
    if(s_category==Category::FOXHUNT){if(s_category_selected==0){auto c=settings_snapshot();s_foxhunt_frequency.store(c.center_mhz,std::memory_order_relaxed);s_foxhunt_level=s_foxhunt_peak_level=0;s_foxhunt_peak_offset_khz=0;s_foxhunt_step_index=0;s_recon_active.store(false);s_looking_glass_active.store(false);s_replay_active.store(false);s_radio_paused.store(false);s_foxhunt_active.store(true);s_detail=Detail::FOXHUNT;s_page=Page::DETAIL;}else s_page=Page::HOME;return;}
    (void)items;(void)count;
}

void handle_detail_event(bool short_press,bool long_press,const Item *items,size_t n) {
    if(s_detail==Detail::SETTINGS){
        if(short_press)s_category_selected=(s_category_selected+1)%22;
        else if(long_press){if(s_category_selected==0){s_page=Page::HOME;s_detail=Detail::NONE;}else{--s_category_selected;adjust_settings();++s_category_selected;}}
        return;
    }
    if(s_detail==Detail::SPECTRUM){if(long_press){s_looking_glass_active.store(false);s_page=Page::CATEGORY;s_detail=Detail::NONE;}else if(short_press){auto c=settings_snapshot();c.center_mhz=c.center_mhz+c.frequency_step>MARAUDER_RADIO_FREQ_MAX_MHZ?MARAUDER_RADIO_FREQ_MIN_MHZ:c.center_mhz+c.frequency_step;settings_commit(c);}return;}
    if(s_detail==Detail::FREQ){if(long_press){s_page=Page::CATEGORY;s_detail=Detail::NONE;}else if(short_press){auto c=settings_snapshot();c.center_mhz=c.center_mhz+c.frequency_step>MARAUDER_RADIO_FREQ_MAX_MHZ?MARAUDER_RADIO_FREQ_MIN_MHZ:c.center_mhz+c.frequency_step;settings_commit(c);}return;}
    if(s_detail==Detail::GAIN||s_detail==Detail::FLASH||s_detail==Detail::DEBUG||s_detail==Detail::STORAGE||s_detail==Detail::DEVICE||s_detail==Detail::DIAG){if(long_press){s_page=Page::CATEGORY;s_detail=Detail::NONE;}return;}
    if(s_detail==Detail::SCANNER){if(short_press)s_recon_active.store(!s_recon_active.load());if(long_press){s_recon_active.store(false);s_page=Page::CATEGORY;s_detail=Detail::NONE;}return;}
    if(s_detail==Detail::CAPTURE_FILES){static char names[24][64];size_t nfiles=list_capture_names(names,24);const size_t choices=nfiles+1;if(short_press){s_capture_delete_confirm=false;s_capture_cursor=(s_capture_cursor+choices+s_nav_dir)%choices;}else if(long_press){if(!nfiles||s_capture_cursor>=nfiles){s_page=Page::CATEGORY;s_detail=Detail::NONE;}else if(!s_capture_delete_confirm){s_capture_delete_confirm=true;snprintf(s_status,sizeof(s_status),"HOLD AGAIN TO DELETE");}else{char path[64],meta[72];snprintf(path,sizeof(path),"%s",names[s_capture_cursor%nfiles]);snprintf(meta,sizeof(meta),"%s.META",path);bool ok=marauder::storage_delete_file(path);marauder::storage_delete_file(meta);s_capture_delete_confirm=false;snprintf(s_status,sizeof(s_status),ok?"CAPTURE DELETED":"DELETE FAILED");s_page=Page::CATEGORY;s_detail=Detail::NONE;}}return;}
    if(s_detail==Detail::REPLAY){static char names[24][64];size_t nfiles=list_capture_names(names,24);if(short_press&&nfiles){s_replay_cursor=(s_replay_cursor+nfiles+s_nav_dir)%nfiles;select_replay(s_replay_cursor);}else if(long_press){if(s_replay_active.load())s_replay_active.store(false);else{char name[64];copy_replay_name(name);if(!name[0]&&!start_replay_next())return;copy_replay_name(name);if(name[0]&&marauder::storage_file_size(name)>=512)s_replay_active.store(true);}}return;}
    if(s_detail==Detail::RF_REPLAY){static char names[24][64];size_t nfiles=list_capture_names(names,24);if(short_press&&nfiles)s_rf_cursor=(s_rf_cursor+nfiles+s_nav_dir)%nfiles;else if(long_press&&nfiles){const char *name=names[s_rf_cursor%nfiles];char cfgpath[80];snprintf(cfgpath,sizeof(cfgpath),"%s",name);char*ext=strrchr(cfgpath,'.');if(ext)strcpy(ext,".RRC");char saved[192];snprintf(saved,sizeof(saved),"MARAUDER_RF_REPLAY_CONFIG=1\nCAPTURE=%s\nMODE=RF_TX\nTRANSMIT_ALLOWED=0\nTEST_ENVIRONMENT=CONTROLLED_LAB_ONLY\n",name);bool ok=marauder::storage_write_file(cfgpath,saved,strlen(saved),false);snprintf(s_status,sizeof(s_status),ok?"CONFIG SAVED; TX UNSUPPORTED":"SD WRITE FAILED");}return;}
    if(s_detail==Detail::RF_CONFIGS){static char names[24][64];size_t n=marauder::storage_list_files(".",".RRC",names,24);if(short_press&&n)s_rf_config_cursor=(s_rf_config_cursor+n+s_nav_dir)%n;return;}
    if(s_detail==Detail::GPS){auto fix=marauder::gps_current_fix();const uint64_t now=(uint64_t)esp_timer_get_time();if(s_gps_view==0){if(short_press)s_gps_cursor=(s_gps_cursor+8+s_nav_dir)%8;else if(long_press){switch(s_gps_cursor){case 0:if(!marauder::storage_gps_track_active()){if(marauder::storage_gps_track_start(s_gps_track_name))snprintf(s_status,sizeof(s_status),"GPS LOG STARTED");else snprintf(s_status,sizeof(s_status),"SD LOG START FAILED");}else{marauder::storage_gps_track_pause(false);snprintf(s_status,sizeof(s_status),"GPS LOG RESUMED");}break;case 1:if(marauder::storage_gps_track_active()){marauder::storage_gps_track_pause(true);snprintf(s_status,sizeof(s_status),"GPS LOG PAUSED");}break;case 2:marauder::storage_gps_track_stop();snprintf(s_status,sizeof(s_status),"GPS LOG STOPPED");break;case 3:if(marauder::storage_gps_save_location(s_gps_track_name,fix))snprintf(s_status,sizeof(s_status),"LOCATION SAVED");else snprintf(s_status,sizeof(s_status),fix.valid?"LOCATION SAVE FAILED":"NO VALID FIX");break;case 4:s_gps_name_editing=!s_gps_name_editing;snprintf(s_status,sizeof(s_status),s_gps_name_editing?"EDIT NAME: UP/DN CHAR":"NAME EDIT DONE");break;case 5:s_gps_coordinate_format=(s_gps_coordinate_format+1)%3;gps_ui_preferences(true);break;case 6:s_gps_imperial_units=!s_gps_imperial_units;gps_ui_preferences(true);break;case 7:snprintf(s_status,sizeof(s_status),fix.valid?"POSITION REFRESHED":"NO RECENT FIX");break;default:break;}}if(marauder::storage_gps_track_active()&&fix.valid&&fix.last_fix_us>s_gps_log_last_fix_us&&now-s_gps_log_last_us>=1000000){s_gps_log_last_us=now;if(!marauder::storage_gps_track_append(fix))snprintf(s_status,sizeof(s_status),"GPS LOG WRITE ERROR");else s_gps_log_last_fix_us=fix.last_fix_us;}}else if(s_gps_view==1){static char tracks[32][64];size_t nt=marauder::storage_list_files("GPS/TRACKS",".CSV",tracks,32);if(short_press&&nt){if(s_gps_point_review){char path[96];snprintf(path,sizeof(path),"GPS/TRACKS/%s",tracks[s_gps_cursor%nt]);auto summary=summarize_track(path,marauder::storage_file_size(path));if(summary.points)s_gps_point_index=(s_gps_point_index+summary.points+s_nav_dir)%summary.points;}else{s_gps_cursor=(s_gps_cursor+nt+s_nav_dir)%nt;s_gps_point_index=0;s_gps_delete_confirm=false;}}else if(long_press&&nt)s_gps_point_review=!s_gps_point_review;}else{char loc[1024]{};size_t z=marauder::storage_read_file("GPS/LOCATIONS.CSV",0,loc,sizeof(loc)-1);size_t count=0;for(size_t i=0;i<z;++i)if(loc[i]==10)++count;if(short_press&&count){s_gps_cursor=(s_gps_cursor+count+s_nav_dir)%count;s_gps_delete_confirm=false;}else if(long_press&&count){if(!s_gps_delete_confirm){s_gps_delete_confirm=true;snprintf(s_status,sizeof(s_status),"CENTER AGAIN TO DELETE");}else{bool ok=marauder::storage_gps_delete_location((uint32_t)s_gps_cursor);s_gps_delete_confirm=false;snprintf(s_status,sizeof(s_status),ok?"LOCATION DELETED":"DELETE FAILED");}}}return;}
    if(s_detail==Detail::PROFILES){static char names[32][64];size_t nfiles=marauder::storage_list_files("RFPROFILES",".RFP",names,32);const size_t action_count=2+nfiles*3;if(short_press){s_profile_delete_confirm=false;s_profile_cursor=(s_profile_cursor+action_count+s_nav_dir)%action_count;}else if(long_press){if(s_profile_cursor==0)save_profile();else if(s_profile_cursor==action_count-1){s_page=Page::CATEGORY;s_detail=Detail::NONE;}else{size_t ix=(s_profile_cursor-1)/3;unsigned op=(unsigned)((s_profile_cursor-1)%3);if(ix<nfiles){snprintf(s_last_profile,sizeof(s_last_profile),"%s",names[ix]);if(op==0){if(load_profile(s_last_profile))snprintf(s_status,sizeof(s_status),"PROFILE LOADED");else snprintf(s_status,sizeof(s_status),"INVALID PROFILE");}else if(op==1){if(load_profile(s_last_profile))save_profile();else snprintf(s_status,sizeof(s_status),"INVALID PROFILE");}else if(!s_profile_delete_confirm){s_profile_delete_confirm=true;snprintf(s_status,sizeof(s_status),"HOLD AGAIN TO DELETE");}else{char path[96];snprintf(path,sizeof(path),"RFPROFILES/%s",s_last_profile);if(marauder::storage_delete_file(path))snprintf(s_status,sizeof(s_status),"PROFILE DELETED");else snprintf(s_status,sizeof(s_status),"DELETE FAILED");s_profile_delete_confirm=false;}}}}return;}
    if(s_detail==Detail::IQ_TRIM){static char names[24][64];size_t nfiles=list_capture_names(names,24);if(short_press)s_capture_cursor=(s_capture_cursor+nfiles+1+s_nav_dir)%(nfiles+1);else if(long_press){if(s_capture_cursor%(nfiles+1)==nfiles){s_page=Page::CATEGORY;s_detail=Detail::NONE;}else if(nfiles)trim_capture(names[s_capture_cursor%nfiles]);}return;}
    if(s_detail==Detail::NOTEPAD){
        if(short_press){s_note_char=(s_note_char+29+s_nav_dir)%29;return;}
        if(long_press){
            if(s_note_char==28){s_page=Page::CATEGORY;s_detail=Detail::NONE;return;}
            if(s_note_char==26){if(s_note_len){--s_note_len;s_note[s_note_len]=0;persist_note();}}
            else if(s_note_char==27){if(s_note_len<sizeof(s_note)-1){s_note[s_note_len++]=' ';s_note[s_note_len]=0;persist_note();}}
            else if(s_note_len<sizeof(s_note)-1){s_note[s_note_len++]=(char)(s_note_char<26?'A'+s_note_char:'?');s_note[s_note_len]=0;persist_note();}
            if(s_note_len>s_note_scroll+220)s_note_scroll=s_note_len-220;
        }
        return;
    }
    if(s_detail==Detail::EXTERNALS){if(short_press)s_external_cursor=(s_external_cursor+24+s_nav_dir)%24;else if(long_press){s_page=Page::CATEGORY;s_detail=Detail::NONE;}return;}
    if(s_detail==Detail::ANTENNA){if(short_press)s_antenna_mode=(s_antenna_mode+1)%3;else if(long_press){s_page=Page::CATEGORY;s_detail=Detail::NONE;}return;}
    if(s_detail==Detail::STOPWATCH){const uint64_t now=(uint64_t)esp_timer_get_time()/1000;if(short_press){if(s_stopwatch_running){s_stopwatch_elapsed_ms+=now-s_stopwatch_started_ms;s_stopwatch_running=false;}else{s_stopwatch_started_ms=now;s_stopwatch_running=true;}}else if(long_press){if(s_stopwatch_running)s_stopwatch_elapsed_ms+=now-s_stopwatch_started_ms;s_stopwatch_running=false;s_page=Page::CATEGORY;s_detail=Detail::NONE;}return;}
    if(s_detail==Detail::RANDPWD){if(short_press)make_password();else if(long_press){s_page=Page::CATEGORY;s_detail=Detail::NONE;}return;}
    if(s_detail==Detail::CALCULATOR){
        if(short_press){s_calc_key=(s_calc_key+1)%17;return;}
        if(long_press){if(s_calc_key<10){int64_t &value=s_calc_rhs_mode?s_calc_rhs:s_calc_lhs;if(value<100000000)value=value*10+(int64_t)s_calc_key;s_calc_error=false;snprintf(s_calc_result,sizeof(s_calc_result),"%lld",(long long)value);}
            else if(s_calc_key<14){s_calc_op=(int)s_calc_key-10;s_calc_rhs_mode=true;}
            else if(s_calc_key==14&&s_calc_op>=0){if(s_calc_op==3&&s_calc_rhs==0)s_calc_error=true;else{int64_t result=s_calc_op==0?s_calc_lhs+s_calc_rhs:s_calc_op==1?s_calc_lhs-s_calc_rhs:s_calc_op==2?s_calc_lhs*s_calc_rhs:s_calc_lhs/s_calc_rhs;snprintf(s_calc_result,sizeof(s_calc_result),"%lld",(long long)result);}}
            else if(s_calc_key==15){s_calc_lhs=s_calc_rhs=0;s_calc_op=-1;s_calc_rhs_mode=s_calc_error=false;snprintf(s_calc_result,sizeof(s_calc_result),"0");}
            else if(s_calc_key==16){s_page=Page::CATEGORY;s_detail=Detail::NONE;return;}
        }
        return;
    }
    if(s_detail==Detail::FREQ_MANAGER){if(short_press){auto c=settings_snapshot();c.center_mhz=c.center_mhz+5>MARAUDER_RADIO_FREQ_MAX_MHZ?MARAUDER_RADIO_FREQ_MIN_MHZ:c.center_mhz+5;settings_commit(c);}if(long_press){s_page=Page::CATEGORY;s_detail=Detail::NONE;}return;}
    (void)items;(void)n;
}


// ---- Transmitter / fox beacon page ---------------------------------------
// Settings are hot: while the beacon runs, changing a row applies immediately.
marauder::FoxConfig s_fox{};
bool s_fox_loaded{},s_fox_armed{},s_fox_rx_paused{};
int s_fox_edit{-1}; // 0 = message text, 1 = callsign, -1 = not editing
unsigned s_fox_freq_step_index{};
size_t s_fox_cursor{};
char s_fox_note[32]{};
constexpr size_t kFoxRows=13;
constexpr uint16_t kFoxIntervals[]={200,500,1000,1500,2000,3000,5000,10000};
constexpr uint16_t kFoxDurations[]={0,1,5,10,30,60,120};
constexpr uint16_t kFoxIdEvery[]={0,60,120,300,600};
constexpr uint16_t kFoxFreqSteps[]={5,20,100};
constexpr char kFoxCharset[]="ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789/";

template<size_t N> uint16_t step_table(const uint16_t (&table)[N],uint16_t current,int dir){
    size_t i=0;while(i+1<N&&table[i]<current)++i;
    if(table[i]!=current&&dir>0&&i>0&&table[i]>current)--i; // between entries: step from the lower one
    return table[(i+N+(dir>0?1:N-1))%N];
}
void foxtx_enter(){
    if(!s_fox_loaded){marauder::fox_config_load(&s_fox);s_fox_loaded=true;}
    marauder::fox_files_load();
    s_fox_cursor=12;s_fox_armed=false;s_fox_edit=-1;s_fox_note[0]=0;
    {const esp_reset_reason_t rr=esp_reset_reason();if(rr==ESP_RST_PANIC||rr==ESP_RST_INT_WDT||rr==ESP_RST_TASK_WDT||rr==ESP_RST_WDT||rr==ESP_RST_BROWNOUT)snprintf(s_fox_note,sizeof(s_fox_note),"LAST RESET: %s",rr==ESP_RST_PANIC?"PANIC":rr==ESP_RST_BROWNOUT?"BROWNOUT":"WATCHDOG");}
}
void foxtx_release_rx(){if(s_fox_rx_paused){s_radio_paused.store(settings_snapshot().paused);s_fox_rx_paused=false;}}
void foxtx_stop(){marauder::fox_stop();foxtx_release_rx();s_fox_armed=false;}
void foxtx_leave(){foxtx_stop();s_fox_edit=-1;marauder::fox_config_save(s_fox);}
void foxtx_poll(){ // the transmitter can end by itself (duration, error)
    if(s_fox_rx_paused&&!marauder::fox_status().running){
        const auto st=marauder::fox_status();
        snprintf(s_fox_note,sizeof(s_fox_note),st.last_err==ESP_OK?"TX ENDED":"TX ERR %.20s",esp_err_to_name(st.last_err));
        foxtx_release_rx();s_fox_armed=false;
    }
}
void foxtx_apply_live(){if(marauder::fox_status().running)marauder::fox_update(s_fox);}
void foxtx_event(marauder::InputKey key,bool confirm){
    using marauder::InputKey;
    const bool running=marauder::fox_status().running;
    s_fox_note[0]=0;
    if(s_fox_edit>=0){
        char *field=s_fox_edit==0?s_fox.text:s_fox.call;
        size_t n=strlen(field);
        if(confirm){s_fox_edit=-1;marauder::fox_config_sanitize(s_fox);foxtx_apply_live();return;}
        if(key==InputKey::UP||key==InputKey::DOWN){
            if(!n){field[0]='A';field[1]=0;return;}
            const char *p=strchr(kFoxCharset,field[n-1]);
            const size_t count=sizeof(kFoxCharset)-1;size_t i=p?(size_t)(p-kFoxCharset):0;
            i=(i+(key==InputKey::UP?1:count-1))%count;field[n-1]=kFoxCharset[i];
        }else if(key==InputKey::RIGHT){if(n<marauder::kFoxTextMax){field[n]='A';field[n+1]=0;}}
        else if(key==InputKey::LEFT){if(n)field[n-1]=0;}
        return;
    }
    if(key==InputKey::UP||key==InputKey::DOWN){s_fox_armed=false;s_fox_cursor=(s_fox_cursor+(key==InputKey::UP?kFoxRows-1:1))%kFoxRows;return;}
    if(confirm){
        if(s_fox_cursor==1){s_fox_freq_step_index=(s_fox_freq_step_index+1)%(sizeof(kFoxFreqSteps)/sizeof(*kFoxFreqSteps));return;}
        if(s_fox_cursor==7){if(marauder::fox_file_message_count())snprintf(s_fox_note,sizeof(s_fox_note),"MESSAGES COME FROM FILE");else s_fox_edit=0;return;}
        if(s_fox_cursor==8){if(marauder::fox_callsign_from_file())snprintf(s_fox_note,sizeof(s_fox_note),"CALLSIGN COMES FROM FILE");else s_fox_edit=1;return;}
        if(s_fox_cursor==12){
            if(running){foxtx_stop();snprintf(s_fox_note,sizeof(s_fox_note),"TX STOPPED");return;}
            marauder::fox_config_sanitize(s_fox);
            if(s_fox.mode==(uint8_t)marauder::FoxMode::MORSE&&!marauder::fox_file_message_count()&&!s_fox.text[0]){snprintf(s_fox_note,sizeof(s_fox_note),"NO MESSAGE (morse.txt)");return;}
            if(!s_fox_armed){s_fox_armed=true;ESP_LOGI(kTag,"fox TX armed");snprintf(s_fox_note,sizeof(s_fox_note),"ARMED: CENTER AGAIN");return;}
            ESP_LOGI(kTag,"fox TX start requested");
            marauder::fox_config_save(s_fox);
            s_radio_paused.store(true);s_fox_rx_paused=true;
            const esp_err_t e=marauder::fox_start(s_fox);
            if(e!=ESP_OK){foxtx_release_rx();snprintf(s_fox_note,sizeof(s_fox_note),"START ERR %.20s",esp_err_to_name(e));}
            s_fox_armed=false;
        }
        snprintf(s_fox_note,sizeof(s_fox_note),"SELECT ACTION ROW TO START");
        return;
    }
    if(key!=InputKey::LEFT&&key!=InputKey::RIGHT)return;
    s_fox_armed=false;
    const int dir=key==InputKey::RIGHT?1:-1;
    auto &f=s_fox;
    switch(s_fox_cursor){
        case 0:f.mode=(uint8_t)(((int)f.mode+(int)marauder::FoxMode::COUNT+dir)%(int)marauder::FoxMode::COUNT);break;
        case 1:f.freq_mhz=(uint16_t)std::clamp<int>((int)f.freq_mhz+dir*(int)kFoxFreqSteps[s_fox_freq_step_index],(int)marauder::kFoxFreqMin,(int)marauder::kFoxFreqMax);break;
        case 2:f.power=(uint8_t)std::clamp<int>((int)f.power+dir,0,(int)marauder::kFoxPowerMax);break;
        case 3:f.hop_set=(uint8_t)(((int)f.hop_set+(int)marauder::fox_hop_set_count()+dir)%(int)marauder::fox_hop_set_count());break;
        case 4:f.interval_ms=step_table(kFoxIntervals,f.interval_ms,dir);break;
        case 5:f.duty_pct=(uint8_t)std::clamp<int>((int)f.duty_pct+dir*10,10,90);break;
        case 6:f.duration_min=step_table(kFoxDurations,f.duration_min,dir);break;
        case 9:f.wpm=(uint8_t)std::clamp<int>((int)f.wpm+dir,5,30);break;
        case 10:f.id_every_s=step_table(kFoxIdEvery,f.id_every_s,dir);break;
        case 11:f.random_order=f.random_order?0:1;break;
        default:return;
    }
    foxtx_apply_live();
}

constexpr int kDspPages=10; // 0 basic, 1..6 categories, 7 signals, 8 IQ, 9 performance
void dsp_row_text(int row,const char *text,bool selected){
    char line[31];snprintf(line,sizeof(line),"%c%-29.29s",selected?'>':' ',text);
    marauder::display_text(0,36+row*11,line,selected?C_WHITE:C_CYAN,selected?0x21A7:C_BLACK);
}
void dsp_event(marauder::InputKey key,bool confirm,bool repeat){
    using K=marauder::InputKey;
    if(s_dsp_group<0){
        if(key==K::UP||key==K::DOWN)s_dsp_menu=(s_dsp_menu+(key==K::UP?kDspPages-1:1))%kDspPages;
        else if(key==K::RIGHT||confirm){s_dsp_group=(int)s_dsp_menu;s_dsp_cursor=0;s_dsp_sig_cursor=0;s_dsp_clear=true;}
        else if(key==K::LEFT){marauder::dsp_cfg_save_if_dirty();s_page=Page::CATEGORY;s_detail=Detail::NONE;}
        return;
    }
    if(s_dsp_group==0){ // original five settings, unchanged behaviour
        if(confirm){s_dsp_group=-1;s_dsp_clear=true;}
        else if(key==K::UP||key==K::DOWN){s_dsp_cursor=(s_dsp_cursor+(key==K::UP?4:1))%5;}
        else if(key==K::LEFT||key==K::RIGHT){
            const int direction=key==K::RIGHT?1:-1;
            auto c=settings_snapshot();
            switch(s_dsp_cursor){
                case 0:{const int current=s_spectrum_filter.load(std::memory_order_relaxed);spectrum_filter_set((uint8_t)((current+kSpectrumFilterCount+direction)%kSpectrumFilterCount));break;}
                case 1:{static constexpr uint8_t values[]={1,2,4,8};size_t index=0;while(index+1<4&&values[index]!=c.averaging)++index;index=(index+(direction>0?1:3))%4;c.averaging=values[index];settings_commit(c);s_avg_initialized=false;break;}
                case 2:c.hann_window=!c.hann_window;settings_commit(c);s_avg_initialized=false;break;
                case 3:c.max_hold=!c.max_hold;settings_commit(c);break;
                case 4:trace_autofit_set(!s_trace_autofit.load(std::memory_order_relaxed));break;
                default:break;
            }
        }
        return;
    }
    if(s_dsp_group>=1&&s_dsp_group<=6){
        const unsigned g=(unsigned)s_dsp_group-1;const size_t count=marauder::dsp_item_count(g);
        if(key==K::UP||key==K::DOWN)s_dsp_cursor=(s_dsp_cursor+(key==K::UP?count:1))%(count+1);
        else if(key==K::LEFT||key==K::RIGHT){if(s_dsp_cursor<count)marauder::dsp_item_adjust(g,s_dsp_cursor,key==K::RIGHT?1:-1,repeat?4:1);}
        else if(confirm){
            if(s_dsp_cursor==count){for(size_t i=0;i<count;++i)marauder::dsp_item_reset(g,i);}
            else{marauder::dsp_cfg_save_if_dirty();s_dsp_group=-1;s_dsp_clear=true;}
        }
        return;
    }
    if(s_dsp_group==7&&(key==K::UP||key==K::DOWN)){s_dsp_sig_cursor=(s_dsp_sig_cursor+(key==K::UP?7:1))%8;return;}
    if(confirm||key==K::LEFT){s_dsp_group=-1;s_dsp_clear=true;}
}
void draw_dsp(bool screen_changed){
    static int drawn_group=-2;
    const bool clear=screen_changed||s_dsp_clear||drawn_group!=s_dsp_group;
    if(clear){marauder::display_fill_rect(0,32,240,264,C_BLACK);s_dsp_clear=false;drawn_group=s_dsp_group;}
    const char *footer=s_dsp_group<0?"U/D PICK  C/R OPEN  L BACK":(s_dsp_group>=1&&s_dsp_group<=6)?"U/D ROW L/R CHANGE C BACK":s_dsp_group==0?"U/D FOCUS L/R CHANGE C BACK":"C / L = BACK";
    if(screen_changed)draw_detail_frame(footer);else draw_footer(footer);
    char line[48],val[24];
    if(s_dsp_group<0){
        draw_header("DSP / SPECTRUM","",screen_changed);
        static const char *names[kDspPages]={"BASIC (ORIGINAL 5)","FFT / WINDOW","FREQUENCY FILTERS","NOISE REDUCTION","SIGNAL DETECTION","TEMPORAL PROCESSING","DISPLAY PROCESSING","SIGNALS (LIVE)","IQ STATS (LIVE)","PERFORMANCE (LIVE)"};
        for(int i=0;i<kDspPages;++i){
            if(i>=1&&i<=6){const unsigned m=marauder::dsp_group_modified((unsigned)i-1);if(m)snprintf(line,sizeof(line),"%-21.21s %2u SET",names[i],m);else snprintf(line,sizeof(line),"%s",names[i]);}
            else snprintf(line,sizeof(line),"%s",names[i]);
            dsp_row_text(i,line,(size_t)i==s_dsp_menu);
        }
        draw_line(12,"LEVELS = RELATIVE FFT MAGNITUDE");
        draw_line(13,"NOT CALIBRATED RF POWER (NO dBm)");
        draw_line(14,"DEFAULTS = ORIGINAL BEHAVIOUR");
        draw_line(15,"SETTINGS SAVE AND APPLY LIVE");
        return;
    }
    if(s_dsp_group==0){
        draw_header("DSP: BASIC","",screen_changed);
        const auto c=settings_snapshot();
        const char *names[]={"BIN SMOOTHING","TEMPORAL AVERAGE","FFT WINDOW (LEGACY)","PEAK HOLD","TRACE AUTO FIT"};
        const char *values[]={spectrum_filter_name(s_spectrum_filter.load(std::memory_order_relaxed)),
                              c.averaging==1?"1 FRAME":c.averaging==2?"2 FRAMES":c.averaging==4?"4 FRAMES":"8 FRAMES",
                              c.hann_window?"HANN":"RECTANGULAR",c.max_hold?"ON":"OFF",
                              s_trace_autofit.load(std::memory_order_relaxed)?"ON":"MANUAL"};
        static uint32_t row_sig[5];
        for(size_t i=0;i<5;++i){
            const int y=42+(int)i*27;const bool selected=i==s_dsp_cursor;
            uint32_t sig=selected?2166136261u:16777619u;
            for(const char *q=values[i];*q;++q)sig=(sig^(uint8_t)*q)*16777619u;
            if(!clear&&row_sig[i]==sig)continue;
            row_sig[i]=sig;
            marauder::display_fill_rect(3,y,234,22,selected?C_CYAN:C_PANEL);
            marauder::display_fill_rect(4,y+1,232,20,selected?C_NAVY:C_BLACK);
            marauder::display_text(9,y+2,names[i],selected?C_CYAN:C_WHITE,selected?C_NAVY:C_BLACK);
            marauder::display_text(9,y+11,values[i],selected?C_WHITE:C_AMBER,selected?C_NAVY:C_BLACK);
        }
        draw_line(16,"AUTO FIT: ROBUST 5-98% RANGE");
        draw_line(17,"SCALE SMOOTHED BETWEEN FRAMES");
        draw_line(19,"MANUAL USES TRACE FLOOR/CEIL");
        return;
    }
    if(s_dsp_group<=6){
        const unsigned g=(unsigned)s_dsp_group-1;const size_t count=marauder::dsp_item_count(g);
        draw_header(marauder::dsp_group_name(g),"",screen_changed);
        for(size_t i=0;i<count;++i){marauder::dsp_item_value(g,i,val,sizeof(val));draw_value_row((int)i,marauder::dsp_item_name(g,i),val,i==s_dsp_cursor);}
        dsp_row_text((int)count,"RESET GROUP  (C)",s_dsp_cursor==count);
        // description / facts for the focused row, always two lines so stale text is overwritten
        char l1[32]="",l2[32]="";
        if(s_dsp_cursor<count){
            const char *name=marauder::dsp_item_name(g,s_dsp_cursor);
            if(g==marauder::DG_FFT&&!strcmp(name,"WINDOW")){
                const auto d=marauder::dsp_cfg_snapshot();const auto &w=marauder::dsp_window_info(d.window);
                if(d.window==marauder::WIN_LEGACY){snprintf(l1,sizeof(l1),"ORIGINAL RECT/HANN (SETTINGS)");snprintf(l2,sizeof(l2),"NO GAIN CORRECTION APPLIED");}
                else{snprintf(l1,sizeof(l1),"SIDELOBE %d dB ENBW %u.%02u",(int)w.sidelobe_db,w.enbw_x100/100,w.enbw_x100%100);
                     snprintf(l2,sizeof(l2),"RES %u kHz SCALLOP %u.%02u dB",w.enbw_x100*625/1000,w.scallop_x100/100,w.scallop_x100%100);}
            }else{
                const char *desc=marauder::dsp_item_desc(g,s_dsp_cursor);size_t n=strlen(desc);
                snprintf(l1,sizeof(l1),"%.30s",desc);if(n>30)snprintf(l2,sizeof(l2),"%.30s",desc+30);
            }
        }
        snprintf(line,sizeof(line),"%-30.30s",l1);draw_line(14,line,C_AMBER);
        snprintf(line,sizeof(line),"%-30.30s",l2);draw_line(15,line,C_AMBER);
        return;
    }
    marauder::DspResults r;marauder::dsp_results_snapshot(&r);
    if(s_dsp_group==7){
        draw_header("SIGNALS (RELATIVE)","",screen_changed);
        snprintf(line,sizeof(line),"%-30.30s","");
        snprintf(line,sizeof(line),"FLOOR %-3u PEAK %-3u TRACKS %u   ",r.frame_floor,r.frame_peak,r.count);draw_line(0,line,C_AMBER);
        for(unsigned i=0;i<marauder::kDspMaxSignals;++i){
            if(i<r.count){
                const auto &s=r.sig[i];
                const int64_t khz=(int64_t)r.center_mhz*1000+marauder::dsp_pos_to_khz_offset(s.pos_q4);
                snprintf(line,sizeof(line),"%c%lld.%03lld L%-3u %+3ddB P%-3u%%   ",i==s_dsp_sig_cursor?'>':' ',(long long)(khz/1000),(long long)(khz%1000),s.level,s.snr_x10/10,(unsigned)s.persistence*100/255);
                draw_line(1+i,line,i==s_dsp_sig_cursor?C_WHITE:C_CYAN);
            }else{snprintf(line,sizeof(line),"%-30.30s","");draw_line(1+i,line);}
        }
        if(s_dsp_sig_cursor<r.count){const auto &s=r.sig[s_dsp_sig_cursor];
            snprintf(line,sizeof(line),"WIDTH %u kHz  PROM %-3u AGE %-5u ",(unsigned)(s.width_q4*625/160),s.prominence,s.age);draw_line(11,line,C_AMBER);
            snprintf(line,sizeof(line),"DRIFT %+d kHz SINCE FIRST SEEN   ",(int)(s.drift_q4*625/160*-1));draw_line(12,line,C_AMBER);
        }else{snprintf(line,sizeof(line),"%-30.30s","");draw_line(11,line);draw_line(12,line);}
        draw_line(14,"L=LEVEL SNR=VS LOCAL NOISE (CFAR)");
        draw_line(15,"RELATIVE FFT DB - NOT dBm");
        draw_line(16,marauder::dsp_cfg_snapshot().d_on?"DETECTION ON               ":"DETECTION OFF (SEE SIGNAL DETECT)");
        return;
    }
    if(s_dsp_group==8){
        draw_header("IQ STATS (SNAPSHOT)","",screen_changed);
        const auto d=marauder::dsp_cfg_snapshot();
        snprintf(line,sizeof(line),"DC I %+4d  Q %+4d  (COUNTS)   ",r.iq.dc_i,r.iq.dc_q);draw_line(0,line);
        snprintf(line,sizeof(line),"I/Q GAIN %+d.%02d dB            ",r.iq.gain_x100_db/100,abs(r.iq.gain_x100_db)%100);draw_line(1,line);
        snprintf(line,sizeof(line),"QUADRATURE %+d.%d DEG          ",r.iq.phase_x10_deg/10,abs(r.iq.phase_x10_deg)%10);draw_line(2,line);
        snprintf(line,sizeof(line),"PEAK SAMPLE %-3u NORM SHIFT %u  ",r.iq.peak,r.iq.norm_shift);draw_line(3,line);
        snprintf(line,sizeof(line),"DC CORR %-3s BALANCE %-3s NORM %-3s",d.iq_dc?"ON":"OFF",d.iq_balance?"ON":"OFF",d.iq_normalize?"ON":"OFF");draw_line(5,line,C_AMBER);
        draw_line(7,"ESTIMATES FROM ONE 256-SAMPLE");
        draw_line(8,"SNAPSHOT, SMOOTHED OVER FRAMES.");
        draw_line(9,"A STRONG REAL TONE OR DC SIGNAL");
        draw_line(10,"BIASES THEM. NOT CONTINUOUS");
        draw_line(11,"STREAM PROCESSING.");
        return;
    }
    // performance: measured on this device
    draw_header("DSP PERFORMANCE","",screen_changed);
    marauder::DspPerf p;marauder::dsp_perf_snapshot(&p);
    snprintf(line,sizeof(line),"FFT %lu us  STAGE %lu us      ",(unsigned long)p.fft_us,(unsigned long)p.stage_us);draw_line(0,line);
    snprintf(line,sizeof(line),"DISPLAY STAGE %lu us           ",(unsigned long)p.display_us);draw_line(1,line);
    snprintf(line,sizeof(line),"DSP TOTAL %lu us  MAX %lu us   ",(unsigned long)(p.fft_us+p.stage_us+p.display_us),(unsigned long)p.total_max_us);draw_line(2,line,C_AMBER);
    snprintf(line,sizeof(line),"CAPTURE+DSP LOOP %lu us        ",(unsigned long)s_last_capture_us.load(std::memory_order_relaxed));draw_line(3,line);
    snprintf(line,sizeof(line),"FRAMES %lu                      ",(unsigned long)p.frames);draw_line(4,line);
    snprintf(line,sizeof(line),"STACK RADIO FREE %u B          ",(unsigned)(s_radio_task_handle?uxTaskGetStackHighWaterMark(s_radio_task_handle):0));draw_line(6,line);
    snprintf(line,sizeof(line),"STACK UI FREE %u B             ",(unsigned)(s_ui_task_handle?uxTaskGetStackHighWaterMark(s_ui_task_handle):0));draw_line(7,line);
    snprintf(line,sizeof(line),"HEAP FREE %u B                 ",(unsigned)esp_get_free_heap_size());draw_line(8,line);
    snprintf(line,sizeof(line),"HEAP MIN FREE %u B             ",(unsigned)esp_get_minimum_free_heap_size());draw_line(9,line);
    snprintf(line,sizeof(line),"CPU %u%%  DIE %d F              ",marauder::cpu_load_pct(),(int)marauder::die_temp_f());draw_line(10,line);
    draw_line(12,"MICROSECONDS MEASURED ON-DEVICE");
    draw_line(13,"EMA OF LAST FRAMES. MAX SINCE BOOT");
}

void draw_foxtx(bool screen_changed){
    using marauder::FoxMode;
    foxtx_poll();
    const auto st=marauder::fox_status();
    if(screen_changed)draw_detail_frame("U/D ROW L/R CHG C OK");
    draw_header("FOX BEACON","",screen_changed);
    const bool hop=s_fox.mode==(uint8_t)FoxMode::HOP,burst=s_fox.mode==(uint8_t)FoxMode::BURST,morse=s_fox.mode==(uint8_t)FoxMode::MORSE,steady=s_fox.mode==(uint8_t)FoxMode::STEADY;
    const char *labels[kFoxRows]={"MODE","CARRIER","POWER","HOP SET","INTERVAL","DUTY","DURATION","MESSAGE","CALLSIGN","WPM","ID EVERY","MSG ORDER","ACTION"};
    char values[kFoxRows][24]{};
    snprintf(values[0],24,"%s",marauder::fox_mode_name(s_fox.mode));
    if(hop)snprintf(values[1],24,"(HOP SET)");else snprintf(values[1],24,"%u MHZ +-%u",s_fox.freq_mhz,kFoxFreqSteps[s_fox_freq_step_index]);
    snprintf(values[2],24,"%+d DBM",marauder::fox_power_dbm(s_fox.power));
    snprintf(values[3],24,"%s",hop?marauder::fox_hop_set_name(s_fox.hop_set):"-");
    if(hop)snprintf(values[4],24,"%u MS DWELL",s_fox.interval_ms);else if(burst)snprintf(values[4],24,"%u MS PERIOD",s_fox.interval_ms);else if(morse)snprintf(values[4],24,"%u MS PAUSE",s_fox.interval_ms);else snprintf(values[4],24,"-");
    if(burst)snprintf(values[5],24,"%u%% ON",s_fox.duty_pct);else snprintf(values[5],24,"-");
    if(steady||!s_fox.duration_min)snprintf(values[6],24,"UNTIL STOPPED");else snprintf(values[6],24,"%u MIN",s_fox.duration_min);
    if(marauder::fox_file_message_count())snprintf(values[7],24,"FILE %u MSG",marauder::fox_file_message_count());
    else snprintf(values[7],24,"%s%s",s_fox.text[0]?s_fox.text:"(NONE)",s_fox_edit==0?"_":"");
    if(marauder::fox_callsign_from_file())snprintf(values[8],24,"%s FILE",marauder::fox_callsign());
    else snprintf(values[8],24,"%s%s",s_fox.call[0]?s_fox.call:"(NONE)",s_fox_edit==1?"_":"");
    snprintf(values[9],24,"%u",s_fox.wpm);
    if(morse)snprintf(values[10],24,"-");else if(s_fox.id_every_s)snprintf(values[10],24,"%u S",s_fox.id_every_s);else snprintf(values[10],24,"OFF");
    snprintf(values[11],24,"%s",morse?(s_fox.random_order?"RANDOM":"IN ORDER"):"-");
    snprintf(values[12],24,"%s",st.running?"STOP TX":s_fox_armed?"CENTER = START":"START (ARM)");
    for(size_t i=0;i<kFoxRows;++i)draw_value_row((int)i,labels[i],values[i],i==s_fox_cursor);
    char line[48];
    if(st.running){
        const unsigned secs=st.elapsed_s;
        snprintf(line,sizeof(line),"TX ON %u MHZ %s   ",st.freq_mhz,st.carrier_on?"KEYED":"OFF  ");draw_line(14,line,C_RED);
        snprintf(line,sizeof(line),"%02u:%02u:%02u HOP %lu ID %lu   ",secs/3600,(secs/60)%60,secs%60,(unsigned long)st.hops,(unsigned long)st.ids);draw_line(15,line);
    }else{
        snprintf(line,sizeof(line),"%-28.28s",s_fox_note[0]?s_fox_note:(s_fox_armed?"ARMED: CENTER AGAIN":st.last_err!=ESP_OK?esp_err_to_name(st.last_err):"TX OFF - NOT ARMED"));
        draw_line(14,line,s_fox_armed?C_AMBER:C_WHITE);
        draw_line(15,"                            ");
    }
    draw_line(17,"SETTINGS APPLY LIVE");
    draw_line(18,"CTR ON CARRIER = STEP SIZE");
    draw_line(19,"1802-2802 MHZ; ONLY CH 1-14");
    draw_line(20,"ARE SDK-DOCUMENTED");
    draw_line(21,"CABLE+ATTENUATOR OR CALL ID",C_AMBER);
    draw_line(22,"HOLD CENTER = BACK + STOP");
}

// ---- SD file browser: the merged media viewer + file manager --------------------------------------------------------
enum class BrMode : uint8_t { LIST, IMAGE, TEXT, INFO };
marauder::BrowserModel s_br;
BrMode s_br_mode = BrMode::LIST;
bool s_br_dirty = true, s_br_confirm = false, s_br_inited = false, s_br_clear = true;
uint32_t s_br_text_off = 0, s_br_text_next = 0;
char s_br_msg[48] = {};

void br_open() {
    if (!s_br_inited) { s_br.init(marauder::storage_dir_page); s_br_inited = true; }
    if (!marauder::storage_accessible()) marauder::storage_retry_mount();
    s_br.open_dir(".");
    s_br_mode = BrMode::LIST; s_br_confirm = false; s_br_dirty = s_br_clear = true; s_br_msg[0] = 0;
}
bool br_path(const marauder::DirEntry &e, char *out, size_t n) { return marauder::browser_join(s_br.dir(), e.name, out, n); }
void br_set_mode(BrMode m) { s_br_mode = m; s_br_confirm = false; s_br_dirty = s_br_clear = true; s_br_msg[0] = 0; }

// returns true when the browser should be left (LEFT at the top-level folder)
bool br_event(const marauder::InputEvent &e, bool move, bool confirm) {
    using K = marauder::InputKey;
    const bool left = e.key == K::LEFT && e.kind == marauder::InputKind::SHORT_RELEASE;
    const bool right = e.key == K::RIGHT && e.kind == marauder::InputKind::PRESS;
    const bool up = move && e.key == K::UP, down = move && e.key == K::DOWN;
    const marauder::DirEntry *sel = s_br.selected();
    switch (s_br_mode) {
    case BrMode::LIST:
        if (up || down) { s_br_confirm = false; const int d = down ? 1 : -1; if (e.repeat_count > 12) s_br.page(d); else s_br.move(d); s_br_dirty = true; }
        else if (left) { if (!s_br.up()) return true; s_br_dirty = s_br_clear = true; }
        else if (!marauder::storage_accessible()) { if (confirm) { br_open(); } }
        else if (right && sel) br_set_mode(BrMode::INFO);
        else if (confirm && sel) {
            const auto kind = marauder::browser_kind(sel->name, sel->is_dir);
            if (kind == marauder::FileKind::FOLDER) { s_br.enter(); s_br_dirty = s_br_clear = true; }
            else if (kind == marauder::FileKind::IMAGE) br_set_mode(BrMode::IMAGE);
            else if (kind == marauder::FileKind::TEXT) { s_br_text_off = 0; br_set_mode(BrMode::TEXT); }
            else br_set_mode(BrMode::INFO);
        }
        break;
    case BrMode::IMAGE: {
        marauder::DirEntry n;
        if ((left || right) && s_br.step_kind(marauder::FileKind::IMAGE, left ? -1 : 1, &n)) br_set_mode(BrMode::IMAGE);
        else if (confirm) br_set_mode(BrMode::LIST);
        break; }
    case BrMode::TEXT:
        if (down && s_br_text_next < (sel ? sel->size : 0)) { s_br_text_off = s_br_text_next; s_br_dirty = s_br_clear = true; }
        else if (up) { s_br_text_off = s_br_text_off > 600 ? s_br_text_off - 600 : 0; s_br_dirty = s_br_clear = true; }
        else if (confirm || left) br_set_mode(BrMode::LIST);
        break;
    case BrMode::INFO:
        if (left) br_set_mode(BrMode::LIST);
        else if (confirm && sel) {
            if (!marauder::browser_deletable(sel->name, sel->is_dir)) { snprintf(s_br_msg, sizeof(s_br_msg), "PROTECTED - NOT DELETABLE"); s_br_dirty = true; }
            else if (!s_br_confirm) { s_br_confirm = true; s_br_dirty = true; }
            else {
                char path[80], pair[96];
                bool ok = br_path(*sel, path, sizeof(path)) && marauder::storage_delete_file(path);
                if (ok && strcasecmp(strrchr(sel->name, '.') ? strrchr(sel->name, '.') : "", ".c8") == 0) {   // IQ capture: remove its sidecars too
                    snprintf(pair, sizeof(pair), "%s.META", path); marauder::storage_delete_file(pair);
                    snprintf(pair, sizeof(pair), "%.*s.META", (int)(strlen(path) - 3), path); marauder::storage_delete_file(pair);
                }
                s_br.reload(); br_set_mode(BrMode::LIST);
                snprintf(s_br_msg, sizeof(s_br_msg), ok ? "DELETED" : "DELETE FAILED");
            }
        }
        break;
    }
    return false;
}

void br_row(int y, const char *text, uint16_t fg, uint16_t bg = C_BLACK) {
    char b[32]; snprintf(b, sizeof(b), "%-30.30s", text); marauder::display_text(0, y, b, fg, bg);
}
void br_wrap(int row, const char *text, int lines, uint16_t fg) {
    for (int i = 0; i < lines; ++i) { char b[32] = {}; const size_t n = strlen(text); if ((size_t)i * 30 < n) snprintf(b, sizeof(b), "%.30s", text + i * 30); draw_line(row + i, b, fg); }
}

void br_draw(bool screen_changed) {
    if (screen_changed) s_br_dirty = s_br_clear = true;
    if (!s_br_dirty) return;
    s_br_dirty = false;
    if (s_br_clear) { marauder::display_fill_rect(0, 32, 240, 264, C_BLACK); s_header_force = s_footer_force = true; s_footer_valid = false; s_br_clear = false; }
    const marauder::DirEntry *sel = s_br.selected();
    char title[32], right[16] = "", path[80] = "";
    if (s_br.at_root()) snprintf(title, sizeof(title), "SD CARD /"); else { const size_t n = strlen(s_br.dir()); snprintf(title, sizeof(title), n > 24 ? "SD ..%s" : "SD /%s", s_br.dir() + (n > 24 ? n - 22 : 0)); }
    if (sel && s_br_mode == BrMode::LIST) snprintf(right, sizeof(right), "%u/%u", (unsigned)s_br.position(), (unsigned)s_br.total());
    if (s_br_mode == BrMode::LIST) {
        draw_header(title, right);
        if (!marauder::storage_accessible()) {
            for (int r = 0; r < 12; ++r) draw_line(r, "");
            draw_line(1, "SD NOT MOUNTED", C_RED);
            char l[32]; snprintf(l, sizeof(l), "MOUNT: %s", esp_err_to_name(marauder::storage_last_error())); draw_line(2, l, C_AMBER);
            draw_line(4, "CHECK CARD / FAT32 / SPI"); draw_line(6, "CENTER = RETRY MOUNT");
            draw_footer("CENTER RETRY  HOLD EXIT"); return;
        }
        for (size_t i = 0; i < marauder::BrowserModel::kRows; ++i) {
            const int y = 34 + (int)i * 11;
            if (i >= s_br.count()) { br_row(y, i == 0 ? " (EMPTY FOLDER)" : "", C_AMBER); continue; }
            const auto &en = s_br.entry(i);
            const auto kind = marauder::browser_kind(en.name, en.is_dir);
            char nm[24], sz[10] = "", line[40];
            marauder::browser_fit_name(en.name, 19, nm, sizeof(nm));
            if (en.is_dir) snprintf(sz, sizeof(sz), "<DIR>"); else marauder::browser_format_size(en.size, sz);
            const char icon = kind == marauder::FileKind::FOLDER ? '>' : kind == marauder::FileKind::IMAGE ? '#' : kind == marauder::FileKind::TEXT ? '=' :
                              kind == marauder::FileKind::CAPTURE ? '~' : kind == marauder::FileKind::MAP ? 'M' : kind == marauder::FileKind::VIDEO ? 'V' : ' ';
            snprintf(line, sizeof(line), "%c %-19s %8s", icon, nm, sz);
            const uint16_t fg = kind == marauder::FileKind::FOLDER ? C_CYAN : kind == marauder::FileKind::IMAGE ? C_GREEN : kind == marauder::FileKind::CAPTURE ? C_AMBER : C_WHITE;
            const bool on = i == s_br.cursor();
            br_row(y, line, on ? C_WHITE : fg, on ? 0x21A7 : C_BLACK);
        }
        br_row(34 + 21 * 11 + 3, s_br_msg, C_AMBER);
        draw_footer("C OPEN  L UP  R INFO  HOLD C EXIT");
        return;
    }
    if (!sel) { br_set_mode(BrMode::LIST); return; }
    br_path(*sel, path, sizeof(path));
    const auto kind = marauder::browser_kind(sel->name, sel->is_dir);
    if (s_br_mode == BrMode::IMAGE) {
        draw_header(sel->name, "", true);
        draw_footer("L/R PREV/NEXT  C LIST");
        char msg[48] = {};
        if (!marauder::media_display_image(path, msg, sizeof(msg))) {
            snprintf(s_br_msg, sizeof(s_br_msg), "%s", msg);
            br_wrap(10, msg, 2, C_RED); br_wrap(13, sel->name, 2, C_AMBER);
            char l[32]; marauder::browser_format_size(sel->size, l); char o[40]; snprintf(o, sizeof(o), "SIZE %s", l); draw_line(16, o);
        }
    } else if (s_br_mode == BrMode::TEXT) {
        draw_header(sel->name, "", true);
        char buf[601]; const size_t got = marauder::storage_read_file(path, s_br_text_off, buf, 600);
        int row = 0; char ln[31]; size_t col = 0, used = 0;
        for (size_t i = 0; i < got && row < 21; ++i) {
            const unsigned char c = buf[i]; used = i + 1;
            if (c == '\n') { ln[col] = 0; draw_line(row++, ln); col = 0; continue; }
            if (c == '\r') continue;
            ln[col++] = (c < 32 || c == 127) ? (c == '\t' ? ' ' : '.') : c;
            if (col == 30) { ln[col] = 0; draw_line(row++, ln); col = 0; }
        }
        if (col && row < 21) { ln[col] = 0; draw_line(row++, ln); }
        s_br_text_next = s_br_text_off + used;
        for (; row < 21; ++row) draw_line(row, "");
        draw_footer(s_br_text_next < sel->size ? "U/D PAGE  C/L LIST" : "END  U BACK  C LIST");
    } else {
        draw_header("FILE INFO", "", true);
        br_wrap(1, sel->name, 3, C_AMBER);
        char l[48];
        snprintf(l, sizeof(l), "IN  /%s", s_br.at_root() ? "" : s_br.dir()); draw_line(5, l);
        snprintf(l, sizeof(l), "TYPE %s", marauder::browser_kind_label(kind)); draw_line(6, l);
        if (!sel->is_dir) { snprintf(l, sizeof(l), "SIZE %lu BYTES", (unsigned long)sel->size); draw_line(7, l); }
        if (kind == marauder::FileKind::VIDEO) draw_line(9, "NO VIDEO DECODER ON ESP32", C_RED);
        const bool del = marauder::browser_deletable(sel->name, sel->is_dir);
        draw_line(11, sel->is_dir ? "FOLDERS CANNOT BE DELETED" : del ? (s_br_confirm ? "CENTER AGAIN = DELETE" : "CENTER = DELETE FILE") : "SYSTEM FILE - PROTECTED", s_br_confirm ? C_RED : del ? C_AMBER : C_WHITE);
        if (s_br_confirm) draw_line(12, "LEFT = CANCEL", C_GREEN); else draw_line(12, "");
        draw_line(14, s_br_msg, C_AMBER);
        draw_footer("C DELETE  L BACK  HOLD EXIT");
    }
}

bool is_map_detail(Detail d){return d==Detail::MAP_VIEW||d==Detail::NAV_VIEW||d==Detail::RID_VIEW;}
void map_radio_hook(bool pause){s_radio_paused.store(pause?true:settings_snapshot().paused,std::memory_order_relaxed);}

void ui_task(void *) {
    bool first=true;
    Page old_page=Page::HOME;Category old_category=Category::RECEIVE;Detail old_detail=Detail::NONE;
    size_t old_home=(size_t)-1,old_category_index=(size_t)-1;
    uint32_t shown_center_mhz=settings_snapshot().center_mhz;
    uint32_t previous_glass_center{};
    uint32_t previous_glass_peak_center{};
    uint8_t previous_glass_view=0xff;
    TickType_t last_refresh=0;
    for(;;){
        const auto event=marauder::input_next_event(input_config(settings_snapshot()));
        s_nav_dir=event.key==marauder::InputKey::UP?-1:1;
        if(s_page==Page::DETAIL&&is_map_detail(s_detail))marauder::map_ui_tick();
        if(s_page==Page::DETAIL&&s_detail==Detail::APPRUN)marauder::app_run_tick((uint32_t)(esp_timer_get_time()/1000));
        size_t count=0;const Item *items=nullptr;
        const bool move=input_direction(event);
        const bool confirm=event.key==marauder::InputKey::CENTER&&event.kind==marauder::InputKind::SHORT_RELEASE;
        const bool gps_name_edit=s_page==Page::DETAIL&&s_detail==Detail::GPS&&s_gps_view==0&&s_gps_cursor==4&&s_gps_name_editing;
        const bool gps_view_actions=s_page==Page::DETAIL&&s_detail==Detail::GPS&&s_gps_view!=0;
        const bool left_is_control=s_detail==Detail::SPECTRUM||s_detail==Detail::FREQ||s_detail==Detail::FOXHUNT||s_detail==Detail::DSP||s_detail==Detail::RSSI_GRAPH||s_detail==Detail::FOXTX||s_detail==Detail::FILES||s_detail==Detail::APPRUN||is_map_detail(s_detail);
        const bool back=(event.key==marauder::InputKey::LEFT&&event.kind==marauder::InputKind::SHORT_RELEASE&&!gps_name_edit&&!left_is_control)||(event.key==marauder::InputKey::CENTER&&event.kind==marauder::InputKind::LONG_PRESS&&!gps_view_actions);
        if(s_page==Page::HOME){count=sizeof(kHome)/sizeof(*kHome);items=kHome;if(move&&(event.key==marauder::InputKey::UP||event.key==marauder::InputKey::DOWN)){if(event.key==marauder::InputKey::UP)s_home_selected=(s_home_selected+count-1)%count;else s_home_selected=(s_home_selected+1)%count;}else if(confirm){s_category=kHomeCategories[s_home_selected];s_category_selected=s_category==Category::SETTINGS?1:0;if(s_category==Category::SETTINGS){s_detail=Detail::SETTINGS;s_page=Page::DETAIL;}else s_page=Page::CATEGORY;}}
        else if(s_page==Page::CATEGORY){items=items_for(s_category,count);if(move&&(event.key==marauder::InputKey::UP||event.key==marauder::InputKey::DOWN)){if(event.key==marauder::InputKey::UP)s_category_selected=(s_category_selected+count-1)%count;else s_category_selected=(s_category_selected+1)%count;}else if(confirm)execute_detail_action(items,count);else if(back){s_page=Page::HOME;s_category_selected=0;}}
        else if(s_page==Page::DETAIL){
            if(back){if(is_map_detail(s_detail))marauder::map_ui_close();if(s_detail==Detail::APPRUN)marauder::app_run_stop();if(s_detail==Detail::FOXTX)foxtx_leave();if(s_detail==Detail::DSP)marauder::dsp_cfg_save_if_dirty();s_looking_glass_active.store(false);s_recon_active.store(false);s_foxhunt_active.store(false);s_rssi_detector_active.store(false);s_tx_armed=false;if(s_detail==Detail::SETTINGS)s_page=Page::HOME;else s_page=Page::CATEGORY;s_detail=Detail::NONE;}
            else if(s_detail==Detail::GAIN&&(move||confirm)){
                bool automatic=marauder_radio_rx_gain_is_auto();unsigned index=marauder_radio_rx_gain_index();
                if(event.key==marauder::InputKey::RIGHT){if(event.kind==marauder::InputKind::PRESS)dc_mode_cycle();}
                else if(confirm){automatic=!automatic;marauder_radio_set_rx_gain(automatic,index);}
                else if(!automatic&&(event.key==marauder::InputKey::UP||event.key==marauder::InputKey::DOWN)){
                    int delta=event.key==marauder::InputKey::UP?1:-1;
                    if(event.repeat_count>16)delta*=4;else if(event.repeat_count>8)delta*=2;
                    index=(unsigned)std::clamp<int>((int)index+delta,0,127);
                    marauder_radio_set_rx_gain(automatic,index);
                }
            }
            else if(s_detail==Detail::TX_TEST&&confirm){
                if(!s_tx_armed){s_tx_armed=true;snprintf(s_status,sizeof(s_status),"LAB CHECK ACKNOWLEDGED");}
                else {s_radio_paused.store(true);esp_err_t tx=marauder_radio_bench_tx_once();s_radio_paused.store(false);s_tx_armed=false;snprintf(s_status,sizeof(s_status),tx==ESP_OK?"ONE-PACKET TEST CALL SENT":"TX TEST ERROR");}
            }
            else if(s_detail==Detail::EXTERNALS&&(move||confirm)){
                const size_t nf=marauder::storage_list_files("APPS",".APP",s_ext_names,24);
                if(move&&nf){s_external_cursor=(s_external_cursor+24+(event.key==marauder::InputKey::UP?-1:1))%24;if(s_external_cursor>=nf)s_external_cursor=event.key==marauder::InputKey::UP?nf-1:0;}
                else if(confirm&&nf){char path[96],why[48]{};snprintf(path,sizeof(path),"APPS/%s",s_ext_names[s_external_cursor%nf]);
                    if(marauder::app_run_start(path,why,sizeof(why))){s_detail=Detail::APPRUN;s_page=Page::DETAIL;}else snprintf(s_status,sizeof(s_status),"%.31s",why);}
            }
            else if(s_detail==Detail::APPRUN){
                if(event.kind==marauder::InputKind::PRESS||event.kind==marauder::InputKind::REPEAT){
                    if(event.key==marauder::InputKey::UP)marauder::app_run_key(1);else if(event.key==marauder::InputKey::DOWN)marauder::app_run_key(2);
                    else if(event.key==marauder::InputKey::LEFT)marauder::app_run_key(3);else if(event.key==marauder::InputKey::RIGHT)marauder::app_run_key(4);}
                else if(confirm)marauder::app_run_key(5);
            }
            else if(is_map_detail(s_detail)){if(marauder::map_ui_event(event,move,confirm)){marauder::map_ui_close();s_page=Page::CATEGORY;s_detail=Detail::NONE;}}
            else if(s_detail==Detail::FILES){if(br_event(event,move,confirm)){s_page=Page::CATEGORY;s_detail=Detail::NONE;}}
            else if(s_detail==Detail::FOXTX&&(move||confirm)){foxtx_event(event.key,confirm);}
            else if(s_detail==Detail::DSP&&(move||confirm)){dsp_event(event.key,confirm,event.kind==marauder::InputKind::REPEAT);}
            else if(s_detail==Detail::RSSI_GRAPH&&(move||confirm)){
                if(confirm)s_rssi_detector_enabled=!s_rssi_detector_enabled;
                else if(event.key==marauder::InputKey::LEFT||event.key==marauder::InputKey::RIGHT){
                    const auto c=settings_snapshot();
                    const int repeat=event.repeat_count>24?8:event.repeat_count>16?4:event.repeat_count>8?2:1;
                    const int delta=(int)c.frequency_step*repeat*(event.key==marauder::InputKey::RIGHT?1:-1);
                    const uint32_t current=s_rssi_detector_frequency.load(std::memory_order_relaxed);
                    const uint32_t next=(uint32_t)std::clamp<int>((int)current+delta,MARAUDER_RADIO_FREQ_MIN_MHZ,MARAUDER_RADIO_FREQ_MAX_MHZ);
                    s_rssi_detector_frequency.store(next,std::memory_order_relaxed);
                    s_rssi_history_count=s_rssi_history_head=0;
                    memset(s_rssi_history,0,sizeof(s_rssi_history));
                    s_rssi_level=0;
                    s_rssi_triggered=false;
                }else if(event.key==marauder::InputKey::UP||event.key==marauder::InputKey::DOWN){
                    const int delta=(event.key==marauder::InputKey::UP?1:-1)*(event.repeat_count>8?12:4);
                    s_rssi_threshold=(uint8_t)std::clamp<int>((int)s_rssi_threshold+delta,0,255);
                }
            }
            else if(s_detail==Detail::FOXHUNT&&(move||confirm)){
                static constexpr uint32_t steps[]={1,5,10,25};
                if(event.key==marauder::InputKey::LEFT||event.key==marauder::InputKey::RIGHT){
                    const int repeat=event.repeat_count>24?8:event.repeat_count>16?4:event.repeat_count>8?2:1;
                    const int delta=(int)steps[s_foxhunt_step_index]*repeat*(event.key==marauder::InputKey::RIGHT?1:-1);
                    const uint32_t current=s_foxhunt_frequency.load(std::memory_order_relaxed);
                    const uint32_t next=(uint32_t)std::clamp<int>((int)current+delta,MARAUDER_RADIO_FREQ_MIN_MHZ,MARAUDER_RADIO_FREQ_MAX_MHZ);
                    s_foxhunt_frequency.store(next,std::memory_order_relaxed);
                    s_foxhunt_level=s_foxhunt_peak_level=0;
                    s_foxhunt_peak_offset_khz=0;
                }else if(event.key==marauder::InputKey::UP||event.key==marauder::InputKey::DOWN){
                    s_foxhunt_step_index=(s_foxhunt_step_index+(event.key==marauder::InputKey::UP?1:3))%4;
                }else if(confirm){
                    s_foxhunt_level=s_foxhunt_peak_level=0;
                    s_foxhunt_peak_offset_khz=0;
                }
            }
            else if(s_detail==Detail::SPECTRUM&&s_looking_glass_active.load(std::memory_order_relaxed)&&(move||confirm)){
                auto c=settings_snapshot();
                const bool arrow=event.key==marauder::InputKey::UP||event.key==marauder::InputKey::DOWN||
                                 event.key==marauder::InputKey::LEFT||event.key==marauder::InputKey::RIGHT;
                if(confirm&&s_glass_setting==8&&!s_glass_editing)s_glass_paused.store(!s_glass_paused.load(std::memory_order_relaxed),std::memory_order_relaxed);
                else if(confirm&&s_glass_setting==9&&!s_glass_editing)s_glass_range_locked=!s_glass_range_locked;
                else if(confirm)s_glass_editing=!s_glass_editing;
                else if(arrow&&!s_glass_editing){
                    const bool previous=event.key==marauder::InputKey::UP||event.key==marauder::InputKey::LEFT;
                    s_glass_setting=(s_glass_setting+(previous?9:1))%10;
                }else if(arrow){
                    const int direction=event.key==marauder::InputKey::UP||event.key==marauder::InputKey::RIGHT?1:-1;
                    switch(s_glass_setting){
                        case 0:{
                            const int step=event.repeat_count>8?10:1;
                            const int old_min=(int)s_glass_min_frequency.load(std::memory_order_relaxed);
                            const int old_max=(int)s_glass_max_frequency.load(std::memory_order_relaxed);
                            const int width=old_max-old_min;
                            const int upper=s_glass_range_locked?(int)MARAUDER_RADIO_FREQ_MAX_MHZ-width:old_max;
                            const int next=std::clamp(old_min+direction*step,(int)MARAUDER_RADIO_FREQ_MIN_MHZ,upper);
                            if(s_glass_range_locked){
                                const uint32_t next_max=(uint32_t)(next+width);
                                if(next>old_min){
                                    s_glass_max_frequency.store(next_max,std::memory_order_relaxed);
                                    s_glass_min_frequency.store((uint32_t)next,std::memory_order_relaxed);
                                }else{
                                    s_glass_min_frequency.store((uint32_t)next,std::memory_order_relaxed);
                                    s_glass_max_frequency.store(next_max,std::memory_order_relaxed);
                                }
                            }else s_glass_min_frequency.store((uint32_t)next,std::memory_order_relaxed);
                            break;
                        }
                        case 1:{
                            const int step=event.repeat_count>8?10:1;
                            const int old_min=(int)s_glass_min_frequency.load(std::memory_order_relaxed);
                            const int old_max=(int)s_glass_max_frequency.load(std::memory_order_relaxed);
                            const int width=old_max-old_min;
                            const int lower=s_glass_range_locked?(int)MARAUDER_RADIO_FREQ_MIN_MHZ+width:old_min;
                            const int next=std::clamp(old_max+direction*step,lower,(int)MARAUDER_RADIO_FREQ_MAX_MHZ);
                            if(s_glass_range_locked){
                                const uint32_t next_min=(uint32_t)(next-width);
                                if(next>old_max){
                                    s_glass_max_frequency.store((uint32_t)next,std::memory_order_relaxed);
                                    s_glass_min_frequency.store(next_min,std::memory_order_relaxed);
                                }else{
                                    s_glass_min_frequency.store(next_min,std::memory_order_relaxed);
                                    s_glass_max_frequency.store((uint32_t)next,std::memory_order_relaxed);
                                }
                            }else s_glass_max_frequency.store((uint32_t)next,std::memory_order_relaxed);
                            break;
                        }
                        case 2:{
                            const int delta=direction*(event.repeat_count>8?4:1);
                            const unsigned gain=(unsigned)std::clamp((int)marauder_radio_rx_gain_index()+delta,0,127);
                            const esp_err_t err=marauder_radio_set_rx_gain(false,gain);
                            if(err!=ESP_OK){snprintf(s_status,sizeof(s_status),"GAIN ERROR: %s",esp_err_to_name(err));ESP_LOGW(kTag,"RX gain update failed: %s",esp_err_to_name(err));}
                            break;
                        }
                        case 3:{
                            static constexpr uint16_t spans[]={1,2,4,8,16};
                            size_t current=0;while(current+1<sizeof(spans)/sizeof(spans[0])&&spans[current]!=c.span_mhz)++current;
                            current=(current+(direction>0?1:sizeof(spans)/sizeof(spans[0])-1))%(sizeof(spans)/sizeof(spans[0]));
                            c.span_mhz=spans[current];settings_commit(c,false);
                            break;
                        }
                        case 4:{
                            const uint8_t current=s_spectrum_filter.load(std::memory_order_relaxed);
                            spectrum_filter_set((uint8_t)((current+kSpectrumFilterCount+direction)%kSpectrumFilterCount));
                            break;
                        }
                        case 5:{
                            const uint8_t current=s_glass_step_index.load(std::memory_order_relaxed);
                            s_glass_step_index.store((uint8_t)((current+(direction>0?1:3))%4),std::memory_order_relaxed);
                            break;
                        }
                        case 6:{
                            const uint8_t current=s_glass_dwell_index.load(std::memory_order_relaxed);
                            s_glass_dwell_index.store((uint8_t)((current+(direction>0?1:3))%4),std::memory_order_relaxed);
                            break;
                        }
                        case 7:{
                            const uint8_t current=s_glass_view_mode.load(std::memory_order_relaxed);
                            s_glass_view_mode.store((uint8_t)((current+(direction>0?1:2))%3),std::memory_order_relaxed);
                            break;
                        }
                        case 8:s_glass_paused.store(!s_glass_paused.load(std::memory_order_relaxed),std::memory_order_relaxed);break;
                        case 9:s_glass_range_locked=!s_glass_range_locked;break;
                        default:break;
                    }
                }
            }
            else if(s_detail==Detail::SPECTRUM&&(move||confirm)){
                auto c=settings_snapshot();if(event.key==marauder::InputKey::LEFT||event.key==marauder::InputKey::RIGHT){int speed=event.repeat_count>24?8:event.repeat_count>16?4:event.repeat_count>8?2:1;int delta=(int)c.frequency_step*speed*(event.key==marauder::InputKey::RIGHT?1:-1);c.center_mhz=(uint32_t)std::clamp<int>((int)c.center_mhz+delta,MARAUDER_RADIO_FREQ_MIN_MHZ,MARAUDER_RADIO_FREQ_MAX_MHZ);settings_commit(c,false);}
                else if(event.key==marauder::InputKey::UP||event.key==marauder::InputKey::DOWN){c.span_mhz=event.key==marauder::InputKey::UP?(c.span_mhz<=1?16:c.span_mhz/2):(c.span_mhz>=16?1:c.span_mhz*2);settings_commit(c,false);}
                else if(confirm&&s_detail==Detail::SPECTRUM){s_radio_paused.store(!s_radio_paused.load());c.paused=s_radio_paused.load();settings_commit(c);}
            }
            else if((s_detail==Detail::SPECTRUM||s_detail==Detail::FREQ)&&(move||confirm)){
                auto c=settings_snapshot();if(event.key==marauder::InputKey::LEFT||event.key==marauder::InputKey::RIGHT){int speed=event.repeat_count>24?8:event.repeat_count>16?4:event.repeat_count>8?2:1;int delta=(int)c.frequency_step*speed*(event.key==marauder::InputKey::RIGHT?1:-1);c.center_mhz=(uint32_t)std::clamp<int>((int)c.center_mhz+delta,MARAUDER_RADIO_FREQ_MIN_MHZ,MARAUDER_RADIO_FREQ_MAX_MHZ);settings_commit(c,false);}
                else if(event.key==marauder::InputKey::UP||event.key==marauder::InputKey::DOWN){c.span_mhz=event.key==marauder::InputKey::UP?(c.span_mhz<=1?16:c.span_mhz/2):(c.span_mhz>=16?1:c.span_mhz*2);settings_commit(c,false);}
                else if(confirm&&s_detail==Detail::SPECTRUM){s_radio_paused.store(!s_radio_paused.load());c.paused=s_radio_paused.load();settings_commit(c);}
            }
            else if((s_detail==Detail::SPECTRUM||s_detail==Detail::FREQ)&&event.kind==marauder::InputKind::SHORT_RELEASE){auto save=settings_snapshot();marauder::settings_save(save);}
            else if(gps_name_edit&&move){if(event.key==marauder::InputKey::LEFT)s_gps_name_cursor=(s_gps_name_cursor+7)%8;else if(event.key==marauder::InputKey::RIGHT)s_gps_name_cursor=(s_gps_name_cursor+1)%8;else{static const char charset[]="ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";const char *p=strchr(charset,s_gps_track_name[s_gps_name_cursor]);int index=p?(int)(p-charset):0;index=(index+36+s_nav_dir)%36;s_gps_track_name[s_gps_name_cursor]=charset[index];}}
            else if(s_detail==Detail::SETTINGS){if(move&&(event.key==marauder::InputKey::UP||event.key==marauder::InputKey::DOWN))s_category_selected=(s_category_selected+(event.key==marauder::InputKey::UP?21:1))%22;else if((move&&event.key==marauder::InputKey::RIGHT)||confirm){if(s_category_selected==0){s_page=Page::HOME;s_detail=Detail::NONE;}else{size_t selected=s_category_selected;--s_category_selected;adjust_settings(confirm);s_category_selected=selected;}}else if(event.kind==marauder::InputKind::SHORT_RELEASE&&event.key==marauder::InputKey::RIGHT){auto save=settings_snapshot();marauder::settings_save(save);}}
            else {const bool center_runs_primary=s_detail==Detail::SCANNER||s_detail==Detail::ANTENNA||s_detail==Detail::STOPWATCH||s_detail==Detail::RANDPWD||s_detail==Detail::FREQ_MANAGER||s_detail==Detail::EXTERNALS;bool short_press=move&&(event.key==marauder::InputKey::UP||event.key==marauder::InputKey::DOWN||event.key==marauder::InputKey::RIGHT);if(confirm&&center_runs_primary)short_press=true;bool long_press=(confirm&&!center_runs_primary)||(gps_view_actions&&event.key==marauder::InputKey::CENTER&&event.kind==marauder::InputKind::LONG_PRESS);handle_detail_event(short_press,long_press,items,count);}
        }
        const TickType_t now=xTaskGetTickCount();
        bool redraw=first||s_page!=old_page||s_category!=old_category||s_detail!=old_detail||s_home_selected!=old_home||s_category_selected!=old_category_index||event.kind!=marauder::InputKind::NONE;
        const bool screen_changed=first||s_page!=old_page||s_category!=old_category||s_detail!=old_detail;
        if(screen_changed&&s_page==Page::DETAIL&&s_detail==Detail::GPS&&s_gps_view==1){
            ESP_LOGI(kTag,"GPS track screen stack watermark: %u",
                     (unsigned)uxTaskGetStackHighWaterMark(nullptr));
        }
        if(s_page==Page::DETAIL&&s_detail!=Detail::SPECTRUM&&(s_detail!=Detail::DSP||s_dsp_group>=7)&&s_detail!=Detail::SETTINGS&&!(s_detail==Detail::GPS&&s_gps_view==1)&&now-last_refresh>=pdMS_TO_TICKS(s_detail==Detail::STOPWATCH||s_detail==Detail::FOXHUNT||s_detail==Detail::DSP||s_detail==Detail::RSSI_GRAPH||s_detail==Detail::FOXTX?250:1000))redraw=true;
        if(s_page==Page::HOME){if(redraw)draw_menu("DOMINION / HOME",kHome,sizeof(kHome)/sizeof(*kHome),s_home_selected,0,screen_changed);}
        else if(s_page==Page::CATEGORY){if(redraw){items=items_for(s_category,count);draw_menu(category_name(s_category),items,count,s_category_selected,0,screen_changed);}}
        else if(s_detail==Detail::FOXTX&&redraw){draw_foxtx(screen_changed);}
        else if(s_detail==Detail::DSP&&redraw){draw_dsp(screen_changed);}
        else if(s_detail==Detail::FOXHUNT){
            if(screen_changed)draw_detail_frame("L/R TUNE U/D STEP C RESET");
            draw_header("LEVEL / SIGNAL METER","",screen_changed);
            marauder::SpectrumFrame frame{};
            if(s_frame_queue&&xQueueReceive(s_frame_queue,&frame,0)==pdTRUE){
                uint8_t level=0;unsigned peak_bin=120;
                for(unsigned i=0;i<240;++i)if(frame.bins[i]>level){level=frame.bins[i];peak_bin=i;}
                s_foxhunt_level=s_foxhunt_level?(uint8_t)(((unsigned)s_foxhunt_level*3+level)/4):level;
                if(level>=s_foxhunt_peak_level){s_foxhunt_peak_level=level;s_foxhunt_peak_offset_khz=(120-(int)peak_bin)*625/10;}
            }
            const uint32_t frequency=s_foxhunt_frequency.load(std::memory_order_relaxed);
            const uint32_t step[]={1,5,10,25};
            char line[40];
            snprintf(line,sizeof(line),"CENTER %lu MHz  STEP %lu",(unsigned long)frequency,(unsigned long)step[s_foxhunt_step_index]);draw_line(0,line,frequency>MARAUDER_RADIO_FREQ_BAND_MAX_MHZ?C_RED:C_AMBER);
            snprintf(line,sizeof(line),"REL LEVEL %3u / 255",s_foxhunt_level);draw_line(2,line,C_CYAN);
            snprintf(line,sizeof(line),"PEAK %3u  OFFSET %+d kHz",s_foxhunt_peak_level,s_foxhunt_peak_offset_khz);draw_line(3,line,C_AMBER);
            marauder::display_fill_rect(18,90,204,8,C_PANEL);
            const int bar_width=(int)((unsigned)s_foxhunt_level*200/255);
            if(bar_width)marauder::display_fill_rect(20,92,bar_width,4,C_CYAN);
            draw_line(6,"VISUAL RELATIVE RSSI ONLY");
            draw_line(7,"ROTATE ANTENNA; COMPARE LEVEL");
            const auto fix=marauder::gps_current_fix();
            if(fix.valid){snprintf(line,sizeof(line),"RX POS %.4f,%.4f",fix.latitude,fix.longitude);draw_line(9,line,C_GREEN);}
            else draw_line(9,"GPS: NO RECEIVER POSITION FIX",C_AMBER);
            draw_line(11,"NOT CALIBRATED dBm / NO TX",C_WHITE);
        }
        else if(s_detail==Detail::RSSI_GRAPH){
            if(screen_changed){draw_detail_frame("L/R TUNE U/D TRIG C DETECT");}
            draw_header("ROLLING RSSI DETECTOR","",screen_changed);
            marauder::SpectrumFrame frame{};
            if(s_frame_queue&&xQueueReceive(s_frame_queue,&frame,0)==pdTRUE){
                uint8_t level=0;
                for(unsigned i=0;i<240;++i)level=std::max(level,frame.bins[i]);
                s_rssi_level=level;
                s_rssi_triggered=s_rssi_detector_enabled&&level>=s_rssi_threshold;
                s_rssi_history[s_rssi_history_head]=level;
                s_rssi_history_head=(s_rssi_history_head+1)%sizeof(s_rssi_history);
                s_rssi_history_count=std::min(s_rssi_history_count+1,sizeof(s_rssi_history));
            }
            const uint32_t frequency=s_rssi_detector_frequency.load(std::memory_order_relaxed);
            char line[40];
            snprintf(line,sizeof(line),"CENTER %lu MHz",(unsigned long)frequency);
            draw_line(0,line,frequency>MARAUDER_RADIO_FREQ_BAND_MAX_MHZ?C_RED:C_AMBER);
            snprintf(line,sizeof(line),"LEVEL %3u  TRIGGER %3u",s_rssi_level,s_rssi_threshold);
            draw_line(1,line,C_CYAN);
            draw_line(2,s_rssi_triggered?"DETECTED - ABOVE THRESHOLD":s_rssi_detector_enabled?"MONITORING":"DETECTOR OFF",
                      s_rssi_triggered?C_RED:s_rssi_detector_enabled?C_GREEN:C_AMBER);
            constexpr int graph_x=20,graph_y=88,graph_width=200,graph_height=128;
            // Render the graph off-screen in 16-row strips and push each strip in one transfer,
            // so the plot never flashes through a black fill.
            static uint16_t strip[graph_width*16];
            static uint32_t drawn_sig=0;
            uint32_t sig=(uint32_t)s_rssi_history_head*2654435761u^((uint32_t)s_rssi_history_count<<8)^((uint32_t)s_rssi_threshold<<20)^(s_rssi_level*40503u);
            if(screen_changed||sig!=drawn_sig){
                drawn_sig=sig;
                const int threshold_y=graph_y+graph_height-(int)((unsigned)s_rssi_threshold*graph_height/255);
                const size_t first=(s_rssi_history_head+sizeof(s_rssi_history)-s_rssi_history_count)%sizeof(s_rssi_history);
                for(int y0=0;y0<graph_height;y0+=16){
                    for(int k=0;k<graph_width*16;++k)strip[k]=C_BLACK;
                    for(unsigned lv=0;lv<=255;lv+=64){
                        const int gy=graph_height-(int)(lv*graph_height/255)-y0;
                        if(gy>=0&&gy<16)for(int x=0;x<graph_width;++x)strip[gy*graph_width+x]=C_PANEL;
                    }
                    {const int gy=threshold_y-graph_y-y0;if(gy>=0&&gy<16)for(int x=0;x<graph_width;++x)strip[gy*graph_width+x]=C_RED;}
                    int previous_y=graph_height;
                    for(size_t i=0;i<s_rssi_history_count&&i<(size_t)graph_width;++i){
                        const uint8_t value=s_rssi_history[(first+i)%sizeof(s_rssi_history)];
                        const int y=graph_height-(int)((unsigned)value*graph_height/255);
                        const int top=i?std::min(previous_y,y):y;
                        const int h=i?std::max(1,std::abs(previous_y-y)):1;
                        const uint16_t col=value>=s_rssi_threshold?C_RED:C_CYAN;
                        for(int yy=std::max(top,y0);yy<std::min(top+h,y0+16);++yy)strip[(yy-y0)*graph_width+(int)i]=col;
                        previous_y=y;
                    }
                    marauder::display_rgb565_image(graph_x,graph_y+y0,graph_width,16,strip);
                }
            }
            draw_line(23,"RELATIVE FFT LEVEL - NOT dBm",C_AMBER);
        }
        else if(s_detail==Detail::SPECTRUM){
            if(screen_changed){marauder::display_clear(C_BLACK);s_header_force=true;s_footer_force=true;s_footer_valid=false;marauder::display_reset_waterfall();if(s_looking_glass_active.load(std::memory_order_relaxed))for(auto &entry:s_glass_quick_cache)entry.valid=false;}
            if(s_looking_glass_active.load(std::memory_order_relaxed))
                draw_footer(s_glass_editing?"ARROWS ADJUST C DONE HOLD EXIT":"ARROWS PICK C EDIT HOLD EXIT");
            else
                draw_footer("L/R TUNE U/D SPAN C PAUSE HOLD EXIT");
            auto c=settings_snapshot();
            if(screen_changed)shown_center_mhz=c.center_mhz;
            marauder::SpectrumFrame frame{};
            const bool have_frame=s_frame_queue&&xQueueReceive(s_frame_queue,&frame,0)==pdTRUE;
            if(have_frame)shown_center_mhz=frame.center_mhz;
            const uint32_t display_center=s_looking_glass_active.load(std::memory_order_relaxed)?
                (have_frame?shown_center_mhz:s_glass_current_frequency.load(std::memory_order_relaxed)):c.center_mhz;
            char title[40];snprintf(title,sizeof(title),"%lu.%03lu GHz Z%u %s%s",(unsigned long)(display_center/1000),(unsigned long)(display_center%1000),c.span_mhz,c.waterfall_palette==0?"MAGMA":c.waterfall_palette==1?"INFERNO":"GRAY",c.max_hold?" MH":"");draw_header(title,"",screen_changed,display_center>MARAUDER_RADIO_FREQ_BAND_MAX_MHZ?C_RED:C_CYAN);
            const int64_t center_khz=(int64_t)display_center*1000;
            const int64_t half_span_khz=(int64_t)c.span_mhz*500;
            const int64_t axis_khz[]={center_khz-half_span_khz,center_khz,center_khz+half_span_khz};
            const int axis_x[]={0,84,168};const char axis_tag[]={'L','C','R'};char axis[32];
            if(!s_looking_glass_active.load(std::memory_order_relaxed))
                for(unsigned a=0;a<3;++a){int64_t f=std::max<int64_t>(0,axis_khz[a]);snprintf(axis,sizeof(axis),"%c %lld.%03lldG",axis_tag[a],(long long)(f/1000000),(long long)((f/1000)%1000));marauder::display_text(axis_x[a],66,axis,C_AMBER,C_BLACK);}
            const uint8_t glass_view=s_glass_view_mode.load(std::memory_order_relaxed);
            if(s_looking_glass_active.load(std::memory_order_relaxed)&&glass_view!=previous_glass_view){
                if(previous_glass_view!=0xff){
                    marauder::display_fill_rect(0,36,240,30,C_BLACK);
                    marauder::display_reset_waterfall();
                    previous_glass_center=0;
                }
                for(auto &entry:s_glass_quick_cache)entry.valid=false;
                previous_glass_view=glass_view;
            }else if(!s_looking_glass_active.load(std::memory_order_relaxed)){
                previous_glass_view=0xff;
            }
            if(have_frame){
                uint8_t display_bins[240],display_peaks[240],water_bins[240],display_floor[240],display_min[240];uint16_t row[240];
                const auto dsp_view=marauder::dsp_cfg_snapshot();
                project_trace(frame,c,display_bins,display_peaks,display_floor,display_min);project_waterfall(frame,c,water_bins);
                for(unsigned i=0;i<240;++i)row[i]=marauder::waterfall_rgb565(water_bins[i],0,255,c.waterfall_palette);
                if(s_looking_glass_active.load(std::memory_order_relaxed)){
                    if(previous_glass_center&&frame.center_mhz<previous_glass_center){marauder::display_reset_waterfall();memset(s_glass_peak_bins,0,sizeof(s_glass_peak_bins));}
                    previous_glass_center=frame.center_mhz;
                    if(glass_view==0)marauder::display_waterfall_line(row);
                    else{
                        if(previous_glass_peak_center!=frame.center_mhz)memset(s_glass_peak_bins,0,sizeof(s_glass_peak_bins));
                        for(unsigned i=0;i<240;++i)s_glass_peak_bins[i]=std::max(s_glass_peak_bins[i],display_bins[i]);
                        previous_glass_peak_center=frame.center_mhz;
                        marauder::display_spectrum_region(glass_view==1?display_bins:s_glass_peak_bins,
                                                          glass_view==1?display_peaks:s_glass_peak_bins,
                                                          C_CYAN,80,100);
                    }
                }else{
                    previous_glass_center=0;
                    previous_glass_peak_center=0;
                    marauder::display_spectrum_region(display_bins,display_peaks,C_CYAN,36,28,
                        (dsp_view.v_floor&&frame.have_floor)?display_floor:nullptr,dsp_view.v_minhold?display_min:nullptr);
                    marauder::display_waterfall_line(row);
                }
            }
            if(s_looking_glass_active.load(std::memory_order_relaxed)){
                auto draw_quick=[&](int x,int y,const char *text,size_t index){
                    const bool focused=s_glass_setting==index;
                    auto &cache=s_glass_quick_cache[index];
                    if(cache.valid&&cache.focused==focused&&cache.editing==s_glass_editing&&strcmp(cache.text,text)==0)return;
                    snprintf(cache.text,sizeof(cache.text),"%s",text);
                    cache.focused=focused;
                    cache.editing=s_glass_editing;
                    cache.valid=true;
                    const int width=110;
                    if(focused){
                        const uint16_t background=s_glass_editing?C_AMBER:C_WHITE;
                        marauder::display_fill_rect(x-2,y-1,width+4,9,C_CYAN);
                        marauder::display_fill_rect(x-1,y,width+2,8,background);
                        marauder::display_text(x,y,text,C_BLACK,background);
                    }else{
                        marauder::display_fill_rect(x-2,y-1,width+4,9,C_BLACK);
                        marauder::display_text(x,y,text,C_WHITE,C_BLACK);
                    }
                };
                char value[24];
                const int y[]={36,44,52,60,68};
                static constexpr uint32_t glass_steps[]={4,8,16,32};
                static constexpr unsigned glass_dwells[]={1,2,4,8};
                static const char *filter_name[]={"OFF","MEAN3","TRI5","MED3","MEAN5","MEAN7","TRI7","GAUSS5","MED5"};
                static const char *view_name[]={"WATER","LIVE-V","PEAK-V"};
                snprintf(value,sizeof(value),"MIN %lu",(unsigned long)s_glass_min_frequency.load(std::memory_order_relaxed));draw_quick(4,y[0],value,0);
                snprintf(value,sizeof(value),"MAX %lu",(unsigned long)s_glass_max_frequency.load(std::memory_order_relaxed));draw_quick(124,y[0],value,1);
                snprintf(value,sizeof(value),"RX %s %u",marauder_radio_rx_gain_is_auto()?"AGC":"MAN",marauder_radio_rx_gain_index());draw_quick(4,y[1],value,2);
                snprintf(value,sizeof(value),"RANGE %uM",c.span_mhz);draw_quick(124,y[1],value,3);
                snprintf(value,sizeof(value),"FILTER %s",filter_name[s_spectrum_filter.load(std::memory_order_relaxed)<kSpectrumFilterCount?s_spectrum_filter.load(std::memory_order_relaxed):0]);draw_quick(4,y[2],value,4);
                snprintf(value,sizeof(value),"STEP %luM",(unsigned long)glass_steps[std::min<uint8_t>(s_glass_step_index.load(std::memory_order_relaxed),3)]);draw_quick(124,y[2],value,5);
                snprintf(value,sizeof(value),"DWELL %u",glass_dwells[std::min<uint8_t>(s_glass_dwell_index.load(std::memory_order_relaxed),3)]);draw_quick(4,y[3],value,6);
                snprintf(value,sizeof(value),"VIEW %s",view_name[std::min<uint8_t>(glass_view,2)]);draw_quick(124,y[3],value,7);
                snprintf(value,sizeof(value),"%s",s_glass_paused.load(std::memory_order_relaxed)?"SWEEP PAUSED":"SWEEP RUN");draw_quick(4,y[4],value,8);
                snprintf(value,sizeof(value),"RANGE LOCK %s",s_glass_range_locked?"ON":"OFF");draw_quick(124,y[4],value,9);
            }else{
                previous_glass_center=0;
            }
            if(!s_looking_glass_active.load(std::memory_order_relaxed))s_radio_paused.store(c.paused,std::memory_order_relaxed);
        } else if(s_detail==Detail::REPLAY){
            auto c=settings_snapshot();char name[64],title[32],footer[31];copy_replay_name(name);
            if(screen_changed){marauder::display_clear(C_BLACK);s_header_force=true;s_footer_force=true;s_footer_valid=false;marauder::display_reset_waterfall();}
            snprintf(title,sizeof(title),"IQ REPLAY %.18s",name[0]?name:"NO FILE");
            snprintf(footer,sizeof(footer),"%s CENTER=PLAY/PAUSE",s_replay_active.load()?"PLAY":"STOP");
            if(redraw){draw_header(title,"");draw_footer(footer);}
            marauder::SpectrumFrame frame{};
            if(s_frame_queue&&xQueueReceive(s_frame_queue,&frame,0)==pdTRUE){uint8_t bins[240],peaks[240],wf[240];uint16_t row[240];project_trace(frame,c,bins,peaks);project_waterfall(frame,c,wf);for(unsigned i=0;i<240;++i)row[i]=marauder::waterfall_rgb565(wf[i],0,255,c.waterfall_palette);marauder::display_spectrum(bins,peaks,C_CYAN);marauder::display_waterfall_line(row);}
        } else if(redraw){
            if(screen_changed){const char *controls=s_detail==Detail::SETTINGS?"UP/DN ROW RIGHT CHANGE LEFT BACK":s_detail==Detail::RF_REPLAY?"UP/DN CAPTURE CENTER SAVE CFG LEFT BACK":s_detail==Detail::RF_CONFIGS?"UP/DN BROWSE CONFIGS LEFT BACK":s_detail==Detail::TX_TEST?"LAB ONLY: CENTER ARM, CENTER SEND":s_detail==Detail::GPS?"UP/DN CONTROL CENTER RUN LEFT BACK":s_detail==Detail::MEDIA?"UP/DN FILE CENTER OPEN HOLD EXIT":"UP/DN/RIGHT ACTION CENTER SELECT LEFT BACK";draw_detail_frame(controls);}
            auto c=settings_snapshot();char line[40];
            switch(s_detail){
                case Detail::SPECTRUM:break;
                case Detail::FREQ:draw_header("FREQUENCY CONTROL");snprintf(line,sizeof(line),"CENTER %lu MHz",(unsigned long)c.center_mhz);draw_line(1,line,c.center_mhz>MARAUDER_RADIO_FREQ_BAND_MAX_MHZ?C_RED:C_AMBER);draw_line(3,"LEFT/RIGHT: TUNE");draw_line(4,"UP/DOWN: DISPLAY SPAN");draw_line(6,"REQUEST LIMIT: 2800 MHz");draw_line(7,c.center_mhz>2651?"THEORY >2651 MHz":c.center_mhz>MARAUDER_RADIO_FREQ_BAND_MAX_MHZ?"ABOVE 2484: UNVERIFIED":"2484 MHz: BAND EDGE",c.center_mhz>MARAUDER_RADIO_FREQ_BAND_MAX_MHZ?C_RED:C_WHITE);break;
                case Detail::GAIN:{draw_header("RX GAIN / CAPTURE");draw_line(1,marauder_radio_rx_gain_is_auto()?"RX GAIN MODE: AGC":"RX GAIN MODE: MANUAL",C_AMBER);snprintf(line,sizeof(line),"MANUAL GAIN INDEX: %u",marauder_radio_rx_gain_index());draw_line(2,line);draw_line(3,"CENTER=AGC / MANUAL");draw_line(4,"U/D GAIN  RIGHT DC MODE");draw_line(5,s_dc_mode.load()?"DC MODE: FILL CENTER BIN":"DC MODE: SHOW AS IS");draw_line(6,"LO OFFSET NOT SUPPORTED");draw_line(7,"16 MS/s / 256 IQ8 SPARSE");break;}
                case Detail::TX_TEST:draw_header("CONTROLLED RF BENCH");draw_line(1,"SHIELDED / CABLED LAB ONLY",C_RED);draw_line(3,"ONE FIXED 802.11 TEST FRAME");draw_line(4,"CH 1 / 1 Mb/s / 32 BYTES");draw_line(5,"MAXIMUM SDK POWER BACKOFF");draw_line(6,s_tx_armed?"CENTER AGAIN TO SEND ONE":"CENTER TO ACK LAB SETUP",C_AMBER);draw_line(8,s_status);break;
                case Detail::SCANNER:draw_header("PASSIVE RECON");draw_line(1,s_recon_active.load()?"SCAN ACTIVE":"SCAN PAUSED",s_recon_active.load()?C_GREEN:C_AMBER);draw_line(3,"14 2.4 GHz CHANNEL PRESETS");draw_line(4,"RECORDS FREQUENCY / LEVEL");draw_line(6,"CENTER=START / STOP");break;
                case Detail::CAPTURE:draw_header("IQ CAPTURE");draw_line(1,s_capture_active.load()?"RECORDING IQ8":"READY",s_capture_active.load()?C_RED:C_CYAN);draw_line(3,"FORMAT: INTERLEAVED IQ8");draw_line(4,"RATE: 16 MS/s SNAPSHOTS");draw_line(5,marauder::storage_ready()?"SD CARD READY":"SD CARD UNAVAILABLE");draw_line(7,"USE CATEGORY ITEM TO START");break;
                case Detail::CAPTURE_FILES:{draw_header("CAPTURE MANAGER");static char names[24][64];size_t nf=list_capture_names(names,24);if(nf&&s_capture_cursor<nf){const char *name=names[s_capture_cursor];draw_line(1,name,C_AMBER);snprintf(line,sizeof(line),"%lu BYTES",(unsigned long)marauder::storage_file_size(name));draw_line(3,line);char meta[72],buf[128]{};snprintf(meta,sizeof(meta),"%s.META",name);size_t mn=marauder::storage_read_file(meta,0,buf,sizeof(buf)-1);buf[mn]=0;const char *freq=strstr(buf,"FREQUENCY_MHZ=");draw_line(4,freq?freq:"METADATA UNAVAILABLE");}else draw_line(1,"BACK",C_AMBER);draw_line(6,s_capture_delete_confirm?"HOLD AGAIN: DELETE PAIR":"HOLD: DELETE  SHORT: NEXT");draw_line(7,"SELECT BACK TO EXIT");break;}
                case Detail::REPLAY:{draw_header("OFFLINE IQ PLAYBACK");char name[64];copy_replay_name(name);draw_line(1,name[0]?name:"SELECT IQ8 CAPTURE",C_AMBER);draw_line(2,"LOCAL FFT VIEW ONLY");draw_line(3,s_replay_active.load()?"PLAYING THROUGH FFT":"PAUSED",s_replay_active.load()?C_GREEN:C_AMBER);if(name[0]){size_t size=marauder::storage_file_size(name);portENTER_CRITICAL(&s_state_mux);size_t pos=s_replay_offset;portEXIT_CRITICAL(&s_state_mux);snprintf(line,sizeof(line),"%u%%  %lu / %lu B",size?(unsigned)(std::min(pos,size)*100/size):0,(unsigned long)std::min(pos,size),(unsigned long)size);draw_line(4,line);}draw_line(6,"UP/DOWN=SELECT FILE");draw_line(7,"CENTER=PLAY / PAUSE");break;}
                case Detail::RF_REPLAY:draw_rf_replay_page();break;
                case Detail::RF_CONFIGS:draw_rf_configs_page();break;
                case Detail::GPS:{if(screen_changed&&s_gps_view==1)ESP_LOGI(kTag,"GPS track viewer entered; SD ready=%u accessible=%u",(unsigned)marauder::storage_ready(),(unsigned)marauder::storage_accessible());draw_header(s_gps_view==0?"GPS DASHBOARD":s_gps_view==1?"GPS TRACK VIEWER":"SAVED LOCATIONS");if(s_gps_view==0){auto f=marauder::gps_current_fix();draw_line(0,f.valid?"FIX VALID":"NO FIX / STALE",f.valid?C_GREEN:C_AMBER);char coord[32];format_coordinate(f.latitude,true,coord);draw_line(1,f.valid?coord:"LAT unavailable");format_coordinate(f.longitude,false,coord);draw_line(2,f.valid?coord:"LON unavailable");snprintf(line,sizeof(line),s_gps_imperial_units?"ALT %.1f ft":"ALT %.1f m",f.has_altitude?(s_gps_imperial_units?f.altitude_m*3.28084:f.altitude_m):0.0);draw_line(3,f.has_altitude?line:"ALT unavailable");snprintf(line,sizeof(line),s_gps_imperial_units?"SPD %.1f mph":"SPD %.1f m/s",f.has_speed?(s_gps_imperial_units?f.speed_mps*2.23694:f.speed_mps):0.0);draw_line(4,f.has_speed?line:"SPD unavailable");snprintf(line,sizeof(line),"COG %.1f deg",f.course_deg);draw_line(5,f.has_course?line:"COG unavailable");snprintf(line,sizeof(line),"QUALITY %u  SAT %u",f.fix_quality,f.satellites);draw_line(6,line);snprintf(line,sizeof(line),"HDOP %.2f",f.hdop);draw_line(7,f.has_hdop?line:"HDOP unavailable");snprintf(line,sizeof(line),"UTC %s",f.utc[0]?f.utc:"unavailable");draw_line(8,line);snprintf(line,sizeof(line),"NMEA %lu/s  AGE %lus",(unsigned long)f.update_hz,f.last_sentence_us?(unsigned long)((esp_timer_get_time()/1000000-f.last_sentence_us/1000000)):999ul);draw_line(9,line);snprintf(line,sizeof(line),"OK %lu BAD %lu UART ERR %lu",(unsigned long)f.valid_sentences,(unsigned long)f.invalid_sentences,(unsigned long)f.uart_errors);draw_line(10,line);const bool active=marauder::storage_gps_track_active();snprintf(line,sizeof(line),"LOG %s %lu PT %lus",!active?"STOP":marauder::storage_gps_track_paused()?"PAUSE":"REC",(unsigned long)marauder::storage_gps_track_points(),(unsigned long)marauder::storage_gps_track_elapsed_seconds());draw_line(11,line,active?C_GREEN:C_AMBER);draw_line(12,"GPS UART2 MODULE ALWAYS ON");snprintf(line,sizeof(line),"ACTION %s",s_gps_cursor==0?"START/RESUME":s_gps_cursor==1?"PAUSE":s_gps_cursor==2?"STOP":s_gps_cursor==3?"SAVE LOCATION":s_gps_cursor==4?s_gps_track_name:s_gps_cursor==5?(s_gps_coordinate_format==0?"DECIMAL DEGREES":s_gps_coordinate_format==1?"DEG+MIN":"DEG+MIN+SEC"):s_gps_cursor==6?(s_gps_imperial_units?"IMPERIAL":"METRIC"):"REFRESH POSITION");draw_line(13,line,C_CYAN);snprintf(line,sizeof(line),"LOG NAME %.8s",s_gps_track_name);draw_line(14,line);draw_line(15,"NAME: LEFT/RIGHT CHAR");draw_line(16,"UP/DN EDIT  LEFT/RIGHT CHAR");snprintf(line,sizeof(line),"COORD %s  UNITS %s",s_gps_coordinate_format==0?"DD":s_gps_coordinate_format==1?"DMM":"DMS",s_gps_imperial_units?"IMPERIAL":"METRIC");draw_line(17,line);}else if(s_gps_view==1){static char tracks[32][64];size_t nt=marauder::storage_list_files("GPS/TRACKS",".CSV",tracks,32);ESP_LOGI(kTag,"GPS track viewer found %u files",(unsigned)nt);if(!marauder::storage_accessible()){draw_line(1,"SD CARD / FILESYSTEM OFFLINE",C_RED);draw_line(3,"CHECK CARD THEN REBOOT");}else if(!nt){draw_line(1,"NO SAVED TRACK CSV FILES",C_AMBER);draw_line(3,"LOG A TRACK FROM GPS PAGE");}else{s_gps_cursor%=nt;const char*name=tracks[s_gps_cursor];snprintf(line,sizeof(line),"FILE %s",name);draw_line(0,line,C_AMBER);char path[96];snprintf(path,sizeof(path),"GPS/TRACKS/%s",name);size_t sz=marauder::storage_file_size(path);auto summary=summarize_track(path,sz);snprintf(line,sizeof(line),"SIZE %lu BYTES  POINTS %lu",(unsigned long)sz,(unsigned long)summary.points);draw_line(1,line);if(summary.have_points){snprintf(line,sizeof(line),"START %.4f,%.4f",summary.start_lat,summary.start_lon);draw_line(3,line);snprintf(line,sizeof(line),"END %.4f,%.4f",summary.end_lat,summary.end_lon);draw_line(4,line);snprintf(line,sizeof(line),s_gps_imperial_units?"END-END %.2f mi":"END-END %.2f km",s_gps_imperial_units?summary.distance_km*0.621371:summary.distance_km);draw_line(5,line);if(summary.has_duration){snprintf(line,sizeof(line),"DURATION %lu s",(unsigned long)summary.duration_s);draw_line(6,line);}}draw_line(7,"POINT COUNT / START / END");draw_line(8,"STRAIGHT-LINE DISTANCE ONLY");if(s_gps_point_review&&summary.points){s_gps_point_index%=summary.points;char point[160]{};if(read_track_point(path,s_gps_point_index,point)){snprintf(line,sizeof(line),"P%lu %.26s",(unsigned long)(s_gps_point_index+1),point);draw_line(10,line,C_CYAN);}draw_line(11,"CENTER EXIT POINT");}else draw_line(9,"UP/DN SELECT CENTER REVIEW");draw_track_graph(path,summary.points,screen_changed);}}else{char loc[1024]{};size_t z=marauder::storage_read_file("GPS/LOCATIONS.CSV",0,loc,sizeof(loc)-1);loc[z]=0;size_t rows=0;for(size_t i=0;i<z;++i)if(loc[i]=='\n')++rows;if(!marauder::storage_accessible())draw_line(1,"SD CARD / FILESYSTEM OFFLINE",C_RED);else if(!rows)draw_line(1,"NO SAVED LOCATIONS",C_AMBER);else{size_t target=s_gps_cursor%rows,start=0;for(size_t r=0;r<target;++r){char*p=strchr(loc+start,'\n');if(!p){start=z;break;}start=(size_t)(p-loc)+1;}if(start<z){char*p=strchr(loc+start,'\n');size_t n=p?(size_t)(p-(loc+start)):strlen(loc+start);if(n>30)n=30;memcpy(line,loc+start,n);line[n]=0;draw_line(1,line,C_AMBER);}snprintf(line,sizeof(line),"LOCATION %lu / %lu",(unsigned long)(s_gps_cursor+1),(unsigned long)rows);draw_line(3,line);draw_line(5,s_gps_delete_confirm?"CENTER AGAIN DELETE":"CENTER HOLD DELETE");}}break;}
                case Detail::PROFILES:{draw_header("RF PROFILE MANAGER");static char names[32][64];size_t np=marauder::storage_list_files("RFPROFILES",".RFP",names,32);const size_t actions=2+np*3;const size_t choice=s_profile_cursor%actions;char action[32];if(choice==0)snprintf(action,sizeof(action),"SAVE CURRENT");else if(choice==actions-1)snprintf(action,sizeof(action),"BACK");else{size_t ix=(choice-1)/3;unsigned op=(unsigned)((choice-1)%3);snprintf(action,sizeof(action),"%s %.17s",op==0?"LOAD":op==1?"DUP":"DELETE",names[ix]);}draw_line(1,action,C_AMBER);draw_line(3,"UP/DN SELECT ACTION");draw_line(4,"CENTER RUN ACTION");draw_line(5,"V1 TEXT FILES IN RFPROFILES");draw_line(6,"IMPORT/EXPORT VIA SD CARD");draw_line(7,s_status);break;}
                case Detail::FILES:br_draw(screen_changed);break;
                case Detail::APPRUN:draw_header(marauder::app_run_name());draw_footer("HOLD CENTER = EXIT");if(screen_changed)marauder::app_run_restart();break;
                case Detail::MAP_VIEW:case Detail::NAV_VIEW:case Detail::RID_VIEW:draw_header(marauder::map_ui_title());draw_footer(marauder::map_ui_footer());marauder::map_ui_draw(screen_changed);break;
                case Detail::FREQ_MANAGER:draw_header("FREQUENCY MANAGER");snprintf(line,sizeof(line),"PRESET CENTER %lu",(unsigned long)c.center_mhz);draw_line(1,line,c.center_mhz>MARAUDER_RADIO_FREQ_BAND_MAX_MHZ?C_RED:C_AMBER);draw_line(3,"CENTER=NEXT 5 MHz PRESET");draw_line(4,"REQUEST LIMIT: 2800 MHz");draw_line(5,c.center_mhz>2651?"THEORY >2651 MHz":c.center_mhz>MARAUDER_RADIO_FREQ_BAND_MAX_MHZ?"ABOVE 2484: UNVERIFIED":"2484 MHz: BAND EDGE",c.center_mhz>MARAUDER_RADIO_FREQ_BAND_MAX_MHZ?C_RED:C_WHITE);break;
                case Detail::ANTENNA:{draw_header("ANTENNA LENGTH");const double factor=s_antenna_mode==0?0.25:s_antenna_mode==1?0.5:0.625;const double length_in=(299792458.0/((double)c.center_mhz*1000000.0))*0.95*factor*39.3700787;snprintf(line,sizeof(line),"CENTER %lu MHz",(unsigned long)c.center_mhz);draw_line(1,line,C_AMBER);snprintf(line,sizeof(line),"%s: %.2f in",s_antenna_mode==0?"QUARTER WAVE":s_antenna_mode==1?"HALF WAVE":"5/8 WAVE",length_in);draw_line(3,line);draw_line(5,"VELOCITY FACTOR 0.95");draw_line(7,"CENTER=TYPE  LEFT=BACK");break;}
                case Detail::STOPWATCH:{draw_header("STOPWATCH");uint64_t ms=s_stopwatch_elapsed_ms;if(s_stopwatch_running)ms+=(uint64_t)esp_timer_get_time()/1000-s_stopwatch_started_ms;snprintf(line,sizeof(line),"%02lu:%02lu.%03lu",(unsigned long)(ms/60000),(unsigned long)((ms/1000)%60),(unsigned long)(ms%1000));draw_line(2,line,C_AMBER);draw_line(4,s_stopwatch_running?"RUNNING":"STOPPED",s_stopwatch_running?C_GREEN:C_CYAN);draw_line(6,"CENTER=START / STOP");draw_line(7,"HOLD CENTER=EXIT");break;}
                case Detail::RANDPWD:draw_header("RANDOM PASSWORD");draw_line(2,s_random_password,C_AMBER);draw_line(4,"16 HEX CHARACTERS");draw_line(6,"CENTER=GENERATE NEW");draw_line(7,"HOLD CENTER=BACK");break;
                case Detail::CALCULATOR:{draw_header("CALCULATOR");static const char *keys[]={"0","1","2","3","4","5","6","7","8","9","+","-","*","/","=","CLEAR","BACK"};snprintf(line,sizeof(line),"KEY: %s",keys[s_calc_key]);draw_line(1,line,C_AMBER);char op=s_calc_op<0?' ':s_calc_op==0?'+':s_calc_op==1?'-':s_calc_op==2?'*':'/';snprintf(line,sizeof(line),"%lld %c %lld",(long long)s_calc_lhs,op,(long long)s_calc_rhs);draw_line(3,line);draw_line(5,s_calc_error?"ERROR: DIVIDE BY ZERO":s_calc_result,s_calc_error?C_RED:C_GREEN);draw_line(7,"UP/DN KEY CENTER ENTER");break;}
                case Detail::IQ_TRIM:{draw_header("IQ TRIM / SILENCE CUT");static char names[24][64];size_t nf=list_capture_names(names,24);draw_line(1,nf&&s_capture_cursor<nf?names[s_capture_cursor]:"BACK",C_AMBER);draw_line(3,"UP/DOWN=SELECT SOURCE");draw_line(4,"CENTER=WRITE TRIMMED COPY");draw_line(5,"THRESHOLD: PEAK IQ > 3/127");draw_line(7,s_status);break;}
                case Detail::NOTEPAD:{draw_header("NOTEPAD / NOTES.TXT");const char *chars="ABCDEFGHIJKLMNOPQRSTUVWXYZ";if(s_note_char==28)snprintf(line,sizeof(line),"ACTION: BACK");else if(s_note_char==26)snprintf(line,sizeof(line),"ACTION: BACKSPACE");else if(s_note_char==27)snprintf(line,sizeof(line),"ACTION: SPACE");else snprintf(line,sizeof(line),"TYPE: %c",chars[s_note_char]);draw_line(1,line,C_AMBER);size_t start=s_note_scroll;for(int r=0;r<19&&start<s_note_len;r++){char row[31]{};size_t take=std::min<size_t>(30,s_note_len-start);memcpy(row,s_note+start,take);marauder::display_text(4,52+r*8,row,C_WHITE,C_BLACK);start+=take;}draw_line(22,"UP/DOWN NEXT  CENTER TYPE");draw_line(23,"HOLD CENTER TO EXIT");break;}
                case Detail::FLASH:draw_header("FLASH UTILITY");draw_line(1,"USE ROM BOOTLOADER");draw_line(3,"FLASH VIA ESP-IDF IN VS CODE");draw_line(4,"ARBITRARY SD BIN DISABLED");break;
                case Detail::DIAG:case Detail::DEBUG:{
                    draw_header("RUNTIME DIAGNOSTICS");
                    const uint32_t frames=s_frames.load(std::memory_order_relaxed);
                    const uint32_t captures=s_radio_capture_successes.load(std::memory_order_relaxed);
                    const uint32_t errors=s_radio_errors.load(std::memory_order_relaxed);
                    const uint32_t total=captures+errors;
                    const uint32_t ok_pct=total?(uint32_t)((uint64_t)captures*100/total):0;
                    const size_t internal_total=heap_caps_get_total_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
                    const size_t internal_free=heap_caps_get_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT);
                    const size_t psram_total=heap_caps_get_total_size(MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
                    const size_t psram_free=heap_caps_get_free_size(MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT);
                    const size_t dma_free=heap_caps_get_free_size(MALLOC_CAP_DMA);
                    const size_t dma_largest=heap_caps_get_largest_free_block(MALLOC_CAP_DMA);
                    const auto diag_settings=settings_snapshot();
                    const uint32_t uptime_s=(uint32_t)(esp_timer_get_time()/1000000);
                    snprintf(line,sizeof(line),"UPTIME %lu s",(unsigned long)uptime_s);draw_line(0,line);
                    snprintf(line,sizeof(line),"CAP OK %lu  FAIL %lu",(unsigned long)captures,(unsigned long)errors);draw_line(1,line);
                    snprintf(line,sizeof(line),"CAP SUCCESS %lu%% LAST %lu us",(unsigned long)ok_pct,(unsigned long)s_last_capture_us.load());draw_line(2,line);
                    snprintf(line,sizeof(line),"RADIO ERR %s",esp_err_to_name((esp_err_t)s_last_radio_error.load()));draw_line(3,line,s_last_radio_error.load()?C_AMBER:C_GREEN);
                    snprintf(line,sizeof(line),"FFT %lu PK %u >F %u %s",(unsigned long)frames,(unsigned)s_last_fft_peak.load(),(unsigned)s_bins_above_trace_floor.load(),s_trace_autoscale.load()?"AUTO":"MAN");draw_line(4,line);
                    snprintf(line,sizeof(line),"TUNE %lu MHz  SPAN %u",(unsigned long)diag_settings.center_mhz,diag_settings.span_mhz);draw_line(5,line);
                    snprintf(line,sizeof(line),"GAIN %s %u",(marauder_radio_rx_gain_is_auto()?"AUTO":"MAN"),marauder_radio_rx_gain_index());draw_line(6,line);
                    snprintf(line,sizeof(line),"RX %s %s",s_radio_paused.load()?"PAUSED":"RUN",s_replay_active.load()?"REPLAY":s_looking_glass_active.load()?"SWEEP":s_recon_active.load()?"SCAN":"LIVE");draw_line(7,line);
                    snprintf(line,sizeof(line),"INT HEAP %u / %u B",(unsigned)internal_free,(unsigned)internal_total);draw_line(8,line);
                    snprintf(line,sizeof(line),"INT LOW %u  MAX BLOCK %u",(unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT),(unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT));draw_line(9,line);
                    snprintf(line,sizeof(line),"DMA FREE %u  MAX %u",(unsigned)dma_free,(unsigned)dma_largest);draw_line(10,line);
                    if(psram_total)snprintf(line,sizeof(line),"PSRAM %u / %u B",(unsigned)psram_free,(unsigned)psram_total);
                    else snprintf(line,sizeof(line),"PSRAM UNAVAILABLE");
                    draw_line(11,line);
                    snprintf(line,sizeof(line),"STACK RADIO %u B",(unsigned)(s_radio_task_handle?uxTaskGetStackHighWaterMark(s_radio_task_handle):0));draw_line(12,line);
                    snprintf(line,sizeof(line),"STACK UI %u  GPS %u B",(unsigned)(s_ui_task_handle?uxTaskGetStackHighWaterMark(s_ui_task_handle):0),(unsigned)(s_gps_task_handle?uxTaskGetStackHighWaterMark(s_gps_task_handle):0));draw_line(13,line);
                    snprintf(line,sizeof(line),"SD %s  %s",marauder::storage_accessible()?"READY":"OFFLINE",esp_err_to_name(marauder::storage_last_error()));draw_line(14,line);
                    snprintf(line,sizeof(line),"QUEUE %u / 1",(unsigned)(s_frame_queue?uxQueueMessagesWaiting(s_frame_queue):0));draw_line(15,line);
                    const auto gps_diag=marauder::gps_current_fix();
                    snprintf(line,sizeof(line),"GPS %s SAT %u AGE %lus",gps_diag.valid?"FIX":"NO FIX",(unsigned)gps_diag.satellites,(unsigned long)(gps_diag.last_sentence_us?(esp_timer_get_time()-gps_diag.last_sentence_us)/1000000:999));draw_line(16,line);
                    snprintf(line,sizeof(line),"NMEA OK %lu BAD %lu",(unsigned long)gps_diag.valid_sentences,(unsigned long)gps_diag.invalid_sentences);draw_line(17,line);
                    break;
                }
                case Detail::STORAGE:{
                    if(screen_changed&&!marauder::storage_accessible())marauder::storage_retry_mount();
                    const bool accessible=marauder::storage_accessible();
                    char error[40];
                    snprintf(error,sizeof(error),"MOUNT: %s",esp_err_to_name(marauder::storage_last_error()));
                    draw_header("STORAGE DIAGNOSTICS");
                    draw_line(1,accessible?"SD CARD + FAT FS OK":"SD CARD / FAT FS OFFLINE",accessible?C_GREEN:C_RED);
                    draw_line(3,error,marauder::storage_ready()?C_CYAN:C_AMBER);
                    snprintf(line,sizeof(line),"STAGE %s",marauder::storage_last_stage());draw_line(4,line);
                    snprintf(line,sizeof(line),"SYS ERRNO %d",marauder::storage_last_errno());draw_line(8,line,marauder::storage_last_errno()?C_AMBER:C_WHITE);
                    snprintf(line,sizeof(line),"SEED APP %s WARD %s",
                             marauder::storage_file_exists("APPS/ANALOG_TV.APP")?"YES":"NO",
                             marauder::storage_file_exists("WARDRIVE.CSV")?"YES":"NO");
                    draw_line(9,line);
                    if(accessible){
                        char root_names[32][64],media_names[32][64];
                        const size_t entries=marauder::storage_list_files(".",nullptr,root_names,32);
                        const size_t media=marauder::media_list_files(media_names,32);
                        snprintf(line,sizeof(line),"ROOT %u MEDIA %u",(unsigned)entries,(unsigned)media);
                        draw_line(5,line);
                        if(entries){snprintf(line,sizeof(line),"ROOT: %.26s",root_names[0]);draw_line(6,line);}
                        else draw_line(6,"ROOT DIRECTORY EMPTY",C_AMBER);
                        if(media){snprintf(line,sizeof(line),"MEDIA: %.24s",media_names[0]);draw_line(7,line,C_CYAN);}
                        else draw_line(7,"NO JPG/PNG/MP4 FOUND",C_AMBER);
                    }else{
                        draw_line(5,"CHECK FAT32 / SD WIRING",C_AMBER);
                        draw_line(6,"RETRY ON ENTRY; NO FORMAT");
                    }
                    break;
                }
                case Detail::DEVICE:{draw_header("GPS / DEVICE INFO");auto fix=marauder::gps_current_fix();draw_line(1,fix.valid?"GPS FIX VALID":"GPS NO FIX",fix.valid?C_GREEN:C_AMBER);if(fix.valid){snprintf(line,sizeof(line),"LAT %.5f",fix.latitude);draw_line(2,line);snprintf(line,sizeof(line),"LON %.5f",fix.longitude);draw_line(3,line);draw_line(4,fix.utc);}draw_line(6,"ESP32 MARAUDER V7.2 BY");draw_line(7,"JUSTCALLMEKOKO");{const int scl=marauder::battery_scl_pin();if(scl>=0)snprintf(line,sizeof(line),"GAUGE SDA33 SCL%-2d %3d%%",scl,marauder::battery_percent());else snprintf(line,sizeof(line),"GAUGE NONE (SDA33)       ");draw_line(9,line,scl>=0?C_GREEN:C_AMBER);}draw_line(10,"HW V7.2 BATTERY POLARITY REV");{const esp_reset_reason_t rr=esp_reset_reason();snprintf(line,sizeof(line),"LAST RESET %s",rr==ESP_RST_POWERON?"POWER ON":rr==ESP_RST_SW?"SOFTWARE":rr==ESP_RST_PANIC?"PANIC":rr==ESP_RST_BROWNOUT?"BROWNOUT":rr==ESP_RST_TASK_WDT?"TASK WDT":rr==ESP_RST_INT_WDT?"INT WDT":rr==ESP_RST_WDT?"WDT":"OTHER");draw_line(11,line);}break;}
                case Detail::SETTINGS:{draw_header("SDR + JOYSTICK SETTINGS");const char *labels[]={"BACK","CENTER MHZ","SPAN ZOOM","STEP MHZ","WF FLOOR","WF CEILING","PALETTE","AVERAGING","HANN WINDOW","MAX HOLD","PAUSE RX","TRACE FLOOR","TRACE CEIL","SAVE PROFILE","DEFAULTS","REPEAT DELAY","REPEAT RATE","LONG PRESS","ACCELERATION","INVERT VERT","INVERT HORIZ","DEBOUNCE"};char values[22][16]{};snprintf(values[0],16,"HOME");snprintf(values[1],16,"%lu",(unsigned long)c.center_mhz);snprintf(values[2],16,"%u MHz",c.span_mhz);snprintf(values[3],16,"%u",c.frequency_step);snprintf(values[4],16,"%u",c.intensity_floor);snprintf(values[5],16,"%u",c.intensity_ceiling);snprintf(values[6],16,"%s",c.waterfall_palette==0?"MAGMA":c.waterfall_palette==1?"INFERNO":"GRAY");snprintf(values[7],16,"%u",c.averaging);snprintf(values[8],16,"%s",c.hann_window?"ON":"OFF");snprintf(values[9],16,"%s",c.max_hold?"ON":"OFF");snprintf(values[10],16,"%s",s_radio_paused.load()?"PAUSED":"LIVE");snprintf(values[11],16,"%u",c.trace_floor);snprintf(values[12],16,"%u",c.trace_ceiling);snprintf(values[13],16,"SAVE");snprintf(values[14],16,"RESET");snprintf(values[15],16,"%ums",c.joystick_repeat_delay_ms);snprintf(values[16],16,"%ums",c.joystick_repeat_period_ms);snprintf(values[17],16,"%ums",c.joystick_long_press_ms);snprintf(values[18],16,"%u",c.joystick_acceleration);snprintf(values[19],16,"%s",c.joystick_invert_vertical?"ON":"OFF");snprintf(values[20],16,"%s",c.joystick_invert_horizontal?"ON":"OFF");snprintf(values[21],16,"%ums",c.joystick_debounce_ms);for(int i=0;i<22;i++)draw_value_row(i,labels[i],values[i],(size_t)i==s_category_selected);break;}
                case Detail::EXTERNALS:{draw_header("EXTERNAL APPS");auto &names=s_ext_names;size_t nf=marauder::storage_list_files("APPS",".APP",names,24);if(!nf){draw_line(1,"NO APPS/*.APP MANIFESTS");draw_line(3,"UP/DOWN NEXT ENTRY");draw_line(4,"LEFT BACK / HOLD CENTER");break;}const char *name=names[s_external_cursor%nf];char path[96],manifest[513]{};snprintf(path,sizeof(path),"APPS/%s",name);size_t bytes=marauder::storage_read_file(path,0,manifest,sizeof(manifest)-1);manifest[bytes]=0;auto field=[&](const char *key)->const char*{static char val[72];char token[32];snprintf(token,sizeof(token),"%s=",key);const char *p=strstr(manifest,token);if(!p)return "(missing)";p+=strlen(token);size_t z=strcspn(p,"\r\n");z=std::min(z,sizeof(val)-1);memcpy(val,p,z);val[z]=0;return val;};draw_line(1,field("NAME"),C_AMBER);char value[40];snprintf(value,sizeof(value),"CATEGORY %s",field("CATEGORY"));draw_line(3,value);snprintf(value,sizeof(value),"VERSION %s",field("VERSION"));draw_line(4,value);draw_line(5,field("DESCRIPTION"));const bool manifest_ok=strstr(manifest,"MARAUDER_APP=1")&&strstr(manifest,"API=1");draw_line(6,manifest_ok?"API 1 METADATA OK":"API MISMATCH / UNKNOWN",manifest_ok?C_GREEN:C_RED);{const bool runnable=manifest_ok&&strstr(manifest,"RUNTIME=VM1");draw_line(7,runnable?"RUNTIME VM1: CENTER = RUN":"DISCOVERY ONLY: NOT RUNNABLE",runnable?C_GREEN:C_AMBER);}draw_line(8,"UP/DN NEXT LEFT BACK");break;}
                default:draw_line(2,"APPLICATION NOT AVAILABLE");break;
            }
        }
        if(redraw){old_page=s_page;old_category=s_category;old_detail=s_detail;old_home=s_home_selected;old_category_index=s_category_selected;last_refresh=now;first=false;}
        draw_status_bar(screen_changed);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
}

extern "C" void app_main(void) {
    using namespace marauder;
    gpio_config_t out{};out.pin_bit_mask=1ULL<<pins::led_a;out.mode=GPIO_MODE_OUTPUT;ESP_ERROR_CHECK(gpio_config(&out));
    ESP_ERROR_CHECK(input_init());
    spi_gate_init();ESP_ERROR_CHECK(display_bus_init());
    esp_err_t sd=storage_init();if(sd!=ESP_OK)ESP_LOGW(kTag,"SD unavailable: %s",esp_err_to_name(sd));
    ESP_ERROR_CHECK(display_init());ESP_ERROR_CHECK(gps_init());ESP_ERROR_CHECK(marauder_radio_init());battery_init();thermal_init();marauder::dsp_cfg_load();gps_ui_preferences();dc_mode_load();
    AppSettings cfg{};ESP_ERROR_CHECK(settings_load(&cfg));portENTER_CRITICAL(&s_state_mux);s_settings=cfg;portEXIT_CRITICAL(&s_state_mux);
    s_radio_paused.store(cfg.paused);spectrum_filter_load();s_frame_queue=xQueueCreateStatic(1,sizeof(SpectrumFrame),s_frame_queue_storage,&s_frame_queue_control);configASSERT(s_frame_queue);
    BaseType_t a=xTaskCreatePinnedToCore(gps_task_entry,"gps",3072,nullptr,3,&s_gps_task_handle,1);
    BaseType_t b=xTaskCreatePinnedToCore(radio_task,"radio_dsp",8192,nullptr,8,&s_radio_task_handle,0);
    BaseType_t c=xTaskCreatePinnedToCore(ui_task,"display_ui",12288,nullptr,5,&s_ui_task_handle,1);
    configASSERT(a==pdPASS&&b==pdPASS&&c==pdPASS);
}
