#include "include/battery.hpp"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <algorithm>
#include <atomic>
#include <cstdint>

namespace marauder {
namespace {
constexpr char kTag[]="battery";
constexpr gpio_num_t kGaugeSda=GPIO_NUM_33;
// Marauder V7 puts the fuel gauge on SCL = GPIO16, V7.1 on GPIO27 (same SDA, same MAX17048 at 0x36).
// V7.2 (battery-polarity revision, the target board) has no documented pin change; both are probed.
// Both are tried so either revision works.
constexpr gpio_num_t kGaugeScl[]={GPIO_NUM_16,GPIO_NUM_27};
constexpr uint8_t kMax17048Address=0x36;
constexpr uint8_t kSocRegister=0x04;
constexpr uint8_t kCmdRegister=0xFE;
constexpr uint16_t kMaxSocQ8=100u<<8;
i2c_master_bus_handle_t s_bus{};
i2c_master_dev_handle_t s_gauge{};
std::atomic<int> s_percent{-1};
std::atomic<int> s_scl{-1};
unsigned s_failures{};
unsigned s_probe_start{};
bool s_reset_sent{};
constexpr unsigned kFailuresBeforeInvalid=3;

void drop_bus(){
    if(s_gauge){i2c_master_bus_rm_device(s_gauge);s_gauge=nullptr;}
    if(s_bus){i2c_del_master_bus(s_bus);s_bus=nullptr;}
}

bool open_bus(gpio_num_t scl){
    i2c_master_bus_config_t bus{};
    bus.i2c_port=I2C_NUM_0;
    bus.sda_io_num=kGaugeSda;
    bus.scl_io_num=scl;
    bus.clk_source=I2C_CLK_SRC_DEFAULT;
    bus.glitch_ignore_cnt=7;
    bus.flags.enable_internal_pullup=true;
    if(i2c_new_master_bus(&bus,&s_bus)!=ESP_OK){s_bus=nullptr;return false;}
    if(i2c_master_probe(s_bus,kMax17048Address,50)!=ESP_OK){i2c_del_master_bus(s_bus);s_bus=nullptr;return false;}
    s_scl.store((int)scl,std::memory_order_relaxed);
    return true;
}

bool ensure_gauge() {
    if(!s_bus){
        // Find the gauge: try each known SCL pin, starting with the one tried least recently.
        bool found=false;
        for(unsigned n=0;n<sizeof(kGaugeScl)/sizeof(*kGaugeScl)&&!found;++n){
            const unsigned i=(s_probe_start+n)%(sizeof(kGaugeScl)/sizeof(*kGaugeScl));
            if(open_bus(kGaugeScl[i])){found=true;ESP_LOGI(kTag,"MAX17048 found on SDA33 SCL%d",(int)kGaugeScl[i]);}
        }
        s_probe_start=(s_probe_start+1)%(sizeof(kGaugeScl)/sizeof(*kGaugeScl));
        if(!found){ESP_LOGW(kTag,"MAX17048 not found (tried SCL16 and SCL27)");return false;}
        s_reset_sent=false;
    }
    if(!s_gauge){
        i2c_device_config_t dev{};
        dev.dev_addr_length=I2C_ADDR_BIT_LEN_7;
        dev.device_address=kMax17048Address;
        dev.scl_speed_hz=100000;
        const esp_err_t error=i2c_master_bus_add_device(s_bus,&dev,&s_gauge);
        if(error!=ESP_OK){s_gauge=nullptr;ESP_LOGW(kTag,"MAX17048 setup failed: %s",esp_err_to_name(error));drop_bus();return false;}
    }
    if(!s_reset_sent){
        // Same first step as Marauder (Adafruit_MAX1704X::begin): power-on reset command. The chip resets
        // before it ACKs, so a NACK is expected and ignored; the state of charge is re-estimated afterwards.
        const uint8_t reset[3]={kCmdRegister,0x54,0x00};
        (void)i2c_master_transmit(s_gauge,reset,sizeof(reset),20);
        s_reset_sent=true;
        vTaskDelay(pdMS_TO_TICKS(150));
    }
    return true;
}
}

void battery_init() { battery_update(); }

// A failed read keeps the last good value for a few attempts (one glitchy transaction should not blank
// the gauge), then reports -1 and tears the I2C device down so the next attempt rebuilds it from scratch.
static void note_failure(){
    if(++s_failures>=kFailuresBeforeInvalid){
        s_percent.store(-1,std::memory_order_relaxed);
        s_scl.store(-1,std::memory_order_relaxed);
        drop_bus(); // rebuild (and re-probe both SCL pins) on the next update
        s_failures=kFailuresBeforeInvalid;
    }
}

void battery_update() {
    if(!ensure_gauge()){note_failure();return;}
    uint8_t raw[2]{};
    const esp_err_t error=i2c_master_transmit_receive(s_gauge,&kSocRegister,1,raw,sizeof(raw),20);
    if(error!=ESP_OK){
        ESP_LOGW(kTag,"MAX17048 SOC read failed: %s",esp_err_to_name(error));
        note_failure();
        return;
    }
    s_failures=0;
    const uint16_t soc_q8=static_cast<uint16_t>((raw[0]<<8)|raw[1]);
    if(soc_q8==0xFFFF){note_failure();return;} // bus floating / no device driving the lines
    const unsigned percent=std::min<unsigned>(soc_q8>>8,kMaxSocQ8>>8);
    s_percent.store(static_cast<int>(percent),std::memory_order_relaxed);
}

int battery_scl_pin(){return s_scl.load(std::memory_order_relaxed);}
int battery_percent(){return s_percent.load(std::memory_order_relaxed);}
}
