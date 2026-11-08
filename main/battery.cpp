#include "include/battery.hpp"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <algorithm>
#include <atomic>
#include <cstdint>

namespace marauder {
namespace {
constexpr char kTag[]="battery";
constexpr gpio_num_t kGaugeSda=GPIO_NUM_33;
constexpr gpio_num_t kGaugeScl=GPIO_NUM_16;
constexpr uint8_t kMax17048Address=0x36;
constexpr uint8_t kSocRegister=0x04;
constexpr unsigned kSocReadCount=5;
constexpr uint16_t kMaxSocQ8=100u<<8;
i2c_master_bus_handle_t s_bus{};
i2c_master_dev_handle_t s_gauge{};
std::atomic<int> s_percent{-1};
int32_t s_filtered_soc_q8=-1;
}

void battery_init() {
    i2c_master_bus_config_t bus{};
    bus.i2c_port=I2C_NUM_0;
    bus.sda_io_num=kGaugeSda;
    bus.scl_io_num=kGaugeScl;
    bus.clk_source=I2C_CLK_SRC_DEFAULT;
    bus.glitch_ignore_cnt=7;
    bus.flags.enable_internal_pullup=true;
    esp_err_t error=i2c_new_master_bus(&bus,&s_bus);
    if(error!=ESP_OK){s_bus=nullptr;ESP_LOGW(kTag,"I2C bus init failed: %s",esp_err_to_name(error));return;}
    i2c_device_config_t dev{};
    dev.dev_addr_length=I2C_ADDR_BIT_LEN_7;
    dev.device_address=kMax17048Address;
    dev.scl_speed_hz=100000;
    error=i2c_master_bus_add_device(s_bus,&dev,&s_gauge);
    if(error!=ESP_OK){s_gauge=nullptr;ESP_LOGW(kTag,"MAX17048 not available: %s",esp_err_to_name(error));}
    battery_update();
}

void battery_update() {
    if(!s_gauge){s_percent.store(-1,std::memory_order_relaxed);return;}
    uint16_t samples[kSocReadCount]{};
    size_t valid_samples=0;
    for(unsigned i=0;i<kSocReadCount;++i){
        uint8_t raw[2]{};
        const esp_err_t error=i2c_master_transmit_receive(
            s_gauge,&kSocRegister,1,raw,sizeof(raw),20);
        if(error!=ESP_OK){
            ESP_LOGW(kTag,"SOC read %u/%u failed: %s",i+1,kSocReadCount,esp_err_to_name(error));
            continue;
        }
        const uint16_t soc_q8=static_cast<uint16_t>((raw[0]<<8)|raw[1]);
        if(soc_q8>kMaxSocQ8)continue;
        samples[valid_samples++]=soc_q8;
    }
    if(valid_samples<3){
        ESP_LOGW(kTag,"only %u valid SOC samples",static_cast<unsigned>(valid_samples));
        s_percent.store(-1,std::memory_order_relaxed);return;
    }
    for(size_t i=0;i<valid_samples;++i){
        for(size_t j=i+1;j<valid_samples;++j){
            if(samples[j]<samples[i]){
                const uint16_t value=samples[i];
                samples[i]=samples[j];
                samples[j]=value;
            }
        }
    }
    const int32_t measured_q8=samples[valid_samples/2];
    if(s_filtered_soc_q8<0||measured_q8<=s_filtered_soc_q8){
        s_filtered_soc_q8=measured_q8;
    }else{
        s_filtered_soc_q8+=std::min<int32_t>(measured_q8-s_filtered_soc_q8,1<<8);
    }
    s_percent.store(s_filtered_soc_q8>>8,std::memory_order_relaxed);
}

int battery_percent(){return s_percent.load(std::memory_order_relaxed);}
}
