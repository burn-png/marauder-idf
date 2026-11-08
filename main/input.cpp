#include "include/input.hpp"
#include "include/pins.hpp"
#include "driver/gpio.h"
#include "esp_timer.h"

namespace {
struct KeyState { gpio_num_t pin; bool stable; bool candidate; int64_t changed_us; int64_t pressed_us; int64_t repeat_us; uint16_t repeats; bool long_sent; };
KeyState s_keys[] = {
    {marauder::pins::joy_up,false,false,0,0,0,0,false}, {marauder::pins::joy_down,false,false,0,0,0,0,false},
    {marauder::pins::joy_left,false,false,0,0,0,0,false}, {marauder::pins::joy_right,false,false,0,0,0,0,false},
    {marauder::pins::joy_center,false,false,0,0,0,0,false}
};
constexpr marauder::InputKey kNames[] = {marauder::InputKey::UP,marauder::InputKey::DOWN,marauder::InputKey::LEFT,marauder::InputKey::RIGHT,marauder::InputKey::CENTER};
}
namespace marauder {
esp_err_t input_init() {
    // GPIO34/35/36/39 are ESP32 input-only pads. The board provides the
    // pull-up network; only GPIO13 uses the ESP32 internal pull-up.
    for (auto &k : s_keys) {
        gpio_config_t c{}; c.pin_bit_mask = 1ULL << k.pin; c.mode=GPIO_MODE_INPUT;
        c.pull_up_en=(k.pin==GPIO_NUM_13)?GPIO_PULLUP_ENABLE:GPIO_PULLUP_DISABLE;
        c.pull_down_en=GPIO_PULLDOWN_DISABLE; c.intr_type=GPIO_INTR_DISABLE;
        esp_err_t e=gpio_config(&c); if(e!=ESP_OK)return e;
        k.stable=false;k.candidate=false;k.changed_us=esp_timer_get_time();
    }
    return ESP_OK;
}
InputEvent input_next_event(const InputConfig &cfg) {
    const int64_t now=esp_timer_get_time();
    const int64_t debounce=(int64_t)cfg.debounce_ms*1000;
    for(size_t i=0;i<sizeof(s_keys)/sizeof(s_keys[0]);++i){
        auto &k=s_keys[i];size_t physical=i;
        if(cfg.invert_vertical){if(i==0)physical=1;else if(i==1)physical=0;}
        if(cfg.invert_horizontal){if(i==2)physical=3;else if(i==3)physical=2;}
        bool raw=gpio_get_level(s_keys[physical].pin)==0;
        if(raw!=k.candidate){k.candidate=raw;k.changed_us=now;}
        if(raw!=k.stable && now-k.changed_us>=debounce){
            k.stable=raw;
            if(raw){k.pressed_us=now;k.repeat_us=now+(int64_t)cfg.repeat_delay_ms*1000;k.repeats=0;k.long_sent=false;return{kNames[i],InputKind::PRESS,0};}
            if(!k.long_sent)return{kNames[i],InputKind::SHORT_RELEASE,k.repeats};
        }
        if(!k.stable)continue;
        if(!k.long_sent && now-k.pressed_us >= (int64_t)cfg.long_press_ms*1000){k.long_sent=true;return{kNames[i],InputKind::LONG_PRESS,k.repeats};}
        if(kNames[i]!=InputKey::CENTER && now>=k.repeat_us){++k.repeats;uint16_t period=cfg.repeat_period_ms;
            if(cfg.acceleration && k.repeats>8){uint16_t divisor=cfg.acceleration==1?2:cfg.acceleration==2?3:4;period=(uint16_t)(period/divisor);if(period<30)period=30;}
            k.repeat_us=now+(int64_t)period*1000;return{kNames[i],InputKind::REPEAT,k.repeats};}
    }
    return{InputKey::NONE,InputKind::NONE,0};
}
}
