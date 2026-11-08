#pragma once
#include <stdint.h>
#include "esp_err.h"

namespace marauder {
struct AppSettings {
    uint32_t center_mhz;
    uint16_t span_mhz;
    uint16_t frequency_step;
    uint8_t intensity_floor;
    uint8_t intensity_ceiling;
    uint8_t waterfall_palette;
    uint8_t averaging;
    uint8_t history_depth;
    bool hann_window;
    bool max_hold;
    bool paused;
    uint8_t trace_floor;
    uint8_t trace_ceiling;
    uint16_t joystick_repeat_delay_ms;
    uint16_t joystick_repeat_period_ms;
    uint16_t joystick_long_press_ms;
    uint16_t joystick_debounce_ms;
    uint8_t joystick_acceleration;
    bool joystick_invert_vertical;
    bool joystick_invert_horizontal;
};

AppSettings settings_defaults();
esp_err_t settings_load(AppSettings *settings);
esp_err_t settings_save(const AppSettings &settings);
}
