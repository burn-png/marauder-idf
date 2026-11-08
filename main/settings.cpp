#include "include/settings.hpp"
#include "radio_backend.h"
#include "nvs.h"
#include <cstring>

namespace marauder {
namespace {
constexpr uint32_t kSettingsMagic = 0x4D415231; // MAR1
constexpr uint16_t kSettingsVersion = 4;
struct SettingsRecord { uint32_t magic; uint16_t version; uint16_t size; AppSettings value; };
struct LegacySettings { uint32_t center_mhz;uint16_t span_mhz;uint16_t frequency_step;uint8_t intensity_floor,intensity_ceiling,waterfall_palette,averaging,history_depth;bool hann_window,max_hold,paused; };
bool valid_except_center(const AppSettings &s) {
    return s.frequency_step >= 1 && s.frequency_step <= 100 &&
        s.span_mhz >= 1 && s.span_mhz <= 16 && s.intensity_floor < s.intensity_ceiling &&
        s.waterfall_palette <= 2 && s.averaging >= 1 && s.averaging <= 8 &&
        s.history_depth >= 1 && s.history_depth <= 64 && s.trace_floor < s.trace_ceiling &&
        s.joystick_repeat_delay_ms >= 100 && s.joystick_repeat_delay_ms <= 1000 &&
        s.joystick_repeat_period_ms >= 40 && s.joystick_repeat_period_ms <= 500 &&
        s.joystick_long_press_ms >= 300 && s.joystick_long_press_ms <= 1500 &&
        s.joystick_debounce_ms >= 5 && s.joystick_debounce_ms <= 100 && s.joystick_acceleration <= 3;
}
bool valid(const AppSettings &s) {
    return s.center_mhz >= MARAUDER_RADIO_FREQ_MIN_MHZ &&
        s.center_mhz <= MARAUDER_RADIO_FREQ_MAX_MHZ && valid_except_center(s);
}
}
AppSettings settings_defaults() {
    return {2412, 16, 5, 20, 220, 0, 1, 24, false, true, false, 20, 220, 220, 100, 650, 12, 2, false, false};
}

esp_err_t settings_load(AppSettings *settings) {
    if (!settings) return ESP_ERR_INVALID_ARG;
    *settings = settings_defaults();
    nvs_handle_t nvs;
    esp_err_t e = nvs_open("marauder", NVS_READONLY, &nvs);
    if (e == ESP_ERR_NVS_NOT_FOUND) return ESP_OK;
    if (e != ESP_OK) return e;
    SettingsRecord record{};
    size_t size = sizeof(record);
    e = nvs_get_blob(nvs, "prefs", &record, &size);
    nvs_close(nvs);
    if (e == ESP_OK && size == sizeof(record) && record.magic == kSettingsMagic &&
        record.version == kSettingsVersion && record.size == sizeof(AppSettings)) {
        if (valid(record.value)) {
            *settings = record.value;
            return ESP_OK;
        }
        if (valid_except_center(record.value)) {
            *settings = record.value;
            if (settings->center_mhz < MARAUDER_RADIO_FREQ_MIN_MHZ)
                settings->center_mhz = MARAUDER_RADIO_FREQ_MIN_MHZ;
            else if (settings->center_mhz > MARAUDER_RADIO_FREQ_MAX_MHZ)
                settings->center_mhz = MARAUDER_RADIO_FREQ_MAX_MHZ;
            settings_save(*settings);
            return ESP_OK;
        }
    }
    // Keep existing v3 preferences while migrating the old slow joystick defaults.
    struct V3Record { uint32_t magic; uint16_t version; uint16_t size; AppSettings value; } v3{};
    size=sizeof(v3);
    if(nvs_open("marauder",NVS_READONLY,&nvs)==ESP_OK){
        esp_err_t old=nvs_get_blob(nvs,"prefs",&v3,&size);nvs_close(nvs);
        if(old==ESP_OK&&size==sizeof(v3)&&v3.magic==kSettingsMagic&&v3.version==3&&v3.size==sizeof(AppSettings)){
            auto upgraded=v3.value;
            if(upgraded.joystick_repeat_delay_ms>250)upgraded.joystick_repeat_delay_ms=250;
            if(upgraded.joystick_debounce_ms>15)upgraded.joystick_debounce_ms=15;
            if(upgraded.joystick_debounce_ms<5)upgraded.joystick_debounce_ms=12;
            if(!valid(upgraded))upgraded=settings_defaults();
            *settings=upgraded;
            settings_save(upgraded);
            return ESP_OK;
        }
    }
    // Preserve configuration saved by firmware v2 while adding joystick settings.
    struct V2Record { uint32_t magic; uint16_t version; uint16_t size; struct V2Settings { uint32_t center_mhz; uint16_t span_mhz,frequency_step; uint8_t intensity_floor,intensity_ceiling,waterfall_palette,averaging,history_depth; bool hann_window,max_hold,paused; uint8_t trace_floor,trace_ceiling; } value; } v2{};
    size=sizeof(v2);
    if((e==ESP_ERR_NVS_INVALID_LENGTH || e==ESP_OK) && nvs_open("marauder",NVS_READONLY,&nvs)==ESP_OK){
        esp_err_t old=nvs_get_blob(nvs,"prefs",&v2,&size);nvs_close(nvs);
        if(old==ESP_OK && size==sizeof(v2) && v2.magic==kSettingsMagic && v2.version==2 && v2.size==sizeof(v2.value)){
            auto upgraded=settings_defaults();upgraded.center_mhz=v2.value.center_mhz;upgraded.span_mhz=v2.value.span_mhz;upgraded.frequency_step=v2.value.frequency_step;upgraded.intensity_floor=v2.value.intensity_floor;upgraded.intensity_ceiling=v2.value.intensity_ceiling;upgraded.waterfall_palette=v2.value.waterfall_palette;upgraded.averaging=v2.value.averaging;upgraded.history_depth=v2.value.history_depth;upgraded.hann_window=v2.value.hann_window;upgraded.max_hold=v2.value.max_hold;upgraded.paused=v2.value.paused;upgraded.trace_floor=v2.value.trace_floor;upgraded.trace_ceiling=v2.value.trace_ceiling;*settings=upgraded;return ESP_OK;
        }
    }
    // Upgrade the original unversioned preferences blob once, if present.
    LegacySettings legacy{};
    size = sizeof(legacy);
    if ((e == ESP_ERR_NVS_INVALID_LENGTH || e==ESP_OK) && nvs_open("marauder", NVS_READONLY, &nvs) == ESP_OK) {
        esp_err_t old = nvs_get_blob(nvs, "prefs", &legacy, &size);
        nvs_close(nvs);
        if (old == ESP_OK && size == sizeof(legacy) &&
            legacy.center_mhz >= MARAUDER_RADIO_FREQ_MIN_MHZ &&
            legacy.center_mhz <= MARAUDER_RADIO_FREQ_MAX_MHZ &&
            legacy.frequency_step>=1&&legacy.frequency_step<=100&&legacy.span_mhz>=1&&legacy.span_mhz<=16&&legacy.intensity_floor<legacy.intensity_ceiling&&legacy.waterfall_palette<=2&&legacy.averaging>=1&&legacy.averaging<=8&&legacy.history_depth>=1&&legacy.history_depth<=64) {
            auto upgraded=settings_defaults();upgraded.center_mhz=legacy.center_mhz;upgraded.span_mhz=legacy.span_mhz;upgraded.frequency_step=legacy.frequency_step;upgraded.intensity_floor=legacy.intensity_floor;upgraded.intensity_ceiling=legacy.intensity_ceiling;upgraded.waterfall_palette=legacy.waterfall_palette;upgraded.averaging=legacy.averaging;upgraded.history_depth=legacy.history_depth;upgraded.hann_window=legacy.hann_window;upgraded.max_hold=legacy.max_hold;upgraded.paused=legacy.paused;*settings=upgraded;
            return ESP_OK;
        }
    }
    *settings = settings_defaults();
    return (e == ESP_ERR_NVS_NOT_FOUND || e == ESP_ERR_NVS_INVALID_LENGTH || e == ESP_OK) ? ESP_OK : e;
}

esp_err_t settings_save(const AppSettings &settings) {
    if (!valid(settings)) return ESP_ERR_INVALID_ARG;
    nvs_handle_t nvs;
    esp_err_t e = nvs_open("marauder", NVS_READWRITE, &nvs);
    if (e != ESP_OK) return e;
    SettingsRecord record{kSettingsMagic, kSettingsVersion, sizeof(AppSettings), settings};
    e = nvs_set_blob(nvs, "prefs", &record, sizeof(record));
    if (e == ESP_OK) e = nvs_commit(nvs);
    nvs_close(nvs);
    return e;
}
}
