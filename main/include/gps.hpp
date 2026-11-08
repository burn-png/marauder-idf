#pragma once
#include "esp_err.h"

namespace marauder {
struct GpsFix {
    bool valid; double latitude; double longitude; char utc[24];
    bool has_altitude, has_speed, has_course, has_hdop;
    double altitude_m, speed_mps, course_deg, hdop;
    uint8_t fix_quality, satellites;
    uint32_t valid_sentences, invalid_sentences, uart_errors, update_hz;
    uint64_t last_sentence_us, last_fix_us;
};
esp_err_t gps_init();
void gps_task(void *);
GpsFix gps_current_fix();
}
