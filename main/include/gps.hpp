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
    // ---- appended (GST/GSA/VTG); older code that only reads the fields above is unaffected ----
    bool has_gst;                       // receiver supplied pseudorange error statistics (GST)
    double sigma_lat_m, sigma_lon_m, sigma_alt_m;
    bool has_dop;                       // GSA
    double pdop, vdop;
    uint8_t fix_type;                   // GSA: 1 = none, 2 = 2D, 3 = 3D, 0 = not reported
    bool has_vtg; double vtg_course_deg, vtg_speed_mps;
    uint32_t fix_seq;                   // increments with every accepted position update
};
esp_err_t gps_init();
void gps_task(void *);
GpsFix gps_current_fix();
}
