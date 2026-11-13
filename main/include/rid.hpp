#pragma once
// Passive Remote ID (ASTM F3411 / Open Drone ID) reception for the map.
//
// Receive-only. Nothing is transmitted, spoofed or jammed. Only what an aircraft broadcasts about ITSELF is
// stored. The broadcast operator-location field is decoded by the shared Marauder decoder but discarded here;
// no operator position is shown, mapped, logged or inferred from aircraft telemetry.
//
// Decoding is Marauder's own RemoteIdDecoder (rid/), fed with frames from:
//   - Wi-Fi promiscuous management frames (Beacon + NAN action frames)   -> available on this hardware
//   - BLE 4 legacy advertisements via the HCI scanner (rid_ble.cpp)       -> only when the BT controller is built in
//   - BLE 5 extended advertisements                                       -> NOT receivable on the original ESP32
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

namespace marauder {

enum class RidState : uint8_t { ACTIVE, STALE, LOST };

constexpr size_t kRidCapacity = 12;
constexpr uint32_t kRidStaleMs = 10000;     // no message for 10 s: marker drawn as stale
constexpr uint32_t kRidLostMs = 30000;      // Marauder's REMOTE_ID_STALE_MS: aircraft considered lost
constexpr uint32_t kRidExpireMs = 120000;   // Marauder's REMOTE_ID_EXPIRE_MS: record removed
constexpr size_t kRidTrail = 10;

struct RidAircraft {
    char id[21];                // UAS ID (serial / registration) or empty
    char operator_id[21];       // operator registration ID text if broadcast (not a location)
    char description[24];
    uint8_t mac[6];
    uint8_t transports;         // bit0 Wi-Fi beacon, bit1 Wi-Fi NAN, bit2 BLE legacy, bit3 BLE extended
    uint8_t id_type, ua_type;
    bool has_id, has_operator_id, has_description;
    // aircraft-REPORTED position (never the receiver's GPS)
    bool has_position; double lat, lon;
    bool has_alt; float alt_geo_m;
    bool has_height; float height_m; bool height_is_agl;
    bool has_speed; float speed_mps;
    bool has_vspeed; float vspeed_mps;
    bool has_heading; uint16_t heading_deg;
    // reported accuracy, decoded from the ASTM code to the upper bound it stands for (metres, <0 = unknown)
    float h_acc_m, v_acc_m;
    bool has_report_time; uint16_t report_time_ds;    // aircraft clock: tenths of a second since the hour
    int8_t rssi; uint8_t wifi_channel;
    uint32_t first_seen_ms, last_seen_ms, age_ms;     // age since the last packet of any kind
    uint32_t position_age_ms;                         // age since the last packet that carried a position
    RidState state;
    uint32_t packets; float rate_hz;
    uint16_t position_revision;
    uint8_t trail_count; int16_t trail_e[kRidTrail], trail_n[kRidTrail];   // previous positions, metres relative to lat/lon
};

struct RidStats {
    bool running; uint32_t frames_decoded, frames_malformed, frames_other;
    uint32_t wifi_frames, ble_frames;
    uint8_t channel; bool ble_available;
};

// ---- decoder-facing entry points (used by the radio callbacks and by the host tests) ----
// Return true when the frame/payload decoded into a Remote ID message.
bool rid_feed_wifi_frame(const uint8_t *frame, size_t length, int8_t rssi, uint8_t channel, uint32_t now_ms);
bool rid_feed_ble_adv(const uint8_t *adv_payload, size_t length, const uint8_t mac[6], int8_t rssi, bool legacy, uint32_t now_ms);
size_t rid_snapshot_at(RidAircraft *out, size_t max, uint32_t now_ms);   // also prunes expired records
void rid_clear();
void rid_stats(RidStats *out);
float rid_h_acc_code_to_m(uint8_t code);
float rid_v_acc_code_to_m(uint8_t code);

#ifndef RID_HOST
// ---- firmware control ----
esp_err_t rid_start();      // pauses nothing itself: the caller must have paused the SDR. Opens the Wi-Fi monitor session.
void rid_stop();
bool rid_running();
void rid_tick();            // call regularly from the UI loop: channel schedule + record lifecycle (no extra task)
size_t rid_snapshot(RidAircraft *out, size_t max);
#endif
}
