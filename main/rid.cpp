// Remote ID adapter. Decoding is Marauder's RemoteIdDecoder (see rid/NOTICE.md); the observation and merge
// logic below follows Marauder's WiFiScanRemoteId.h (processRemoteIdWifiFrame / processRemoteIdBlePayload /
// mergeRemoteIdRecord) without Arduino types. Passive receive only.
#include "include/rid.hpp"
#include "rid/RemoteIdDecoder.h"
#include "rid/RemoteIdModel.h"
#include <string.h>
#include <math.h>
#ifdef RID_HOST
#define RID_LOCK()   do {} while (0)
#define RID_UNLOCK() do {} while (0)
#else
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "radio_backend.h"
#endif

namespace marauder {
namespace {
#ifndef RID_HOST
portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
#define RID_LOCK()   portENTER_CRITICAL(&s_mux)
#define RID_UNLOCK() portEXIT_CRITICAL(&s_mux)
const char kTag[] = "rid";
#endif
RemoteIdRecord s_records[kRidCapacity];
RemoteIdStore s_store{s_records, kRidCapacity};
const RemoteIdDecoder s_decoder;
uint32_t s_decoded, s_malformed, s_other, s_wifi, s_ble;

// Bounded copy that always terminates (memcpy of a measured length: no strncpy truncation diagnostics).
void copy_cstr(char *dst, size_t n, const char *src, size_t src_max) {
    size_t len = strnlen(src, src_max);
    if (len > n - 1) len = n - 1;
    memcpy(dst, src, len);
    dst[len] = '\0';
}
template <size_t N, size_t M>
void copy_if_present(char (&dst)[N], const char (&src)[M], bool present) {
    if (present) copy_cstr(dst, N, src, M);
}

// Adapted from Marauder mergeRemoteIdRecord(). Operator-location / area / authentication fields are
// intentionally NOT merged: they never enter this firmware's store.
void merge(RemoteIdRecord &d, const RemoteIdRecord &s) {
    const bool accept_id = s.hasUasId && remoteIdShouldReplaceBasicId(d.idType, d.hasUasId, s.idType);
    copy_if_present(d.uasId, s.uasId, accept_id);
    copy_if_present(d.operatorId, s.operatorId, s.hasOperatorId);
    copy_if_present(d.description, s.description, s.hasDescription);
    if (accept_id) { d.hasUasId = true; d.idType = s.idType; d.uaType = s.uaType; }
    if (s.hasOperatorId) d.hasOperatorId = true;
    if (s.hasDescription) d.hasDescription = true;
    if (s.hasLocation) {
        if (!d.hasLocation && s.historyCount != 0) {
            memcpy(d.history, s.history, sizeof(d.history));
            d.historyCount = s.historyCount;
        }
        remoteIdUpdateLocation(d, s.latitudeE7, s.longitudeE7);
        d.altitudeGeoM = s.altitudeGeoM;          d.altitudePressureM = s.altitudePressureM;
        d.heightM = s.heightM;                    d.horizontalSpeedMps = s.horizontalSpeedMps;
        d.verticalSpeedMps = s.verticalSpeedMps;  d.directionDeg = s.directionDeg;
        d.operationalStatus = s.operationalStatus;
        d.horizontalAccuracy = s.horizontalAccuracy; d.verticalAccuracy = s.verticalAccuracy; d.speedAccuracy = s.speedAccuracy;
        d.locationTimestampDeciseconds = s.locationTimestampDeciseconds;
        d.heightIsAboveGround = s.heightIsAboveGround;
        d.directionValid = s.directionValid;      d.horizontalSpeedValid = s.horizontalSpeedValid;
        d.verticalSpeedValid = s.verticalSpeedValid; d.altitudePressureValid = s.altitudePressureValid;
        d.altitudeGeoValid = s.altitudeGeoValid;  d.heightValid = s.heightValid;
        d.locationTimestampValid = s.locationTimestampValid;
    }
}

// `lastGpxLoggedMs` is a logging field in Marauder's record that this firmware does not use; it holds the time
// of the last message that carried a position so a marker's position age can be reported separately.
void ingest(const RemoteIdRecord &decoded, const uint8_t mac[6], RemoteIdTransport transport, int8_t rssi,
            uint8_t channel, uint32_t now) {
    RID_LOCK();
    RemoteIdRecord *rec = decoded.hasUasId ? s_store.findByUasId(decoded.uasId) : nullptr;
    RemoteIdRecord *provisional = s_store.findByMac(mac);
    if (!rec) rec = &s_store.observe(mac, transport, rssi, now);
    else {
        if (provisional && provisional != rec) {
            merge(*rec, *provisional);
            rec->transportMask |= provisional->transportMask;
            rec->packetCount += provisional->packetCount;
            rec->lostCount += provisional->lostCount;
            rec->reacquiredCount += provisional->reacquiredCount;
            if (provisional->lastGpxLoggedMs) rec->lastGpxLoggedMs = provisional->lastGpxLoggedMs;
            s_store.eraseByMac(mac, rec);
            rec = s_store.findByUasId(decoded.uasId);
        }
        // known aircraft heard again (same logic as Marauder's updateKnownRemoteIdObservation)
        memcpy(rec->mac, mac, 6);
        rec->transportMask |= (uint8_t)transport;
        rec->rssi = rssi;
        if (rec->isLost) { rec->isLost = false; ++rec->reacquiredCount; rec->lastReacquiredMs = now; }
        rec->lastSeenMs = now;
        ++rec->packetCount;
        if (rec->rateWindowStartedMs == 0) rec->rateWindowStartedMs = now;
        ++rec->rateWindowPackets;
        const uint32_t el = now - rec->rateWindowStartedMs;
        if (el >= 1000) { rec->packetRateHz = rec->rateWindowPackets * 1000.0f / (float)el; rec->rateWindowStartedMs = now; rec->rateWindowPackets = 0; }
    }
    if (rec) {
        merge(*rec, decoded);
        if (decoded.hasLocation) rec->lastGpxLoggedMs = now ? now : 1;
        if (channel) rec->lastWifiChannel = channel;
    }
    RID_UNLOCK();
}

void count(RemoteIdDecodeResult r, bool wifi) {
    RID_LOCK();
    if (r == RemoteIdDecodeResult::Decoded) ++s_decoded;
    else if (r == RemoteIdDecodeResult::Malformed) ++s_malformed;
    else ++s_other;
    if (r == RemoteIdDecodeResult::Decoded) { if (wifi) ++s_wifi; else ++s_ble; }
    RID_UNLOCK();
}
}  // namespace

// ASTM F3411 horizontal accuracy codes -> the upper bound the code stands for, metres (<0 = unknown).
float rid_h_acc_code_to_m(uint8_t c) {
    static const float t[] = {-1.f, 18520.f, 7408.f, 3704.f, 1852.f, 926.f, 555.6f, 185.2f, 92.6f, 30.f, 10.f, 3.f, 1.f};
    return c < sizeof(t) / sizeof(*t) ? t[c] : -1.f;
}
float rid_v_acc_code_to_m(uint8_t c) {
    static const float t[] = {-1.f, 150.f, 45.f, 25.f, 10.f, 3.f, 1.f};
    return c < sizeof(t) / sizeof(*t) ? t[c] : -1.f;
}

bool rid_feed_wifi_frame(const uint8_t *frame, size_t length, int8_t rssi, uint8_t channel, uint32_t now_ms) {
    if (!frame || length < 24) return false;
    RemoteIdRecord decoded;
    RemoteIdDecodeResult r = s_decoder.decodeWifiBeacon(frame, length, decoded);
    RemoteIdTransport t = RemoteIdTransport::WifiBeacon;
    if (r != RemoteIdDecodeResult::Decoded) {          // not a Beacon carrying Remote ID: try a NAN action frame
        RemoteIdRecord nan_decoded;
        const RemoteIdDecodeResult rn = s_decoder.decodeWifiNan(frame, length, nan_decoded);
        if (rn == RemoteIdDecodeResult::Decoded) { r = rn; decoded = nan_decoded; t = RemoteIdTransport::WifiNan; }
        else if (r == RemoteIdDecodeResult::Ignored) r = rn;
    }
    if (r != RemoteIdDecodeResult::Decoded) {
        if (r == RemoteIdDecodeResult::Malformed) count(r, true);   // rejected: signature seen but content invalid / truncated
        return false;
    }
    count(r, true);
    ingest(decoded, frame + 10, t, rssi, channel, now_ms);
    return true;
}

bool rid_feed_ble_adv(const uint8_t *p, size_t length, const uint8_t mac[6], int8_t rssi, bool legacy, uint32_t now_ms) {
    if (!p || !mac) return false;
    size_t off = 0;
    while (off < length) {
        const size_t ad_len = p[off];
        if (ad_len == 0) break;
        if (off + ad_len + 1 > length) { count(RemoteIdDecodeResult::Malformed, false); return false; }
        if (p[off + 1] == 0x16 && ad_len >= 4) {                  // Service Data - 16-bit UUID
            RemoteIdRecord decoded;
            const RemoteIdDecodeResult r = s_decoder.decodeBleServiceData(p + off + 2, ad_len - 1, decoded);
            if (r == RemoteIdDecodeResult::Decoded) {
                count(r, false);
                ingest(decoded, mac, legacy ? RemoteIdTransport::BleLegacy : RemoteIdTransport::BleExtended, rssi, 0, now_ms);
                return true;
            }
            if (r == RemoteIdDecodeResult::Malformed) count(r, false);
        }
        off += ad_len + 1;
    }
    return false;
}

static void to_aircraft(const RemoteIdRecord &r, uint32_t now, RidAircraft *a) {
    memset(a, 0, sizeof(*a));
    copy_cstr(a->id, sizeof(a->id), r.uasId, sizeof(r.uasId));
    copy_cstr(a->operator_id, sizeof(a->operator_id), r.operatorId, sizeof(r.operatorId));
    copy_cstr(a->description, sizeof(a->description), r.description, sizeof(r.description));
    memcpy(a->mac, r.mac, 6);
    a->transports = r.transportMask; a->id_type = r.idType; a->ua_type = r.uaType;
    a->has_id = r.hasUasId; a->has_operator_id = r.hasOperatorId; a->has_description = r.hasDescription;
    a->has_position = r.hasLocation && remoteIdCoordinatesValid(r.latitudeE7, r.longitudeE7);
    a->lat = r.latitudeE7 * 1e-7; a->lon = r.longitudeE7 * 1e-7;
    a->has_alt = r.hasLocation && r.altitudeGeoValid; a->alt_geo_m = r.altitudeGeoM;
    a->has_height = r.hasLocation && r.heightValid; a->height_m = r.heightM; a->height_is_agl = r.heightIsAboveGround;
    a->has_speed = r.hasLocation && r.horizontalSpeedValid; a->speed_mps = r.horizontalSpeedMps;
    a->has_vspeed = r.hasLocation && r.verticalSpeedValid; a->vspeed_mps = r.verticalSpeedMps;
    a->has_heading = r.hasLocation && r.directionValid; a->heading_deg = r.directionDeg % 360;
    a->h_acc_m = r.hasLocation ? rid_h_acc_code_to_m(r.horizontalAccuracy) : -1.f;
    a->v_acc_m = r.hasLocation ? rid_v_acc_code_to_m(r.verticalAccuracy) : -1.f;
    a->has_report_time = r.hasLocation && r.locationTimestampValid; a->report_time_ds = r.locationTimestampDeciseconds;
    a->rssi = r.rssi; a->wifi_channel = r.lastWifiChannel;
    a->first_seen_ms = r.firstSeenMs; a->last_seen_ms = r.lastSeenMs;
    a->age_ms = now - r.lastSeenMs;
    a->position_age_ms = r.lastGpxLoggedMs ? now - r.lastGpxLoggedMs : 0xFFFFFFFFu;
    a->state = (r.isLost || a->age_ms >= kRidLostMs) ? RidState::LOST : a->age_ms >= kRidStaleMs ? RidState::STALE : RidState::ACTIVE;
    a->packets = r.packetCount; a->rate_hz = r.packetRateHz; a->position_revision = r.locationRevision;
    a->trail_count = r.historyCount > kRidTrail ? kRidTrail : r.historyCount;
    for (uint8_t i = 0; i < a->trail_count; ++i) { a->trail_e[i] = r.history[i].eastM; a->trail_n[i] = r.history[i].northM; }
}

size_t rid_snapshot_at(RidAircraft *out, size_t max, uint32_t now) {
    size_t n = 0;
    RID_LOCK();
    s_store.updateLifecycle(now, kRidLostMs);
    s_store.pruneStale(now, kRidExpireMs);
    for (size_t i = 0; i < s_store.size() && n < max; ++i) to_aircraft(*s_store.at(i), now, &out[n++]);
    RID_UNLOCK();
    return n;
}

void rid_clear() { RID_LOCK(); s_store.clear(); RID_UNLOCK(); }

#ifndef RID_HOST
namespace {
bool s_running; uint8_t s_channel = 6; uint32_t s_step, s_sweep = 1; int64_t s_last_hop_us;
constexpr uint32_t kDwellMs = 300;
uint32_t now_ms() { return (uint32_t)(esp_timer_get_time() / 1000); }

void wifi_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
    if (type != WIFI_PKT_MGMT || !buf) return;
    const wifi_promiscuous_pkt_t *p = (const wifi_promiscuous_pkt_t *)buf;
    size_t len = p->rx_ctrl.sig_len;
    if (len >= 4) len -= 4;                       // FCS
    if (len < 24 || (p->payload[0] != 0x80 && p->payload[0] != 0xD0)) return;   // Beacon / Action only
    rid_feed_wifi_frame(p->payload, len, (int8_t)p->rx_ctrl.rssi, (uint8_t)p->rx_ctrl.channel, now_ms());
}
}  // namespace

bool rid_ble_start();   // rid_ble.cpp (weak default: not built in)
void rid_ble_stop();
bool rid_ble_available();

esp_err_t rid_start() {
    if (s_running) return ESP_OK;
    const esp_err_t e = marauder_radio_monitor_begin(wifi_cb, 6);
    if (e != ESP_OK) { ESP_LOGW(kTag, "monitor begin failed: %s", esp_err_to_name(e)); return e; }
    s_running = true; s_channel = 6; s_step = 0; s_sweep = 1; s_last_hop_us = esp_timer_get_time();
    (void)rid_ble_start();
    ESP_LOGI(kTag, "Remote ID scan started (Wi-Fi beacon + NAN%s)", rid_ble_available() ? " + BLE legacy" : "");
    return ESP_OK;
}
void rid_stop() {
    if (!s_running) return;
    rid_ble_stop();
    s_running = false;
    marauder_radio_monitor_end();
    ESP_LOGI(kTag, "Remote ID scan stopped");
}
bool rid_running() { return s_running; }

void rid_tick() {
    if (!s_running) return;
    const int64_t now = esp_timer_get_time();
    if (now - s_last_hop_us < (int64_t)kDwellMs * 1000) return;
    s_last_hop_us = now;
    // Same shape as Marauder's hopRemoteIdChannel(): half of the dwell periods on channel 6 (ASTM NAN default),
    // the others sweep channels 1-11 so Wi-Fi Beacon transmitters on other channels remain discoverable.
    const uint32_t phase = s_step++ & 3u;
    uint8_t ch;
    if (phase == 0 || phase == 2) ch = 6;
    else { ch = (uint8_t)s_sweep; s_sweep = s_sweep >= 11 ? 1 : s_sweep + 1; }
    if (marauder_radio_monitor_channel(ch) == ESP_OK) s_channel = ch;
}

size_t rid_snapshot(RidAircraft *out, size_t max) { return rid_snapshot_at(out, max, now_ms()); }
#endif

void rid_stats(RidStats *o) {
    memset(o, 0, sizeof(*o));
    RID_LOCK();
    o->frames_decoded = s_decoded; o->frames_malformed = s_malformed; o->frames_other = s_other;
    o->wifi_frames = s_wifi; o->ble_frames = s_ble;
    RID_UNLOCK();
#ifndef RID_HOST
    o->running = s_running; o->channel = s_channel; o->ble_available = rid_ble_available();
#endif
}
}  // namespace marauder
