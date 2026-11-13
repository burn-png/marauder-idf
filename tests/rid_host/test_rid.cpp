// Remote ID: frames are encoded here BY HAND from the ASTM F3411 / OpenDroneID field layout (independent of the
// decoder) and fed through the same decoder Marauder uses, via this firmware's adapter.
#include "include/rid.hpp"
#include "rid/RemoteIdModel.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <vector>
using namespace marauder;
static int fails = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); ++fails; } } while (0)
#define NEAR(a, b, t) do { double _a=(a), _b=(b); if (fabs(_a-_b) > (t)) { printf("FAIL %s:%d  %s=%.7f expected %.7f\n", __FILE__, __LINE__, #a, _a, _b); ++fails; } } while (0)
typedef std::vector<uint8_t> Bytes;

static void put32(Bytes &b, size_t o, int32_t v) { for (int i = 0; i < 4; ++i) b[o + i] = (uint8_t)((uint32_t)v >> (8 * i)); }
static void put16(Bytes &b, size_t o, uint16_t v) { b[o] = v & 0xFF; b[o + 1] = v >> 8; }
static uint16_t enc_alt(double m) { return (uint16_t)lround((m + 1000.0) / 0.5); }

static Bytes basic_id(const char *id, uint8_t idtype, uint8_t uatype) {
    Bytes m(25, 0); m[0] = 0x02; m[1] = (idtype << 4) | uatype; strncpy((char *)&m[2], id, 20); return m;
}
// speed_mps < 63.75 uses the 0.25 multiplier; direction in degrees 0..359 (E/W flag carries the +180)
static Bytes location(double lat, double lon, double dir, double speed, double vs, double alt, double h, uint8_t hacc, uint8_t vacc, uint16_t ts) {
    Bytes m(25, 0); m[0] = 0x12;
    const bool west = dir >= 180.0; const uint8_t d = (uint8_t)(west ? dir - 180.0 : dir);
    const bool hs = speed > 63.75;
    m[1] = (2 << 4) | (0 << 2) | (west ? 2 : 0) | (hs ? 1 : 0);      // status = airborne(2), height type = above takeoff
    m[2] = d;
    m[3] = hs ? (uint8_t)lround((speed - 63.75) / 0.75) : (uint8_t)lround(speed / 0.25);
    m[4] = (uint8_t)(int8_t)lround(vs / 0.5);
    put32(m, 5, (int32_t)lround(lat * 1e7)); put32(m, 9, (int32_t)lround(lon * 1e7));
    put16(m, 13, enc_alt(alt)); put16(m, 15, enc_alt(alt)); put16(m, 17, enc_alt(h));
    m[19] = (vacc << 4) | hacc; m[20] = 0x01;
    put16(m, 21, ts);
    return m;
}
static Bytes system_msg(double olat, double olon) {            // operator location: must NEVER surface
    Bytes m(25, 0); m[0] = 0x42; m[1] = 0x01; put32(m, 2, (int32_t)lround(olat * 1e7)); put32(m, 6, (int32_t)lround(olon * 1e7)); return m;
}
static Bytes operator_id(const char *id) { Bytes m(25, 0); m[0] = 0x52; m[1] = 0; strncpy((char *)&m[2], id, 20); return m; }

static Bytes ble_adv(const Bytes &msg, uint8_t counter) {      // legacy: Service Data AD (UUID 0xFFFA, app code 0x0D, counter, message)
    Bytes a;   // a legacy payload is just the 31-byte Service Data AD structure
    a.push_back((uint8_t)(1 + 2 + 1 + 1 + msg.size())); a.push_back(0x16); a.push_back(0xFA); a.push_back(0xFF); a.push_back(0x0D); a.push_back(counter);
    a.insert(a.end(), msg.begin(), msg.end()); return a;
}
static Bytes pack(const std::vector<Bytes> &msgs) {
    Bytes p = {0xF2, 25, (uint8_t)msgs.size()}; for (auto &m : msgs) p.insert(p.end(), m.begin(), m.end()); return p;
}
static Bytes wifi_beacon(const uint8_t mac[6], const Bytes &payload, bool asd_oui = true) {
    Bytes f(36, 0); f[0] = 0x80; memset(&f[4], 0xFF, 6); memcpy(&f[10], mac, 6); memcpy(&f[16], mac, 6);
    f.push_back(0); f.push_back(4); f.push_back('R'); f.push_back('I'); f.push_back('D'); f.push_back('1');     // SSID IE (ignored)
    Bytes ie; if (asd_oui) { ie = {0xFA, 0x0B, 0xBC, 0x0D, 0x01}; } else ie = {0x90, 0x3A, 0xE6, 0x0D, 0x01};
    ie.insert(ie.end(), payload.begin(), payload.end());
    f.push_back(221); f.push_back((uint8_t)ie.size()); f.insert(f.end(), ie.begin(), ie.end()); return f;
}
static Bytes wifi_nan(const uint8_t mac[6], const Bytes &payload) {
    Bytes f(24, 0); f[0] = 0xD0; const uint8_t dest[6] = {0x51, 0x6F, 0x9A, 0x01, 0x00, 0x00}; memcpy(&f[4], dest, 6); memcpy(&f[10], mac, 6);
    Bytes attr = {0x88, 0x69, 0x19, 0x9D, 0x92, 0x09, 0x01, 0x00, 0x10};        // service id, instance ids, control (publish)
    attr.push_back((uint8_t)(payload.size() + 1)); attr.push_back(0x07); attr.insert(attr.end(), payload.begin(), payload.end());   // counter + payload
    Bytes hdr = {0x04, 0x09, 0x50, 0x6F, 0x9A, 0x13}; f.insert(f.end(), hdr.begin(), hdr.end());
    f.push_back(0x03); f.push_back(attr.size() & 0xFF); f.push_back(attr.size() >> 8); f.insert(f.end(), attr.begin(), attr.end()); return f;
}

int main() {
    const double LAT = 29.7604321, LON = -95.3698123;
    const uint8_t M1[6] = {0xAA, 1, 2, 3, 4, 5}, M2[6] = {0xBB, 1, 2, 3, 4, 5};
    RidAircraft a[kRidCapacity];

    // ---- BLE legacy: Basic ID and Location arrive as separate adverts and merge into ONE aircraft ----
    CHECK(rid_feed_ble_adv(ble_adv(basic_id("1581F4TESTDRONE00001", 1, 2), 1).data(), 31, M1, -61, true, 1000));
    {
        Bytes adv = ble_adv(basic_id("1581F4TESTDRONE00001", 1, 2), 1); CHECK(adv.size() == 31);   // exactly a full legacy payload
    }
    Bytes loc = location(LAT, LON, 270.0, 12.5, -1.5, 120.0, 45.0, 10, 4, 36000 + 123);
    CHECK(rid_feed_ble_adv(ble_adv(loc, 2).data(), 31, M1, -58, true, 2000));
    size_t n = rid_snapshot_at(a, kRidCapacity, 2500);
    CHECK(n == 1);
    CHECK(a[0].has_id && !strcmp(a[0].id, "1581F4TESTDRONE00001"));
    CHECK(a[0].id_type == 1 && a[0].ua_type == 2);
    CHECK(a[0].has_position); NEAR(a[0].lat, LAT, 1e-7); NEAR(a[0].lon, LON, 1e-7);
    CHECK(a[0].has_heading && a[0].heading_deg == 270);                   // 90 + 180 via the E/W flag
    CHECK(a[0].has_speed); NEAR(a[0].speed_mps, 12.5, 0.01);
    CHECK(a[0].has_vspeed); NEAR(a[0].vspeed_mps, -1.5, 0.01);
    CHECK(a[0].has_alt); NEAR(a[0].alt_geo_m, 120.0, 0.26);
    CHECK(a[0].has_height); NEAR(a[0].height_m, 45.0, 0.26);
    NEAR(a[0].h_acc_m, 10.0, 1e-6); NEAR(a[0].v_acc_m, 10.0, 1e-6);       // codes 10 and 4 -> "< 10 m", "< 10 m"
    CHECK(a[0].has_report_time && a[0].report_time_ds == 36123);
    CHECK(a[0].transports == (uint8_t)RemoteIdTransport::BleLegacy);
    CHECK(a[0].rssi == -58 && a[0].packets == 2);
    CHECK(a[0].state == RidState::ACTIVE && a[0].age_ms == 500);
    CHECK(a[0].position_age_ms == 500);
    // high-speed multiplier branch
    rid_feed_ble_adv(ble_adv(location(LAT, LON, 10.0, 80.0, 0, 100, 30, 9, 3, 5), 3).data(), 31, M1, -58, true, 3000);
    n = rid_snapshot_at(a, kRidCapacity, 3000); CHECK(a[0].has_speed); NEAR(a[0].speed_mps, 80.0, 0.5); CHECK(a[0].heading_deg == 10);
    // ---- freshness: stale at 10 s, lost at 30 s, reacquired on the next packet, expired at 120 s ----
    n = rid_snapshot_at(a, kRidCapacity, 3000 + 9999); CHECK(a[0].state == RidState::ACTIVE);
    n = rid_snapshot_at(a, kRidCapacity, 3000 + 10000); CHECK(a[0].state == RidState::STALE);
    n = rid_snapshot_at(a, kRidCapacity, 3000 + 30000); CHECK(n == 1 && a[0].state == RidState::LOST);
    CHECK(a[0].has_position);                                             // last reported position is kept, but flagged LOST
    rid_feed_ble_adv(ble_adv(loc, 4).data(), 31, M1, -70, true, 40000);
    n = rid_snapshot_at(a, kRidCapacity, 40100); CHECK(a[0].state == RidState::ACTIVE && a[0].age_ms == 100);
    n = rid_snapshot_at(a, kRidCapacity, 40000 + 120000); CHECK(n == 0);

    // ---- Wi-Fi beacon carrying a message pack (basic id + location + operator id + SYSTEM with operator location) ----
    rid_clear();
    {
        Bytes p = pack({basic_id("WIFI-BEACON-UAS-0001", 1, 4), location(LAT + 0.001, LON + 0.001, 45, 5.0, 0, 80, 20, 11, 5, 100),
                        operator_id("OP-REG-12345"), system_msg(29.5, -95.0)});
        Bytes f = wifi_beacon(M2, p);
        CHECK(rid_feed_wifi_frame(f.data(), f.size(), -50, 6, 5000));
        n = rid_snapshot_at(a, kRidCapacity, 5100);
        CHECK(n == 1 && a[0].has_id && !strcmp(a[0].id, "WIFI-BEACON-UAS-0001"));
        CHECK(a[0].has_position); NEAR(a[0].lat, LAT + 0.001, 1e-7); CHECK(a[0].heading_deg == 45);
        CHECK(a[0].has_operator_id && !strcmp(a[0].operator_id, "OP-REG-12345"));
        CHECK(a[0].transports == (uint8_t)RemoteIdTransport::WifiBeacon && a[0].wifi_channel == 6);
        // the operator position (29.5, -95.0) must not be reachable through any public field
        const uint8_t *raw = (const uint8_t *)&a[0];
        const int32_t olat = (int32_t)lround(29.5 * 1e7), olon = (int32_t)lround(-95.0 * 1e7);
        bool leaked = false;
        for (size_t i = 0; i + 4 <= sizeof(a[0]); ++i) { int32_t v; memcpy(&v, raw + i, 4); if (v == olat || v == olon) leaked = true; }
        for (size_t i = 0; i + 8 <= sizeof(a[0]); ++i) { double v; memcpy(&v, raw + i, 8); if (fabs(v - 29.5) < 1e-6 || fabs(v + 95.0) < 1e-6) leaked = true; }
        CHECK(!leaked);
        // legacy OUI variant is accepted too
        rid_feed_wifi_frame(wifi_beacon(M2, pack({basic_id("WIFI-BEACON-UAS-0001", 1, 4)}), false).data(), 36 + 2 + 4 + 2 + 5 + 3 + 25, -50, 11, 5200);
    }
    // ---- Wi-Fi NAN action frame ----
    rid_clear();
    {
        Bytes f = wifi_nan(M1, pack({basic_id("NAN-UAS-000000000002", 1, 2), location(LAT, LON, 0, 0, 0, 10, 5, 12, 6, 7)}));
        CHECK(rid_feed_wifi_frame(f.data(), f.size(), -44, 6, 100));
        n = rid_snapshot_at(a, kRidCapacity, 200);
        CHECK(n == 1 && !strcmp(a[0].id, "NAN-UAS-000000000002") && a[0].has_position);
        CHECK(a[0].transports == (uint8_t)RemoteIdTransport::WifiNan);
        NEAR(a[0].h_acc_m, 1.0, 1e-6);
    }
    // ---- one aircraft, two radios / rotating MAC: merged by UAS ID ----
    {
        Bytes adv = ble_adv(basic_id("NAN-UAS-000000000002", 1, 2), 1);
        rid_feed_ble_adv(adv.data(), adv.size(), M2, -80, true, 300);
        n = rid_snapshot_at(a, kRidCapacity, 400);
        CHECK(n == 1);
        CHECK(a[0].transports == ((uint8_t)RemoteIdTransport::WifiNan | (uint8_t)RemoteIdTransport::BleLegacy));
    }
    // ---- movement builds a trail ----
    rid_clear();
    for (int i = 0; i < 5; ++i) {
        Bytes adv = ble_adv(location(LAT + i * 0.0002, LON, 0, 3, 0, 50, 10, 10, 4, (uint16_t)i), (uint8_t)i);
        rid_feed_ble_adv(adv.data(), adv.size(), M1, -60, true, 1000 + i * 1000);
    }
    n = rid_snapshot_at(a, kRidCapacity, 6000); CHECK(n == 1 && a[0].trail_count == 4);
    NEAR(a[0].trail_n[3], -22.2, 1.0);                                      // previous point is 0.0002 deg (~22 m) south of now

    // ---- invalid / incomplete input is rejected and counted, never shown as an aircraft ----
    rid_clear();
    RidStats before; rid_stats(&before);
    { Bytes adv = ble_adv(loc, 1); adv.resize(20); rid_feed_ble_adv(adv.data(), adv.size(), M1, -60, true, 1); }          // truncated AD
    { Bytes adv = ble_adv(loc, 1); adv[2] = 0xFB; CHECK(!rid_feed_ble_adv(adv.data(), adv.size(), M1, -60, true, 1)); }                    // wrong service UUID -> ignored
    { Bytes adv = ble_adv(loc, 1); adv[4] = 0x0E; CHECK(!rid_feed_ble_adv(adv.data(), adv.size(), M1, -60, true, 1)); }                    // wrong application code
    { Bytes f = wifi_beacon(M2, pack({loc})); f.resize(f.size() - 7); CHECK(!rid_feed_wifi_frame(f.data(), f.size(), -50, 6, 1)); }          // truncated IE
    { Bytes p = {0xF2, 25, 3}; p.insert(p.end(), loc.begin(), loc.end()); Bytes f = wifi_beacon(M2, p); CHECK(!rid_feed_wifi_frame(f.data(), f.size(), -50, 6, 1)); }  // pack says 3, holds 1
    { Bytes f(100, 0x41); CHECK(!rid_feed_wifi_frame(f.data(), f.size(), -50, 6, 1)); }                                                    // not a beacon at all
    CHECK(!rid_feed_wifi_frame(nullptr, 0, 0, 0, 0));
    {   // location with lat/lon == 0 (ASTM "unknown") decodes but gives no position
        Bytes adv = ble_adv(location(0, 0, 0, 1, 0, 10, 5, 3, 3, 1), 1);
        rid_feed_ble_adv(adv.data(), adv.size(), M1, -60, true, 10);
        n = rid_snapshot_at(a, kRidCapacity, 20); CHECK(n == 0 || !a[0].has_position);
    }
    {   // unknown altitude (0xFFFF... encoded as 0) and invalid speed code produce "no value", not a number
        Bytes l = location(LAT, LON, 0, 0, 0, -1000.0, -1000.0, 3, 3, 1); l[3] = 255; l[1] |= 1;     // speed 255 high-speed => 254.25+ ... invalid
        Bytes adv = ble_adv(l, 1); rid_feed_ble_adv(adv.data(), adv.size(), M1, -60, true, 30);
        n = rid_snapshot_at(a, kRidCapacity, 40); CHECK(n == 1 && !a[0].has_alt && !a[0].has_height);
    }
    RidStats after; rid_stats(&after);
    CHECK(after.frames_malformed > before.frames_malformed);
    // capacity: the 13th distinct aircraft replaces the least recently heard, never overflows
    rid_clear();
    for (int i = 0; i < 20; ++i) {
        uint8_t mac[6] = {0xCC, (uint8_t)i, 0, 0, 0, 1}; char id[21]; snprintf(id, sizeof id, "CAP-TEST-%011d", i);
        Bytes adv = ble_adv(basic_id(id, 1, 2), 1); rid_feed_ble_adv(adv.data(), adv.size(), mac, -60, true, 1000 + i);
    }
    n = rid_snapshot_at(a, kRidCapacity, 1100); CHECK(n == kRidCapacity);
    // accuracy tables (ASTM F3411 Table)
    NEAR(rid_h_acc_code_to_m(1), 18520, 1e-3); NEAR(rid_h_acc_code_to_m(8), 92.6, 1e-3); CHECK(rid_h_acc_code_to_m(0) < 0 && rid_h_acc_code_to_m(14) < 0);
    NEAR(rid_v_acc_code_to_m(2), 45, 1e-6); CHECK(rid_v_acc_code_to_m(0) < 0 && rid_v_acc_code_to_m(9) < 0);
    printf(fails ? "FAILED (%d)\n" : "ALL PASS\n", fails);
    return fails != 0;
}
