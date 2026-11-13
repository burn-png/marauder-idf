#pragma once
// RF propagation-time -> distance estimation (range rings). Pure math + bookkeeping, no hardware access.
//
//   one-way, clocks synchronised :  d = c * (t_rx - t_tx - cal)
//   round trip                   :  d = c * (t_rtt - turnaround - cal) / 2
//
// This module ONLY accepts genuine time-of-flight measurements: hardware-timestamped, and (for one-way)
// synchronised to a stated accuracy. Packet arrival times, software timestamps, Wi-Fi scheduling/processing
// delays, buffering and similar are rejected, never converted. The original ESP32 on this board cannot make such
// a measurement itself (no hardware RX/TX timestamping on the RF path, no FTM), so on this hardware the only way
// to feed it is an external timing-capable receiver via /maps/rf_range.csv. See README "RF range rings".
//
// The result is a RANGE RING around the observer (distance +/- uncertainty). It is never a transmitter position.
#include <stdint.h>
#include <stddef.h>
#include "osm.hpp"

namespace marauder {

enum class RangeSource : uint8_t {
    HW_ROUND_TRIP,          // hardware-timestamped round trip (e.g. FTM / UWB / dedicated radar-style timer)
    HW_ONE_WAY_SYNCED,      // hardware-timestamped one-way with a clock sync of stated accuracy (GPS-disciplined, PTP-class)
    SOFTWARE_TIMESTAMP,     // REJECTED: CPU/driver timestamps
    PACKET_ARRIVAL,         // REJECTED: packet arrival / beacon timestamp / RSSI-derived timing
    UNKNOWN                 // REJECTED
};

enum class RangeStatus : uint8_t {
    OK, REJECT_SOURCE, REJECT_NOT_HW_TIMESTAMPED, REJECT_UNSYNCED, REJECT_NEGATIVE, REJECT_NOT_FINITE, REJECT_NO_UNCERTAINTY,
    REJECT_TOO_FAR, REJECT_NO_OBSERVER, STORE_FULL
};
const char *range_status_text(RangeStatus s);

struct RangeInput {
    RangeSource source;
    bool hw_timestamped;        // the timestamps come from a hardware counter at the antenna/PHY
    double time_s;              // one-way: t_rx - t_tx. round trip: total round-trip time
    double turnaround_s;        // round trip only: responder's known processing delay (0 when none)
    double cal_offset_s;        // fixed system delay (cables, front-end) measured by calibration; subtracted
    double time_unc_s;          // 1-sigma timing uncertainty of the measurement (required, > 0)
    double cal_unc_s;           // 1-sigma uncertainty of the calibration offset
    double sync_unc_s;          // one-way only: 1-sigma clock synchronisation uncertainty (required, > 0)
};

struct RangeEstimate {
    bool valid;
    double distance_m;          // central estimate
    double sigma_m;             // 1-sigma
    double ring_inner_m, ring_outer_m;   // drawn band: distance +/- 2 sigma (about 95 %), inner clamped at 0
};

// Pure calculation. Returns OK and fills `out` only for an acceptable measurement.
RangeStatus range_compute(const RangeInput &in, RangeEstimate *out);

struct RangeRing {
    bool used;
    char label[12];
    double obs_lat, obs_lon;    // where the observer (this receiver) was when the measurement was made
    RangeEstimate est;
    uint32_t stamp_ms;
};
constexpr size_t kRangeRings = 8;

class RangeStore {
public:
    RangeStore() : head_(0), count_(0) {}
    RangeStatus submit(const RangeInput &in, double obs_lat, double obs_lon, const char *label, uint32_t now_ms);
    size_t snapshot(RangeRing *out, size_t max) const;   // newest last
    void clear() { head_ = 0; count_ = 0; }
    size_t count() const { return count_; }
private:
    RangeRing r_[kRangeRings]; size_t head_, count_;
};

// External measurements: /maps/rf_range.csv. One row per measurement:
//   label,mode,time_ns,turnaround_ns,cal_ns,time_unc_ns,cal_unc_ns,sync_unc_ns,hw_timestamped,obs_lat,obs_lon
//   mode: RT (round trip) or OW (one way, synchronised)
// Returns the number of rows accepted; rejected rows are counted in *rejected with the last reason.
size_t range_import_csv(osm::FileIo *io, const char *path, RangeStore &store, uint32_t now_ms, size_t *rejected, RangeStatus *last_reject);

// Always false on this board; the reason text is shown on the map screen.
bool range_hardware_supported(const char **why_not);

#ifndef RANGE_HOST
RangeStore &range_store();                // firmware singleton
#endif
}
