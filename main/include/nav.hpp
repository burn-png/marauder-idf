#pragma once
// GPS navigation service: the one interface other modules (map, Remote ID overlay, RF range, UI) use for
// position, speed, heading, accuracy, trace and trip statistics. Nothing outside gps.cpp touches the UART.
// The engine only ever works from real receiver data; where a value is derived or estimated it says so.
#include <stdint.h>
#include <stddef.h>
#include "gps.hpp"

namespace marauder {

enum class HeadingSrc : uint8_t { NONE, GNSS, TRACK, HELD };   // course over ground / bearing of recent track / last value kept
enum class SpeedSrc : uint8_t { NONE, GNSS, DERIVED };          // receiver RMC/VTG speed / distance over time between fixes
enum class AccSrc : uint8_t { NONE, GST, ESTIMATE };            // measured by receiver / HDOP x assumed 5 m UERE (estimate)

struct NavPoint { int32_t lat_e7, lon_e7; };

struct NavSnapshot {
    GpsFix fix;                  // latest raw receiver data (valid only while a fix is current)
    bool have_position;          // a valid position has been received at some time since boot
    bool stale;                  // the fix is currently lost; position below is the last valid one
    double lat, lon;             // last valid position
    uint32_t fix_age_ms;         // time since that position (UINT32_MAX when none)
    double speed_mps; SpeedSrc speed_src;                 // NONE while stale
    double heading_deg; HeadingSrc heading_src; uint32_t heading_age_ms;
    double acc_m; AccSrc acc_src;                          // horizontal, 1-sigma-class
    // trip statistics (distance excludes standing-still jitter)
    double trip_dist_m, max_speed_mps; uint32_t trip_moving_s, trip_elapsed_s;
    uint16_t trace_count; bool trace_enabled;
};

constexpr size_t kTraceCapacity = 256;

class NavEngine {
public:
    NavEngine();
    void on_fix(const GpsFix &f, uint64_t now_us);          // call after each accepted position update
    void snapshot(NavSnapshot *out, const GpsFix &raw, uint64_t now_us) const;
    void trip_reset(uint64_t now_us);
    void trace_enable(bool on);
    void trace_clear();
    size_t trace_count() const { return count_; }
    bool trace_get(size_t index, NavPoint *p) const;        // index 0 = oldest
    void set_trace_min_m(float m) { trace_min_m_ = m < 1 ? 1 : m; }
private:
    bool have_, have_prev_;
    double lat_, lon_, prev_lat_, prev_lon_;
    uint64_t last_us_, prev_us_;
    GpsFix last_fix_;
    double speed_; SpeedSrc speed_src_;
    double heading_; HeadingSrc heading_src_; uint64_t heading_us_;
    double anchor_lat_, anchor_lon_; bool have_anchor_;
    double acc_; AccSrc acc_src_;
    double dist_, vmax_; uint64_t moving_us_, trip_start_us_;
    NavPoint trace_[kTraceCapacity]; size_t head_, count_; bool trace_on_; float trace_min_m_;
    NavPoint last_trace_;  bool have_trace_;
};

void nav_on_fix_internal(const GpsFix &f);   // gps.cpp only: feed one accepted position update
// ---- firmware singleton (gps.cpp feeds it; any task may read) ----------------------------------------
void nav_snapshot(NavSnapshot *out);
void nav_trip_reset();
void nav_trace_enable(bool on);
void nav_trace_clear();
size_t nav_trace_copy(NavPoint *out, size_t max);        // oldest first
bool nav_trace_enabled();
}
