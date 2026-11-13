#include "include/nav.hpp"
#include "include/geo.hpp"
#include <math.h>
#include <string.h>
#ifndef NAV_HOST
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

namespace marauder {
namespace {
constexpr double kMaxPlausibleMps = 340.0;     // anything faster is a glitch for a handheld/vehicle receiver
constexpr double kMovingMps = 0.5;             // below this the receiver's course is noise
constexpr double kCourseTrustMps = 1.0;        // minimum speed for the receiver course to be used as-is
constexpr double kUereM = 5.0;                 // assumed user-equivalent range error for the HDOP estimate
constexpr double kHeadingMinMoveM = 8.0;
}

NavEngine::NavEngine() { memset(this, 0, sizeof(*this)); trace_min_m_ = 4.0f; trip_start_us_ = 0; }

void NavEngine::trip_reset(uint64_t now_us) { dist_ = 0; vmax_ = 0; moving_us_ = 0; trip_start_us_ = now_us; }
void NavEngine::trace_enable(bool on) { trace_on_ = on; }
void NavEngine::trace_clear() { head_ = 0; count_ = 0; have_trace_ = false; }

bool NavEngine::trace_get(size_t index, NavPoint *p) const {
    if (index >= count_) return false;
    const size_t first = (head_ + kTraceCapacity - count_) % kTraceCapacity;
    *p = trace_[(first + index) % kTraceCapacity];
    return true;
}

void NavEngine::on_fix(const GpsFix &f, uint64_t now_us) {
    if (!f.valid || !geo::coords_valid(f.latitude, f.longitude)) return;
    if (!trip_start_us_) trip_start_us_ = now_us;

    // accuracy: measured (GST) beats estimated (HDOP); neither is invented when absent
    if (f.has_gst && f.sigma_lat_m > 0 && f.sigma_lon_m > 0) {
        acc_ = sqrt(f.sigma_lat_m * f.sigma_lat_m + f.sigma_lon_m * f.sigma_lon_m); acc_src_ = AccSrc::GST;
    } else if (f.has_hdop && f.hdop > 0) { acc_ = f.hdop * kUereM; acc_src_ = AccSrc::ESTIMATE; }
    else { acc_ = 0; acc_src_ = AccSrc::NONE; }

    double step = 0, dt = 0;
    if (have_prev_) {
        step = geo::distance_m(prev_lat_, prev_lon_, f.latitude, f.longitude);
        dt = (double)(now_us - prev_us_) / 1e6;
    }

    // speed: receiver value when present, otherwise distance over time between consecutive fixes
    if (f.has_speed && f.speed_mps >= 0 && f.speed_mps < kMaxPlausibleMps) { speed_ = f.speed_mps; speed_src_ = SpeedSrc::GNSS; }
    else if (f.has_vtg && f.vtg_speed_mps >= 0 && f.vtg_speed_mps < kMaxPlausibleMps) { speed_ = f.vtg_speed_mps; speed_src_ = SpeedSrc::GNSS; }
    else if (have_prev_ && dt >= 0.2 && dt <= 10.0 && step / dt < kMaxPlausibleMps) { speed_ = step / dt; speed_src_ = SpeedSrc::DERIVED; }
    else { speed_ = 0; speed_src_ = SpeedSrc::NONE; }
    const bool moving = speed_src_ != SpeedSrc::NONE && speed_ >= kMovingMps;

    // trip statistics: standing-still jitter is excluded by requiring motion and a step above the noise floor
    const double noise_floor = fmax(2.0, acc_src_ != AccSrc::NONE ? acc_ * 0.5 : 0.0);
    if (have_prev_ && moving && step >= noise_floor && dt > 0 && dt <= 10.0) {
        dist_ += step;
        moving_us_ += (uint64_t)(dt * 1e6);
    }
    if (speed_src_ != SpeedSrc::NONE && speed_ > vmax_) vmax_ = speed_;

    // heading: receiver course when moving fast enough; otherwise bearing of the recent track once the
    // receiver has moved clear of its own position noise; otherwise the last value, labelled HELD
    const double course = f.has_course ? f.course_deg : (f.has_vtg ? f.vtg_course_deg : -1.0);
    if (course >= 0 && course < 360.0 && speed_src_ == SpeedSrc::GNSS && speed_ >= kCourseTrustMps) {
        heading_ = course; heading_src_ = HeadingSrc::GNSS; heading_us_ = now_us;
        anchor_lat_ = f.latitude; anchor_lon_ = f.longitude; have_anchor_ = true;
    } else {
        if (!have_anchor_) { anchor_lat_ = f.latitude; anchor_lon_ = f.longitude; have_anchor_ = true; }
        const double need = fmax(kHeadingMinMoveM, acc_src_ != AccSrc::NONE ? 2.0 * acc_ : 0.0);
        const double moved = geo::distance_m(anchor_lat_, anchor_lon_, f.latitude, f.longitude);
        if (moved >= need) {
            const double b = geo::bearing_deg(anchor_lat_, anchor_lon_, f.latitude, f.longitude);
            if (heading_src_ == HeadingSrc::NONE || fabs(geo::angle_diff_deg(heading_, b)) > 90.0) heading_ = b;
            else { heading_ += 0.5 * geo::angle_diff_deg(heading_, b); if (heading_ < 0) heading_ += 360.0; if (heading_ >= 360.0) heading_ -= 360.0; }
            heading_src_ = HeadingSrc::TRACK; heading_us_ = now_us;
            anchor_lat_ = f.latitude; anchor_lon_ = f.longitude;
        } else if (heading_src_ != HeadingSrc::NONE && now_us - heading_us_ > 10000000ull) {
            heading_src_ = HeadingSrc::HELD;
        }
    }

    // trace: store a point only after moving clear of both the minimum spacing and the position noise
    if (trace_on_) {
        const double spacing = fmax((double)trace_min_m_, acc_src_ != AccSrc::NONE ? 1.5 * acc_ : 0.0);
        const int32_t la = (int32_t)llround(f.latitude * 1e7), lo = (int32_t)llround(f.longitude * 1e7);
        if (!have_trace_ || geo::distance_m(last_trace_.lat_e7 / 1e7, last_trace_.lon_e7 / 1e7, f.latitude, f.longitude) >= spacing) {
            trace_[head_] = {la, lo};
            head_ = (head_ + 1) % kTraceCapacity;
            if (count_ < kTraceCapacity) ++count_;
            last_trace_ = {la, lo}; have_trace_ = true;
        }
    }

    prev_lat_ = lat_ = f.latitude; prev_lon_ = lon_ = f.longitude;
    prev_us_ = last_us_ = now_us; have_prev_ = have_ = true; last_fix_ = f;
}

void NavEngine::snapshot(NavSnapshot *o, const GpsFix &raw, uint64_t now_us) const {
    memset(o, 0, sizeof(*o));
    o->fix = raw;
    o->have_position = have_;
    o->lat = lat_; o->lon = lon_;
    o->fix_age_ms = have_ ? (uint32_t)((now_us - last_us_) / 1000ull) : 0xFFFFFFFFu;
    o->stale = !have_ || o->fix_age_ms > 3000u;
    if (!o->stale) { o->speed_mps = speed_; o->speed_src = speed_src_; o->acc_m = acc_; o->acc_src = acc_src_; }
    o->heading_deg = heading_; o->heading_src = heading_src_;
    o->heading_age_ms = heading_src_ == HeadingSrc::NONE ? 0xFFFFFFFFu : (uint32_t)((now_us - heading_us_) / 1000ull);
    // a heading that has not been refreshed for a while is shown as held, even if no new fix has arrived
    if (heading_src_ != HeadingSrc::NONE && o->heading_age_ms > 10000u) o->heading_src = HeadingSrc::HELD;
    o->trip_dist_m = dist_; o->max_speed_mps = vmax_;
    o->trip_moving_s = (uint32_t)(moving_us_ / 1000000ull);
    o->trip_elapsed_s = trip_start_us_ ? (uint32_t)((now_us - trip_start_us_) / 1000000ull) : 0;
    o->trace_count = (uint16_t)count_; o->trace_enabled = trace_on_;
}

#ifndef NAV_HOST
namespace {
portMUX_TYPE s_nav_mux = portMUX_INITIALIZER_UNLOCKED;
NavEngine s_engine;
}
void nav_on_fix_internal(const GpsFix &f) {   // called by gps.cpp (declared there)
    portENTER_CRITICAL(&s_nav_mux);
    s_engine.on_fix(f, (uint64_t)esp_timer_get_time());
    portEXIT_CRITICAL(&s_nav_mux);
}
void nav_snapshot(NavSnapshot *out) {
    const GpsFix raw = gps_current_fix();
    portENTER_CRITICAL(&s_nav_mux);
    s_engine.snapshot(out, raw, (uint64_t)esp_timer_get_time());
    portEXIT_CRITICAL(&s_nav_mux);
}
void nav_trip_reset() { portENTER_CRITICAL(&s_nav_mux); s_engine.trip_reset((uint64_t)esp_timer_get_time()); portEXIT_CRITICAL(&s_nav_mux); }
void nav_trace_enable(bool on) { portENTER_CRITICAL(&s_nav_mux); s_engine.trace_enable(on); portEXIT_CRITICAL(&s_nav_mux); }
void nav_trace_clear() { portENTER_CRITICAL(&s_nav_mux); s_engine.trace_clear(); portEXIT_CRITICAL(&s_nav_mux); }
bool nav_trace_enabled() { NavSnapshot s; nav_snapshot(&s); return s.trace_enabled; }
size_t nav_trace_copy(NavPoint *out, size_t max) {
    size_t n = 0;
    portENTER_CRITICAL(&s_nav_mux);
    const size_t c = s_engine.trace_count();
    for (size_t i = 0; i < c && n < max; ++i) s_engine.trace_get(i, &out[n++]);
    portEXIT_CRITICAL(&s_nav_mux);
    return n;
}
#endif
}
