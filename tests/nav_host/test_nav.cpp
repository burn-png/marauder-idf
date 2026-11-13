#include "include/nav.hpp"
#include "include/geo.hpp"
#include <stdio.h>
#include <string.h>
#include <math.h>
using namespace marauder;
static int fails = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); ++fails; } } while (0)
#define NEAR(a, b, t) do { double _a=(a), _b=(b); if (fabs(_a-_b) > (t)) { printf("FAIL %s:%d  %s=%.6f expected %.6f\n", __FILE__, __LINE__, #a, _a, _b); ++fails; } } while (0)

static GpsFix fix(double lat, double lon, double speed, double course, bool has_speed = true) {
    GpsFix f{}; f.valid = true; f.latitude = lat; f.longitude = lon;
    f.has_speed = has_speed; f.speed_mps = speed; f.has_course = has_speed; f.course_deg = course;
    return f;
}
// metres north/east -> degrees at lat 29.76 (Houston)
static const double LAT0 = 29.7604, LON0 = -95.3698;
static double dlat(double m) { return m / (geo::kEarthRadiusM * geo::kDeg); }
static double dlon(double m) { return m / (geo::kEarthRadiusM * geo::kDeg * cos(LAT0 * geo::kDeg)); }

int main() {
    // ---- geo ----
    NEAR(geo::distance_m(0, 0, 0, 1), 111194.9, 5);               // 1 deg at the equator
    NEAR(geo::distance_m(29.7604, -95.3698, 29.7604 + dlat(1000), -95.3698), 1000.0, 0.5);
    NEAR(geo::bearing_deg(0, 0, 1, 0), 0.0, 1e-6);
    NEAR(geo::bearing_deg(0, 0, 0, 1), 90.0, 1e-6);
    NEAR(geo::bearing_deg(0, 0, -1, 0), 180.0, 1e-6);
    NEAR(geo::bearing_deg(0, 0, 0, -1), 270.0, 1e-6);
    NEAR(geo::angle_diff_deg(350, 10), 20.0, 1e-9);
    NEAR(geo::angle_diff_deg(10, 350), -20.0, 1e-9);
    CHECK(!geo::coords_valid(0, 0)); CHECK(!geo::coords_valid(91, 0)); CHECK(geo::coords_valid(29.7, -95.3));
    {   // projection round trip, north-up and heading-up
        geo::View v{}; geo::view_set(v, LAT0, LON0, 5.0); v.cx = 120; v.cy = 150; v.north_up = true;
        float x, y; geo::to_px(v, LAT0 + dlat(500), LON0, &x, &y);
        NEAR(x, 120, 0.01); NEAR(y, 150 - 100, 0.01);                // 500 m north at 5 m/px = 100 px up
        geo::to_px(v, LAT0, LON0 + dlon(250), &x, &y);
        NEAR(x, 120 + 50, 0.01); NEAR(y, 150, 0.01);
        double la, lo; geo::from_px(v, 200, 60, &la, &lo);
        geo::to_px(v, la, lo, &x, &y); NEAR(x, 200, 0.01); NEAR(y, 60, 0.01);
        v.north_up = false; v.rot_deg = 90;                           // heading east: east is screen-up
        geo::to_px(v, LAT0, LON0 + dlon(500), &x, &y);
        NEAR(x, 120, 0.05); NEAR(y, 150 - 100, 0.05);
        geo::from_px(v, 33, 77, &la, &lo); geo::to_px(v, la, lo, &x, &y); NEAR(x, 33, 0.01); NEAR(y, 77, 0.01);
        int px; double m = geo::scale_bar_m(5.0, 80, &px);
        NEAR(m, 200.0, 1e-9); CHECK(px == 40);
        v.north_up = true; geo::pan_px(v, 50, 0);                     // pan 50 px east = 250 m
        NEAR(geo::distance_m(LAT0, LON0, v.lat, v.lon), 250.0, 0.5);
    }

    // ---- nav: receiver speed/course used as reported ----
    {
        NavEngine e; e.trace_enable(true);
        uint64_t t = 1000000;
        double lat = LAT0;
        for (int i = 0; i < 30; ++i) { e.on_fix(fix(lat, LON0, 10.0, 0.0), t); lat += dlat(10.0); t += 1000000; }
        NavSnapshot s; e.snapshot(&s, GpsFix{}, t);
        CHECK(s.have_position && !s.stale);
        CHECK(s.heading_src == HeadingSrc::GNSS); NEAR(s.heading_deg, 0.0, 1e-9);
        CHECK(s.speed_src == SpeedSrc::GNSS); NEAR(s.speed_mps, 10.0, 1e-9);
        NEAR(s.trip_dist_m, 29 * 10.0, 1.0);                          // 29 steps of 10 m
        NEAR(s.max_speed_mps, 10.0, 1e-9);
        CHECK(s.trace_count >= 20 && s.trace_count <= 30);
        CHECK(s.acc_src == AccSrc::NONE);                              // no HDOP/GST -> no accuracy invented
    }
    // ---- nav: stationary jitter must not add distance, must not invent a heading ----
    {
        NavEngine e; uint64_t t = 1000000; unsigned r = 1;
        for (int i = 0; i < 60; ++i) {
            r = r * 1664525u + 1013904223u; const double n = ((int)(r >> 16) % 7 - 3) * 0.6, ee = ((int)(r >> 8) % 7 - 3) * 0.6;
            GpsFix f = fix(LAT0 + dlat(n), LON0 + dlon(ee), 0.1, 123.0); f.has_hdop = true; f.hdop = 1.2;
            e.on_fix(f, t); t += 1000000;
        }
        NavSnapshot s; e.snapshot(&s, GpsFix{}, t);
        NEAR(s.trip_dist_m, 0.0, 1e-9);
        CHECK(s.heading_src == HeadingSrc::NONE);                      // COG at 0.1 m/s is noise: not used
        CHECK(s.acc_src == AccSrc::ESTIMATE); NEAR(s.acc_m, 6.0, 1e-9);
    }
    // ---- nav: heading estimated from track when the receiver gives no course ----
    {
        NavEngine e; uint64_t t = 1000000; double la = LAT0, lo = LON0;
        for (int i = 0; i < 40; ++i) { e.on_fix(fix(la, lo, 0, 0, false), t); la += dlat(3.0); lo += dlon(3.0); t += 1000000; }  // 45 degrees
        NavSnapshot s; e.snapshot(&s, GpsFix{}, t);
        CHECK(s.speed_src == SpeedSrc::DERIVED); NEAR(s.speed_mps, 4.24, 0.1);
        CHECK(s.heading_src == HeadingSrc::TRACK); NEAR(s.heading_deg, 45.0, 2.0);
        // then standing still: heading is kept but labelled HELD after 10 s, never refreshed
        for (int i = 0; i < 15; ++i) { e.on_fix(fix(la, lo, 0, 0, false), t); t += 1000000; }
        e.snapshot(&s, GpsFix{}, t);
        CHECK(s.heading_src == HeadingSrc::HELD); NEAR(s.heading_deg, 45.0, 2.0);
    }
    // ---- nav: fix loss keeps last position, drops live speed ----
    {
        NavEngine e; e.on_fix(fix(LAT0, LON0, 5.0, 90.0), 1000000);
        NavSnapshot s; e.snapshot(&s, GpsFix{}, 2000000); CHECK(!s.stale && s.speed_src == SpeedSrc::GNSS);
        e.snapshot(&s, GpsFix{}, 9000000);
        CHECK(s.stale && s.have_position); NEAR(s.lat, LAT0, 1e-12); CHECK(s.speed_src == SpeedSrc::NONE);
        CHECK(s.fix_age_ms == 8000);
    }
    // ---- nav: invalid input rejected, GST accuracy preferred, trace ring wraps, trip reset ----
    {
        NavEngine e; GpsFix bad = fix(0, 0, 1, 1); e.on_fix(bad, 1); NavSnapshot s; e.snapshot(&s, GpsFix{}, 2); CHECK(!s.have_position);
        bad = fix(95, 10, 1, 1); e.on_fix(bad, 1); e.snapshot(&s, GpsFix{}, 2); CHECK(!s.have_position);
        GpsFix f = fix(LAT0, LON0, 2, 0); f.has_gst = true; f.sigma_lat_m = 3; f.sigma_lon_m = 4; f.has_hdop = true; f.hdop = 9;
        e.on_fix(f, 1000000); e.snapshot(&s, GpsFix{}, 1500000); CHECK(s.acc_src == AccSrc::GST); NEAR(s.acc_m, 5.0, 1e-9);
        e.trace_enable(true); double la = LAT0;
        for (int i = 0; i < 400; ++i) { e.on_fix(fix(la, LON0, 5, 0), 2000000 + i * 1000000ull); la += dlat(10); }
        CHECK(e.trace_count() == kTraceCapacity);
        NavPoint a, b; CHECK(e.trace_get(0, &a) && e.trace_get(kTraceCapacity - 1, &b)); CHECK(b.lat_e7 > a.lat_e7);
        e.trip_reset(500000000); e.snapshot(&s, GpsFix{}, 501000000); NEAR(s.trip_dist_m, 0, 1e-9); CHECK(s.trip_elapsed_s == 1);
        e.trace_clear(); CHECK(e.trace_count() == 0);
    }
    printf(fails ? "FAILED (%d)\n" : "ALL PASS\n", fails);
    return fails != 0;
}
