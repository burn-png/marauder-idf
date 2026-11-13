#pragma once
// Small geodesy helpers shared by the GPS service, the offline map and the Remote ID overlay.
// Pure functions, no hardware access (host-tested in tests/nav_host).
#include <initializer_list>
#include <math.h>
#include <stdint.h>

namespace marauder::geo {

inline constexpr double kEarthRadiusM = 6371008.8;   // IUGG mean radius
inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kDeg = kPi / 180.0;
inline constexpr double kC_mps = 299792458.0;        // speed of light in vacuum

inline bool coords_valid(double lat, double lon) {
    return lat >= -90.0 && lat <= 90.0 && lon >= -180.0 && lon <= 180.0 && !(lat == 0.0 && lon == 0.0);
}

// Great-circle distance (haversine), metres.
inline double distance_m(double lat1, double lon1, double lat2, double lon2) {
    const double p1 = lat1 * kDeg, p2 = lat2 * kDeg, dp = (lat2 - lat1) * kDeg, dl = (lon2 - lon1) * kDeg;
    const double a = sin(dp / 2) * sin(dp / 2) + cos(p1) * cos(p2) * sin(dl / 2) * sin(dl / 2);
    return 2.0 * kEarthRadiusM * asin(sqrt(a < 1.0 ? a : 1.0));
}

// Initial bearing 0..360 degrees true, from point 1 to point 2.
inline double bearing_deg(double lat1, double lon1, double lat2, double lon2) {
    const double p1 = lat1 * kDeg, p2 = lat2 * kDeg, dl = (lon2 - lon1) * kDeg;
    const double y = sin(dl) * cos(p2);
    const double x = cos(p1) * sin(p2) - sin(p1) * cos(p2) * cos(dl);
    double b = atan2(y, x) / kDeg;
    if (b < 0) b += 360.0;
    if (b >= 360.0) b -= 360.0;
    return b;
}

// Smallest signed difference b - a in degrees, range (-180, 180].
inline double angle_diff_deg(double a, double b) {
    double d = fmod(b - a, 360.0);
    if (d > 180.0) d -= 360.0;
    if (d <= -180.0) d += 360.0;
    return d;
}

// Local tangent-plane offsets in metres of (lat,lon) relative to (lat0,lon0): east, north. Accurate for map-sized areas.
inline void offset_m(double lat0, double lon0, double lat, double lon, double *east, double *north) {
    *north = (lat - lat0) * kDeg * kEarthRadiusM;
    *east = (lon - lon0) * kDeg * kEarthRadiusM * cos(lat0 * kDeg);
}

// ---- Web-Mercator style view ------------------------------------------------------------------------
// The map view is defined by a centre and a scale in metres per pixel at the centre latitude. Using one local
// equirectangular scale across a 240-pixel-wide view is exact enough (error < 0.1% for views under ~50 km).
struct View {
    double lat, lon;          // centre
    double m_per_px;          // metres per pixel at the centre latitude
    double cos_lat;           // cached
    int cx, cy;               // pixel position of the centre on the canvas
    bool north_up;            // false: heading-up
    double rot_deg;           // heading-up rotation (screen "up" = this true bearing)
};

inline void view_set(View &v, double lat, double lon, double m_per_px) {
    v.lat = lat; v.lon = lon; v.m_per_px = m_per_px; v.cos_lat = cos(lat * kDeg);
    if (v.cos_lat < 0.01) v.cos_lat = 0.01;
}

// Geographic -> canvas pixel (float). Returns false when the point is non-finite.
inline bool to_px(const View &v, double lat, double lon, float *x, float *y) {
    const double e = (lon - v.lon) * kDeg * kEarthRadiusM * v.cos_lat;
    const double n = (lat - v.lat) * kDeg * kEarthRadiusM;
    double sx = e / v.m_per_px, sy = n / v.m_per_px;
    if (!v.north_up) {
        const double r = v.rot_deg * kDeg, c = cos(r), s = sin(r);
        const double rx = sx * c - sy * s;
        const double ry = sx * s + sy * c;
        sx = rx; sy = ry;
    }
    if (!isfinite(sx) || !isfinite(sy)) return false;
    *x = (float)(v.cx + sx);
    *y = (float)(v.cy - sy);
    return true;
}

// Canvas pixel -> geographic.
inline void from_px(const View &v, float x, float y, double *lat, double *lon) {
    double sx = x - v.cx, sy = v.cy - y;
    if (!v.north_up) {
        const double r = -v.rot_deg * kDeg, c = cos(r), s = sin(r);
        const double rx = sx * c - sy * s, ry = sx * s + sy * c;
        sx = rx; sy = ry;
    }
    *lat = v.lat + (sy * v.m_per_px) / kEarthRadiusM / kDeg;
    *lon = v.lon + (sx * v.m_per_px) / (kEarthRadiusM * v.cos_lat) / kDeg;
}

// Pan the view centre by a number of screen pixels (screen right/up), honouring rotation.
inline void pan_px(View &v, double dx, double dy) {
    double lat, lon;
    from_px(v, (float)(v.cx + dx), (float)(v.cy - dy), &lat, &lon);
    view_set(v, lat, lon, v.m_per_px);
}

// Rounded 1-2-5 scale-bar length (metres) that is no wider than max_px at this scale.
inline double scale_bar_m(double m_per_px, int max_px, int *px_out) {
    const double limit = m_per_px * max_px;
    double base = pow(10.0, floor(log10(limit)));
    double best = base;
    for (double m : {1.0, 2.0, 5.0, 10.0}) if (m * base <= limit) best = m * base;
    *px_out = (int)(best / m_per_px + 0.5);
    return best;
}

// Standard zoom ladder, metres per pixel (index 0 = widest).
inline constexpr double kZoomMpp[] = {160.0, 80.0, 40.0, 20.0, 10.0, 5.0, 2.5, 1.25, 0.6};
inline constexpr int kZoomCount = (int)(sizeof(kZoomMpp) / sizeof(*kZoomMpp));

// Speed of light in air: refractive index ~1.0003 is a 0.03% effect; documented, not applied by default.
}  // namespace marauder::geo
