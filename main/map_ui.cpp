#include "include/map_ui.hpp"
#include "include/canvas_draw.hpp"
#include "include/display.hpp"
#include "include/geo.hpp"
#include "include/gps.hpp"
#include "include/nav.hpp"
#include "include/osm.hpp"
#include "include/rf_range.hpp"
#include "include/rid.hpp"
#include "include/storage.hpp"
#include "font_cp437.hpp"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>
#include <new>
#include <stdio.h>
#include <string.h>

namespace marauder {
namespace {
constexpr char kTag[] = "map";
constexpr const char *kSrc = "maps/regional_map.osm";
constexpr const char *kCache = "map_cache/regional_map";
constexpr const char *kRangeCsv = "maps/rf_range.csv";

constexpr int kMapTop = 44, kBandH = 56, kBands = 4, kMapH = kBandH * kBands;       // map area y 44..267
constexpr int kCenterX = 120, kCenterY = kMapH / 2;                                  // screen centre inside the map area
constexpr uint16_t C_BLACK = 0, C_WHITE = 0xFFFF, C_CYAN = 0x07FF, C_AMBER = 0xFEA0, C_RED = 0xF800, C_GREEN = 0x07E0, C_SEL = 0x21A7;

// Palette: OSM classes 0..8, overlay classes 9..15 (see canvas_draw.hpp).
const uint16_t kPalette[16] = {0x0000, 0x0200, 0x0014, 0x2104, 0x6B4D, 0x7BEF, 0xA145, 0xBDF7, 0xFD20,
                               0x07FF, 0xFFFF, 0xF81F, 0x9813, 0xF800, 0x87F0, 0xFFFF};

// ---- SD access for the OSM module (uses the existing storage layer; the SPI gate is taken inside it) ----
class SdIo : public osm::FileIo {
public:
    void *open_read(const char *p) override { return storage_file_open(p, "rb"); }
    void *open_write(const char *p, bool append) override { return storage_file_open(p, append ? "ab" : "wb"); }
    size_t read(void *h, void *b, size_t n) override { return storage_file_read(h, b, n); }
    bool write(void *h, const void *b, size_t n) override { return storage_file_write(h, b, n); }
    bool seek(void *h, size_t o) override { return storage_file_seek(h, o); }
    void close(void *h) override { storage_file_close(h); }
    size_t size(const char *p) override { return storage_file_size(p); }
    bool remove(const char *p) override { return storage_delete_file(p); }
    bool mkdir(const char *p) override { return storage_make_dir(p); }
};
SdIo s_io;

enum class MapState : uint8_t { NO_SD, NO_FILE, NEED_INDEX, INDEXING, READY, FAILED };
enum class Mode : uint8_t { VIEW, PAN, QUICK };

void (*s_pause_hook)(bool) = nullptr;
MapScreen s_screen = MapScreen::MAP;
bool s_open = false, s_imperial = false;

// map state
MapState s_ms = MapState::NO_FILE;
Mode s_mode = Mode::VIEW;
osm::Meta s_meta{};
osm::Indexer *s_idx = nullptr;
osm::Renderer *s_rend = nullptr;
uint8_t *s_cvmem = nullptr;
osm::Canvas s_cv{};
int s_zoom = 4;
bool s_follow = true, s_north_up = true, s_hud = true, s_show_rid = true, s_show_rings = true, s_show_trace = true;
int s_layer_preset = 0;
double s_clat = 0, s_clon = 0;               // view centre when not following (and last centre when no fix has existed)
bool s_have_centre = false;
geo::View s_render_view{};
bool s_view_dirty = true;
int s_band = kBands;                          // kBands: no render in progress
uint32_t s_last_cycle_ms = 0, s_last_rid_rev = 0;
char s_msg[40] = {};
osm::Error s_err = osm::Error::NONE;
uint32_t s_need_k = 0, s_have_k = 0;
bool s_text_dirty = true, s_full_clear = true;
int s_quick = 0;
bool s_quick_drawn = false;

// per-cycle snapshots (so every band overlays the same data)
NavSnapshot s_nav{};
NavPoint s_trace[kTraceCapacity];
size_t s_trace_n = 0;
RidAircraft s_rid[kRidCapacity];
size_t s_rid_n = 0;
RangeRing s_rings[kRangeRings];
size_t s_ring_n = 0;

// rid list state
size_t s_rid_sel = 0;
char s_rid_msg[40] = {};

constexpr uint16_t kLayerPresets[] = {osm::L_ALL, osm::L_ROADS | osm::L_PATHS, osm::L_ROADS | osm::L_WATER | osm::L_GREEN,
                                      osm::L_ROADS | osm::L_BUILDINGS, osm::L_ROADS};
const char *const kLayerNames[] = {"ALL", "ROADS+PATHS", "ROADS WATER GREEN", "ROADS BUILDINGS", "ROADS ONLY"};

uint32_t now_ms() { return (uint32_t)(esp_timer_get_time() / 1000); }
double mpp() { return geo::kZoomMpp[s_zoom]; }
size_t free_internal() { return heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); }

void set_msg(const char *t) { snprintf(s_msg, sizeof(s_msg), "%s", t); s_text_dirty = true; }

// ---------------------------------------------------------------------------------------------------------------
// units / text helpers
void fmt_speed(char *o, size_t n, double mps) {
    if (s_imperial) snprintf(o, n, "%.1f mph", mps * 2.2369363); else snprintf(o, n, "%.1f km/h", mps * 3.6);
}
void fmt_dist(char *o, size_t n, double m) {
    if (s_imperial) { const double ft = m * 3.28084; if (ft < 1000) snprintf(o, n, "%.0f ft", ft); else snprintf(o, n, "%.2f mi", m / 1609.344); }
    else { if (m < 1000) snprintf(o, n, "%.0f m", m); else snprintf(o, n, "%.2f km", m / 1000.0); }
}
const char *compass(double deg) {
    static const char *const n[] = {"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};
    deg = fmod(deg + 360.0 * 4, 360.0);
    return n[(int)((deg + 11.25) / 22.5) & 15];
}
void fmt_lat(char *o, size_t n, double v) { snprintf(o, n, "%.6f %c", fabs(v), v >= 0 ? 'N' : 'S'); }
void fmt_lon(char *o, size_t n, double v) { snprintf(o, n, "%.6f %c", fabs(v), v >= 0 ? 'E' : 'W'); }
void text(int x, int y, const char *s, uint16_t fg, uint16_t bg = C_BLACK) {
    char b[32]; snprintf(b, sizeof(b), "%-*.*s", (240 - x) / 8 < 30 ? (240 - x) / 8 : 30, (240 - x) / 8 < 30 ? (240 - x) / 8 : 30, s);
    display_text(x, y, b, fg, bg);
}
void row(int r, const char *s, uint16_t fg = C_WHITE, uint16_t bg = C_BLACK) { text(0, 34 + r * 11, s, fg, bg); }

// ---------------------------------------------------------------------------------------------------------------
// map lifecycle
void free_map_objects() {
    if (s_rend) { s_rend->cancel(); delete s_rend; s_rend = nullptr; }
    if (s_idx) { s_idx->cancel(); delete s_idx; s_idx = nullptr; }
    free(s_cvmem); s_cvmem = nullptr;
}
void detect_map() {
    s_err = osm::Error::NONE;
    if (!storage_accessible()) { s_ms = MapState::NO_SD; return; }
    if (osm::load_meta(&s_io, kCache, kSrc, &s_meta)) { s_ms = MapState::READY; return; }
    s_ms = s_io.size(kSrc) ? MapState::NEED_INDEX : MapState::NO_FILE;
}
void ensure_canvas() {                       // (re)allocate the render canvas and renderer after indexing freed them
    if (!s_cvmem) {
        s_cvmem = (uint8_t *)malloc(osm::canvas_bytes(240, kBandH));
        if (s_cvmem) s_cv = {s_cvmem, 240, kBandH};
    }
    if (s_ms == MapState::READY && !s_rend) s_rend = new (std::nothrow) osm::Renderer();
}
void start_index() {
    // Free everything the renderer holds first: the indexer needs the RAM and nothing renders while it runs.
    if (s_rend) { s_rend->cancel(); delete s_rend; s_rend = nullptr; }
    free(s_cvmem); s_cvmem = nullptr; s_band = kBands;
    if (s_idx) { s_idx->cancel(); delete s_idx; s_idx = nullptr; }
    s_idx = new (std::nothrow) osm::Indexer();
    if (!s_idx) { s_ms = MapState::FAILED; s_err = osm::Error::NO_MEMORY; return; }
    const size_t need = s_idx->heap_estimate() + 6 * 1024;       // small reserve for the UI and Wi-Fi
    const size_t have = free_internal();
    if (have < need) {
        s_need_k = (uint32_t)(need / 1024); s_have_k = (uint32_t)(have / 1024);
        delete s_idx; s_idx = nullptr; s_ms = MapState::FAILED; s_err = osm::Error::NO_MEMORY; ensure_canvas(); return;
    }
    if (!s_idx->start(&s_io, kSrc, kCache)) {
        osm::IndexProgress p{}; s_idx->progress(&p); s_err = p.error;
        delete s_idx; s_idx = nullptr; s_ms = MapState::FAILED; ensure_canvas(); return;
    }
    s_ms = MapState::INDEXING;
}
void centre_from_data() {
    if (s_have_centre) return;
    if (s_nav.have_position) { s_clat = s_nav.lat; s_clon = s_nav.lon; s_have_centre = true; return; }
    if (s_ms == MapState::READY) { s_clat = (s_meta.min_lat_e7 + (double)s_meta.max_lat_e7) / 2e7; s_clon = (s_meta.min_lon_e7 + (double)s_meta.max_lon_e7) / 2e7; s_have_centre = true; }
}
void desired_view(geo::View *v) {
    double lat = s_clat, lon = s_clon;
    if (s_follow && s_nav.have_position) { lat = s_nav.lat; lon = s_nav.lon; }
    geo::view_set(*v, lat, lon, mpp());
    v->cx = kCenterX; v->cy = kCenterY;
    v->north_up = s_north_up || s_nav.heading_src == HeadingSrc::NONE;
    v->rot_deg = v->north_up ? 0 : s_nav.heading_deg;
}

// ---------------------------------------------------------------------------------------------------------------
// band rendering
void overlay_band(const geo::View &v) {
    using namespace cdraw;
    float x, y;
    if (s_show_rings) for (size_t i = 0; i < s_ring_n; ++i) {
        const auto &r = s_rings[i];
        if (!geo::to_px(v, r.obs_lat, r.obs_lon, &x, &y)) continue;
        circle(s_cv, x, y, (float)(r.est.ring_inner_m / v.m_per_px), K_RING);
        circle(s_cv, x, y, (float)(r.est.ring_outer_m / v.m_per_px), K_RING, 6);
    }
    if (s_show_trace && s_trace_n > 1) {
        float px = 0, py = 0; bool have = false;
        for (size_t i = 0; i < s_trace_n; ++i) {
            if (!geo::to_px(v, s_trace[i].lat_e7 / 1e7, s_trace[i].lon_e7 / 1e7, &x, &y)) { have = false; continue; }
            if (have) line(s_cv, (int)px, (int)py, (int)x, (int)y, K_TRACE);
            px = x; py = y; have = true;
        }
    }
    if (s_show_rid) for (size_t i = 0; i < s_rid_n; ++i) {
        const auto &a = s_rid[i];
        if (!a.has_position || !geo::coords_valid(a.lat, a.lon)) continue;      // aircraft-REPORTED position only
        const uint8_t k = a.state == RidState::ACTIVE ? K_RID_ACTIVE : a.state == RidState::STALE ? K_RID_STALE : K_RID_LOST;
        for (unsigned t = 0; t < a.trail_count; ++t) {                          // previous reported positions
            double la = a.lat + a.trail_n[t] / 111320.0, lo = a.lon + a.trail_e[t] / (111320.0 * v.cos_lat);
            if (geo::to_px(v, la, lo, &x, &y)) put(s_cv, (int)x, (int)y, k);
        }
        if (!geo::to_px(v, a.lat, a.lon, &x, &y)) continue;
        if (a.state == RidState::ACTIVE) diamond(s_cv, (int)x, (int)y, 4, k, true);
        else if (a.state == RidState::STALE) diamond(s_cv, (int)x, (int)y, 4, k, false);
        else cross(s_cv, (int)x, (int)y, 3, k);
        if (a.has_heading && a.state != RidState::LOST) {
            const float b = (float)a.heading_deg - (float)v.rot_deg;
            line(s_cv, (int)x, (int)y, (int)(x + sinf(b * 0.0174533f) * 11), (int)(y - cosf(b * 0.0174533f) * 11), k);
        }
    }
    if (s_nav.have_position && geo::to_px(v, s_nav.lat, s_nav.lon, &x, &y)) {   // receiver GPS (distinct shape from aircraft)
        if (s_nav.acc_src != AccSrc::NONE) { const float r = (float)(s_nav.acc_m / v.m_per_px); if (r >= 4 && r <= 120) circle(s_cv, x, y, r, K_RX, 4); }
        if (s_nav.stale) { circle(s_cv, x, y, 6, K_RX, 4); cross(s_cv, (int)x, (int)y, 2, K_RX); }
        else if (s_nav.heading_src != HeadingSrc::NONE) arrow(s_cv, x, y, (float)(s_nav.heading_deg - v.rot_deg), 9, K_RX);
        else { circle(s_cv, x, y, 5, K_RX); put(s_cv, (int)x, (int)y, K_RX); }
    }
    if (s_mode == Mode::PAN) {
        const int cx = v.cx, cy = v.cy;
        cdraw::line(s_cv, cx - 8, cy, cx + 8, cy, K_UI); cdraw::line(s_cv, cx, cy - 8, cx, cy + 8, K_UI);
    }
}
void north_and_scale(int band, const geo::View &v) {
    using namespace cdraw;
    if (band == 0) {                                   // north arrow, top right
        arrow(s_cv, 222, 14, (float)(-v.rot_deg), 9, K_UI);
        glyph(s_cv, font::glyphs[(uint8_t)'N'], 217, 25, 1, K_UI);
    }
    if (band == kBands - 1) {                          // scale bar, bottom left
        int px; geo::scale_bar_m(v.m_per_px, 80, &px);
        const int y = s_cv.h - 5;
        line(s_cv, 8, y, 8 + px, y, K_UI); line(s_cv, 8, y - 3, 8, y + 1, K_UI); line(s_cv, 8 + px, y - 3, 8 + px, y + 1, K_UI);
    }
}
void blit_canvas(int top) {
    uint16_t buf[240 * 4];
    for (int y = 0; y < s_cv.h; y += 4) {
        const int n = s_cv.h - y < 4 ? s_cv.h - y : 4;
        for (int r = 0; r < n; ++r) for (int x = 0; x < 240; ++x) buf[r * 240 + x] = kPalette[osm::canvas_get(s_cv, x, y + r)];
        display_rgb565_image(0, top + y, 240, n, buf);
    }
}
void snapshot_cycle() {
    nav_snapshot(&s_nav);
    s_trace_n = nav_trace_copy(s_trace, kTraceCapacity);
    s_rid_n = s_show_rid ? rid_snapshot(s_rid, kRidCapacity) : 0;
    s_ring_n = range_store().snapshot(s_rings, kRangeRings);
}
void begin_band(int band) {
    geo::View v = s_render_view; v.cy = kCenterY - band * kBandH;
    osm::canvas_clear(s_cv);
    if (s_ms == MapState::READY && s_rend) {
        if (!s_rend->begin(&s_io, kCache, s_meta, v, s_cv, osm::layers_for_scale(v.m_per_px, kLayerPresets[s_layer_preset]))) {
            ESP_LOGW(kTag, "renderer begin failed");
        }
    }
}
void finish_band(int band) {
    geo::View v = s_render_view; v.cy = kCenterY - band * kBandH;
    overlay_band(v); north_and_scale(band, v);
    blit_canvas(kMapTop + band * kBandH);
}
void start_cycle() {
    centre_from_data();
    if (!s_have_centre && s_ms != MapState::READY) { desired_view(&s_render_view); }
    snapshot_cycle(); desired_view(&s_render_view);
    s_band = 0; s_view_dirty = false; s_last_cycle_ms = now_ms(); s_text_dirty = true;
    if (s_rend) s_rend->cancel();
    begin_band(0);
}

bool view_changed() {
    geo::View v; desired_view(&v);
    if (!geo::coords_valid(v.lat, v.lon)) return false;
    const double dpx = geo::distance_m(v.lat, v.lon, s_render_view.lat, s_render_view.lon) / v.m_per_px;
    return dpx >= 6.0 || v.m_per_px != s_render_view.m_per_px || v.north_up != s_render_view.north_up ||
           (!v.north_up && fabs(geo::angle_diff_deg(v.rot_deg, s_render_view.rot_deg)) >= 10.0);
}

void map_tick() {
    if (s_ms == MapState::INDEXING && s_idx) {
        const bool more = s_idx->step(6144);
        osm::IndexProgress p{}; s_idx->progress(&p);
        s_text_dirty = true;
        if (!more) {
            if (p.state == osm::State::DONE) { delete s_idx; s_idx = nullptr; detect_map(); ensure_canvas(); s_view_dirty = true; set_msg("MAP INDEX COMPLETE"); }
            else { s_err = p.error; delete s_idx; s_idx = nullptr; s_ms = MapState::FAILED; ensure_canvas(); }
        }
        return;
    }
    if (!s_cvmem) return;
    if (s_band < kBands) {
        bool more = false;
        if (s_ms == MapState::READY && s_rend && s_rend->active()) more = s_rend->step(24);
        if (more) return;
        finish_band(s_band);
        if (++s_band < kBands) begin_band(s_band);
        else { s_text_dirty = true; }
        return;
    }
    // idle: restart when the view moved, or periodically so aircraft and the receiver marker stay current
    const uint32_t t = now_ms();
    RidStats rs{}; rid_stats(&rs);
    const uint32_t rev = rs.frames_decoded;
    if (s_view_dirty || view_changed() || (t - s_last_cycle_ms > 1000 && (rev != s_last_rid_rev || s_nav.have_position || s_rid_n)) ) {
        s_last_rid_rev = rev; start_cycle();
    }
}

// ---------------------------------------------------------------------------------------------------------------
// map screen drawing (text parts)
void status_bar() {
    char b[96], fx[8], mp[10];
    NavSnapshot &n = s_nav;
    nav_snapshot(&n);
    if (!n.have_position) snprintf(fx, sizeof(fx), "NOFIX");
    else if (n.stale) snprintf(fx, sizeof(fx), "LOST");
    else snprintf(fx, sizeof(fx), "%s", n.fix.fix_type == 3 ? "3D" : n.fix.fix_type == 2 ? "2D" : "FIX");
    if (s_ms == MapState::READY) snprintf(mp, sizeof(mp), "MAP");
    else if (s_ms == MapState::INDEXING) snprintf(mp, sizeof(mp), "IDX");
    else if (s_ms == MapState::NO_SD) snprintf(mp, sizeof(mp), "NOSD");
    else snprintf(mp, sizeof(mp), "NOMAP");
    RidStats rs{}; rid_stats(&rs);
    char ridb[8]; if (rs.running) snprintf(ridb, sizeof(ridb), "R%u", (unsigned)s_rid_n); else snprintf(ridb, sizeof(ridb), "R-");
    char z[10]; if (mpp() >= 10) snprintf(z, sizeof(z), "%.0fm", mpp()); else snprintf(z, sizeof(z), "%.1fm", mpp());
    snprintf(b, sizeof(b), "%s S%02u %s %s %s%s", fx, n.fix.satellites, mp, ridb, z, s_follow ? " F" : " P");
    text(0, 33, b, n.stale || !n.have_position ? C_AMBER : C_GREEN, 0x18E3);
}
void hud_lines() {
    NavSnapshot &n = s_nav;
    char a[96], b[96], c[96], t1[24], t2[24];
    if (!s_hud) { text(0, 269, "", C_WHITE); text(0, 278, "", C_WHITE); text(0, 287, "", C_WHITE); return; }
    if (n.have_position) {
        fmt_lat(t1, sizeof(t1), n.lat); fmt_lon(t2, sizeof(t2), n.lon);
        snprintf(a, sizeof(a), "%.12s %.13s", t1, t2);
        char sp[16]; if (n.speed_src != SpeedSrc::NONE) fmt_speed(sp, sizeof(sp), n.speed_mps); else snprintf(sp, sizeof(sp), "---");
        if (n.heading_src != HeadingSrc::NONE) snprintf(b, sizeof(b), "%s %03.0f%c%s", sp, n.heading_deg, 0xF8, compass(n.heading_deg));
        else snprintf(b, sizeof(b), "%s HDG ---", sp);
        if (n.stale) snprintf(c, sizeof(c), "FIX LOST %lus AGO", (unsigned long)(n.fix_age_ms / 1000));
        else if (n.acc_src != AccSrc::NONE) snprintf(c, sizeof(c), "ALT %s +-%.0fm%s", n.fix.has_altitude ? "ok" : "--", n.acc_m, n.acc_src == AccSrc::ESTIMATE ? "~" : "");
        else snprintf(c, sizeof(c), "ACC ---");
        if (!n.stale && n.fix.has_altitude) { char d[16]; fmt_dist(d, sizeof(d), n.fix.altitude_m); snprintf(c, sizeof(c), "ALT %s  +-%.0fm%s", d, n.acc_m, n.acc_src == AccSrc::ESTIMATE ? "~" : ""); }
    } else {
        snprintf(a, sizeof(a), "NO GPS FIX YET"); snprintf(b, sizeof(b), "RX POSITION UNKNOWN"); snprintf(c, sizeof(c), "SATS %u", n.fix.satellites);
    }
    text(0, 269, a, C_WHITE); text(0, 278, b, n.stale ? C_AMBER : C_CYAN); text(0, 287, c, n.stale ? C_AMBER : C_WHITE);
}

struct QuickItem { const char *name; };
constexpr int kQuickN = 15;
void quick_value(int i, char *o, size_t n) {
    switch (i) {
        case 0: snprintf(o, n, "%s%.4g m/px", "", mpp()); break;
        case 1: snprintf(o, n, "%s", s_follow ? "ON" : "OFF"); break;
        case 2: snprintf(o, n, "%s", s_north_up ? "NORTH-UP" : "HEADING-UP"); break;
        case 3: snprintf(o, n, "%s %u", nav_trace_enabled() ? "ON" : "OFF", (unsigned)s_nav.trace_count); break;
        case 5: snprintf(o, n, "%s", s_hud ? "ON" : "OFF"); break;
        case 6: snprintf(o, n, "%s", s_show_rid ? "ON" : "OFF"); break;
        case 7: snprintf(o, n, "%s", rid_running() ? "SCANNING" : "OFF"); break;
        case 8: snprintf(o, n, "%s %u", s_show_rings ? "ON" : "OFF", (unsigned)range_store().count()); break;
        case 9: snprintf(o, n, "%s", kLayerNames[s_layer_preset]); break;
        case 10: snprintf(o, n, "%s", s_imperial ? "IMPERIAL" : "METRIC"); break;
        default: o[0] = 0; break;
    }
}
const char *const kQuickNames[kQuickN] = {"ZOOM", "FOLLOW GPS", "ORIENTATION", "TRACE PATH", "CLEAR TRACE", "HUD", "RID MARKERS", "RID SCAN",
                                          "RANGE RINGS", "MAP LAYERS", "UNITS", "RESET TRIP", "REBUILD MAP INDEX", "LOAD RF RANGE CSV", "CLOSE"};
void draw_quick() {
    if (!s_quick_drawn) { display_fill_rect(0, 32, 240, 264, C_BLACK); s_quick_drawn = true; }
    row(0, "QUICK SETTINGS", C_CYAN);
    for (int i = 0; i < kQuickN; ++i) {
        char v[24], l[40]; quick_value(i, v, sizeof(v));
        snprintf(l, sizeof(l), "%-17s %12s", kQuickNames[i], v);
        row(1 + i, l, i == s_quick ? C_WHITE : C_CYAN, i == s_quick ? C_SEL : C_BLACK);
    }
    row(17, s_msg, C_AMBER);
    row(19, "UP/DN ROW  L/R CHANGE", C_WHITE);
    row(20, "CENTER = TOGGLE / RUN", C_WHITE);
}
void draw_map_status_panel() {
    // shown instead of the map when the OSM cache is not usable; Remote ID, trace and receiver GPS still render over black
    char l[96];
    if (s_ms == MapState::NO_SD) text(0, 48, "SD NOT MOUNTED - NO BASE MAP", C_RED);
    else if (s_ms == MapState::NO_FILE) { text(0, 48, "NO BASE MAP (OVERLAYS ONLY)", C_AMBER); text(0, 58, "PUT maps/regional_map.osm ON SD", C_WHITE); text(0, 68, "THEN OPEN QUICK > REBUILD INDEX", C_WHITE); }
    else if (s_ms == MapState::NEED_INDEX) {
        snprintf(l, sizeof(l), "OSM FILE %lu KB NOT INDEXED", (unsigned long)(s_io.size(kSrc) / 1024)); text(0, 48, l, C_AMBER);
        text(0, 58, "QUICK > REBUILD MAP INDEX", C_WHITE);
    } else if (s_ms == MapState::INDEXING && s_idx) {
        osm::IndexProgress p{}; s_idx->progress(&p);
        const unsigned pct = p.bytes_total ? (unsigned)((uint64_t)p.bytes_done * 100 / p.bytes_total) : 0;
        snprintf(l, sizeof(l), "INDEXING %s %u%%", p.state == osm::State::NODES ? "NODES" : "WAYS", pct); text(0, 48, l, C_CYAN);
        snprintf(l, sizeof(l), "NODES %lu WAYS %lu", (unsigned long)p.nodes, (unsigned long)p.ways); text(0, 58, l, C_WHITE);
        snprintf(l, sizeof(l), "FEATURES %lu MISSING %lu", (unsigned long)p.features, (unsigned long)p.missing_refs); text(0, 68, l, C_WHITE);
        text(0, 78, "HOLD CENTER CANCELS", C_AMBER);
    } else if (s_ms == MapState::FAILED) {
        if (s_err == osm::Error::NO_MEMORY && s_need_k) snprintf(l, sizeof(l), "NO MEMORY: NEED %luK HAVE %luK", (unsigned long)s_need_k, (unsigned long)s_have_k);
        else snprintf(l, sizeof(l), "MAP ERROR: %.20s", osm::error_text(s_err));
        text(0, 48, l, C_RED); text(0, 58, "QUICK > REBUILD TO RETRY", C_WHITE);
    }
}

// ---------------------------------------------------------------------------------------------------------------
// NAV (speedometer) page
void nav_dial() {
    using namespace cdraw;
    osm::canvas_clear(s_cv);
    const double sp = s_nav.speed_src == SpeedSrc::NONE ? 0 : s_nav.speed_mps * (s_imperial ? 2.2369363 : 3.6);
    static const double steps[] = {10, 20, 50, 100, 200, 400};
    double mx = steps[5]; for (double st : steps) if (sp <= st * 0.92) { mx = st; break; }
    const float cx = 120, cy = (float)s_cv.h - 4, r = 50;
    for (int d = 0; d <= 180; d += 2) { const float a = (180 - d) * 0.0174533f; put(s_cv, (int)(cx + r * cosf(a)), (int)(cy - r * sinf(a)), K_TRACE); }
    for (int i = 0; i <= 10; ++i) {
        const float a = (180 - i * 18) * 0.0174533f, r0 = i % 5 == 0 ? r - 9 : r - 5;
        line(s_cv, (int)(cx + r0 * cosf(a)), (int)(cy - r0 * sinf(a)), (int)(cx + r * cosf(a)), (int)(cy - r * sinf(a)), K_UI);
    }
    char num[8]; snprintf(num, sizeof(num), "%.0f", sp); const int len = (int)strlen(num), w = len * 16;
    for (int i = 0; i < len; ++i) glyph(s_cv, font::glyphs[(uint8_t)num[i]], (int)cx - w / 2 + i * 16, (int)cy - 30, 2, K_UI);
    double frac = sp / mx; if (frac > 1) frac = 1; if (frac < 0) frac = 0;
    const float na = (float)((1.0 - frac) * 3.14159265);
    line(s_cv, (int)cx, (int)cy, (int)(cx + (r - 4) * cosf(na)), (int)(cy - (r - 4) * sinf(na)), K_RID_LOST);
    diamond(s_cv, (int)cx, (int)cy, 2, K_RID_LOST, true);
    // scale end labels (0 and max) as tiny glyphs under the arc ends
    char mxs[8]; snprintf(mxs, sizeof(mxs), "%.0f", mx);
    glyph(s_cv, font::glyphs[(uint8_t)'0'], 66, s_cv.h - 9, 1, K_UI);
    for (size_t i = 0; i < strlen(mxs); ++i) glyph(s_cv, font::glyphs[(uint8_t)mxs[i]], 170 + (int)i * 8, s_cv.h - 9, 1, K_UI);
    blit_canvas(36);
}
void draw_nav() {
    nav_snapshot(&s_nav);
    const NavSnapshot &n = s_nav;
    if (s_full_clear) { display_fill_rect(0, 32, 240, 264, C_BLACK); s_full_clear = false; }
    if (s_cvmem) nav_dial();
    char l[96], a[24], b[24];
    if (n.speed_src != SpeedSrc::NONE) { fmt_speed(a, sizeof(a), n.speed_mps); snprintf(l, sizeof(l), "SPD %s  [%s]", a, n.speed_src == SpeedSrc::GNSS ? "GNSS" : "DERIVED"); }
    else snprintf(l, sizeof(l), "SPD ---  %s", n.stale ? "(FIX LOST)" : "(NO SPEED)");
    row(6, l, C_CYAN);
    if (n.heading_src != HeadingSrc::NONE) {
        snprintf(l, sizeof(l), "HDG %03.0f %-3s [%s%s]", n.heading_deg, compass(n.heading_deg), n.heading_src == HeadingSrc::GNSS ? "GNSS" : n.heading_src == HeadingSrc::TRACK ? "TRACK EST" : "HELD",
                 n.heading_src == HeadingSrc::HELD ? " OLD" : "");
    } else snprintf(l, sizeof(l), "HDG --- (NEEDS MOVEMENT)");
    row(7, l, n.heading_src == HeadingSrc::HELD ? C_AMBER : C_CYAN);
    if (n.have_position) {
        fmt_lat(a, sizeof(a), n.lat); fmt_lon(b, sizeof(b), n.lon);
        snprintf(l, sizeof(l), "LAT %s", a); row(8, l, C_WHITE);
        snprintf(l, sizeof(l), "LON %s", b); row(9, l, C_WHITE);
    } else { row(8, "LAT ---", C_AMBER); row(9, "LON ---", C_AMBER); }
    if (n.fix.has_altitude && !n.stale) { char d[16]; fmt_dist(d, sizeof(d), n.fix.altitude_m); snprintf(l, sizeof(l), "ALT %s (MSL)", d); } else snprintf(l, sizeof(l), "ALT ---");
    row(10, l, C_WHITE);
    const char *ft = n.fix.fix_type == 3 ? "3D" : n.fix.fix_type == 2 ? "2D" : n.fix.fix_type == 1 ? "NONE" : "?";
    snprintf(l, sizeof(l), "FIX %s  SAT %u  Q%u", ft, n.fix.satellites, n.fix.fix_quality); row(11, l, n.stale ? C_AMBER : C_GREEN);
    if (n.fix.has_dop) snprintf(l, sizeof(l), "H%.1f P%.1f V%.1f DOP", n.fix.hdop, n.fix.pdop, n.fix.vdop);
    else if (n.fix.has_hdop) snprintf(l, sizeof(l), "HDOP %.1f (PDOP/VDOP N/A)", n.fix.hdop); else snprintf(l, sizeof(l), "DOP ---");
    row(12, l, C_WHITE);
    if (n.acc_src != AccSrc::NONE) { char d[16]; fmt_dist(d, sizeof(d), n.acc_m); snprintf(l, sizeof(l), "ACC +-%s [%s]", d, n.acc_src == AccSrc::GST ? "GST" : "HDOPx5 EST"); } else snprintf(l, sizeof(l), "ACC ---");
    row(13, l, C_WHITE);
    snprintf(l, sizeof(l), "UTC %.23s", n.fix.utc[0] ? n.fix.utc : "---"); row(14, l, C_WHITE);
    if (n.fix_age_ms == UINT32_MAX) snprintf(l, sizeof(l), "FIX AGE ---  NMEA %lu/s", (unsigned long)n.fix.update_hz);
    else snprintf(l, sizeof(l), "FIX AGE %.1fs  NMEA %lu/s", n.fix_age_ms / 1000.0, (unsigned long)n.fix.update_hz);
    row(15, l, n.stale ? C_AMBER : C_WHITE);
    fmt_dist(a, sizeof(a), n.trip_dist_m);
    snprintf(l, sizeof(l), "TRIP %s  %02lu:%02lu:%02lu", a, (unsigned long)(n.trip_elapsed_s / 3600), (unsigned long)(n.trip_elapsed_s / 60 % 60), (unsigned long)(n.trip_elapsed_s % 60)); row(16, l, C_WHITE);
    const double avg = n.trip_moving_s ? n.trip_dist_m / n.trip_moving_s : 0; char mxs[20];
    fmt_speed(a, sizeof(a), avg); fmt_speed(mxs, sizeof(mxs), n.max_speed_mps);
    snprintf(l, sizeof(l), "AVG %s MAX %s", a, mxs); row(17, l, C_WHITE);
    snprintf(l, sizeof(l), "TRACE %s %u PTS", n.trace_enabled ? "ON" : "OFF", (unsigned)n.trace_count); row(18, l, n.trace_enabled ? C_GREEN : C_WHITE);
    row(19, n.stale ? "NO CURRENT FIX - LAST KNOWN SHOWN" : n.have_position ? "FIX CURRENT" : "WAITING FOR GPS FIX", n.stale || !n.have_position ? C_AMBER : C_GREEN);
    row(20, s_msg, C_AMBER);
    row(21, "(SPEED/HEADING FROM RECEIVER OR", C_WHITE); row(22, "GPS TRACK; NOT RF MEASUREMENTS)", C_WHITE);
}

// ---------------------------------------------------------------------------------------------------------------
// Remote ID page
void draw_rid() {
    RidStats rs{}; rid_stats(&rs);
    RidAircraft list[kRidCapacity]; const size_t n = rid_snapshot(list, kRidCapacity);
    nav_snapshot(&s_nav);
    if (s_full_clear) { display_fill_rect(0, 32, 240, 264, C_BLACK); s_full_clear = false; }
    char l[96];
    if (rs.running) snprintf(l, sizeof(l), "SCAN ON CH%u  OK %lu BAD %lu", rs.channel, (unsigned long)rs.frames_decoded, (unsigned long)rs.frames_malformed);
    else snprintf(l, sizeof(l), "SCAN OFF  (SDR RX RUNS)");
    row(0, l, rs.running ? C_GREEN : C_AMBER);
    snprintf(l, sizeof(l), "WIFI BEACON+NAN  %s", rs.ble_available ? "BLE LEGACY ON" : "BLE OFF"); row(1, l, C_WHITE);
    if (s_rid_sel >= n) s_rid_sel = n ? n - 1 : 0;
    for (size_t i = 0; i < 8; ++i) {
        if (i >= n) { row(3 + (int)i, i == 0 ? (rs.running ? "NO AIRCRAFT RECEIVED YET" : "CENTER = START PASSIVE SCAN") : "", C_AMBER); continue; }
        const auto &a = list[i];
        char id[14]; if (a.has_id) snprintf(id, sizeof(id), "%.12s", a.id); else snprintf(id, sizeof(id), "%02X%02X%02X%02X%02X%02X", a.mac[0], a.mac[1], a.mac[2], a.mac[3], a.mac[4], a.mac[5]);
        snprintf(l, sizeof(l), "%c %-12s %4d %4lus", a.state == RidState::ACTIVE ? 'A' : a.state == RidState::STALE ? 'S' : 'L', id, a.rssi, (unsigned long)(a.age_ms / 1000));
        const uint16_t fg = a.state == RidState::ACTIVE ? C_GREEN : a.state == RidState::STALE ? C_AMBER : C_RED;
        row(3 + (int)i, l, i == s_rid_sel ? C_WHITE : fg, i == s_rid_sel ? C_SEL : C_BLACK);
    }
    if (n) {
        const auto &a = list[s_rid_sel];
        snprintf(l, sizeof(l), "ID %s", a.has_id ? a.id : "(NONE)"); row(12, l, C_CYAN);
        snprintf(l, sizeof(l), "OPERATOR ID %s", a.has_operator_id ? a.operator_id : "(NONE)"); row(13, l, C_WHITE);
        if (a.has_position) { snprintf(l, sizeof(l), "AIRCRAFT %.5f %.5f", a.lat, a.lon); row(14, l, C_WHITE); } else row(14, "AIRCRAFT POSITION: NONE", C_AMBER);
        char alt[24] = "", hgt[24] = "";
        if (a.has_alt) snprintf(alt, sizeof(alt), "ALT %.0fm", a.alt_geo_m);
        if (a.has_height) snprintf(hgt, sizeof(hgt), "%s %.0fm", a.height_is_agl ? "AGL" : "ABV TO", a.height_m);
        snprintf(l, sizeof(l), "%s %s", alt, hgt); row(15, l, C_WHITE);
        char sp[16] = "", hd[16] = "", vs[16] = "";
        if (a.has_speed) fmt_speed(sp, sizeof(sp), a.speed_mps);
        if (a.has_heading) snprintf(hd, sizeof(hd), "%u%c", a.heading_deg, 0xF8);
        if (a.has_vspeed) snprintf(vs, sizeof(vs), "VS %.1f", a.vspeed_mps);
        snprintf(l, sizeof(l), "%s %s %s", sp, hd, vs); row(16, l, C_WHITE);
        if (a.h_acc_m >= 0) snprintf(l, sizeof(l), "REPORTED ACC H<%.0fm V<%.0fm", a.h_acc_m, a.v_acc_m >= 0 ? a.v_acc_m : 0.0f); else snprintf(l, sizeof(l), "REPORTED ACC: UNKNOWN");
        row(17, l, C_WHITE);
        snprintf(l, sizeof(l), "%s POS AGE %s", a.state == RidState::ACTIVE ? "ACTIVE" : a.state == RidState::STALE ? "STALE" : "LOST",
                 a.has_position ? "" : "N/A"); if (a.has_position) snprintf(l + strlen(l), sizeof(l) - strlen(l), "%lus", (unsigned long)(a.position_age_ms / 1000));
        row(18, l, a.state == RidState::ACTIVE ? C_GREEN : a.state == RidState::STALE ? C_AMBER : C_RED);
        if (a.has_position && s_nav.have_position && geo::coords_valid(a.lat, a.lon)) {
            char d[16]; fmt_dist(d, sizeof(d), geo::distance_m(s_nav.lat, s_nav.lon, a.lat, a.lon));
            snprintf(l, sizeof(l), "FROM RX GPS: %s @%03.0f%s", d, geo::bearing_deg(s_nav.lat, s_nav.lon, a.lat, a.lon), s_nav.stale ? " (OLD)" : "");
            row(19, l, C_WHITE);
        } else row(19, s_nav.have_position ? "RX->AIRCRAFT: NO POSITION" : "RX GPS NO FIX: NO RANGE", C_AMBER);
        snprintf(l, sizeof(l), "VIA %s%s%s%s", a.transports & 1 ? "BCN " : "", a.transports & 2 ? "NAN " : "", a.transports & 4 ? "BLE " : "", a.transports & 8 ? "BLE5" : "");
        row(20, l, C_WHITE);
    } else for (int r = 12; r <= 20; ++r) row(r, "", C_WHITE);
    row(21, s_rid_msg, C_AMBER);
    row(22, "PASSIVE RX ONLY. POSITIONS ARE", C_WHITE); row(23, "AIRCRAFT-REPORTED, NOT RX GPS.", C_WHITE);
}

// ---------------------------------------------------------------------------------------------------------------
// Remote ID control
void rid_set_scan(bool on) {
    if (on && !rid_running()) {
        if (s_pause_hook) s_pause_hook(true);
        const esp_err_t e = rid_start();
        if (e != ESP_OK) { if (s_pause_hook) s_pause_hook(false); snprintf(s_rid_msg, sizeof(s_rid_msg), "SCAN START: %s", esp_err_to_name(e)); set_msg(s_rid_msg); return; }
        snprintf(s_rid_msg, sizeof(s_rid_msg), "SCANNING (SDR PAUSED)"); set_msg("RID SCAN ON - SDR PAUSED");
    } else if (!on && rid_running()) {
        rid_stop(); if (s_pause_hook) s_pause_hook(false);
        snprintf(s_rid_msg, sizeof(s_rid_msg), "SCAN STOPPED - SDR RESUMED"); set_msg("RID SCAN OFF");
    }
}
}  // namespace

// ===============================================================================================================
void map_ui_set_radio_hook(void (*h)(bool)) { s_pause_hook = h; }

void map_ui_open(MapScreen screen, bool imperial) {
    if (s_open) map_ui_close();
    s_open = true; s_screen = screen; s_imperial = imperial; s_mode = Mode::VIEW; s_msg[0] = 0; s_rid_msg[0] = 0;
    s_full_clear = true; s_quick_drawn = false; s_text_dirty = true; s_view_dirty = true; s_band = kBands; s_have_centre = false;
    s_cvmem = (uint8_t *)malloc(osm::canvas_bytes(240, kBandH));
    if (s_cvmem) s_cv = {s_cvmem, 240, kBandH};
    nav_snapshot(&s_nav);
    if (screen == MapScreen::MAP) {
        detect_map();
        if (s_ms == MapState::READY) { s_rend = new (std::nothrow) osm::Renderer(); }
        if (!s_cvmem) set_msg("NO MEMORY FOR MAP CANVAS");
        range_import_csv(&s_io, kRangeCsv, range_store(), now_ms(), nullptr, nullptr);   // silently ignored when the file is absent
    }
    s_rid_sel = 0;
}
void map_ui_close() {
    if (!s_open) return;
    if (rid_running()) { rid_stop(); if (s_pause_hook) s_pause_hook(false); }
    free_map_objects();
    s_open = false; s_band = kBands;
}
const char *map_ui_title() { return s_screen == MapScreen::MAP ? "OFFLINE MAP" : s_screen == MapScreen::NAV ? "NAV / SPEEDOMETER" : "REMOTE ID (PASSIVE)"; }
const char *map_ui_footer() {
    if (s_screen == MapScreen::NAV) return "L UNITS R TRIP C TRACE";
    if (s_screen == MapScreen::RID) return "UP/DN SEL C SCAN R CLEAR";
    if (s_mode == Mode::QUICK) return "UP/DN ROW L/R VALUE C RUN";
    if (s_mode == Mode::PAN) return "ARROWS PAN  C DONE";
    return "UD ZOOM L FOLLOW R PAN C MENU";
}

bool map_ui_event(const InputEvent &e, bool move, bool confirm) {
    using K = InputKey;
    const bool left = e.key == K::LEFT && e.kind == InputKind::SHORT_RELEASE;
    const bool right = e.key == K::RIGHT && e.kind == InputKind::PRESS;
    const bool up = move && e.key == K::UP, down = move && e.key == K::DOWN;
    s_msg[0] = 0;
    if (s_screen == MapScreen::NAV) {
        if (left) { s_imperial = !s_imperial; }
        else if (right) { nav_trip_reset(); set_msg("TRIP RESET"); }
        else if (confirm) { nav_trace_enable(!nav_trace_enabled()); set_msg(nav_trace_enabled() ? "TRACE ON" : "TRACE OFF"); }
        return false;
    }
    if (s_screen == MapScreen::RID) {
        if (up && s_rid_sel > 0) --s_rid_sel;
        else if (down) ++s_rid_sel;
        else if (confirm) rid_set_scan(!rid_running());
        else if (right) { rid_clear(); snprintf(s_rid_msg, sizeof(s_rid_msg), "LIST CLEARED"); s_rid_sel = 0; }
        else if (left) return true;
        return false;
    }
    // ---- MAP ----
    if (s_mode == Mode::VIEW) {
        if (up && s_zoom < geo::kZoomCount - 1) { ++s_zoom; s_view_dirty = true; }
        else if (down && s_zoom > 0) { --s_zoom; s_view_dirty = true; }
        else if (left) { s_follow = true; s_view_dirty = true; set_msg(s_nav.have_position ? "FOLLOWING GPS" : "NO GPS FIX TO FOLLOW"); }
        else if (right) { s_mode = Mode::PAN; s_follow = false; geo::View v; desired_view(&v); s_clat = v.lat; s_clon = v.lon; s_have_centre = true; s_view_dirty = true; }
        else if (confirm) { s_mode = Mode::QUICK; s_quick_drawn = false; }
        // note: zoom direction: UP = closer (larger index = smaller metres per pixel)
    } else if (s_mode == Mode::PAN) {
        const int step = 24 * (1 + (e.repeat_count > 24 ? 3 : e.repeat_count > 10 ? 1 : 0));
        double dx = 0, dy = 0;
        if (move && e.key == K::UP) dy = step; else if (move && e.key == K::DOWN) dy = -step;
        else if (move && e.key == K::RIGHT) dx = step; else if (left || (move && e.key == K::LEFT)) dx = -step;
        if (dx || dy) {
            geo::View v; desired_view(&v); v.lat = s_clat; v.lon = s_clon; geo::view_set(v, s_clat, s_clon, mpp());
            geo::pan_px(v, dx, dy);
            if (geo::coords_valid(v.lat, v.lon)) { s_clat = v.lat; s_clon = v.lon; }
            s_view_dirty = true;
        } else if (confirm) { s_mode = Mode::VIEW; s_view_dirty = true; }
    } else {   // QUICK
        auto change = [&](int d) {
            switch (s_quick) {
                case 0: s_zoom = s_zoom + d < 0 ? 0 : s_zoom + d >= geo::kZoomCount ? geo::kZoomCount - 1 : s_zoom + d; break;
                case 1: s_follow = !s_follow; if (!s_follow) { geo::View v; desired_view(&v); s_clat = v.lat; s_clon = v.lon; s_have_centre = true; } break;
                case 2: s_north_up = !s_north_up; break;
                case 3: nav_trace_enable(!nav_trace_enabled()); break;
                case 5: s_hud = !s_hud; break;
                case 6: s_show_rid = !s_show_rid; break;
                case 7: rid_set_scan(!rid_running()); break;
                case 8: s_show_rings = !s_show_rings; break;
                case 9: s_layer_preset = (s_layer_preset + d + 5) % 5; break;
                case 10: s_imperial = !s_imperial; break;
                default: return false;
            }
            return true;
        };
        if (up) s_quick = (s_quick + kQuickN - 1) % kQuickN;
        else if (down) s_quick = (s_quick + 1) % kQuickN;
        else if (left || right) { if (change(right ? 1 : -1)) s_view_dirty = true; }
        else if (confirm) {
            switch (s_quick) {
                case 4: nav_trace_clear(); set_msg("TRACE CLEARED"); s_view_dirty = true; break;
                case 11: nav_trip_reset(); set_msg("TRIP RESET"); break;
                case 12:
                    if (s_ms == MapState::INDEXING) break;
                    if (!storage_accessible()) { set_msg("SD NOT MOUNTED"); break; }
                    if (!s_io.size(kSrc)) { set_msg("maps/regional_map.osm MISSING"); break; }
                    if (s_rend) s_rend->cancel();
                    start_index(); s_mode = Mode::VIEW; s_view_dirty = true; s_full_clear = true; break;
                case 13: {
                    size_t rej = 0; RangeStatus last{};
                    const size_t ok = range_import_csv(&s_io, kRangeCsv, range_store(), now_ms(), &rej, &last);
                    char m[40]; snprintf(m, sizeof(m), "RANGE CSV: %u OK %u REJECTED", (unsigned)ok, (unsigned)rej); set_msg(m); s_view_dirty = true; break; }
                case 14: s_mode = Mode::VIEW; s_view_dirty = true; s_quick_drawn = false; break;
                default: if (change(1)) s_view_dirty = true; break;
            }
        }
    }
    return false;
}

void map_ui_tick() {
    if (!s_open) return;
    if (s_screen == MapScreen::RID) { if (rid_running()) rid_tick(); return; }
    if (s_screen != MapScreen::MAP) return;
    if (rid_running()) rid_tick();
    if (s_mode == Mode::QUICK) return;                 // the settings list replaces the map; no rendering while it is up
    map_tick();
}

void map_ui_draw(bool screen_changed) {
    if (!s_open) return;
    if (screen_changed) { s_full_clear = true; s_quick_drawn = false; s_text_dirty = true; s_view_dirty = true; }
    switch (s_screen) {
        case MapScreen::NAV: draw_nav(); break;
        case MapScreen::RID: draw_rid(); break;
        case MapScreen::MAP:
            if (s_mode == Mode::QUICK) { draw_quick(); break; }
            if (s_quick_drawn || s_full_clear) { display_fill_rect(0, 32, 240, 264, C_BLACK); s_quick_drawn = false; s_full_clear = false; s_view_dirty = true; s_band = kBands; }
            status_bar();
            if (s_band >= kBands) { if (s_ms != MapState::READY || !s_have_centre) draw_map_status_panel(); hud_lines(); }
            if (s_ms == MapState::INDEXING) draw_map_status_panel();
            if (s_msg[0]) text(0, 259, s_msg, C_AMBER);
            if (s_band >= kBands && s_ms == MapState::READY) {                 // scale label (the bar itself is in the canvas)
                int px; const double m = geo::scale_bar_m(mpp(), 80, &px); char d[16]; fmt_dist(d, sizeof(d), m); text(12 + px, 258, d, C_WHITE, C_BLACK);
            }
            break;
    }
}
}  // namespace marauder
