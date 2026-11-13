#pragma once
// Bounds-safe drawing into the 4-bpp osm::Canvas (two pixels per byte, even x = low nibble). Used for map overlays,
// the speedometer dial and scale bar. Every primitive clips; nothing here allocates.
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include "osm.hpp"

namespace marauder::cdraw {

// Palette classes above the OSM classes (osm::C_COUNT == 9).
enum : uint8_t { K_TRACE = 9, K_RX = 10, K_RID_ACTIVE = 11, K_RID_STALE = 12, K_RID_LOST = 13, K_RING = 14, K_UI = 15 };

inline void put(osm::Canvas &c, int x, int y, uint8_t k) {
    if (x < 0 || y < 0 || x >= c.w || y >= c.h) return;
    uint8_t &b = c.px[((size_t)y * c.w + x) >> 1];
    b = (x & 1) ? (uint8_t)((b & 0x0F) | (k << 4)) : (uint8_t)((b & 0xF0) | (k & 15));
}
inline void line(osm::Canvas &c, int x0, int y0, int x1, int y1, uint8_t k) {
    // reject lines entirely outside by a wide margin (also protects against huge coordinates)
    const int lim = 4096;
    if (abs(x0) > lim || abs(y0) > lim || abs(x1) > lim || abs(y1) > lim) return;
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1, dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1, err = dx + dy;
    for (int guard = 0; guard < 16384; ++guard) {
        put(c, x0, y0, k);
        if (x0 == x1 && y0 == y1) break;
        const int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}
inline void rect_fill(osm::Canvas &c, int x0, int y0, int x1, int y1, uint8_t k) {
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 >= c.w) x1 = c.w - 1;
    if (y1 >= c.h) y1 = c.h - 1;
    for (int y = y0; y <= y1; ++y) for (int x = x0; x <= x1; ++x) put(c, x, y, k);
}
inline void circle(osm::Canvas &c, float cx, float cy, float r, uint8_t k, int dash = 0) {
    if (!(r >= 0.5f) || !isfinite(r) || !isfinite(cx) || !isfinite(cy)) return;
    if (r > 3000.0f) return;
    if (cx + r < 0 || cy + r < 0 || cx - r >= c.w || cy - r >= c.h) return;       // fully off the canvas
    int steps = (int)(r * 6.2832f * 1.5f); if (steps < 12) steps = 12; if (steps > 720) steps = 720;
    for (int i = 0; i < steps; ++i) {
        if (dash && ((i * 2 / dash) & 1)) continue;
        const float a = 6.2831853f * (float)i / (float)steps;
        put(c, (int)lroundf(cx + r * cosf(a)), (int)lroundf(cy + r * sinf(a)), k);
    }
}
inline void diamond(osm::Canvas &c, int x, int y, int r, uint8_t k, bool fill) {
    for (int dy = -r; dy <= r; ++dy) {
        const int w = r - abs(dy);
        if (fill) for (int dx = -w; dx <= w; ++dx) put(c, x + dx, y + dy, k);
        else { put(c, x - w, y + dy, k); put(c, x + w, y + dy, k); }
    }
}
inline void cross(osm::Canvas &c, int x, int y, int r, uint8_t k) { line(c, x - r, y - r, x + r, y + r, k); line(c, x - r, y + r, x + r, y - r, k); }
// Triangle arrow whose tip points at `bearing` degrees clockwise from screen-up.
inline void arrow(osm::Canvas &c, float x, float y, float bearing_deg, int len, uint8_t k) {
    const float a = bearing_deg * 0.01745329f;
    const float tx = x + sinf(a) * len, ty = y - cosf(a) * len;
    const float lx = x + sinf(a + 2.5f) * len * 0.8f, ly = y - cosf(a + 2.5f) * len * 0.8f;
    const float rx = x + sinf(a - 2.5f) * len * 0.8f, ry = y - cosf(a - 2.5f) * len * 0.8f;
    line(c, (int)tx, (int)ty, (int)lx, (int)ly, k); line(c, (int)tx, (int)ty, (int)rx, (int)ry, k);
    line(c, (int)lx, (int)ly, (int)x, (int)y, k);   line(c, (int)rx, (int)ry, (int)x, (int)y, k);
}
// 8x8 glyph (bit 7 = leftmost) scaled by an integer factor.
inline void glyph(osm::Canvas &c, const uint8_t g[8], int x, int y, int scale, uint8_t k) {
    for (int r = 0; r < 8; ++r) for (int b = 0; b < 8; ++b)
        if (g[r] & (0x80 >> b)) rect_fill(c, x + b * scale, y + r * scale, x + b * scale + scale - 1, y + r * scale + scale - 1, k);
}
}
