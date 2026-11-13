#include "../../main/include/canvas_draw.hpp"
#include <assert.h>
#include <stdio.h>
#include <vector>
using namespace marauder;
int main() {
    std::vector<uint8_t> mem(osm::canvas_bytes(40, 30) + 16, 0xEE);          // guard bytes past the end must stay intact
    osm::Canvas c{mem.data(), 40, 30}; osm::canvas_clear(c);
    for (int y = 0; y < 30; ++y) for (int x = 0; x < 40; ++x) assert(osm::canvas_get(c, x, y) == 0);
    cdraw::put(c, 3, 4, 9); cdraw::put(c, 4, 4, 12); assert(osm::canvas_get(c, 3, 4) == 9 && osm::canvas_get(c, 4, 4) == 12);
    cdraw::put(c, -1, 0, 5); cdraw::put(c, 40, 0, 5); cdraw::put(c, 0, 30, 5);
    cdraw::line(c, 0, 0, 39, 29, 10); assert(osm::canvas_get(c, 0, 0) == 10 && osm::canvas_get(c, 39, 29) == 10);
    cdraw::line(c, -500, -500, 900, 700, 11); cdraw::line(c, 0, 0, 1000000, 5, 11);            // clipped / rejected, no crash
    cdraw::circle(c, 20, 15, 10, 14); assert(osm::canvas_get(c, 30, 15) == 14);
    cdraw::circle(c, 20, 15, 1e9f, 14); cdraw::circle(c, NAN, 0, 5, 14); cdraw::circle(c, -100, -100, 10, 14);
    cdraw::diamond(c, 5, 25, 3, 11, true); assert(osm::canvas_get(c, 5, 25) == 11);
    cdraw::cross(c, 35, 5, 3, 13); cdraw::arrow(c, 20, 15, 45, 8, 10);
    uint8_t g[8] = {0x80, 0, 0, 0, 0, 0, 0, 1}; cdraw::glyph(c, g, 30, 20, 2, 15); assert(osm::canvas_get(c, 30, 20) == 15 && osm::canvas_get(c, 31, 21) == 15);
    for (size_t i = osm::canvas_bytes(40, 30); i < mem.size(); ++i) assert(mem[i] == 0xEE);
    puts("ALL PASS");
}
