#pragma once
#include <stdint.h>
#include "esp_err.h"

namespace marauder {
inline constexpr int screen_width = 240;
inline constexpr int screen_height = 320;
esp_err_t display_init();
esp_err_t display_bus_init();
void display_text(int x, int y, const char *text, uint16_t fg, uint16_t bg);
void display_clear(uint16_t color);
void display_fill_rect(int x, int y, int width, int height, uint16_t color);
void display_rgb565_image(int x,int y,int width,int height,const uint16_t *pixels);
void display_waterfall_line(const uint16_t *colors);
void display_reset_waterfall();
void display_spectrum(const uint8_t *bins, const uint8_t *peaks, uint16_t color);
// floor_marks (amber dots) and min_marks (green line) are optional overlays in the same 0..255 units as bins.
void display_spectrum_region(const uint8_t *bins, const uint8_t *peaks, uint16_t color,
                             int top, int height, const uint8_t *floor_marks = nullptr, const uint8_t *min_marks = nullptr);
}
