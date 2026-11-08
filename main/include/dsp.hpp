#pragma once
#include <stdint.h>

namespace marauder {
struct SpectrumFrame { uint8_t bins[240]; uint8_t peaks[240]; uint32_t center_mhz; };
// 256 complex int8 IQ points to a centered 240-pixel magnitude trace.
void fft256_fixed(const int8_t *iq, uint8_t bins[240], uint8_t peaks[240]);
void fft256_fixed_windowed(const int8_t *iq, uint8_t bins[240], uint8_t peaks[240], bool hann_window);
uint16_t heatmap_rgb565(uint8_t level);
uint16_t waterfall_rgb565(uint8_t level, uint8_t floor, uint8_t ceiling, uint8_t palette);
}
