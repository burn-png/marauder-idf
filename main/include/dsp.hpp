#pragma once
#include <stdint.h>

namespace marauder {
// bins/peaks: trace path. wf: waterfall path (processed independently). mins: min-hold trace, floor: per-bin
// noise floor estimate (both 0 unless enabled). floor_level: robust frame noise floor. All RELATIVE magnitudes.
struct SpectrumFrame { uint8_t bins[240]; uint8_t peaks[240]; uint32_t center_mhz;
                       uint8_t wf[240]; uint8_t mins[240]; uint8_t floor[240]; uint8_t floor_level; uint8_t have_floor; };
// 256 complex int8 IQ points to a centered 240-pixel magnitude trace.
void fft256_fixed(const int8_t *iq, uint8_t bins[240], uint8_t peaks[240]);
void fft256_fixed_windowed(const int8_t *iq, uint8_t bins[240], uint8_t peaks[240], bool hann_window);
// Parameters of the extended FFT front end. window_q15 may be null (rectangular). Output is scaled by
// out_gain_q8/256 (coherent-gain correction so a bin-centred tone reads the same for every window) and
// 2^out_shift (IQ normalise). DC / quadrature correction is applied while the samples are loaded.
struct FftParams { const int16_t *window_q15; uint16_t out_gain_q8; uint8_t out_shift;
                   int16_t dc_i, dc_q; bool balance; int16_t kc_q10; uint16_t g_q8; };
void fft256_fixed_custom(const int8_t *iq, uint8_t bins[240], uint8_t peaks[240], const FftParams &p);
uint16_t heatmap_rgb565(uint8_t level);
uint16_t waterfall_rgb565(uint8_t level, uint8_t floor, uint8_t ceiling, uint8_t palette);
}
