#pragma once
// Advanced passive-receive DSP (receive / offline analysis only; nothing here transmits).
//
// Data flow per snapshot (all stages are independently switchable; with every option at its default
// the output is bit-identical to the original pipeline):
//
//   256 IQ8 samples --(IQ stats, optional DC / balance correction, window)--> fixed-point FFT --> 240 bins
//   240 bins --> [A] spectrum stage: DC suppress, adjacent-bin rejection, frequency mask / notches,
//                    transient / impulse / outlier rejection, noise-floor tracking + subtraction,
//                    temporal filter (EMA / adaptive / median), signal detection + tracking
//            --> original smoothing / DC fill / averaging (unchanged)
//            --> [B] display stage: trace path and waterfall path processed independently
//
// Bin values are LINEAR FFT MAGNITUDES (|re|+|im|, 0..255). They are RELATIVE measurements. They are not
// calibrated RF power (no gain, antenna or ADC calibration exists on this board) and dB values derived
// from them are ratios between bins/frames, never dBm.
#include <stdint.h>
#include <stddef.h>
#include "dsp.hpp"

namespace marauder {

enum : uint8_t { WIN_RECT, WIN_HANN, WIN_HAMMING, WIN_BLACKMAN, WIN_BLACKMAN_HARRIS, WIN_FLATTOP, WIN_BARTLETT,
                 WIN_TRIANGULAR, WIN_LEGACY, WIN_COUNT };

// Every field is a uint8_t so settings can be described by one table (dsp_cfg.cpp) used for the menu,
// validation, defaults and persistence.
struct DspConfig {
    // FFT / WINDOW
    uint8_t window, iq_dc, iq_balance, iq_normalize;
    // FREQUENCY FILTERS
    uint8_t f_mode, f_center, f_width, f_roll, f_stop, notch1, notch2, notch3, notch_width, dc_suppress, adj_reject;
    // NOISE REDUCTION
    uint8_t nr_strength, nr_rate, nr_guard, nr_impulse, nr_transient, nr_outlier;
    // SIGNAL DETECTION
    uint8_t d_on, d_type, d_guard, d_train, d_sens, d_prom, d_drop, d_persist;
    // TEMPORAL PROCESSING
    uint8_t t_mode, t_ema, t_adapt;
    // DISPLAY PROCESSING
    uint8_t v_smooth, v_decay, v_minhold, v_floor, v_range, v_span, w_suppress, w_interp, w_contrast, w_smooth;
};

// ---- configuration store (dsp_cfg.cpp) ----------------------------------------------------------
enum DspGroup : uint8_t { DG_FFT, DG_FILTER, DG_NOISE, DG_DETECT, DG_TEMPORAL, DG_DISPLAY, DG_COUNT };
DspConfig dsp_cfg_defaults();
void dsp_cfg_sanitize(DspConfig &c);                 // clamps every field to its legal range
DspConfig dsp_cfg_snapshot();                        // thread-safe copy
void dsp_cfg_load();                                 // NVS -> active (defaults if absent / invalid)
bool dsp_cfg_save_if_dirty();                        // NVS write, only if something changed
size_t dsp_item_count(unsigned group);
const char *dsp_group_name(unsigned group);
const char *dsp_item_name(unsigned group, size_t index);
const char *dsp_item_desc(unsigned group, size_t index);
void dsp_item_value(unsigned group, size_t index, char *out, size_t n);
void dsp_item_adjust(unsigned group, size_t index, int direction, unsigned magnitude = 1);
void dsp_item_reset(unsigned group, size_t index);
unsigned dsp_group_modified(unsigned group);         // number of items not at their default
// Window facts computed offline from the exact coefficients (see README): index = WIN_*.
struct DspWindowInfo { const char *name; int16_t sidelobe_db; uint16_t enbw_x100; uint16_t scallop_x100; uint16_t gain_q8; };
const DspWindowInfo &dsp_window_info(uint8_t id);

// ---- processing (dsp_adv.cpp) -------------------------------------------------------------------
struct DspIqStats {
    int16_t dc_i, dc_q;        // mean of I and Q over the snapshot, in ADC counts
    int16_t gain_x100_db;      // 10*log10(P_I/P_Q) * 100, after DC removal (0 = balanced)
    int16_t phase_x10_deg;     // estimated quadrature error * 10 (small-angle estimate)
    uint8_t peak;              // max |I|,|Q| after correction
    uint8_t norm_shift;        // output scale shift applied by IQ NORMALIZE
    bool valid;
};
struct DspSignal {
    bool active;
    int32_t pos_q4;            // tracked peak position, FFT-bin index * 16 (source coordinates)
    int32_t drift_q4;          // pos minus first-seen position, bins * 16
    uint8_t level;             // relative FFT magnitude
    uint8_t prominence;
    uint16_t width_q4;         // half-prominence width, bins * 16
    int16_t snr_x10;           // 20*log10(level / local CFAR noise) * 10, RELATIVE
    uint8_t persistence;       // 0..255 fraction of recent frames in which the peak was seen
    uint16_t age;              // frames since first seen
};
constexpr unsigned kDspMaxSignals = 8;
struct DspResults {
    DspSignal sig[kDspMaxSignals];
    uint8_t count;
    uint8_t frame_floor;       // robust median of passband bins (relative magnitude)
    uint8_t frame_peak;
    uint32_t center_mhz;
    uint32_t impulses, transients, outliers;   // events rejected since boot
    DspIqStats iq;
};
struct DspPerf {
    uint32_t fft_us, stage_us, display_us;     // exponential moving averages (microseconds)
    uint32_t total_max_us;                     // worst fft+stage+display time seen
    uint32_t frames;
};

// Builds bins/peaks for one snapshot. Falls through to the original FFT when nothing is enabled.
void dsp_fft_frame(const int8_t *iq, uint8_t bins[240], uint8_t peaks[240], bool legacy_hann, const DspConfig &c);
void dsp_stage_spectrum(uint8_t bins[240], const DspConfig &c, uint32_t center_mhz);
void dsp_stage_display(const uint8_t avg[240], const uint8_t peaks[240], bool legacy_max_hold,
                       const DspConfig &c, SpectrumFrame &out);
const int16_t *dsp_window_table(uint8_t id); // Q15 coefficients actually used (id != WIN_RECT/LEGACY)
void dsp_reset_state();
void dsp_results_snapshot(DspResults *out);
void dsp_perf_snapshot(DspPerf *out);
void dsp_perf_note(uint32_t fft_us, uint32_t stage_us, uint32_t display_us);
// Frequency of a source bin (position * 16), in kHz relative to the tuned centre, matching the on-screen axis.
int32_t dsp_pos_to_khz_offset(int32_t pos_q4);
// 20*log10(num/den)*10 for amplitude ratios (integer, table based). den==0 -> 0.
int dsp_ratio_db_x10(uint32_t num, uint32_t den);
}
