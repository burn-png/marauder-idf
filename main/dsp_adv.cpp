// Advanced passive-receive DSP. See include/dsp_adv.hpp for the pipeline overview and units.
//
// Mathematical assumptions (documented once, here):
//  * A snapshot is 256 complex int8 samples at a nominal 16 MS/s, so FFT bin spacing is 62.5 kHz and the
//    240 kept bins span about 15 MHz. Snapshots are sparse and are NEVER stitched together; every
//    time-domain operation below is across successive snapshot results (frames), not across samples.
//  * Bin values are linear magnitudes (|re|+|im| of the FFT, clipped to 255). dB figures are 20*log10 of the
//    ratio of two such magnitudes. They are relative, not dBm.
//  * Retuning (or any change of centre frequency) resets all temporal state, because frames at different
//    frequencies are unrelated. Looking Glass therefore degrades to single-frame processing, by design.
#include "include/dsp_adv.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
#define LOCK() portENTER_CRITICAL(&s_lock)
#define UNLOCK() portEXIT_CRITICAL(&s_lock)
#else
#define LOCK()
#define UNLOCK()
#endif

namespace marauder {
namespace {
constexpr unsigned N = 240;
constexpr unsigned kMaxTracks = kDspMaxSignals;
constexpr unsigned kMaxCand = 12;

struct Track {
    bool act;
    int32_t pos_q4, first_q4;
    uint8_t level, prom;
    uint16_t width_q4;
    int16_t snr_x10;
    uint8_t persist, miss;
    uint16_t age;
};

struct State {
    // spectrum stage
    uint32_t center; bool have_center;
    uint8_t prev_in[N], prev_out[N]; bool prev_valid;
    uint16_t floor_q8[N]; uint16_t ff_q8; bool floor_valid;
    uint32_t energy_ema; uint8_t energy_n, transient_run;
    uint16_t t_y[N]; bool t_valid; uint8_t t_mode_prev;
    uint8_t hist[5][N]; uint8_t hist_n, hist_pos;
    uint16_t mask[N]; uint32_t mask_key; bool mask_valid;
    Track tr[kMaxTracks];
    // display stage
    uint16_t trace_y[N]; bool trace_valid;
    uint16_t wf_y[N]; bool wf_valid;
    uint16_t peak_y[N]; bool peak_valid;
    uint8_t minhold[N]; bool min_valid;
    // iq
    int32_t kc_q10, g_q8; bool iq_est_valid; uint8_t norm_s, norm_pending, norm_count;
    // outputs
    DspResults res; DspPerf perf;
};
State S;
DspResults g_pub;   // published copy (guarded by the lock)
DspPerf g_perf;
uint8_t g_lg[128];  // log2(1 + m/128) in Q8
bool g_lg_ready;
int16_t g_win[256]; uint8_t g_win_id = 0xFF;

void init_log() {
    if (g_lg_ready) return;
    for (unsigned m = 0; m < 128; ++m) g_lg[m] = (uint8_t)std::min(255.0f, std::floor(std::log2(1.0f + m / 128.0f) * 256.0f + 0.5f));
    g_lg_ready = true;
}
int ilog2_q8(uint32_t x) { // log2(x) in Q8 (x > 0)
    init_log();
    const int e = 31 - __builtin_clz(x);
    const uint32_t m = e >= 7 ? (x >> (e - 7)) & 127u : (x << (7 - e)) & 127u;
    return (e << 8) + g_lg[m];
}
uint32_t isqrt64(uint64_t v) {
    uint64_t r = 0, bit = 1ull << 62;
    while (bit > v) bit >>= 2;
    while (bit) { if (v >= r + bit) { v -= r + bit; r = (r >> 1) + bit; } else r >>= 1; bit >>= 2; }
    return (uint32_t)r;
}

void build_window(uint8_t id) {
    if (g_win_id == id) return;
    const float tau = 6.28318530718f;
    for (unsigned n = 0; n < 256; ++n) {
        const float x = tau * (float)n / 256.0f;
        float w;
        switch (id) {
            case WIN_HANN: w = 0.5f - 0.5f * cosf(x); break;
            case WIN_HAMMING: w = 0.54f - 0.46f * cosf(x); break;
            case WIN_BLACKMAN: w = 0.42f - 0.5f * cosf(x) + 0.08f * cosf(2 * x); break;
            case WIN_BLACKMAN_HARRIS: w = 0.35875f - 0.48829f * cosf(x) + 0.14128f * cosf(2 * x) - 0.01168f * cosf(3 * x); break;
            case WIN_FLATTOP: w = 0.21557895f - 0.41663158f * cosf(x) + 0.277263158f * cosf(2 * x) - 0.083578947f * cosf(3 * x) + 0.006947368f * cosf(4 * x); break;
            case WIN_BARTLETT: w = 1.0f - fabsf(2.0f * (float)n / 256.0f - 1.0f); break;
            case WIN_TRIANGULAR: w = 1.0f - fabsf((float)n - 128.0f) / 129.0f; break;
            default: w = 1.0f; break;
        }
        g_win[n] = (int16_t)lrintf(w * 32767.0f);
    }
    g_win_id = id;
}

void clear_tracks() { memset(S.tr, 0, sizeof(S.tr)); }
void reset_internal() {
    S.prev_valid = false; S.floor_valid = false; S.energy_n = 0; S.transient_run = 0; S.t_valid = false;
    S.hist_n = 0; S.hist_pos = 0; S.trace_valid = S.wf_valid = S.peak_valid = S.min_valid = false;
    clear_tracks();
}

// ---- frequency mask (source-bin coordinates; the UI works in display coordinates d = 239 - s) ----
constexpr uint16_t kStopGain[4] = {0, 64, 128, 181};
uint32_t mask_key_of(const DspConfig &c) {
    uint32_t h = 2166136261u;
    const uint8_t f[] = {c.f_mode, c.f_center, c.f_width, c.f_roll, c.f_stop, c.notch1, c.notch2, c.notch3, c.notch_width};
    for (uint8_t b : f) h = (h ^ b) * 16777619u;
    return h;
}
// gain for a bin at "dist2" (2*|offset| in bins) relative to a region edge at limit2 (full width).
int ramp_gain(int dist2, int limit2, bool inside_is_pass, int roll, int stop) {
    const bool inside = dist2 <= limit2;
    const int in_g = inside_is_pass ? 256 : stop, out_g = inside_is_pass ? stop : 256;
    if (inside) return in_g;
    const int d = (dist2 - limit2 + 1) / 2; // bins outside the edge
    if (roll > 0 && d <= roll) return in_g + (out_g - in_g) * d / (roll + 1);
    return out_g;
}
void build_mask(const DspConfig &c) {
    const int stop = kStopGain[c.f_stop & 3];
    for (unsigned s = 0; s < N; ++s) {
        const int d = 239 - (int)s; // display coordinate, 120 = tuned centre
        int g = 256;
        switch (c.f_mode) {
            case 1: g = ramp_gain(2 * std::abs(d - 120), c.f_width, true, c.f_roll, stop); break;           // low-pass
            case 2: g = ramp_gain(2 * std::abs(d - 120), c.f_width - 1, false, c.f_roll, stop); break;      // high-pass
            case 3: g = ramp_gain(2 * std::abs(d - (int)c.f_center), c.f_width, true, c.f_roll, stop); break; // band-pass
            case 4: g = ramp_gain(2 * std::abs(d - (int)c.f_center), c.f_width, false, c.f_roll, stop); break; // band-stop
            default: break;
        }
        const uint8_t notches[3] = {c.notch1, c.notch2, c.notch3};
        for (uint8_t nv : notches) {
            if (!nv) continue;
            const int ng = ramp_gain(2 * std::abs(d - ((int)nv - 1)), c.notch_width, false, 0, stop);
            g = g * ng / 256;
        }
        S.mask[s] = (uint16_t)g;
    }
    S.mask_key = mask_key_of(c);
    S.mask_valid = true;
}

inline uint8_t u8(int v) { return (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : v); }

void sort_small(uint8_t *v, unsigned n) {
    for (unsigned i = 1; i < n; ++i) { const uint8_t x = v[i]; unsigned j = i; while (j && v[j - 1] > x) { v[j] = v[j - 1]; --j; } v[j] = x; }
}

constexpr uint8_t kSensQ4[8] = {24, 32, 40, 48, 64, 96, 128, 192};
constexpr uint8_t kDropFrames[4] = {4, 8, 16, 32};
constexpr uint8_t kPersistShift[4] = {2, 3, 4, 5};

struct Cand { int pos_q4; uint8_t level, prom; uint16_t width_q4; int16_t snr_x10; };

void detect(const uint8_t *b, const DspConfig &c, DspResults &r) {
    Cand cand[kMaxCand];
    unsigned nc = 0;
    if (c.d_on) {
        const int g = c.d_guard, n = c.d_train;
        const unsigned T = kSensQ4[c.d_sens & 7];
        for (int i = 1; i < (int)N - 1; ++i) {
            if (!(b[i] > b[i - 1] && b[i] >= b[i + 1] && b[i] >= 3)) continue;
            uint8_t ref[40];
            unsigned cl = 0, cr = 0, sl = 0, sr = 0, nref = 0;
            for (int j = i - g - 1; j >= 0 && j >= i - g - n; --j) { sl += b[j]; ++cl; if (nref < 40) ref[nref++] = b[j]; }
            for (int j = i + g + 1; j < (int)N && j <= i + g + n; ++j) { sr += b[j]; ++cr; if (nref < 40) ref[nref++] = b[j]; }
            if (cl + cr < 4) continue;
            unsigned noise_q4;
            if (c.d_type == 0) noise_q4 = (16u * (sl + sr)) / (cl + cr);
            else if (c.d_type == 1) {
                const unsigned al = cl ? (16u * sl) / cl : 0, ar = cr ? (16u * sr) / cr : 0;
                noise_q4 = cl && cr ? std::max(al, ar) : (cl ? al : ar);
            } else {
                sort_small(ref, nref);
                noise_q4 = 16u * ref[(nref * 3) / 4 >= nref ? nref - 1 : (nref * 3) / 4];
            }
            if (noise_q4 < 16) noise_q4 = 16; // at least one level: never divide by an empty floor
            if (((unsigned)b[i] << 8) <= noise_q4 * T) continue;
            // prominence: lowest point on each side before higher terrain (bounded scan)
            int lmin = b[i], rmin = b[i];
            for (int j = i - 1; j >= 0 && j >= i - 48; --j) { if (b[j] > b[i]) break; lmin = std::min<int>(lmin, b[j]); }
            for (int j = i + 1; j < (int)N && j <= i + 48; ++j) { if (b[j] > b[i]) break; rmin = std::min<int>(rmin, b[j]); }
            const int prom = (int)b[i] - std::max(lmin, rmin);
            if (prom < c.d_prom) continue;
            // width at half prominence, linear interpolation
            const int ref_lv = (int)b[i] - prom / 2;
            int lj = i - 1; while (lj > 0 && lj > i - 48 && b[lj] > ref_lv) --lj;
            int rj = i + 1; while (rj < (int)N - 1 && rj < i + 48 && b[rj] > ref_lv) ++rj;
            auto cross = [&](int a, int bq, bool left) {
                const int va = b[a], vb = b[bq];
                if (vb == va) return a * 16;
                const int t = (ref_lv - va) * 16 / (vb - va);
                (void)left;
                return a * 16 + std::clamp(t, 0, 16) * (bq > a ? 1 : -1);
            };
            const int lx = cross(lj, lj + 1, true), rx = cross(rj, rj - 1, false);
            const int width = std::max(16, rx - lx);
            // parabolic peak refinement
            int off = 0;
            const int den = 2 * (2 * (int)b[i] - b[i - 1] - b[i + 1]);
            if (den > 0) off = std::clamp(16 * ((int)b[i + 1] - (int)b[i - 1]) / den, -8, 8);
            Cand cd{i * 16 + off, b[i], (uint8_t)std::min(prom, 255), (uint16_t)width, (int16_t)dsp_ratio_db_x10((uint32_t)b[i] * 16u, noise_q4)};
            if (nc < kMaxCand) cand[nc++] = cd;
            else { unsigned w = 0; for (unsigned k = 1; k < nc; ++k) if (cand[k].level < cand[w].level) w = k; if (cand[w].level < cd.level) cand[w] = cd; }
        }
    }
    // ---- tracking: nearest-neighbour association within +-3 bins ----
    const uint8_t pshift = kPersistShift[c.d_persist & 3];
    const unsigned drop = kDropFrames[c.d_drop & 3];
    bool hit[kMaxTracks] = {};
    for (unsigned k = 0; k < nc; ++k) {
        int best = -1, bd = 3 * 16 + 1;
        for (unsigned t = 0; t < kMaxTracks; ++t) {
            if (!S.tr[t].act || hit[t]) continue;
            const int d = std::abs(S.tr[t].pos_q4 - cand[k].pos_q4);
            if (d < bd) { bd = d; best = (int)t; }
        }
        if (best < 0) { // new track: a free slot, else replace the weakest-persisting one
            for (unsigned t = 0; t < kMaxTracks && best < 0; ++t) if (!S.tr[t].act) best = (int)t;
            if (best < 0) { unsigned w = 0; for (unsigned t = 1; t < kMaxTracks; ++t) if (S.tr[t].persist < S.tr[w].persist) w = t; if (S.tr[w].persist < 160) best = (int)w; }
            if (best < 0) continue;
            Track &n = S.tr[best];
            n = Track{true, cand[k].pos_q4, cand[k].pos_q4, cand[k].level, cand[k].prom, cand[k].width_q4, cand[k].snr_x10, 64, 0, 0};
            hit[best] = true;
            continue;
        }
        Track &t = S.tr[best];
        t.pos_q4 += (cand[k].pos_q4 - t.pos_q4) / 4 ? (cand[k].pos_q4 - t.pos_q4) / 4 : (cand[k].pos_q4 > t.pos_q4 ? 1 : cand[k].pos_q4 < t.pos_q4 ? -1 : 0);
        t.level = cand[k].level; t.prom = cand[k].prom; t.width_q4 = cand[k].width_q4; t.snr_x10 = cand[k].snr_x10;
        t.persist = (uint8_t)(t.persist + ((255 - t.persist) >> pshift));
        t.miss = 0; hit[best] = true;
        if (t.age < 65535) ++t.age;
    }
    for (unsigned t = 0; t < kMaxTracks; ++t) {
        Track &k = S.tr[t];
        if (!k.act || hit[t]) continue;
        k.persist = (uint8_t)(k.persist - (k.persist >> pshift));
        if (k.age < 65535) ++k.age;
        if (++k.miss > drop) k.act = false;
    }
    r.count = 0;
    for (unsigned t = 0; t < kMaxTracks; ++t) {
        if (!S.tr[t].act) continue;
        DspSignal &o = r.sig[r.count++];
        o = DspSignal{true, S.tr[t].pos_q4, S.tr[t].pos_q4 - S.tr[t].first_q4, S.tr[t].level, S.tr[t].prom, S.tr[t].width_q4,
                      S.tr[t].snr_x10, S.tr[t].persist, S.tr[t].age};
    }
    std::sort(r.sig, r.sig + r.count, [](const DspSignal &a, const DspSignal &b) { return a.level > b.level; });
}
}

int dsp_ratio_db_x10(uint32_t num, uint32_t den) {
    if (!den) return 0;
    if (!num) return -600;
    const int64_t lg = (int64_t)ilog2_q8(num) - ilog2_q8(den);
    return (int)((lg * 60206) / 256000 + (lg >= 0 ? 0 : 0));
}
int32_t dsp_pos_to_khz_offset(int32_t pos_q4) { return (((239 * 16) - pos_q4 - 120 * 16) * 625) / 160; }
const int16_t *dsp_window_table(uint8_t id) { build_window(id); return g_win; }
void dsp_reset_state() { reset_internal(); S.have_center = false; }

// ---- IQ statistics and the extended FFT front end ----------------------------------------------
void dsp_fft_frame(const int8_t *iq, uint8_t bins[240], uint8_t peaks[240], bool legacy_hann, const DspConfig &c) {
    int32_t si = 0, sq = 0, sii = 0, sqq = 0, siq = 0;
    for (unsigned n = 0; n < 256; ++n) {
        const int32_t i = iq[2 * n], q = iq[2 * n + 1];
        si += i; sq += q; sii += i * i; sqq += q * q; siq += i * q;
    }
    const int32_t dci = (si >= 0 ? si + 128 : si - 128) / 256, dcq = (sq >= 0 ? sq + 128 : sq - 128) / 256;
    const int64_t pi = (int64_t)sii - (((int64_t)si * si) >> 8);
    const int64_t pq = (int64_t)sqq - (((int64_t)sq * sq) >> 8);
    const int64_t cc = (int64_t)siq - (((int64_t)si * sq) >> 8);
    int peak = 0;
    for (unsigned n = 0; n < 256; ++n) peak = std::max<int>(peak, std::max<int>(std::abs((int)iq[2 * n] - (int)dci), std::abs((int)iq[2 * n + 1] - (int)dcq)));
    DspIqStats st{};
    st.dc_i = (int16_t)dci; st.dc_q = (int16_t)dcq; st.peak = (uint8_t)std::min(peak, 255); st.valid = true;
    if (pi > 256 && pq > 256) { // enough power for a meaningful estimate
        st.gain_x100_db = (int16_t)(((int64_t)(ilog2_q8((uint32_t)pi) - ilog2_q8((uint32_t)pq)) * 30103) / 25600);
        const uint32_t den = isqrt64((uint64_t)pi * (uint64_t)pq);
        const int32_t x_q10 = den ? (int32_t)((cc * 1024) / (int64_t)den) : 0;
        st.phase_x10_deg = (int16_t)(x_q10 * 573 / 1024);
        const int64_t pq2 = pq - (cc * cc) / pi;
        if (pq2 > 64) {
            const int32_t kc = (int32_t)std::clamp<int64_t>((cc * 1024) / pi, -512, 512);
            const int32_t g = (int32_t)std::clamp<int64_t>((int64_t)isqrt64(((uint64_t)pi << 16) / (uint64_t)pq2), 128, 512);
            if (!S.iq_est_valid) { S.kc_q10 = kc; S.g_q8 = g; S.iq_est_valid = true; }
            else { S.kc_q10 += (kc - S.kc_q10) / 4; S.g_q8 += (g - S.g_q8) / 4; }
        }
    }
    // normalisation shift with a 3-frame confirmation so the scale does not chatter
    const uint8_t target = st.peak >= 64 ? 0 : st.peak >= 32 ? 1 : st.peak >= 16 ? 2 : 3;
    if (target == S.norm_s) S.norm_count = 0;
    else if (target == S.norm_pending) { if (++S.norm_count >= 3) { S.norm_s = target; S.norm_count = 0; } }
    else { S.norm_pending = target; S.norm_count = 1; }
    st.norm_shift = c.iq_normalize ? S.norm_s : 0;
    S.res.iq = st;

    const bool custom = c.window != WIN_LEGACY || c.iq_dc || c.iq_balance || c.iq_normalize;
    if (!custom) { fft256_fixed_windowed(iq, bins, peaks, legacy_hann); return; }
    FftParams p{};
    const uint8_t wid = c.window == WIN_LEGACY ? (uint8_t)(legacy_hann ? WIN_HANN : WIN_RECT) : c.window;
    if (wid != WIN_RECT) { build_window(wid); p.window_q15 = g_win; }
    p.out_gain_q8 = c.window == WIN_LEGACY ? 256 : dsp_window_info(wid).gain_q8;
    p.out_shift = c.iq_normalize ? S.norm_s : 0;
    if (c.iq_dc) { p.dc_i = (int16_t)dci; p.dc_q = (int16_t)dcq; }
    if (c.iq_balance && S.iq_est_valid) { p.balance = true; p.kc_q10 = (int16_t)S.kc_q10; p.g_q8 = (uint16_t)S.g_q8; }
    fft256_fixed_custom(iq, bins, peaks, p);
}

// ---- [A] spectrum stage --------------------------------------------------------------------------
void dsp_stage_spectrum(uint8_t b[240], const DspConfig &c, uint32_t center_mhz) {
    if (!S.have_center || S.center != center_mhz) { reset_internal(); S.center = center_mhz; S.have_center = true; }
    const bool need_floor = c.nr_strength || c.d_on || c.v_floor || c.w_suppress || c.v_range;
    const bool filt = c.f_mode || c.notch1 || c.notch2 || c.notch3;
    const bool any = filt || c.dc_suppress || c.adj_reject || c.nr_impulse || c.nr_transient || c.nr_outlier || c.t_mode || need_floor;
    DspResults r = S.res; // keeps the IQ stats written by dsp_fft_frame
    if (!any) { S.floor_valid = false; S.prev_valid = false; r.count = 0; r.center_mhz = center_mhz; LOCK(); g_pub = r; S.res = r; UNLOCK(); return; }

    uint8_t t[N];
    // 1. DC-spike interpolation (uses only bins outside the suppressed span)
    if (c.dc_suppress) {
        const int h = c.dc_suppress == 1 ? 0 : c.dc_suppress == 2 ? 1 : 2;
        const int l = b[120 - h - 1], rr = b[120 + h + 1];
        for (int k = -h; k <= h; ++k) b[120 + k] = (uint8_t)(l + (rr - l) * (k + h + 1) / (2 * h + 2));
    }
    // 2. adjacent-bin rejection: weak skirts next to a much stronger bin are halved; local maxima are never touched
    if (c.adj_reject) {
        static const uint8_t thr[4] = {0, 64, 96, 128};
        memcpy(t, b, N);
        for (unsigned i = 1; i < N - 1; ++i) {
            const unsigned m = std::max(t[i - 1], t[i + 1]);
            if (t[i] < ((m * thr[c.adj_reject]) >> 8)) b[i] = t[i] >> 1;
        }
    }
    // 3. frequency mask and notches
    if (filt) {
        if (!S.mask_valid || S.mask_key != mask_key_of(c)) build_mask(c);
        for (unsigned i = 0; i < N; ++i) b[i] = (uint8_t)((b[i] * S.mask[i]) >> 8);
    }
    uint8_t cur_in[N];
    memcpy(cur_in, b, N);
    // 4. wideband transient rejection (holds the previous output, at most three frames in a row)
    uint32_t energy = 0;
    for (unsigned i = 0; i < N; ++i) energy += b[i];
    if (c.nr_transient && S.prev_valid && S.energy_n >= 4) {
        static const uint8_t factor[4] = {0, 48, 32, 24}; // 3x, 2x, 1.5x of the running mean energy
        if (energy * 16u > S.energy_ema * factor[c.nr_transient] && S.transient_run < 3) {
            ++S.transient_run; ++S.res.transients; ++r.transients;
            memcpy(b, S.prev_out, N);
            r.center_mhz = center_mhz; LOCK(); g_pub = r; S.res = r; UNLOCK();
            return;
        }
    }
    S.transient_run = 0;
    S.energy_ema = S.energy_n ? (uint32_t)((int32_t)S.energy_ema + ((int32_t)energy - (int32_t)S.energy_ema) / 8) : energy;
    if (S.energy_n < 255) ++S.energy_n;
    // 5. impulse rejection: single-bin spike that was not there in the previous frame
    if (c.nr_impulse && S.prev_valid) {
        static const uint8_t k[4] = {0, 4, 3, 2};
        memcpy(t, b, N);
        for (unsigned i = 1; i < N - 1; ++i) {
            const unsigned m = std::max(t[i - 1], t[i + 1]);
            if (t[i] > 8 && t[i] > m * k[c.nr_impulse] && S.prev_in[i] < t[i] / 2) { b[i] = (uint8_t)m; ++r.impulses; ++S.res.impulses; }
        }
    }
    // 6. outlier rejection: clip (not delete) bins far above the local median unless they persisted from the last frame
    if (c.nr_outlier && S.prev_valid) {
        static const uint8_t kk[4] = {0, 6, 4, 3};
        memcpy(t, b, N);
        for (unsigned i = 2; i < N - 2; ++i) {
            uint8_t v[5] = {t[i - 2], t[i - 1], t[i], t[i + 1], t[i + 2]};
            sort_small(v, 5);
            const int med = v[2];
            uint8_t dv[5];
            for (int q = 0; q < 5; ++q) dv[q] = (uint8_t)std::abs((int)v[q] - med);
            sort_small(dv, 5);
            const int limit = med + (int)kk[c.nr_outlier] * ((int)dv[2] + 1);
            if (t[i] > limit && S.prev_in[i] <= limit) { b[i] = u8(limit); ++r.outliers; ++S.res.outliers; }
        }
    }
    // 7. noise floor: robust frame median over passband bins + per-bin lower-envelope tracker
    uint8_t ff_level = 0;
    if (need_floor) {
        static uint16_t hist[256];
        memset(hist, 0, sizeof(hist));
        unsigned total = 0;
        for (unsigned i = 0; i < N; ++i) if (!filt || S.mask[i] >= 256) { ++hist[b[i]]; ++total; }
        unsigned raw = S.ff_q8 >> 8;
        if (total >= 8) { unsigned acc = 0, v = 0; while (v < 255 && acc + hist[v] < (total + 1) / 2) acc += hist[v++]; raw = v; }
        if (!S.floor_valid) S.ff_q8 = (uint16_t)(raw << 8);
        else { const int d = (int)(raw << 8) - (int)S.ff_q8; S.ff_q8 = (uint16_t)((int)S.ff_q8 + (d > 0 ? d >> 3 : d >> 1)); }
        ff_level = (uint8_t)(S.ff_q8 >> 8);
        const unsigned cap = std::min<unsigned>(65535, (unsigned)S.ff_q8 * 3 / 2 + 512); // a carrier can never become "the floor"
        static const uint8_t up[3] = {7, 5, 3};
        for (unsigned i = 0; i < N; ++i) {
            const int x = (int)b[i] << 8;
            int fl;
            if (!S.floor_valid) fl = std::min<int>(x, (int)cap);
            else {
                fl = S.floor_q8[i];
                fl += (x < fl) ? ((x - fl) >> 1) : ((x - fl) >> up[c.nr_rate % 3]);
                if (fl > (int)cap) fl = (int)cap;
            }
            S.floor_q8[i] = (uint16_t)fl;
        }
        S.floor_valid = true;
        if (c.nr_strength) {
            for (unsigned i = 0; i < N; ++i) {
                unsigned sub = ((unsigned)S.floor_q8[i] * c.nr_strength) / (10u * 256u);
                if (c.nr_guard && b[i] >= 2u * std::max<unsigned>(ff_level, 1)) sub = std::min<unsigned>(sub, b[i] >> 3);
                b[i] = (uint8_t)(b[i] > sub ? b[i] - sub : 0);
            }
        }
    } else S.floor_valid = false;
    // 8. temporal filtering across frames
    if (c.t_mode != S.t_mode_prev) { S.t_valid = false; S.hist_n = 0; S.t_mode_prev = c.t_mode; }
    if (c.t_mode == 1 || c.t_mode == 2) {
        const int slow = c.t_ema, thr = (int)c.t_adapt * 4 * 256;
        for (unsigned i = 0; i < N; ++i) {
            const int x = (int)b[i] << 8;
            if (!S.t_valid) S.t_y[i] = (uint16_t)x;
            else {
                const int y = S.t_y[i], d = x - y;
                const int sh = (c.t_mode == 2 && std::abs(d) > thr) ? 1 : slow;
                S.t_y[i] = (uint16_t)(y + (d >> sh));
            }
            b[i] = (uint8_t)((S.t_y[i] + 128) >> 8);
        }
        S.t_valid = true;
    } else if (c.t_mode == 3 || c.t_mode == 4) {
        const unsigned depth = c.t_mode == 3 ? 3 : 5;
        memcpy(S.hist[S.hist_pos], b, N);
        S.hist_pos = (uint8_t)((S.hist_pos + 1) % 5);
        if (S.hist_n < 5) ++S.hist_n;
        const unsigned n = std::min<unsigned>(S.hist_n, depth);
        for (unsigned i = 0; i < N; ++i) {
            uint8_t v[5];
            for (unsigned q = 0; q < n; ++q) v[q] = S.hist[(S.hist_pos + 5 - 1 - q) % 5][i];
            sort_small(v, n);
            b[i] = v[(n - 1) / 2];
        }
    }
    // 9. remember this frame, detect, publish
    memcpy(S.prev_in, cur_in, N);
    memcpy(S.prev_out, b, N);
    S.prev_valid = true;
    r.frame_floor = ff_level;
    r.frame_peak = *std::max_element(b, b + N);
    r.center_mhz = center_mhz;
    detect(b, c, r);
    LOCK(); g_pub = r; S.res = r; UNLOCK();
}

// ---- [B] display stage: trace path and waterfall path are processed independently --------------
void dsp_stage_display(const uint8_t avg[240], const uint8_t peaks[240], bool legacy_max_hold, const DspConfig &c, SpectrumFrame &o) {
    // trace
    if (c.v_smooth) {
        for (unsigned i = 0; i < N; ++i) {
            const int x = (int)avg[i] << 8;
            S.trace_y[i] = S.trace_valid ? (uint16_t)(S.trace_y[i] + ((x - (int)S.trace_y[i]) >> c.v_smooth)) : (uint16_t)x;
            o.bins[i] = (uint8_t)((S.trace_y[i] + 128) >> 8);
        }
        S.trace_valid = true;
    } else { memcpy(o.bins, avg, N); S.trace_valid = false; }
    if (c.v_decay && !legacy_max_hold) {
        static const uint16_t dec[7] = {0, 64, 128, 256, 512, 1024, 2048};
        for (unsigned i = 0; i < N; ++i) {
            const int x = (int)o.bins[i] << 8;
            int p = S.peak_valid ? (int)S.peak_y[i] - dec[c.v_decay] : x;
            if (p < x) p = x;
            S.peak_y[i] = (uint16_t)p;
            o.peaks[i] = (uint8_t)(p >> 8);
        }
        S.peak_valid = true;
    } else { memcpy(o.peaks, peaks, N); S.peak_valid = false; }
    if (c.v_minhold) {
        for (unsigned i = 0; i < N; ++i) S.minhold[i] = S.min_valid ? std::min(S.minhold[i], o.bins[i]) : o.bins[i];
        S.min_valid = true;
        memcpy(o.mins, S.minhold, N);
    } else { S.min_valid = false; memset(o.mins, 0, N); }
    o.have_floor = S.floor_valid;
    o.floor_level = S.floor_valid ? (uint8_t)(S.ff_q8 >> 8) : 0;
    if (S.floor_valid) for (unsigned i = 0; i < N; ++i) o.floor[i] = (uint8_t)std::min<unsigned>(255, S.floor_q8[i] >> 8);
    else memset(o.floor, 0, N);
    // waterfall
    if (!(c.w_smooth || c.w_suppress || c.w_contrast)) { memcpy(o.wf, avg, N); S.wf_valid = false; return; }
    static const uint8_t supp[4] = {0, 128, 255, 255}; // 0, 0.5x, 1x, 1.5x (third value applied below)
    static const uint8_t gain_q4[4] = {16, 24, 32, 48};
    const int pivot = S.floor_valid ? (int)(S.ff_q8 >> 8) : 0;
    for (unsigned i = 0; i < N; ++i) {
        int v = avg[i];
        if (c.w_smooth) {
            const int x = v << 8;
            S.wf_y[i] = S.wf_valid ? (uint16_t)(S.wf_y[i] + ((x - (int)S.wf_y[i]) >> c.w_smooth)) : (uint16_t)x;
            v = (S.wf_y[i] + 128) >> 8;
        }
        if (c.w_suppress && S.floor_valid) {
            const int f = S.floor_q8[i] >> 8;
            const int sub = c.w_suppress == 1 ? f / 2 : c.w_suppress == 2 ? f : f + f / 2;
            v = std::max(0, v - sub);
        }
        if (c.w_contrast && S.floor_valid) {
            const int g = gain_q4[c.w_contrast];
            v = v > pivot ? pivot + (((v - pivot) * g) >> 4) : pivot - (((pivot - v) * g) >> 4);
        }
        o.wf[i] = u8(v);
    }
    (void)supp;
    S.wf_valid = c.w_smooth != 0;
}

void dsp_results_snapshot(DspResults *out) { LOCK(); *out = g_pub; UNLOCK(); }
void dsp_perf_snapshot(DspPerf *out) { LOCK(); *out = g_perf; UNLOCK(); }
void dsp_perf_note(uint32_t fft_us, uint32_t stage_us, uint32_t display_us) {
    LOCK();
    auto ema = [](uint32_t &a, uint32_t v) { a = g_perf.frames ? a + (int32_t)(v - a) / 8 : v; };
    ema(g_perf.fft_us, fft_us); ema(g_perf.stage_us, stage_us); ema(g_perf.display_us, display_us);
    g_perf.total_max_us = std::max(g_perf.total_max_us, fft_us + stage_us + display_us);
    ++g_perf.frames;
    UNLOCK();
}
}
