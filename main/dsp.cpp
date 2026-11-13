#include <cstddef>
#include "include/dsp.hpp"
#include <array>

namespace {
struct Twiddle { int16_t re, im; };
constexpr Twiddle kTwiddle[256] = {
    {32767, 0},
    {32757, -804},
    {32728, -1608},
    {32678, -2410},
    {32609, -3212},
    {32521, -4011},
    {32412, -4808},
    {32285, -5602},
    {32137, -6393},
    {31971, -7179},
    {31785, -7962},
    {31580, -8739},
    {31356, -9512},
    {31113, -10278},
    {30852, -11039},
    {30571, -11793},
    {30273, -12539},
    {29956, -13279},
    {29621, -14010},
    {29268, -14732},
    {28898, -15446},
    {28510, -16151},
    {28105, -16846},
    {27683, -17530},
    {27245, -18204},
    {26790, -18868},
    {26319, -19519},
    {25832, -20159},
    {25329, -20787},
    {24811, -21403},
    {24279, -22005},
    {23731, -22594},
    {23170, -23170},
    {22594, -23731},
    {22005, -24279},
    {21403, -24811},
    {20787, -25329},
    {20159, -25832},
    {19519, -26319},
    {18868, -26790},
    {18204, -27245},
    {17530, -27683},
    {16846, -28105},
    {16151, -28510},
    {15446, -28898},
    {14732, -29268},
    {14010, -29621},
    {13279, -29956},
    {12539, -30273},
    {11793, -30571},
    {11039, -30852},
    {10278, -31113},
    {9512, -31356},
    {8739, -31580},
    {7962, -31785},
    {7179, -31971},
    {6393, -32137},
    {5602, -32285},
    {4808, -32412},
    {4011, -32521},
    {3212, -32609},
    {2410, -32678},
    {1608, -32728},
    {804, -32757},
    {0, -32767},
    {-804, -32757},
    {-1608, -32728},
    {-2410, -32678},
    {-3212, -32609},
    {-4011, -32521},
    {-4808, -32412},
    {-5602, -32285},
    {-6393, -32137},
    {-7179, -31971},
    {-7962, -31785},
    {-8739, -31580},
    {-9512, -31356},
    {-10278, -31113},
    {-11039, -30852},
    {-11793, -30571},
    {-12539, -30273},
    {-13279, -29956},
    {-14010, -29621},
    {-14732, -29268},
    {-15446, -28898},
    {-16151, -28510},
    {-16846, -28105},
    {-17530, -27683},
    {-18204, -27245},
    {-18868, -26790},
    {-19519, -26319},
    {-20159, -25832},
    {-20787, -25329},
    {-21403, -24811},
    {-22005, -24279},
    {-22594, -23731},
    {-23170, -23170},
    {-23731, -22594},
    {-24279, -22005},
    {-24811, -21403},
    {-25329, -20787},
    {-25832, -20159},
    {-26319, -19519},
    {-26790, -18868},
    {-27245, -18204},
    {-27683, -17530},
    {-28105, -16846},
    {-28510, -16151},
    {-28898, -15446},
    {-29268, -14732},
    {-29621, -14010},
    {-29956, -13279},
    {-30273, -12539},
    {-30571, -11793},
    {-30852, -11039},
    {-31113, -10278},
    {-31356, -9512},
    {-31580, -8739},
    {-31785, -7962},
    {-31971, -7179},
    {-32137, -6393},
    {-32285, -5602},
    {-32412, -4808},
    {-32521, -4011},
    {-32609, -3212},
    {-32678, -2410},
    {-32728, -1608},
    {-32757, -804},
    {-32767, 0},
    {-32757, 804},
    {-32728, 1608},
    {-32678, 2410},
    {-32609, 3212},
    {-32521, 4011},
    {-32412, 4808},
    {-32285, 5602},
    {-32137, 6393},
    {-31971, 7179},
    {-31785, 7962},
    {-31580, 8739},
    {-31356, 9512},
    {-31113, 10278},
    {-30852, 11039},
    {-30571, 11793},
    {-30273, 12539},
    {-29956, 13279},
    {-29621, 14010},
    {-29268, 14732},
    {-28898, 15446},
    {-28510, 16151},
    {-28105, 16846},
    {-27683, 17530},
    {-27245, 18204},
    {-26790, 18868},
    {-26319, 19519},
    {-25832, 20159},
    {-25329, 20787},
    {-24811, 21403},
    {-24279, 22005},
    {-23731, 22594},
    {-23170, 23170},
    {-22594, 23731},
    {-22005, 24279},
    {-21403, 24811},
    {-20787, 25329},
    {-20159, 25832},
    {-19519, 26319},
    {-18868, 26790},
    {-18204, 27245},
    {-17530, 27683},
    {-16846, 28105},
    {-16151, 28510},
    {-15446, 28898},
    {-14732, 29268},
    {-14010, 29621},
    {-13279, 29956},
    {-12539, 30273},
    {-11793, 30571},
    {-11039, 30852},
    {-10278, 31113},
    {-9512, 31356},
    {-8739, 31580},
    {-7962, 31785},
    {-7179, 31971},
    {-6393, 32137},
    {-5602, 32285},
    {-4808, 32412},
    {-4011, 32521},
    {-3212, 32609},
    {-2410, 32678},
    {-1608, 32728},
    {-804, 32757},
    {0, 32767},
    {804, 32757},
    {1608, 32728},
    {2410, 32678},
    {3212, 32609},
    {4011, 32521},
    {4808, 32412},
    {5602, 32285},
    {6393, 32137},
    {7179, 31971},
    {7962, 31785},
    {8739, 31580},
    {9512, 31356},
    {10278, 31113},
    {11039, 30852},
    {11793, 30571},
    {12539, 30273},
    {13279, 29956},
    {14010, 29621},
    {14732, 29268},
    {15446, 28898},
    {16151, 28510},
    {16846, 28105},
    {17530, 27683},
    {18204, 27245},
    {18868, 26790},
    {19519, 26319},
    {20159, 25832},
    {20787, 25329},
    {21403, 24811},
    {22005, 24279},
    {22594, 23731},
    {23170, 23170},
    {23731, 22594},
    {24279, 22005},
    {24811, 21403},
    {25329, 20787},
    {25832, 20159},
    {26319, 19519},
    {26790, 18868},
    {27245, 18204},
    {27683, 17530},
    {28105, 16846},
    {28510, 16151},
    {28898, 15446},
    {29268, 14732},
    {29621, 14010},
    {29956, 13279},
    {30273, 12539},
    {30571, 11793},
    {30852, 11039},
    {31113, 10278},
    {31356, 9512},
    {31580, 8739},
    {31785, 7962},
    {31971, 7179},
    {32137, 6393},
    {32285, 5602},
    {32412, 4808},
    {32521, 4011},
    {32609, 3212},
    {32678, 2410},
    {32728, 1608},
    {32757, 804}
};

struct Rgb { uint8_t r, g, b; };
constexpr Rgb kMagmaStops[] = {{0,0,4},{24,15,61},{59,15,112},{100,26,128},{147,38,103},
    {192,58,75},{224,100,42},{252,165,10},{252,253,191}};
constexpr Rgb kInfernoStops[] = {{0,0,4},{30,12,75},{91,19,110},{160,35,88},{220,66,45},
    {249,132,15},{252,255,164}};
constexpr Rgb kGrayStops[] = {{0,0,0},{255,255,255}};
template<size_t N> constexpr std::array<uint16_t, 256> make_palette(const Rgb (&stops)[N]) {
    std::array<uint16_t, 256> out{};
    for (unsigned i = 0; i < 256; ++i) {
        const unsigned scaled = i * (N - 1) * 256 / 255;
        unsigned seg = scaled >> 8;
        unsigned frac = scaled & 255;
        if (seg >= N - 1) { seg = N - 2; frac = 255; }
        const Rgb a = stops[seg], b = stops[seg + 1];
        const unsigned r = (a.r * (256 - frac) + b.r * frac) >> 8;
        const unsigned g = (a.g * (256 - frac) + b.g * frac) >> 8;
        const unsigned bl = (a.b * (256 - frac) + b.b * frac) >> 8;
        out[i] = (uint16_t)(((r >> 3) << 11) | ((g >> 2) << 5) | (bl >> 3));
    }
    return out;
}
constexpr auto kMagma = make_palette(kMagmaStops);
constexpr auto kInferno = make_palette(kInfernoStops);
constexpr auto kGray = make_palette(kGrayStops);
struct Complex { int32_t re, im; };
}

namespace {
void fft_core(Complex a[256]) {
    // Four radix-4 stages for 256 points. Scale by 1/4 at each stage to
    // maintain headroom; all multiply and butterfly math is integer-only.
    for (unsigned len = 4; len <= 256; len *= 4) {
        unsigned quarter = len >> 2;
        unsigned stride = 256 / len;
        for (unsigned base = 0; base < 256; base += len) {
            for (unsigned k = 0; k < quarter; ++k) {
                const Complex x0 = a[base + k];
                const Complex u1 = a[base + k + quarter];
                const Complex u2 = a[base + k + 2 * quarter];
                const Complex u3 = a[base + k + 3 * quarter];
                const Twiddle w1 = kTwiddle[k * stride];
                const Twiddle w2 = kTwiddle[2 * k * stride];
                const Twiddle w3 = kTwiddle[3 * k * stride];
                const Complex x1 = {(u1.re * w1.re - u1.im * w1.im) >> 15,
                                    (u1.re * w1.im + u1.im * w1.re) >> 15};
                const Complex x2 = {(u2.re * w2.re - u2.im * w2.im) >> 15,
                                    (u2.re * w2.im + u2.im * w2.re) >> 15};
                const Complex x3 = {(u3.re * w3.re - u3.im * w3.im) >> 15,
                                    (u3.re * w3.im + u3.im * w3.re) >> 15};
                const Complex t0 = {x0.re + x2.re, x0.im + x2.im};
                const Complex t1 = {x0.re - x2.re, x0.im - x2.im};
                const Complex t2 = {x1.re + x3.re, x1.im + x3.im};
                const Complex t3 = {x1.re - x3.re, x1.im - x3.im};
                a[base + k] = {(t0.re + t2.re) >> 2, (t0.im + t2.im) >> 2};
                a[base + k + quarter] = {(t1.re + t3.im) >> 2, (t1.im - t3.re) >> 2};
                a[base + k + 2 * quarter] = {(t0.re - t2.re) >> 2, (t0.im - t2.im) >> 2};
                a[base + k + 3 * quarter] = {(t1.re - t3.im) >> 2, (t1.im + t3.re) >> 2};
            }
        }
    }
}
}

namespace marauder {
uint16_t waterfall_rgb565(uint8_t level, uint8_t floor, uint8_t ceiling, uint8_t palette) {
    if (ceiling <= floor) ceiling = (uint8_t)(floor + 1);
    unsigned v = level <= floor ? 0 : level >= ceiling ? 255 :
        ((unsigned)(level - floor) * 255u) / (ceiling - floor);
    switch (palette) {
        case 1: return kInferno[v];
        case 2: return kGray[v];
        default: return kMagma[v];
    }
}
uint16_t heatmap_rgb565(uint8_t level) { return waterfall_rgb565(level, 20, 220, 0); }

void fft256_fixed_windowed(const int8_t *iq, uint8_t bins[240], uint8_t peaks[240], bool hann_window) {
    Complex a[256];
    for (unsigned n = 0; n < 256; ++n) {
        unsigned r = 0, x = n;
        for (unsigned b = 0; b < 4; ++b) { r = (r << 2) | (x & 3u); x >>= 2; }
        int32_t re = (int32_t)iq[2 * n] * 128;
        int32_t im = (int32_t)iq[2 * n + 1] * 128;
        if (hann_window) {
            const int32_t weight = (32767 - kTwiddle[n].re) / 2;
            re = (re * weight) >> 15;
            im = (im * weight) >> 15;
        }
        a[r] = {re, im};
    }
    fft_core(a);
    for (unsigned x = 0; x < 240; ++x) {
        // The 240-bin output drops eight bins at each edge, keeping DC at
        // index 120 (the center of the cropped spectrum and waterfall).
        const unsigned i = (x + 136) & 255u;
        const uint32_t re = (uint32_t)(a[i].re < 0 ? -a[i].re : a[i].re);
        const uint32_t im = (uint32_t)(a[i].im < 0 ? -a[i].im : a[i].im);
        // The radix-4 stages normalize the FFT. Preserve the resulting bin
        // magnitude; shifting it down pushed most bins below the UI floor.
        uint32_t mag = re + im;
        if (mag > 255) mag = 255;
        bins[x] = (uint8_t)mag;
        if (bins[x] >= peaks[x]) peaks[x] = bins[x];
        else if (peaks[x]) --peaks[x];
    }
}
void fft256_fixed_custom(const int8_t *iq, uint8_t bins[240], uint8_t peaks[240], const FftParams &p) {
    Complex a[256];
    for (unsigned n = 0; n < 256; ++n) {
        unsigned r = 0, x = n;
        for (unsigned b = 0; b < 4; ++b) { r = (r << 2) | (x & 3u); x >>= 2; }
        int32_t i = (int32_t)iq[2 * n] - p.dc_i;
        int32_t q = (int32_t)iq[2 * n + 1] - p.dc_q;
        if (p.balance) q = (((q - ((p.kc_q10 * i) >> 10))) * (int32_t)p.g_q8) >> 8;
        int32_t re = i * 128, im = q * 128;
        if (p.window_q15) {
            const int32_t w = p.window_q15[n];
            re = (re * w) >> 15;
            im = (im * w) >> 15;
        }
        a[r] = {re, im};
    }
    fft_core(a);
    for (unsigned x = 0; x < 240; ++x) {
        const unsigned i = (x + 136) & 255u;
        const uint32_t re = (uint32_t)(a[i].re < 0 ? -a[i].re : a[i].re);
        const uint32_t im = (uint32_t)(a[i].im < 0 ? -a[i].im : a[i].im);
        uint32_t mag = (((re + im) * (uint32_t)p.out_gain_q8) >> 8) << p.out_shift;
        if (mag > 255) mag = 255;
        bins[x] = (uint8_t)mag;
        if (bins[x] >= peaks[x]) peaks[x] = bins[x];
        else if (peaks[x]) --peaks[x];
    }
}
void fft256_fixed(const int8_t *iq, uint8_t bins[240], uint8_t peaks[240]) {
    fft256_fixed_windowed(iq, bins, peaks, false);
}
}
