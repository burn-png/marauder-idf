// DSP configuration: one table drives defaults, validation, the on-device menu text and persistence.
#include "include/dsp_adv.hpp"
#include <cstdio>
#include <cstring>
#include <cstddef>
#ifdef ESP_PLATFORM
#include "nvs.h"
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
enum Fmt : uint8_t { F_NAME, F_NUM, F_BIN_OFFSET, F_BIN_WIDTH, F_NOTCH, F_ALPHA, F_LEVELS4, F_OFFNUM };
struct Item {
    const char *name; uint8_t group; uint8_t off; uint8_t lo, hi, step, def; Fmt fmt;
    const char *const *names; const char *desc;
};
#define OFS(f) ((uint8_t)offsetof(DspConfig, f))
const char *const kOnOff[] = {"OFF", "ON"};
const char *const kWin[] = {"RECT", "HANN", "HAMMING", "BLACKMAN", "BLK-HARRIS", "FLAT-TOP", "BARTLETT", "TRIANGULAR", "LEGACY"};
const char *const kFilter[] = {"OFF", "LOW-PASS", "HIGH-PASS", "BAND-PASS", "BAND-STOP"};
const char *const kStop[] = {"FULL", "-12 dB", "-6 dB", "-3 dB"};
const char *const kDc[] = {"OFF", "1 BIN", "3 BINS", "5 BINS"};
const char *const kAdj[] = {"OFF", "MILD 25%", "MEDIUM 37%", "STRONG 50%"};
const char *const kRate[] = {"SLOW", "MEDIUM", "FAST"};
const char *const kLevel[] = {"OFF", "LOW", "MEDIUM", "HIGH"};
const char *const kCfar[] = {"CA", "GO", "OS"};
const char *const kSens[] = {"1.5x", "2x", "2.5x", "3x", "4x", "6x", "8x", "12x"};
const char *const kDrop[] = {"4 FRAMES", "8 FRAMES", "16 FRAMES", "32 FRAMES"};
const char *const kPersist[] = {"FAST", "MEDIUM", "SLOW", "VERY SLOW"};
const char *const kTMode[] = {"OFF", "EMA", "ADAPTIVE", "MEDIAN 3", "MEDIAN 5"};
const char *const kDecay[] = {"LEGACY", "0.25 /FRAME", "0.5 /FRAME", "1 /FRAME", "2 /FRAME", "4 /FRAME", "8 /FRAME"};
const char *const kRange[] = {"LEGACY", "FLOOR+SPAN", "FLOOR+PEAK"};
const char *const kSpan[] = {"32", "64", "96", "128", "192"};
const char *const kInterp[] = {"NEAREST", "LINEAR"};
const char *const kSmooth[] = {"OFF", "1/2", "1/4", "1/8", "1/16"};
const char *const kWfSmooth[] = {"OFF", "1/2", "1/4", "1/8"};

const Item kItems[] = {
    // FFT / WINDOW
    {"WINDOW", DG_FFT, OFS(window), 0, 8, 1, WIN_LEGACY, F_NAME, kWin, "LEAKAGE VS RESOLUTION"},
    {"IQ DC CORR", DG_FFT, OFS(iq_dc), 0, 1, 1, 0, F_NAME, kOnOff, "REMOVE I/Q MEAN. ALSO REMOVES A TRUE DC TONE"},
    {"IQ BALANCE", DG_FFT, OFS(iq_balance), 0, 1, 1, 0, F_NAME, kOnOff, "BLIND GAIN+PHASE FIX FROM 256-SAMPLE STATS"},
    {"IQ NORMALIZE", DG_FFT, OFS(iq_normalize), 0, 1, 1, 0, F_NAME, kOnOff, "SCALES WEAK CAPTURES UP. LEVELS STAY RELATIVE"},
    // FREQUENCY FILTERS
    {"FILTER", DG_FILTER, OFS(f_mode), 0, 4, 1, 0, F_NAME, kFilter, "SPECTRUM MASK ON PROCESSED BINS. IQ UNTOUCHED"},
    {"CENTER", DG_FILTER, OFS(f_center), 0, 239, 2, 120, F_BIN_OFFSET, nullptr, "BAND-PASS / STOP CENTRE. LOW/HIGH-PASS USE 0"},
    {"WIDTH", DG_FILTER, OFS(f_width), 2, 240, 2, 60, F_BIN_WIDTH, nullptr, "FULL WIDTH. LOW/HIGH-PASS: CUTOFF = WIDTH/2"},
    {"ROLLOFF", DG_FILTER, OFS(f_roll), 0, 16, 1, 4, F_NUM, nullptr, "EDGE RAMP IN BINS (62.5 kHz EACH)"},
    {"REJECT DEPTH", DG_FILTER, OFS(f_stop), 0, 3, 1, 0, F_NAME, kStop, "STOP-BAND ATTENUATION (AMPLITUDE)"},
    {"NOTCH 1", DG_FILTER, OFS(notch1), 0, 240, 2, 0, F_NOTCH, nullptr, "FIXED NOTCH (E.G. A KNOWN SPUR)"},
    {"NOTCH 2", DG_FILTER, OFS(notch2), 0, 240, 2, 0, F_NOTCH, nullptr, "SECOND NOTCH"},
    {"NOTCH 3", DG_FILTER, OFS(notch3), 0, 240, 2, 0, F_NOTCH, nullptr, "THIRD NOTCH"},
    {"NOTCH WIDTH", DG_FILTER, OFS(notch_width), 1, 32, 1, 3, F_NUM, nullptr, "ALL NOTCHES, BINS WIDE"},
    {"DC SUPPRESS", DG_FILTER, OFS(dc_suppress), 0, 3, 1, 0, F_NAME, kDc, "INTERPOLATE ACROSS THE DC SPIKE"},
    {"ADJ BIN REJ", DG_FILTER, OFS(adj_reject), 0, 3, 1, 0, F_NAME, kAdj, "TRIM WEAK SKIRTS NEXT TO STRONG BINS"},
    // NOISE REDUCTION
    {"NR STRENGTH", DG_NOISE, OFS(nr_strength), 0, 10, 1, 0, F_OFFNUM, nullptr, "FLOOR SUBTRACTION: 10 = FULL TRACKED FLOOR"},
    {"FLOOR RATE", DG_NOISE, OFS(nr_rate), 0, 2, 1, 1, F_NAME, kRate, "HOW FAST THE FLOOR RISES (FALLS ARE FAST)"},
    {"NB GUARD", DG_NOISE, OFS(nr_guard), 0, 1, 1, 1, F_NAME, kOnOff, "SIGNALS ABOVE 2x FLOOR KEEP >=87% LEVEL"},
    {"IMPULSE REJ", DG_NOISE, OFS(nr_impulse), 0, 3, 1, 0, F_NAME, kLevel, "ONE-FRAME ONE-BIN SPIKES. PERSISTENT KEPT"},
    {"TRANSIENT REJ", DG_NOISE, OFS(nr_transient), 0, 3, 1, 0, F_NAME, kLevel, "WIDEBAND BURSTS: HOLD LAST FRAME (MAX 3)"},
    {"OUTLIER REJ", DG_NOISE, OFS(nr_outlier), 0, 3, 1, 0, F_NAME, kLevel, "CLIP BINS FAR ABOVE LOCAL MEDIAN+MAD"},
    // SIGNAL DETECTION
    {"DETECT", DG_DETECT, OFS(d_on), 0, 1, 1, 1, F_NAME, kOnOff, "CFAR PEAK FINDER. DOES NOT CHANGE THE TRACE"},
    {"CFAR TYPE", DG_DETECT, OFS(d_type), 0, 2, 1, 2, F_NAME, kCfar, "CA MEAN, GO GREATER-OF, OS ORDERED (ROBUST)"},
    {"GUARD CELLS", DG_DETECT, OFS(d_guard), 1, 4, 1, 2, F_NUM, nullptr, "BINS SKIPPED NEXT TO THE TEST BIN"},
    {"TRAIN CELLS", DG_DETECT, OFS(d_train), 4, 16, 2, 8, F_NUM, nullptr, "REFERENCE BINS PER SIDE"},
    {"THRESHOLD", DG_DETECT, OFS(d_sens), 0, 7, 1, 5, F_NAME, kSens, "PEAK MUST EXCEED NOISE BY THIS FACTOR"},
    {"MIN PROM", DG_DETECT, OFS(d_prom), 1, 64, 1, 8, F_NUM, nullptr, "LEVELS ABOVE THE HIGHER SURROUNDING SADDLE"},
    {"TRACK DROP", DG_DETECT, OFS(d_drop), 0, 3, 1, 1, F_NAME, kDrop, "FRAMES WITHOUT A HIT BEFORE A TRACK ENDS"},
    {"PERSISTENCE", DG_DETECT, OFS(d_persist), 0, 3, 1, 1, F_NAME, kPersist, "HOW LONG HISTORY COUNTS TOWARD PERSIST %"},
    // TEMPORAL
    {"TEMPORAL", DG_TEMPORAL, OFS(t_mode), 0, 4, 1, 0, F_NAME, kTMode, "FRAME-TO-FRAME FILTER, BEFORE LEGACY AVG"},
    {"EMA ALPHA", DG_TEMPORAL, OFS(t_ema), 1, 5, 1, 2, F_ALPHA, nullptr, "SMALLER = SMOOTHER BUT SLOWER"},
    {"ADAPT THRESH", DG_TEMPORAL, OFS(t_adapt), 1, 8, 1, 3, F_LEVELS4, nullptr, "CHANGE BIGGER THAN THIS TRACKS FAST"},
    // DISPLAY
    {"TRACE SMOOTH", DG_DISPLAY, OFS(v_smooth), 0, 4, 1, 0, F_NAME, kSmooth, "TRACE ONLY. WATERFALL HAS ITS OWN"},
    {"PEAK DECAY", DG_DISPLAY, OFS(v_decay), 0, 6, 1, 0, F_NAME, kDecay, "PEAK MARKER FALL RATE (IGNORED IN MAX HOLD)"},
    {"MIN HOLD", DG_DISPLAY, OFS(v_minhold), 0, 1, 1, 0, F_NAME, kOnOff, "GREEN LINE: LOWEST LEVEL SINCE RETUNE"},
    {"FLOOR LINE", DG_DISPLAY, OFS(v_floor), 0, 1, 1, 0, F_NAME, kOnOff, "AMBER DOTS: TRACKED NOISE FLOOR"},
    {"RANGE MODE", DG_DISPLAY, OFS(v_range), 0, 2, 1, 0, F_NAME, kRange, "TRACE VERTICAL SCALE. LEGACY = OLD AUTO FIT"},
    {"RANGE SPAN", DG_DISPLAY, OFS(v_span), 0, 4, 1, 3, F_NAME, kSpan, "LEVELS SHOWN ABOVE THE FLOOR (FLOOR+SPAN)"},
    {"WF SUPPRESS", DG_DISPLAY, OFS(w_suppress), 0, 3, 1, 0, F_NAME, kLevel, "WATERFALL ONLY: SUBTRACT TRACKED FLOOR"},
    {"WF INTERP", DG_DISPLAY, OFS(w_interp), 0, 1, 1, 0, F_NAME, kInterp, "WATERFALL BIN INTERPOLATION WHEN ZOOMED"},
    {"WF CONTRAST", DG_DISPLAY, OFS(w_contrast), 0, 3, 1, 0, F_NAME, kLevel, "STRETCH ABOVE / SQUASH BELOW THE FLOOR"},
    {"WF SMOOTH", DG_DISPLAY, OFS(w_smooth), 0, 3, 1, 0, F_NAME, kWfSmooth, "WATERFALL ONLY TEMPORAL SMOOTHING"},
};
constexpr size_t kItemTotal = sizeof(kItems) / sizeof(*kItems);
const char *const kGroupNames[DG_COUNT] = {"FFT / WINDOW", "FREQ FILTERS", "NOISE REDUCTION", "SIGNAL DETECT", "TEMPORAL", "DISPLAY"};

DspConfig s_active;
bool s_loaded = false;
bool s_dirty = false;

uint8_t &field(DspConfig &c, uint8_t off) { return *((uint8_t *)&c + off); }
const Item *find(unsigned group, size_t index) {
    size_t seen = 0;
    for (size_t i = 0; i < kItemTotal; ++i)
        if (kItems[i].group == group) { if (seen == index) return &kItems[i]; ++seen; }
    return nullptr;
}
void ensure_loaded() { if (!s_loaded) { s_active = dsp_cfg_defaults(); s_loaded = true; } }
}

DspConfig dsp_cfg_defaults() {
    DspConfig c{};
    for (const auto &it : kItems) field(c, it.off) = it.def;
    return c;
}
void dsp_cfg_sanitize(DspConfig &c) {
    for (const auto &it : kItems) {
        uint8_t &v = field(c, it.off);
        if (v < it.lo || v > it.hi) v = it.def; // out of range (corrupt / older blob): default, never clamp blindly
    }
}
DspConfig dsp_cfg_snapshot() { LOCK(); ensure_loaded(); DspConfig c = s_active; UNLOCK(); return c; }
size_t dsp_item_count(unsigned group) {
    size_t n = 0;
    for (const auto &it : kItems) if (it.group == group) ++n;
    return n;
}
const char *dsp_group_name(unsigned group) { return group < DG_COUNT ? kGroupNames[group] : ""; }
const char *dsp_item_name(unsigned g, size_t i) { const Item *it = find(g, i); return it ? it->name : ""; }
const char *dsp_item_desc(unsigned g, size_t i) { const Item *it = find(g, i); return it ? it->desc : ""; }

void dsp_item_value(unsigned g, size_t i, char *out, size_t n) {
    const Item *it = find(g, i);
    if (!it || !n) { if (n) out[0] = 0; return; }
    const DspConfig c = dsp_cfg_snapshot();
    const uint8_t v = field(const_cast<DspConfig &>(c), it->off);
    switch (it->fmt) {
        case F_NAME: snprintf(out, n, "%s", it->names[v - it->lo]); break;
        case F_NUM: snprintf(out, n, "%u", v); break;
        case F_OFFNUM: if (v) snprintf(out, n, "%u/10", v); else snprintf(out, n, "OFF"); break;
        case F_BIN_OFFSET: snprintf(out, n, "%+d kHz", ((int)v - 120) * 625 / 10); break;
        case F_BIN_WIDTH: snprintf(out, n, "%d kHz", (int)v * 625 / 10); break;
        case F_NOTCH: if (!v) snprintf(out, n, "OFF"); else snprintf(out, n, "%+d kHz", ((int)v - 1 - 120) * 625 / 10); break;
        case F_ALPHA: snprintf(out, n, "1/%u", 1u << v); break;
        case F_LEVELS4: snprintf(out, n, "%u LVL", v * 4u); break;
    }
}
void dsp_item_adjust(unsigned g, size_t i, int dir, unsigned magnitude) {
    const Item *it = find(g, i);
    if (!it) return;
    LOCK();
    ensure_loaded();
    uint8_t &v = field(s_active, it->off);
    const unsigned mag = it->fmt == F_NAME ? 1u : (magnitude ? magnitude : 1u); // enumerations never skip entries
    int next = (int)v + dir * (int)it->step * (int)mag;
    const bool wrap = it->fmt == F_NAME; // enumerations wrap, numbers saturate
    if (wrap) { const int span = it->hi - it->lo + 1; next = (int)it->lo + (((next - it->lo) % span) + span) % span; }
    else if (next < it->lo) next = it->lo; else if (next > it->hi) next = it->hi;
    if (next != v) { v = (uint8_t)next; s_dirty = true; }
    UNLOCK();
}
void dsp_item_reset(unsigned g, size_t i) {
    const Item *it = find(g, i);
    if (!it) return;
    LOCK(); ensure_loaded();
    if (field(s_active, it->off) != it->def) { field(s_active, it->off) = it->def; s_dirty = true; }
    UNLOCK();
}
unsigned dsp_group_modified(unsigned group) {
    const DspConfig c = dsp_cfg_snapshot();
    unsigned n = 0;
    for (const auto &it : kItems) if (it.group == group && field(const_cast<DspConfig &>(c), it.off) != it.def) ++n;
    return n;
}

// Values measured offline in Python from the exact N=256 periodic windows used by the firmware
// (see TEST_DSP.md): highest sidelobe, equivalent noise bandwidth, worst-case scalloping, 256/coherent gain.
const DspWindowInfo &dsp_window_info(uint8_t id) {
    static const DspWindowInfo k[WIN_COUNT] = {
        {"RECT", -13, 100, 392, 256},      {"HANN", -31, 150, 142, 512},     {"HAMMING", -43, 136, 175, 474},
        {"BLACKMAN", -58, 173, 110, 610},  {"BLK-HARRIS", -92, 200, 83, 713}, {"FLAT-TOP", -93, 377, 1, 1187},
        {"BARTLETT", -26, 133, 182, 512},  {"TRIANGULAR", -26, 132, 185, 508}, {"LEGACY", 0, 0, 0, 256}};
    return k[id < WIN_COUNT ? id : WIN_COUNT - 1];
}

#ifdef ESP_PLATFORM
namespace {
constexpr uint32_t kMagic = 0x44535041; // DSPA
constexpr uint16_t kVersion = 1;
struct Record { uint32_t magic; uint16_t version; uint16_t size; DspConfig value; };
}
void dsp_cfg_load() {
    LOCK(); s_active = dsp_cfg_defaults(); s_loaded = true; s_dirty = false; UNLOCK();
    nvs_handle_t h;
    if (nvs_open("dsp_adv", NVS_READONLY, &h) != ESP_OK) return;
    Record r{};
    size_t sz = sizeof(r);
    const esp_err_t e = nvs_get_blob(h, "cfg", &r, &sz);
    nvs_close(h);
    if (e != ESP_OK || sz != sizeof(r) || r.magic != kMagic || r.version != kVersion || r.size != sizeof(DspConfig)) return;
    dsp_cfg_sanitize(r.value);
    LOCK(); s_active = r.value; UNLOCK();
}
bool dsp_cfg_save_if_dirty() {
    LOCK(); const bool dirty = s_dirty; const DspConfig c = s_active; s_dirty = false; UNLOCK();
    if (!dirty) return true;
    nvs_handle_t h;
    if (nvs_open("dsp_adv", NVS_READWRITE, &h) != ESP_OK) { LOCK(); s_dirty = true; UNLOCK(); return false; }
    Record r{kMagic, kVersion, (uint16_t)sizeof(DspConfig), c};
    esp_err_t e = nvs_set_blob(h, "cfg", &r, sizeof(r));
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    if (e != ESP_OK) { LOCK(); s_dirty = true; UNLOCK(); }
    return e == ESP_OK;
}
#else
void dsp_cfg_load() { s_active = dsp_cfg_defaults(); s_loaded = true; }
bool dsp_cfg_save_if_dirty() { s_dirty = false; return true; }
#endif
}
