#include "include/rf_range.hpp"
#include "include/geo.hpp"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace marauder {
const char *range_status_text(RangeStatus s) {
    switch (s) {
        case RangeStatus::OK: return "OK";
        case RangeStatus::REJECT_SOURCE: return "NOT A RF TIME-OF-FLIGHT SOURCE";
        case RangeStatus::REJECT_NOT_HW_TIMESTAMPED: return "NOT HARDWARE TIMESTAMPED";
        case RangeStatus::REJECT_UNSYNCED: return "ONE-WAY NEEDS SYNCED CLOCKS";
        case RangeStatus::REJECT_NEGATIVE: return "TIME <= DELAYS (NEGATIVE)";
        case RangeStatus::REJECT_NOT_FINITE: return "INVALID NUMBER";
        case RangeStatus::REJECT_NO_UNCERTAINTY: return "UNCERTAINTY MISSING";
        case RangeStatus::REJECT_TOO_FAR: return "OUT OF RANGE (>1000 KM)";
        case RangeStatus::REJECT_NO_OBSERVER: return "NO OBSERVER POSITION";
        case RangeStatus::STORE_FULL: return "STORE FULL";
    }
    return "?";
}

RangeStatus range_compute(const RangeInput &in, RangeEstimate *out) {
    memset(out, 0, sizeof(*out));
    if (in.source != RangeSource::HW_ROUND_TRIP && in.source != RangeSource::HW_ONE_WAY_SYNCED) return RangeStatus::REJECT_SOURCE;
    if (!in.hw_timestamped) return RangeStatus::REJECT_NOT_HW_TIMESTAMPED;
    const double v[] = {in.time_s, in.turnaround_s, in.cal_offset_s, in.time_unc_s, in.cal_unc_s, in.sync_unc_s};
    for (double x : v) if (!isfinite(x)) return RangeStatus::REJECT_NOT_FINITE;
    if (!(in.time_unc_s > 0) || in.cal_unc_s < 0) return RangeStatus::REJECT_NO_UNCERTAINTY;
    const bool rt = in.source == RangeSource::HW_ROUND_TRIP;
    double flight, var;
    if (rt) {
        flight = (in.time_s - in.turnaround_s - in.cal_offset_s) / 2.0;                       // out and back: halve
        var = (in.time_unc_s * in.time_unc_s + in.cal_unc_s * in.cal_unc_s) / 4.0;
    } else {
        if (!(in.sync_unc_s > 0)) return RangeStatus::REJECT_UNSYNCED;
        flight = in.time_s - in.cal_offset_s;
        var = in.time_unc_s * in.time_unc_s + in.cal_unc_s * in.cal_unc_s + in.sync_unc_s * in.sync_unc_s;
    }
    if (flight < 0) return RangeStatus::REJECT_NEGATIVE;
    const double d = geo::kC_mps * flight, sd = geo::kC_mps * sqrt(var);
    if (d > 1.0e6) return RangeStatus::REJECT_TOO_FAR;
    out->valid = true; out->distance_m = d; out->sigma_m = sd;
    out->ring_outer_m = d + 2.0 * sd; out->ring_inner_m = d - 2.0 * sd < 0 ? 0 : d - 2.0 * sd;
    return RangeStatus::OK;
}

RangeStatus RangeStore::submit(const RangeInput &in, double lat, double lon, const char *label, uint32_t now_ms) {
    if (!geo::coords_valid(lat, lon)) return RangeStatus::REJECT_NO_OBSERVER;
    RangeEstimate e; const RangeStatus s = range_compute(in, &e);
    if (s != RangeStatus::OK) return s;
    RangeRing &r = r_[head_];
    memset(&r, 0, sizeof(r)); r.used = true; snprintf(r.label, sizeof(r.label), "%s", label ? label : "");
    r.obs_lat = lat; r.obs_lon = lon; r.est = e; r.stamp_ms = now_ms;
    head_ = (head_ + 1) % kRangeRings; if (count_ < kRangeRings) ++count_;
    return RangeStatus::OK;
}
size_t RangeStore::snapshot(RangeRing *out, size_t max) const {
    size_t n = 0; const size_t first = (head_ + kRangeRings - count_) % kRangeRings;
    for (size_t i = 0; i < count_ && n < max; ++i) out[n++] = r_[(first + i) % kRangeRings];
    return n;
}

size_t range_import_csv(osm::FileIo *io, const char *path, RangeStore &store, uint32_t now_ms, size_t *rejected, RangeStatus *last_reject) {
    size_t ok = 0, bad = 0; if (rejected) *rejected = 0; if (last_reject) *last_reject = RangeStatus::OK;
    void *h = io->open_read(path);
    if (!h) return 0;
    char line[160]; size_t len = 0; char buf[256]; size_t got;
    auto row = [&](char *l) {
        if (!*l || *l == '#' || !strncmp(l, "label,", 6)) return;
        char *f[11]; size_t n = 0; char *p = l; f[n++] = p;
        while (*p && n < 11) { if (*p == ',') { *p = 0; f[n++] = p + 1; } ++p; }
        RangeStatus st = RangeStatus::OK;
        if (n != 11) st = RangeStatus::REJECT_NOT_FINITE;
        else {
            RangeInput in{}; const bool rt = !strcmp(f[1], "RT"), ow = !strcmp(f[1], "OW");
            in.source = rt ? RangeSource::HW_ROUND_TRIP : ow ? RangeSource::HW_ONE_WAY_SYNCED : RangeSource::UNKNOWN;
            in.time_s = atof(f[2]) * 1e-9; in.turnaround_s = atof(f[3]) * 1e-9; in.cal_offset_s = atof(f[4]) * 1e-9;
            in.time_unc_s = atof(f[5]) * 1e-9; in.cal_unc_s = atof(f[6]) * 1e-9; in.sync_unc_s = atof(f[7]) * 1e-9;
            in.hw_timestamped = atoi(f[8]) == 1;
            st = store.submit(in, atof(f[9]), atof(f[10]), f[0], now_ms);
        }
        if (st == RangeStatus::OK) ++ok; else { ++bad; if (last_reject) *last_reject = st; }
    };
    while ((got = io->read(h, buf, sizeof(buf))) > 0) {
        for (size_t i = 0; i < got; ++i) {
            const char c = buf[i];
            if (c == '\n' || c == '\r') { line[len] = 0; if (len) row(line); len = 0; }
            else if (len < sizeof(line) - 1) line[len++] = c;
            else len = sizeof(line);                      // overlong line: ignore the rest of it
            if (len == sizeof(line)) { len = 0; while (false) {} }
        }
    }
    if (len && len < sizeof(line)) { line[len] = 0; row(line); }
    io->close(h);
    if (rejected) *rejected = bad;
    return ok;
}

bool range_hardware_supported(const char **why) {
    if (why) *why = "NO HW TIME-OF-FLIGHT: ESP32 RF PATH HAS NO HW TIMESTAMPS/FTM";
    return false;
}

#ifndef RANGE_HOST
RangeStore &range_store() { static RangeStore s; return s; }
#endif
}
