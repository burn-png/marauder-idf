#include "include/rf_range.hpp"
#include <stdio.h>
#include <string.h>
#include <math.h>
using namespace marauder;
static int fails = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); ++fails; } } while (0)
#define NEAR(a, b, t) do { double _a=(a), _b=(b); if (fabs(_a-_b) > (t)) { printf("FAIL %s:%d  %s=%.6f expected %.6f\n", __FILE__, __LINE__, #a, _a, _b); ++fails; } } while (0)
struct MemIo : osm::FileIo {
    const char *data; size_t pos = 0;
    explicit MemIo(const char *d) : data(d) {}
    void *open_read(const char *) override { pos = 0; return this; }
    void *open_write(const char *, bool) override { return nullptr; }
    size_t read(void *, void *b, size_t n) override { size_t r = strlen(data) - pos; if (r > n) r = n; memcpy(b, data + pos, r); pos += r; return r; }
    bool write(void *, const void *, size_t) override { return false; }
    bool seek(void *, size_t) override { return false; }
    void close(void *) override {}
    size_t size(const char *) override { return 0; }
    bool remove(const char *) override { return false; }
    bool mkdir(const char *) override { return true; }
};
int main() {
    const double c = 299792458.0;
    RangeEstimate e; RangeInput in{};
    // round trip: 2 us total, 0.5 us responder turnaround, 20 ns calibration -> (2000-500-20)/2 ns = 740 ns -> 221.85 m
    in.source = RangeSource::HW_ROUND_TRIP; in.hw_timestamped = true; in.time_s = 2000e-9; in.turnaround_s = 500e-9; in.cal_offset_s = 20e-9; in.time_unc_s = 10e-9; in.cal_unc_s = 4e-9;
    CHECK(range_compute(in, &e) == RangeStatus::OK && e.valid);
    NEAR(e.distance_m, c * 740e-9, 1e-6); NEAR(e.sigma_m, c * sqrt(100e-18 + 16e-18) / 2.0, 1e-6);
    NEAR(e.ring_outer_m - e.distance_m, 2 * e.sigma_m, 1e-6);
    // one way: 1 us, 10 ns calibration, sync 5 ns, timing 3 ns
    in = RangeInput{}; in.source = RangeSource::HW_ONE_WAY_SYNCED; in.hw_timestamped = true; in.time_s = 1000e-9; in.cal_offset_s = 10e-9; in.time_unc_s = 3e-9; in.sync_unc_s = 5e-9;
    CHECK(range_compute(in, &e) == RangeStatus::OK); NEAR(e.distance_m, c * 990e-9, 1e-6); NEAR(e.sigma_m, c * sqrt(9e-18 + 25e-18), 1e-6);
    // one-way with no stated sync accuracy is refused
    in.sync_unc_s = 0; CHECK(range_compute(in, &e) == RangeStatus::REJECT_UNSYNCED && !e.valid);
    in.sync_unc_s = 5e-9;
    // sources that are not RF time of flight are refused, however plausible the numbers
    for (RangeSource s : {RangeSource::SOFTWARE_TIMESTAMP, RangeSource::PACKET_ARRIVAL, RangeSource::UNKNOWN}) { in.source = s; CHECK(range_compute(in, &e) == RangeStatus::REJECT_SOURCE && !e.valid); }
    in.source = RangeSource::HW_ONE_WAY_SYNCED; in.hw_timestamped = false; CHECK(range_compute(in, &e) == RangeStatus::REJECT_NOT_HW_TIMESTAMPED);
    in.hw_timestamped = true;
    in.time_s = 5e-9; in.cal_offset_s = 10e-9; CHECK(range_compute(in, &e) == RangeStatus::REJECT_NEGATIVE);   // delays exceed the measurement: not clamped to zero
    in.time_s = NAN; CHECK(range_compute(in, &e) == RangeStatus::REJECT_NOT_FINITE);
    in.time_s = 1e-6; in.time_unc_s = 0; CHECK(range_compute(in, &e) == RangeStatus::REJECT_NO_UNCERTAINTY);
    in.time_unc_s = 1e-9; in.time_s = 0.01; in.cal_offset_s = 0; CHECK(range_compute(in, &e) == RangeStatus::REJECT_TOO_FAR);
    // inner ring clamps at zero when the uncertainty exceeds the distance
    in = RangeInput{}; in.source = RangeSource::HW_ROUND_TRIP; in.hw_timestamped = true; in.time_s = 20e-9; in.time_unc_s = 50e-9;
    CHECK(range_compute(in, &e) == RangeStatus::OK && e.ring_inner_m == 0 && e.ring_outer_m > e.distance_m);
    // store: observer required, ring buffer wraps, newest last
    RangeStore st; in = RangeInput{}; in.source = RangeSource::HW_ROUND_TRIP; in.hw_timestamped = true; in.time_s = 2000e-9; in.time_unc_s = 10e-9;
    CHECK(st.submit(in, 0, 0, "x", 1) == RangeStatus::REJECT_NO_OBSERVER);
    for (int i = 0; i < 12; ++i) { in.time_s = (2000 + i) * 1e-9; CHECK(st.submit(in, 29.7, -95.3, "t", i) == RangeStatus::OK); }
    RangeRing r[kRangeRings]; CHECK(st.snapshot(r, kRangeRings) == kRangeRings); CHECK(r[kRangeRings - 1].stamp_ms == 11 && r[0].stamp_ms == 4);
    // CSV import: header, comment, good rows, and rows that must be rejected (software source, wrong column count, no HW flag)
    MemIo io("label,mode,time_ns,turnaround_ns,cal_ns,time_unc_ns,cal_unc_ns,sync_unc_ns,hw_timestamped,obs_lat,obs_lon\n# comment\n"
             "A,RT,2000,500,20,10,4,0,1,29.76,-95.37\nB,OW,1000,0,10,3,0,5,1,29.76,-95.37\nC,PK,1000,0,0,3,0,5,1,29.76,-95.37\nD,RT,2000,0,0,10,0,0,0,29.76,-95.37\nE,RT,1,2\nF,OW,1000,0,0,3,0,0,1,29.76,-95.37");
    RangeStore s2; size_t rej = 0; RangeStatus last = RangeStatus::OK;
    CHECK(range_import_csv(&io, "x", s2, 7, &rej, &last) == 2); CHECK(rej == 4); CHECK(s2.count() == 2);
    const char *why = nullptr; CHECK(!range_hardware_supported(&why) && why);
    printf(fails ? "FAILED (%d)\n" : "ALL PASS\n", fails);
    return fails != 0;
}
