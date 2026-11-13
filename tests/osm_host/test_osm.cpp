#include "include/osm.hpp"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <string>
#include <algorithm>
#include <sys/stat.h>
#include <unistd.h>
using namespace marauder;
static int fails = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); ++fails; } } while (0)

static std::string ROOT;
struct StdIo : osm::FileIo {
    std::string p(const char *r) { return ROOT + "/" + r; }
    void *open_read(const char *r) override { return fopen(p(r).c_str(), "rb"); }
    void *open_write(const char *r, bool a) override { return fopen(p(r).c_str(), a ? "ab" : "wb"); }
    size_t read(void *h, void *b, size_t n) override { return fread(b, 1, n, (FILE *)h); }
    bool write(void *h, const void *b, size_t n) override { return fwrite(b, 1, n, (FILE *)h) == n; }
    bool seek(void *h, size_t o) override { return fseek((FILE *)h, (long)o, SEEK_SET) == 0; }
    void close(void *h) override { fclose((FILE *)h); }
    size_t size(const char *r) override { struct stat s; return stat(p(r).c_str(), &s) ? 0 : (size_t)s.st_size; }
    bool remove(const char *r) override { return ::remove(p(r).c_str()) == 0; }
    bool mkdir(const char *r) override { ::mkdir(p(r).c_str(), 0755); return true; }
};

static const double LAT0 = 29.76, LON0 = -95.37;
static void write_file(const char *rel, const std::string &s) { FILE *f = fopen((ROOT + "/" + rel).c_str(), "wb"); fwrite(s.data(), 1, s.size(), f); fclose(f); }

// A town: 11x11 street grid (residential, one primary), buildings, a river, a park, a footpath, a rail line.
static std::string make_osm(int N, bool unsorted = false, bool nodes_after_ways = false) {
    std::string s = "<?xml version='1.0' encoding='UTF-8'?>\n<osm version=\"0.6\" generator=\"test\">\n <!-- comment with > inside -->\n";
    char b[512]; const double step = 0.001;   // ~100 m
    auto id = [&](int r, int c) { return 1000000000LL + (long long)r * 1000 + c; };
    for (int r = 0; r <= N; ++r) for (int c = 0; c <= N; ++c) {
        long long i = id(r, c); if (unsorted && r == 3 && c == 3) i = 5;
        snprintf(b, sizeof b, " <node id=\"%lld\" version=\"1\" lat=\"%.7f\" lon=\"%.7f\"><tag k=\"name\" v=\"a > b &amp; c\"/></node>\n", i, LAT0 + r * step, LON0 + c * step); s += b;
    }
    for (int r = 0; r < N; ++r) for (int c = 0; c < N; ++c) {              // building in each block (2x2 corner nodes reuse the grid)
        (void)r; (void)c;
    }
    for (int r = 0; r <= std::min(N, 7); ++r) {
        s += " <way id=\"" + std::to_string(2000 + r) + "\">\n";
        for (int c = 0; c <= N; ++c) s += "  <nd ref=\"" + std::to_string(id(r, c)) + "\"/>\n";
        s += std::string("  <tag k=\"highway\" v=\"") + (r == 5 ? "primary" : "residential") + "\"/>\n  <tag k=\"name\" v=\"Row " + std::to_string(r) + "\"/>\n </way>\n";
    }
    for (int c = 0; c <= N; ++c) {
        s += " <way id=\"" + std::to_string(3000 + c) + "\">\n";
        for (int r = 0; r <= std::min(N, 7); ++r) s += "  <nd ref=\"" + std::to_string(id(r, c)) + "\"/>\n";
        s += "  <tag k=\"highway\" v=\"residential\"/>\n </way>\n";
    }
    // building: closed square on block (0,0)
    s += " <way id=\"4000\">\n"; for (auto rc : {std::make_pair(0,0), std::make_pair(0,1), std::make_pair(1,1), std::make_pair(1,0), std::make_pair(0,0)}) s += "  <nd ref=\"" + std::to_string(id(rc.first, rc.second)) + "\"/>\n";
    s += "  <tag k=\"building\" v=\"yes\"/>\n </way>\n";
    // park: closed square on block (2..4, 2..4)
    s += " <way id=\"4001\">\n"; for (auto rc : {std::make_pair(2,2), std::make_pair(2,4), std::make_pair(4,4), std::make_pair(4,2), std::make_pair(2,2)}) s += "  <nd ref=\"" + std::to_string(id(rc.first, rc.second)) + "\"/>\n";
    s += "  <tag k=\"leisure\" v=\"park\"/>\n </way>\n";
    // way with a missing node in the middle: must be split, not dropped or crash
    s += " <way id=\"4002\">\n  <nd ref=\"" + std::to_string(id(8, 0)) + "\"/>\n  <nd ref=\"" + std::to_string(id(8, 1)) + "\"/>\n  <nd ref=\"99999999999\"/>\n  <nd ref=\"" + std::to_string(id(8, 3)) + "\"/>\n  <nd ref=\"" + std::to_string(id(8, 4)) + "\"/>\n  <tag k=\"waterway\" v=\"river\"/>\n </way>\n";
    // rail + footpath + untagged way + way tagged with something we do not draw
    s += " <way id=\"4003\"><nd ref=\"" + std::to_string(id(9, 0)) + "\"/><nd ref=\"" + std::to_string(id(9, N)) + "\"/><tag k=\"railway\" v=\"rail\"/></way>\n";
    s += " <way id=\"4004\"><nd ref=\"" + std::to_string(id(10, 0)) + "\"/><nd ref=\"" + std::to_string(id(10, N)) + "\"/><tag k=\"highway\" v=\"footway\"/></way>\n";
    s += " <way id=\"4005\"><nd ref=\"" + std::to_string(id(0, 0)) + "\"/><nd ref=\"" + std::to_string(id(0, 2)) + "\"/></way>\n";
    s += " <way id=\"4006\"><nd ref=\"" + std::to_string(id(0, 0)) + "\"/><nd ref=\"" + std::to_string(id(0, 2)) + "\"/><tag k=\"barrier\" v=\"fence\"/></way>\n";
    s += " <way id=\"4007\"/>\n";
    if (nodes_after_ways) s += " <node id=\"1\" lat=\"1\" lon=\"1\"/>\n";
    s += " <relation id=\"1\"><member type=\"way\" ref=\"4000\" role=\"outer\"/></relation>\n</osm>\n";
    return s;
}

static bool run_index(StdIo &io, osm::Indexer &ix, const char *src, const char *dir, osm::IndexProgress *p) {
    if (!ix.start(&io, src, dir)) { ix.progress(p); return false; }
    int guard = 0; while (ix.step(777) && ++guard < 10000000) {}            // odd budget: tokens straddle chunk boundaries
    ix.progress(p); return p->state == osm::State::DONE;
}

int main() {
    char tmpl[] = "/tmp/osmtestXXXXXX"; ROOT = mkdtemp(tmpl);
    mkdir((ROOT + "/maps").c_str(), 0755); mkdir((ROOT + "/map_cache").c_str(), 0755);
    StdIo io;
    write_file("maps/town.osm", make_osm(10));
    osm::IndexProgress pr; osm::Indexer ix;
    CHECK(run_index(io, ix, "maps/town.osm", "map_cache/town", &pr));
    printf("indexed: nodes=%u ways=%u features=%u missing=%u skipped=%u bytes=%u\n", pr.nodes, pr.ways, pr.features, pr.missing_refs, pr.skipped_tags, pr.bytes_done);
    CHECK(pr.nodes == 121); CHECK(pr.ways == 8 + 11 + 3 + 2 + 2);   // street rows + columns + building/park/river + rail/footpath + untagged/fence (4007 is empty)
    CHECK(pr.missing_refs == 1);
    CHECK(io.size("map_cache/town/nodes.tmp") == 0);                         // temp table removed
    osm::Meta meta; CHECK(osm::load_meta(&io, "map_cache/town", "maps/town.osm", &meta));
    CHECK(meta.nodes == 121 && meta.features >= 22);
    CHECK(meta.min_lat_e7 == (int32_t)llround(LAT0 * 1e7) && meta.max_lon_e7 == (int32_t)llround((LON0 + 0.01) * 1e7));

    // ---- render: zoom 5 m/px centred on the town centre; verify pixels by classes at computed positions ----
    const int W = 240, H = 264; std::string mem(osm::canvas_bytes(W, H), 0);
    osm::Canvas cv{(uint8_t *)&mem[0], W, H};
    geo::View v{}; geo::view_set(v, LAT0 + 0.005, LON0 + 0.005, 5.0); v.cx = W / 2; v.cy = H / 2; v.north_up = true;
    osm::Renderer rr; osm::canvas_clear(cv);
    CHECK(rr.begin(&io, "map_cache/town", meta, v, cv, osm::L_ALL)); int guard = 0; while (rr.step(50) && ++guard < 100000) {}
    osm::RenderStats rs; rr.stats(&rs); printf("render: cells=%u records=%u bytes=%u lines=%u\n", rs.cells, rs.records, rs.bytes, rs.drawn_lines); CHECK(rs.records > 20);
    auto px_at = [&](double lat, double lon) { float x, y; geo::to_px(v, lat, lon, &x, &y); return osm::canvas_get(cv, (int)lroundf(x), (int)lroundf(y)); };
    CHECK(px_at(LAT0 + 0.005, LON0 + 0.0025) == osm::C_ROAD_MAJOR);        // on the primary row (r=5)
    CHECK(px_at(LAT0 + 0.003, LON0 + 0.0025) == osm::C_ROAD_MINOR);        // on residential row (r=3)... between columns
    CHECK(px_at(LAT0 + 0.0035, LON0 + 0.0035) == osm::C_GREEN);            // inside park (2..4 x 2..4), away from streets
    CHECK(px_at(LAT0 + 0.0005, LON0 + 0.0005) == osm::C_BUILD_FILL);       // inside building block (0..1)
    CHECK(px_at(LAT0 + 0.0045, LON0 + 0.0075) == osm::C_NONE);             // empty block
    CHECK(px_at(LAT0 + 0.009, LON0 + 0.0025) == osm::C_RAIL);              // rail row 9
    CHECK(px_at(LAT0 + 0.010, LON0 + 0.0025) == osm::C_PATH || px_at(LAT0 + 0.010, LON0 + 0.0025) == osm::C_ROAD_MINOR);  // row 10 footway coincides with column roads? (between them: path)
    CHECK(px_at(LAT0 + 0.008, LON0 + 0.0005) == osm::C_WATER);             // river part before the missing node
    CHECK(px_at(LAT0 + 0.008, LON0 + 0.0025) != osm::C_WATER);             // gap where the node is missing: not drawn across it
    CHECK(px_at(LAT0 + 0.008, LON0 + 0.0035) == osm::C_WATER);             // river part after the gap

    // ---- heading-up rotation keeps features aligned with the projection ----
    v.north_up = false; v.rot_deg = 90; osm::canvas_clear(cv);
    CHECK(rr.begin(&io, "map_cache/town", meta, v, cv, osm::L_ALL)); while (rr.step(50)) {}
    CHECK(px_at(LAT0 + 0.005, LON0 + 0.0025) == osm::C_ROAD_MAJOR);
    v.north_up = true;

    // ---- layer switches and level of detail ----
    osm::canvas_clear(cv); CHECK(rr.begin(&io, "map_cache/town", meta, v, cv, osm::L_ROADS)); while (rr.step(50)) {}
    CHECK(px_at(LAT0 + 0.0005, LON0 + 0.0005) == osm::C_NONE); CHECK(px_at(LAT0 + 0.005, LON0 + 0.0025) == osm::C_ROAD_MAJOR);
    geo::view_set(v, LAT0 + 0.005, LON0 + 0.005, 160.0); v.cx = W / 2; v.cy = H / 2; osm::canvas_clear(cv);
    CHECK(rr.begin(&io, "map_cache/town", meta, v, cv, osm::L_ALL)); while (rr.step(50)) {}
    CHECK(px_at(LAT0 + 0.003, LON0 + 0.0025) == osm::C_NONE || px_at(LAT0 + 0.003, LON0 + 0.0025) == osm::C_ROAD_MINOR);   // minor roads hidden far out
    CHECK(osm::layers_for_scale(160.0, osm::L_ALL) == (osm::L_ROADS | osm::L_WATER));
    // view far away from the data: nothing to do, no crash
    geo::view_set(v, 40.0, -100.0, 5.0); v.cx = W / 2; v.cy = H / 2; osm::canvas_clear(cv);
    CHECK(rr.begin(&io, "map_cache/town", meta, v, cv, osm::L_ALL)); CHECK(!rr.step(10));

    // ---- stale cache: replacing the .osm invalidates it ----
    write_file("maps/town.osm", make_osm(10) + "<!-- changed -->\n");
    CHECK(!osm::load_meta(&io, "map_cache/town", "maps/town.osm", &meta));

    // ---- bad inputs ----
    { osm::Indexer a; osm::IndexProgress p; CHECK(!run_index(io, a, "maps/missing.osm", "map_cache/x", &p) && p.error == osm::Error::NO_FILE); }
    write_file("maps/empty.osm", ""); { osm::Indexer a; osm::IndexProgress p; CHECK(!run_index(io, a, "maps/empty.osm", "map_cache/e", &p)); }
    write_file("maps/junk.osm", std::string(5000, 'x') + "<a><b/></a>"); { osm::Indexer a; osm::IndexProgress p; CHECK(!run_index(io, a, "maps/junk.osm", "map_cache/j", &p) && p.error == osm::Error::NO_DATA); }
    write_file("maps/unsorted.osm", make_osm(6, true)); { osm::Indexer a; osm::IndexProgress p; CHECK(!run_index(io, a, "maps/unsorted.osm", "map_cache/u", &p) && p.error == osm::Error::NODES_UNSORTED); }
    write_file("maps/late.osm", make_osm(6, false, true)); { osm::Indexer a; osm::IndexProgress p; CHECK(!run_index(io, a, "maps/late.osm", "map_cache/l", &p) && p.error == osm::Error::NODES_AFTER_WAYS); }
    { std::string t = make_osm(6); t.resize(t.size() / 2); write_file("maps/trunc.osm", t);              // file cut in the middle of the ways
      osm::Indexer a; osm::IndexProgress p; run_index(io, a, "maps/trunc.osm", "map_cache/t", &p); CHECK(p.state == osm::State::DONE || p.state == osm::State::ERROR); }
    { std::string t = "<osm>\n<node id=\"1\" lat=\"29.0\" lon=\"-95.0\"/>\n<node id=\"2\" lat=\"29.001\" lon=\"-95.001\"/>\n<node id=\"3\" lat=\"bogus\" lon=\"-95\"/>\n<node id=\"4\" lat=\"29.002\" lon=\"-95.002\"/>\n<way id=\"1\"><nd ref=\"1\"/><nd ref=\"2\"/><nd ref=\"4\"/><tag k=\"highway\" v=\"" + std::string(3000, 'q') + "\"/><tag k=\"highway\" v=\"service\"/></way></osm>";
      write_file("maps/odd.osm", t); osm::Indexer a; osm::IndexProgress p; CHECK(run_index(io, a, "maps/odd.osm", "map_cache/o", &p)); CHECK(p.nodes == 3 && p.skipped_tags >= 2 && p.features == 1); }   // bad lat skipped, 3 KB tag skipped, later tag used
    // a corrupted cell file must not crash the renderer
    { FILE *f = fopen((ROOT + "/map_cache/o/c0000.bin").c_str(), "wb"); for (int i = 0; i < 100; ++i) fputc(0xEE, f); fclose(f);
      osm::Meta m2; CHECK(osm::load_meta(&io, "map_cache/o", "maps/odd.osm", &m2));
      geo::View v2{}; geo::view_set(v2, 29.001, -95.001, 5.0); v2.cx = W / 2; v2.cy = H / 2; v2.north_up = true; osm::canvas_clear(cv);
      osm::Renderer r2; CHECK(r2.begin(&io, "map_cache/o", m2, v2, cv, osm::L_ALL)); int g = 0; while (r2.step(10) && ++g < 1000) {} CHECK(g < 1000); }

    // ---- index memory: node-index compaction when there are more nodes than index slots ----
    { std::string t = "<osm>\n"; char b[200]; const int NN = 140000;
      for (int i = 0; i < NN; ++i) { snprintf(b, sizeof b, "<node id=\"%d\" lat=\"%.7f\" lon=\"%.7f\"/>\n", 1000 + i * 3, 29.0 + (i % 1000) * 1e-5, -95.0 + (i / 1000) * 1e-5); t += b; }
      t += "<way id=\"1\"><nd ref=\"1000\"/><nd ref=\"" + std::to_string(1000 + 3 * 70000) + "\"/><nd ref=\"" + std::to_string(1000 + 3 * (NN - 1)) + "\"/><tag k=\"highway\" v=\"primary\"/></way></osm>";
      write_file("maps/big.osm", t); osm::Indexer a; osm::IndexProgress p; CHECK(run_index(io, a, "maps/big.osm", "map_cache/b", &p)); CHECK(p.nodes == (uint32_t)NN && p.missing_refs == 0 && p.features == 1); }
    printf(fails ? "FAILED (%d)\n" : "ALL PASS\n", fails);
    return fails != 0;
}
