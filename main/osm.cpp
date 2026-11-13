#include "include/osm.hpp"
#include <new>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

namespace marauder::osm {
namespace {
constexpr uint32_t kMagic = 0x50414D44u;     // "DMAP"
constexpr uint32_t kVersion = 1;
constexpr size_t kChunk = 1024;
constexpr size_t kTagMax = 512;
constexpr size_t kIdxCap = 512;              // sparse node index entries (4 KB)
constexpr size_t kWbuf = 1024;                // node-table write buffer
constexpr uint32_t kWin = 64;                 // node records per read window while resolving way references (1 KB)
constexpr size_t kCellBuf = 64;              // per-cell write buffer
constexpr size_t kNodeRec = 16;
constexpr size_t kRecHdr = 12;               // cls, flags, npts(2), lat0(4), lon0(4)
constexpr size_t kRecMax = kRecHdr + 4 * kRecPts;

bool eq(const char *s, size_t n, const char *lit) { const size_t m = strlen(lit); return n == m && memcmp(s, lit, m) == 0; }

// decimal degrees -> 1e-7 degrees without floating point; accepts "-95.3698123", "29", "1e-5" is rejected
bool parse_e7(const char *s, size_t n, int32_t *out) {
    size_t i = 0; bool neg = false; int64_t ip = 0, frac = 0; int fd = 0;
    if (i < n && (s[i] == '-' || s[i] == '+')) { neg = s[i] == '-'; ++i; }
    size_t digits = 0;
    while (i < n && s[i] >= '0' && s[i] <= '9') { ip = ip * 10 + (s[i] - '0'); if (ip > 1000) return false; ++i; ++digits; }
    if (i < n && s[i] == '.') {
        ++i;
        while (i < n && s[i] >= '0' && s[i] <= '9') { if (fd < 7) { frac = frac * 10 + (s[i] - '0'); ++fd; } ++i; ++digits; }
    }
    if (i != n || !digits) return false;
    while (fd < 7) { frac *= 10; ++fd; }
    int64_t v = ip * 10000000ll + frac;
    *out = (int32_t)(neg ? -v : v);
    return true;
}
bool parse_i64(const char *s, size_t n, int64_t *out) {
    size_t i = 0; bool neg = false; int64_t v = 0;
    if (i < n && s[i] == '-') { neg = true; ++i; }
    if (i >= n) return false;
    for (; i < n; ++i) { if (s[i] < '0' || s[i] > '9') return false; if (v > 900000000000000000ll) return false; v = v * 10 + (s[i] - '0'); }
    *out = neg ? -v : v; return true;
}

struct Attrs { const char *id, *lat, *lon, *ref, *k, *v; size_t idn, latn, lonn, refn, kn, vn; };
// One pass over `name a="x" b='y' .../`: pick out the attributes we use.
void parse_attrs(const char *p, const char *end, Attrs *a) {
    memset(a, 0, sizeof(*a));
    while (p < end && *p != ' ' && *p != '\t' && *p != '\n' && *p != '\r' && *p != '/') ++p;      // element name
    while (p < end) {
        while (p < end && (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r' || *p == '/')) ++p;
        const char *ks = p;
        while (p < end && *p != '=' && *p != ' ' && *p != '/' && *p != '\t' && *p != '\n' && *p != '\r') ++p;
        const size_t kn = (size_t)(p - ks);
        while (p < end && (*p == ' ' || *p == '\t')) ++p;
        if (p >= end || *p != '=') continue;
        ++p;
        while (p < end && (*p == ' ' || *p == '\t')) ++p;
        if (p >= end || (*p != '"' && *p != '\'')) continue;
        const char q = *p++; const char *vs = p;
        while (p < end && *p != q) ++p;
        if (p >= end) return;                                  // unterminated value: ignore the rest
        const size_t vn = (size_t)(p - vs); ++p;
        if (kn == 2 && ks[0] == 'i' && ks[1] == 'd') { a->id = vs; a->idn = vn; }
        else if (kn == 3 && !memcmp(ks, "lat", 3)) { a->lat = vs; a->latn = vn; }
        else if (kn == 3 && !memcmp(ks, "lon", 3)) { a->lon = vs; a->lonn = vn; }
        else if (kn == 3 && !memcmp(ks, "ref", 3)) { a->ref = vs; a->refn = vn; }
        else if (kn == 1 && ks[0] == 'k') { a->k = vs; a->kn = vn; }
        else if (kn == 1 && ks[0] == 'v') { a->v = vs; a->vn = vn; }
    }
}

// Priority for choosing between several matching tags on one way (higher wins).
int class_priority(uint8_t c) {
    switch (c) { case C_ROAD_MAJOR: return 9; case C_ROAD_MINOR: return 8; case C_PATH: return 7; case C_RAIL: return 6;
                 case C_BUILD: return 5; case C_WATER: return 4; case C_GREEN: return 3; default: return 0; }
}
uint8_t classify(const char *k, size_t kn, const char *v, size_t vn) {
    if (eq(k, kn, "highway")) {
        if (eq(v, vn, "motorway") || eq(v, vn, "trunk") || eq(v, vn, "primary") || eq(v, vn, "secondary") ||
            eq(v, vn, "motorway_link") || eq(v, vn, "trunk_link") || eq(v, vn, "primary_link") || eq(v, vn, "secondary_link")) return C_ROAD_MAJOR;
        if (eq(v, vn, "tertiary") || eq(v, vn, "tertiary_link") || eq(v, vn, "unclassified") || eq(v, vn, "residential") ||
            eq(v, vn, "living_street") || eq(v, vn, "service") || eq(v, vn, "road")) return C_ROAD_MINOR;
        if (eq(v, vn, "footway") || eq(v, vn, "path") || eq(v, vn, "cycleway") || eq(v, vn, "bridleway") || eq(v, vn, "steps") ||
            eq(v, vn, "pedestrian") || eq(v, vn, "track") || eq(v, vn, "corridor")) return C_PATH;
        return C_NONE;
    }
    if (eq(k, kn, "building")) return eq(v, vn, "no") ? C_NONE : C_BUILD;
    if (eq(k, kn, "waterway")) return (eq(v, vn, "river") || eq(v, vn, "stream") || eq(v, vn, "canal") || eq(v, vn, "drain") || eq(v, vn, "ditch")) ? C_WATER : C_NONE;
    if (eq(k, kn, "natural")) {
        if (eq(v, vn, "water") || eq(v, vn, "coastline") || eq(v, vn, "bay")) return C_WATER;
        if (eq(v, vn, "wood") || eq(v, vn, "scrub") || eq(v, vn, "grassland") || eq(v, vn, "heath")) return C_GREEN;
        return C_NONE;
    }
    if (eq(k, kn, "railway")) return (eq(v, vn, "rail") || eq(v, vn, "light_rail") || eq(v, vn, "subway") || eq(v, vn, "tram") || eq(v, vn, "narrow_gauge")) ? C_RAIL : C_NONE;
    if (eq(k, kn, "leisure")) return (eq(v, vn, "park") || eq(v, vn, "garden") || eq(v, vn, "playground") || eq(v, vn, "pitch") || eq(v, vn, "golf_course")) ? C_GREEN : C_NONE;
    if (eq(k, kn, "landuse")) return (eq(v, vn, "forest") || eq(v, vn, "grass") || eq(v, vn, "meadow") || eq(v, vn, "village_green") || eq(v, vn, "recreation_ground") || eq(v, vn, "cemetery")) ? C_GREEN :
                                     (eq(v, vn, "reservoir") || eq(v, vn, "basin")) ? C_WATER : C_NONE;
    return C_NONE;
}

void cell_path(char *out, size_t n, const char *dir, unsigned row, unsigned col) { snprintf(out, n, "%s/c%02u%02u.bin", dir, row, col); }
}  // namespace

const char *error_text(Error e) {
    switch (e) {
        case Error::NONE: return "OK";
        case Error::NO_FILE: return "NO .OSM FILE IN /maps";
        case Error::EMPTY: return "MAP FILE IS EMPTY";
        case Error::OPEN_FAILED: return "CANNOT OPEN MAP / CACHE";
        case Error::NO_MEMORY: return "NOT ENOUGH FREE RAM";
        case Error::TOO_MANY_NODES: return "TOO MANY NODES (2M MAX)";
        case Error::NODES_UNSORTED: return "NODES NOT SORTED BY ID";
        case Error::NODES_AFTER_WAYS: return "NOT STANDARD OSM ORDER";
        case Error::WRITE_FAILED: return "SD WRITE FAILED";
        case Error::NO_DATA: return "NO NODES FOUND (NOT OSM XML?)";
    }
    return "?";
}

// ================================================================== Indexer
struct Indexer::Impl {
    FileIo *io = nullptr; void *in = nullptr; void *nodes_w = nullptr; void *nodes_r = nullptr;
    char dir[72]{}; char nodes_path[96]{}; char src[96]{};
    State st = State::IDLE; Error err = Error::NONE;
    size_t src_size = 0, done = 0;
    enum Phase : uint8_t { P_CLEAN, P_PARSE, P_FLUSH, P_FINISH } phase = P_CLEAN;
    unsigned clean_cursor = 0;
    // tokenizer
    uint8_t *chunk = nullptr; char *tag = nullptr; size_t tag_len = 0; bool in_tag = false, overflow = false, in_comment = false, seen_root = false; char quote = 0;
    // node table + index
    uint8_t *wbuf = nullptr; size_t wlen = 0; uint64_t node_count = 0; int64_t last_id = INT64_MIN;
    int64_t *idx = nullptr; uint32_t idx_n = 0, stride = 64;
    int32_t min_lat = INT32_MAX, min_lon = INT32_MAX, max_lat = INT32_MIN, max_lon = INT32_MIN;
    uint8_t *blk = nullptr; int64_t blk_cached = -1; uint32_t blk_count = 0;
    // way state
    int64_t *refs = nullptr; int32_t *wlat = nullptr; int32_t *wlon = nullptr;
    uint32_t nref = 0; uint8_t cls = C_NONE; bool truncated = false, in_way = false;
    uint8_t *cbuf = nullptr; uint8_t cused[kGrid * kGrid]{};
    int32_t cell_dlat = 1, cell_dlon = 1;
    // stats
    uint32_t ways = 0, features = 0, missing = 0, trunc_ways = 0, skipped_tags = 0, bytes_out = 0;

    ~Impl() { release(); }
    void release() {
        if (in) { io->close(in); in = nullptr; }
        if (nodes_w) { io->close(nodes_w); nodes_w = nullptr; }
        if (nodes_r) { io->close(nodes_r); nodes_r = nullptr; }
        free(chunk); free(tag); free(wbuf); free(idx); free(blk); free(refs); free(wlat); free(wlon); free(cbuf);
        chunk = nullptr; tag = nullptr; wbuf = nullptr; idx = nullptr; blk = nullptr; refs = nullptr; wlat = wlon = nullptr; cbuf = nullptr;
    }
    void fail(Error e) { err = e; st = State::ERROR; release(); }

    bool alloc() {
        chunk = (uint8_t *)malloc(kChunk); tag = (char *)malloc(kTagMax + 1); wbuf = (uint8_t *)malloc(kWbuf);
        idx = (int64_t *)malloc(kIdxCap * sizeof(int64_t)); refs = (int64_t *)malloc(kMaxRefs * sizeof(int64_t));
        wlat = (int32_t *)malloc(kMaxRefs * 4); wlon = (int32_t *)malloc(kMaxRefs * 4); cbuf = (uint8_t *)malloc(kGrid * kGrid * kCellBuf);
        return chunk && tag && wbuf && idx && refs && wlat && wlon && cbuf;
    }

    // ---------------- node phase ----------------
    void push_index() {
        if (idx_n == kIdxCap) {                                  // out of index slots: keep every second one, double the stride
            for (uint32_t i = 0; i < kIdxCap / 2; ++i) idx[i] = idx[2 * i];
            idx_n = kIdxCap / 2; stride *= 2;
        }
    }
    bool flush_nodes() {
        if (!wlen) return true;
        const bool ok = io->write(nodes_w, wbuf, wlen); wlen = 0; return ok;
    }
    void add_node(int64_t id, int32_t lat, int32_t lon) {
        if (id <= last_id) { fail(Error::NODES_UNSORTED); return; }
        if (node_count >= kMaxNodes) { fail(Error::TOO_MANY_NODES); return; }
        last_id = id;
        if (node_count % stride == 0) { push_index(); if (node_count % stride == 0) idx[idx_n++] = id; }
        if (wlen + kNodeRec > kWbuf && !flush_nodes()) { fail(Error::WRITE_FAILED); return; }
        memcpy(wbuf + wlen, &id, 8); memcpy(wbuf + wlen + 8, &lat, 4); memcpy(wbuf + wlen + 12, &lon, 4); wlen += kNodeRec;
        ++node_count;
        if (lat < min_lat) min_lat = lat;
        if (lat > max_lat) max_lat = lat;
        if (lon < min_lon) min_lon = lon;
        if (lon > max_lon) max_lon = lon;
    }
    bool start_ways() {
        if (!node_count) { fail(Error::NO_DATA); return false; }
        if (!flush_nodes()) { fail(Error::WRITE_FAILED); return false; }
        io->close(nodes_w); nodes_w = nullptr;
        nodes_r = io->open_read(nodes_path);
        if (!nodes_r) { fail(Error::OPEN_FAILED); return false; }
        blk = (uint8_t *)malloc((size_t)kWin * kNodeRec);
        if (!blk) { fail(Error::NO_MEMORY); return false; }
        free(wbuf); wbuf = nullptr;
        cell_dlat = (int32_t)(((int64_t)max_lat - min_lat) / kGrid + 1);
        cell_dlon = (int32_t)(((int64_t)max_lon - min_lon) / kGrid + 1);
        st = State::WAYS;
        return true;
    }
    // Record r of the node table, through a small aligned read window (kWin records) so RAM use does not depend on the
    // sparse-index stride. Returns false on a read error.
    bool node_at(uint64_t r, int64_t *id, int32_t *lat, int32_t *lon) {
        const uint64_t base = r - r % kWin;
        if (blk_cached != (int64_t)base) {
            const uint32_t count = (uint32_t)((node_count - base) < kWin ? (node_count - base) : kWin);
            if (!io->seek(nodes_r, (size_t)(base * kNodeRec)) || io->read(nodes_r, blk, (size_t)count * kNodeRec) != (size_t)count * kNodeRec) { blk_cached = -1; return false; }
            blk_cached = (int64_t)base; blk_count = count;
        }
        const uint8_t *q = blk + (size_t)(r - base) * kNodeRec;
        memcpy(id, q, 8); memcpy(lat, q + 8, 4); memcpy(lon, q + 12, 4);
        return true;
    }
    bool lookup(int64_t id, int32_t *lat, int32_t *lon) {
        uint32_t lo = 0, hi = idx_n;
        while (lo < hi) { const uint32_t mid = (lo + hi) / 2; if (idx[mid] <= id) lo = mid + 1; else hi = mid; }
        if (!lo) return false;
        const uint32_t j = lo - 1;
        uint64_t a = (uint64_t)j * stride, b = a + stride;
        if (b > node_count) b = node_count;
        while (a < b) {
            const uint64_t m = (a + b) / 2; int64_t v; int32_t la, lo2;
            if (!node_at(m, &v, &la, &lo2)) return false;
            if (v == id) { *lat = la; *lon = lo2; return true; }
            if (v < id) a = m + 1; else b = m;
        }
        return false;
    }

    // ---------------- way phase ----------------
    bool flush_cell(unsigned c) {
        if (!cused[c]) return true;
        char path[96]; cell_path(path, sizeof(path), dir, c / kGrid, c % kGrid);
        void *h = io->open_write(path, true);
        if (!h) return false;
        const bool ok = io->write(h, cbuf + (size_t)c * kCellBuf, cused[c]);
        io->close(h); bytes_out += cused[c]; cused[c] = 0; return ok;
    }
    bool append_cell(unsigned c, const uint8_t *rec, size_t n) {
        if (n > kCellBuf) {
            if (!flush_cell(c)) return false;
            char path[96]; cell_path(path, sizeof(path), dir, c / kGrid, c % kGrid);
            void *h = io->open_write(path, true); if (!h) return false;
            const bool ok = io->write(h, rec, n); io->close(h); bytes_out += (uint32_t)n; return ok;
        }
        if (cused[c] + n > kCellBuf && !flush_cell(c)) return false;
        memcpy(cbuf + (size_t)c * kCellBuf + cused[c], rec, n); cused[c] = (uint8_t)(cused[c] + n); return true;
    }
    unsigned cell_of(int32_t lat, int32_t lon) const {
        int64_t r = ((int64_t)lat - min_lat) / cell_dlat, c = ((int64_t)lon - min_lon) / cell_dlon;
        if (r < 0) r = 0;
        if (r >= (int64_t)kGrid) r = kGrid - 1;
        if (c < 0) c = 0;
        if (c >= (int64_t)kGrid) c = kGrid - 1;
        return (unsigned)(r * kGrid + c);
    }
    // Write one run of resolved points [a,b) as one or more records.
    bool emit_run(uint32_t a, uint32_t b, bool closed_whole) {
        while (b - a >= 2) {
            uint8_t rec[kRecMax];
            uint32_t n = 1;
            int32_t cur_lat = wlat[a], cur_lon = wlon[a];
            memcpy(rec + 4, &cur_lat, 4); memcpy(rec + 8, &cur_lon, 4);
            int32_t qlat[kRecPts], qlon[kRecPts]; qlat[0] = cur_lat; qlon[0] = cur_lon;
            uint32_t i = a + 1;
            for (; i < b && n < kRecPts; ++i) {
                const int32_t dl = (int32_t)llround((double)((int64_t)wlat[i] - cur_lat) / 10.0), dn = (int32_t)llround((double)((int64_t)wlon[i] - cur_lon) / 10.0);
                if (dl > 32767 || dl < -32768 || dn > 32767 || dn < -32768) break;        // segment too long for 16-bit deltas: start a new record here
                if (dl == 0 && dn == 0) continue;                                          // duplicate within 1e-6 deg
                const int16_t sl = (int16_t)dl, sn = (int16_t)dn;
                memcpy(rec + kRecHdr + 4 * (n - 1), &sl, 2); memcpy(rec + kRecHdr + 4 * (n - 1) + 2, &sn, 2);
                cur_lat += dl * 10; cur_lon += dn * 10; qlat[n] = cur_lat; qlon[n] = cur_lon; ++n;
            }
            const bool whole = (a == 0 && i == b);
            if (n >= 2) {
                rec[0] = cls; rec[1] = (closed_whole && whole) ? 1 : 0; const uint16_t np = (uint16_t)n; memcpy(rec + 2, &np, 2);
                // every cell the polyline touches (segments are sampled finer than a cell so none is skipped)
                uint32_t bits[kGrid * kGrid / 32] = {};
                for (uint32_t k = 0; k < n; ++k) {
                    const unsigned c0 = cell_of(qlat[k], qlon[k]); bits[c0 >> 5] |= 1u << (c0 & 31);
                    if (!k) continue;
                    const double span = fmax(fabs((double)qlat[k] - qlat[k - 1]) / cell_dlat, fabs((double)qlon[k] - qlon[k - 1]) / cell_dlon);
                    const int steps = (int)ceil(span * 2.0);
                    for (int s = 1; s < steps; ++s) {
                        const double f = (double)s / steps;
                        const unsigned c1 = cell_of((int32_t)(qlat[k - 1] + (qlat[k] - qlat[k - 1]) * f), (int32_t)(qlon[k - 1] + (qlon[k] - qlon[k - 1]) * f));
                        bits[c1 >> 5] |= 1u << (c1 & 31);
                    }
                }
                const size_t len = kRecHdr + 4 * (n - 1);
                for (unsigned c = 0; c < kGrid * kGrid; ++c) if (bits[c >> 5] & (1u << (c & 31))) { if (!append_cell(c, rec, len)) return false; }
                ++features;
            }
            if (i >= b) break;
            a = i - 1;                                                                      // next record starts at the last point written
            if (n < 2) a = i;                                                               // nothing was written (all duplicates): move on
        }
        return true;
    }
    bool emit_way() {
        if (cls == C_NONE || nref < 2) return true;
        // resolve references; a node missing from the extract splits the polyline
        uint32_t run_start = 0, n = 0; bool all = true;
        for (uint32_t i = 0; i < nref; ++i) {
            int32_t la, lo;
            if (lookup(refs[i], &la, &lo)) { wlat[n] = la; wlon[n] = lo; ++n; }
            else {
                ++missing; all = false;
                if (n - run_start >= 2 && !emit_run(run_start, n, false)) return false;
                run_start = n;
            }
        }
        const bool closed = all && nref >= 4 && refs[0] == refs[nref - 1] && !truncated;
        if (n - run_start >= 2 && !emit_run(run_start, n, closed)) return false;
        return true;
    }

    // ---------------- tokenizer ----------------
    void handle_tag(char *t, size_t n) {
        if (!n) return;
        if (t[0] == '?' || t[0] == '!') return;
        const bool closing = t[0] == '/';
        const char *name = t + (closing ? 1 : 0);
        size_t nl = 0; while (nl < n - (closing ? 1 : 0) && name[nl] != ' ' && name[nl] != '/' && name[nl] != '\t' && name[nl] != '\n' && name[nl] != '\r') ++nl;
        t[n] = 0;
        if (st == State::NODES) {
            if (!closing && eq(name, nl, "node")) {
                Attrs a; parse_attrs(t, t + n, &a);
                int64_t id; int32_t lat, lon;
                if (a.id && a.lat && a.lon && parse_i64(a.id, a.idn, &id) && parse_e7(a.lat, a.latn, &lat) && parse_e7(a.lon, a.lonn, &lon) &&
                    lat >= -900000000 && lat <= 900000000 && lon >= -1800000000 && lon <= 1800000000) add_node(id, lat, lon);
                else ++skipped_tags;
            } else if (!closing && eq(name, nl, "way")) {
                if (start_ways()) handle_tag(t, n);
            } else if (!closing && eq(name, nl, "relation")) { if (start_ways()) { phase = P_FLUSH; } }
            return;
        }
        if (st != State::WAYS) return;
        if (closing) {
            if (eq(name, nl, "way")) { if (in_way) { ++ways; if (!emit_way()) fail(Error::WRITE_FAILED); } in_way = false; }
            else if (eq(name, nl, "osm")) phase = P_FLUSH;
            return;
        }
        if (eq(name, nl, "way")) { in_way = true; nref = 0; cls = C_NONE; truncated = false; if (t[n - 1] == '/') in_way = false; }
        else if (eq(name, nl, "nd")) {
            if (!in_way) return;
            Attrs a; parse_attrs(t, t + n, &a); int64_t r;
            if (a.ref && parse_i64(a.ref, a.refn, &r)) { if (nref < kMaxRefs) refs[nref++] = r; else { if (!truncated) ++trunc_ways; truncated = true; } }
        } else if (eq(name, nl, "tag")) {
            if (!in_way) return;
            Attrs a; parse_attrs(t, t + n, &a);
            if (a.k && a.v) { const uint8_t c = classify(a.k, a.kn, a.v, a.vn); if (class_priority(c) > class_priority(cls)) cls = c; }
        } else if (eq(name, nl, "node")) fail(Error::NODES_AFTER_WAYS);
        else if (eq(name, nl, "relation")) phase = P_FLUSH;
    }
    void feed(const uint8_t *d, size_t n) {
        for (size_t i = 0; i < n && st != State::ERROR && phase == P_PARSE; ++i) {
            const char c = (char)d[i];
            if (!in_tag) { if (c == '<') { in_tag = true; tag_len = 0; overflow = false; quote = 0; in_comment = false; } continue; }
            if (in_comment) {                                                         // <!-- ... -->
                if (c == '>' && tag_len >= 2 && tag[tag_len - 1] == '-' && tag[tag_len - 2] == '-') { in_tag = false; continue; }
                if (tag_len < kTagMax) tag[tag_len++] = c; else { tag[0] = tag[1]; tag[1] = tag[2]; tag_len = 3; tag[2] = c; }
                continue;
            }
            if (quote) { if (c == quote) quote = 0; }
            else if (c == '"' || c == '\'') quote = c;
            else if (c == '>') {
                in_tag = false;
                if (overflow) { ++skipped_tags; continue; }
                if (tag_len && tag[tag_len - 1] == '/') --tag_len;                    // keep the name/attrs, drop the self-close marker
                handle_tag(tag, tag_len);
                continue;
            }
            if (tag_len < kTagMax) { tag[tag_len++] = c; if (tag_len == 3 && !memcmp(tag, "!--", 3)) in_comment = true; }
            else overflow = true;
        }
    }
};

Indexer::Indexer() : m_(nullptr) {}
Indexer::~Indexer() { delete m_; }
size_t Indexer::heap_estimate() const {
    return kChunk + kTagMax + kWbuf + kIdxCap * 8 + kMaxRefs * 16 + (size_t)kGrid * kGrid * kCellBuf + 1024;
}
void Indexer::cancel() { if (m_) { m_->release(); m_->st = State::IDLE; } }

bool Indexer::start(FileIo *io, const char *src_path, const char *cache_dir) {
    delete m_; m_ = new (std::nothrow) Impl();
    if (!m_) return false;
    Impl &m = *m_;
    m.io = io;
    snprintf(m.dir, sizeof(m.dir), "%s", cache_dir); snprintf(m.src, sizeof(m.src), "%s", src_path);
    snprintf(m.nodes_path, sizeof(m.nodes_path), "%s/nodes.tmp", cache_dir);
    m.src_size = io->size(src_path);
    if (!m.src_size) { m.fail(io->size(src_path) ? Error::EMPTY : Error::NO_FILE); return false; }
    if (!m.alloc()) { m.fail(Error::NO_MEMORY); return false; }
    m.in = io->open_read(src_path);
    if (!m.in) { m.fail(Error::OPEN_FAILED); return false; }
    char parent[72]; snprintf(parent, sizeof(parent), "%s", cache_dir);
    char *slash = strrchr(parent, '/'); if (slash) { *slash = 0; io->mkdir(parent); }
    io->mkdir(cache_dir);
    m.nodes_w = io->open_write(m.nodes_path, false);
    if (!m.nodes_w) { m.fail(Error::OPEN_FAILED); return false; }
    m.st = State::NODES; m.phase = Impl::P_CLEAN; m.clean_cursor = 0;
    char meta[96]; snprintf(meta, sizeof(meta), "%s/meta.bin", cache_dir); io->remove(meta);       // an unfinished cache must not look valid
    return true;
}

bool Indexer::step(size_t max_bytes) {
    if (!m_) return false;
    Impl &m = *m_;
    if (m.st == State::DONE || m.st == State::ERROR || m.st == State::IDLE) return false;
    if (m.phase == Impl::P_CLEAN) {                                       // remove stale cell files, 64 per call
        for (unsigned n = 0; n < 64 && m.clean_cursor < kGrid * kGrid; ++n, ++m.clean_cursor) {
            char p[96]; cell_path(p, sizeof(p), m.dir, m.clean_cursor / kGrid, m.clean_cursor % kGrid); m.io->remove(p);
        }
        if (m.clean_cursor >= kGrid * kGrid) m.phase = Impl::P_PARSE;
        return true;
    }
    if (m.phase == Impl::P_PARSE) {
        size_t budget = max_bytes;
        while (budget && m.phase == Impl::P_PARSE && m.st != State::ERROR) {
            const size_t want = budget < kChunk ? budget : kChunk;
            const size_t got = m.io->read(m.in, m.chunk, want);
            if (!got) { m.phase = Impl::P_FLUSH; break; }               // end of file
            m.done += got; budget -= got;
            m.feed(m.chunk, got);
        }
        if (m.st == State::ERROR) return false;
        if (m.phase == Impl::P_PARSE) return true;
    }
    if (m.phase == Impl::P_FLUSH) {
        if (m.st == State::NODES && !m.start_ways()) return false;        // file had nodes only
        if (m.st == State::ERROR) return false;
        for (unsigned c = 0; c < kGrid * kGrid; ++c) if (!m.flush_cell(c)) { m.fail(Error::WRITE_FAILED); return false; }
        Meta meta{}; meta.magic = kMagic; meta.version = kVersion; meta.src_size = (uint32_t)m.src_size;
        meta.min_lat_e7 = m.min_lat; meta.min_lon_e7 = m.min_lon; meta.max_lat_e7 = m.max_lat; meta.max_lon_e7 = m.max_lon;
        meta.nodes = (uint32_t)m.node_count; meta.ways_seen = m.ways; meta.features = m.features; meta.missing_refs = m.missing;
        meta.truncated_ways = m.trunc_ways; meta.bytes_out = m.bytes_out;
        char path[96]; snprintf(path, sizeof(path), "%s/meta.bin", m.dir);
        void *h = m.io->open_write(path, false);
        const bool ok = h && m.io->write(h, &meta, sizeof(meta));
        if (h) m.io->close(h);
        if (m.nodes_r) { m.io->close(m.nodes_r); m.nodes_r = nullptr; }
        m.io->remove(m.nodes_path);
        if (!ok) { m.fail(Error::WRITE_FAILED); return false; }
        m.st = State::DONE; m.release();
        return false;
    }
    return false;
}

void Indexer::progress(IndexProgress *p) const {
    memset(p, 0, sizeof(*p));
    if (!m_) return;
    p->state = m_->st; p->error = m_->err; p->bytes_done = (uint32_t)m_->done; p->bytes_total = (uint32_t)m_->src_size;
    p->nodes = (uint32_t)m_->node_count; p->ways = m_->ways; p->features = m_->features; p->missing_refs = m_->missing; p->skipped_tags = m_->skipped_tags;
}

bool load_meta(FileIo *io, const char *cache_dir, const char *src_path, Meta *out) {
    char path[96]; snprintf(path, sizeof(path), "%s/meta.bin", cache_dir);
    void *h = io->open_read(path);
    if (!h) return false;
    const size_t n = io->read(h, out, sizeof(*out));
    io->close(h);
    if (n != sizeof(*out) || out->magic != kMagic || out->version != kVersion) return false;
    return out->src_size == (uint32_t)io->size(src_path);       // the .osm was replaced: cache is stale
}

// ================================================================== canvas + renderer
size_t canvas_bytes(int w, int h) { return ((size_t)w * h + 1) / 2; }
void canvas_clear(Canvas &c) { memset(c.px, 0, canvas_bytes(c.w, c.h)); }
namespace {
inline void put(Canvas &c, int x, int y, uint8_t cls) {
    if ((unsigned)x >= (unsigned)c.w || (unsigned)y >= (unsigned)c.h) return;
    uint8_t &b = c.px[((size_t)y * c.w + x) >> 1];
    if (x & 1) { if ((b >> 4) <= cls) b = (uint8_t)((b & 0x0F) | (cls << 4)); }
    else { if ((b & 15) <= cls) b = (uint8_t)((b & 0xF0) | cls); }
}
// Liang-Barsky clip, then Bresenham.
bool clip_line(float &x0, float &y0, float &x1, float &y1, float xmax, float ymax) {
    float t0 = 0, t1 = 1; const float dx = x1 - x0, dy = y1 - y0;
    const float p[4] = {-dx, dx, -dy, dy}, q[4] = {x0, xmax - x0, y0, ymax - y0};
    for (int i = 0; i < 4; ++i) {
        if (p[i] == 0) { if (q[i] < 0) return false; }
        else { const float r = q[i] / p[i]; if (p[i] < 0) { if (r > t1) return false; if (r > t0) t0 = r; } else { if (r < t0) return false; if (r < t1) t1 = r; } }
    }
    x1 = x0 + t1 * dx; y1 = y0 + t1 * dy; x0 += t0 * dx; y0 += t0 * dy; return true;
}
void line(Canvas &c, float fx0, float fy0, float fx1, float fy1, uint8_t cls, int width) {
    if (!isfinite(fx0) || !isfinite(fy0) || !isfinite(fx1) || !isfinite(fy1)) return;
    if (!clip_line(fx0, fy0, fx1, fy1, (float)c.w - 1, (float)c.h - 1)) return;
    int x0 = (int)lroundf(fx0), y0 = (int)lroundf(fy0), x1 = (int)lroundf(fx1), y1 = (int)lroundf(fy1);
    const int dx = abs(x1 - x0), dy = -abs(y1 - y0), sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
    const bool steep = dx > -dy;
    int err = dx + dy;
    for (;;) {
        put(c, x0, y0, cls);
        if (width > 1) { if (steep) put(c, x0, y0 + 1, cls); else put(c, x0 + 1, y0, cls); }
        if (x0 == x1 && y0 == y1) break;
        const int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}
// Even-odd scanline fill of a closed polygon given in canvas pixels.
void fill_poly(Canvas &c, const float *px, const float *py, int n, uint8_t cls) {
    if (n < 3) return;
    float ymin = py[0], ymax = py[0];
    for (int i = 1; i < n; ++i) { if (py[i] < ymin) ymin = py[i]; if (py[i] > ymax) ymax = py[i]; }
    const int y0 = (int)fmaxf(0.f, ceilf(ymin)), y1 = (int)fminf((float)c.h - 1, floorf(ymax));
    float xs[kRecPts + 2];
    for (int y = y0; y <= y1; ++y) {
        int m = 0; const float yc = y + 0.5f;
        for (int i = 0, j = n - 1; i < n; j = i++) {
            if ((py[i] > yc) != (py[j] > yc)) { if (m < (int)(sizeof(xs) / sizeof(*xs))) xs[m++] = px[j] + (yc - py[j]) * (px[i] - px[j]) / (py[i] - py[j]); }
        }
        for (int a = 1; a < m; ++a) { const float v = xs[a]; int b = a - 1; while (b >= 0 && xs[b] > v) { xs[b + 1] = xs[b]; --b; } xs[b + 1] = v; }
        for (int a = 0; a + 1 < m; a += 2) {
            const int xa = (int)fmaxf(0.f, ceilf(xs[a])), xb = (int)fminf((float)c.w - 1, floorf(xs[a + 1]));
            for (int x = xa; x <= xb; ++x) put(c, x, y, cls);
        }
    }
}
bool class_visible(uint8_t cls, uint16_t layers, double mpp) {
    switch (cls) {
        case C_ROAD_MAJOR: return layers & L_ROADS;
        case C_ROAD_MINOR: return (layers & L_ROADS) && mpp < 80.0;
        case C_PATH: return (layers & L_PATHS) && mpp < 20.0;
        case C_BUILD: case C_BUILD_FILL: return (layers & L_BUILDINGS) && mpp < 10.0;
        case C_WATER: return layers & L_WATER;
        case C_GREEN: return (layers & L_GREEN) && mpp < 80.0;
        case C_RAIL: return (layers & L_RAIL) && mpp < 80.0;
    }
    return false;
}
}  // namespace

uint16_t layers_for_scale(double mpp, uint16_t enabled) {
    uint16_t l = enabled;
    if (mpp >= 20.0) l &= (uint16_t)~L_PATHS;
    if (mpp >= 10.0) l &= (uint16_t)~L_BUILDINGS;
    if (mpp >= 80.0) l &= (uint16_t)~(L_GREEN | L_RAIL);
    return l;
}

struct Renderer::Impl {
    FileIo *io = nullptr; char dir[72]{}; Meta meta{}; geo::View view{}; Canvas cv{}; uint16_t layers = L_ALL;
    uint16_t cells[kGrid * kGrid]; uint32_t ncells = 0, cursor = 0; void *file = nullptr;
    uint8_t buf[1024]; size_t have = 0, pos = 0; bool eof = false;
    float px[kRecPts], py[kRecPts]; bool run = false;
    RenderStats st{};
    void close_cell() { if (file) { io->close(file); file = nullptr; } have = pos = 0; eof = false; }
    bool ensure(size_t n) {
        if (have - pos >= n) return true;
        if (pos) { memmove(buf, buf + pos, have - pos); have -= pos; pos = 0; }
        while (have < n && !eof) {
            const size_t got = io->read(file, buf + have, sizeof(buf) - have);
            if (!got) eof = true; else { have += got; st.bytes += (uint32_t)got; }
        }
        return have >= n;
    }
    void draw_record(uint8_t cls, bool closed, int npts, const int32_t lat0, const int32_t lon0, const uint8_t *deltas) {
        if (!class_visible(cls, layers, view.m_per_px)) return;
        int32_t la = lat0, lo = lon0; float minx = 1e9f, maxx = -1e9f, miny = 1e9f, maxy = -1e9f;
        for (int i = 0; i < npts; ++i) {
            if (i) { int16_t dl, dn; memcpy(&dl, deltas + 4 * (i - 1), 2); memcpy(&dn, deltas + 4 * (i - 1) + 2, 2); la += dl * 10; lo += dn * 10; }
            if (!geo::to_px(view, la / 1e7, lo / 1e7, &px[i], &py[i])) return;
            if (px[i] < minx) minx = px[i];
            if (px[i] > maxx) maxx = px[i];
            if (py[i] < miny) miny = py[i];
            if (py[i] > maxy) maxy = py[i];
        }
        if (maxx < 0 || maxy < 0 || minx > cv.w || miny > cv.h) return;                // entirely outside the canvas
        const double m = view.m_per_px;
        if (closed && (cls == C_BUILD)) {
            fill_poly(cv, px, py, npts, C_BUILD_FILL);
            for (int i = 1; i < npts; ++i) { line(cv, px[i - 1], py[i - 1], px[i], py[i], C_BUILD, 1); ++st.drawn_lines; }
        } else if (closed && (cls == C_GREEN || cls == C_WATER)) {
            fill_poly(cv, px, py, npts, cls);
        } else {
            const int w = cls == C_ROAD_MAJOR ? (m < 40.0 ? 2 : 1) : (cls == C_ROAD_MINOR && m < 2.5) ? 2 : 1;
            for (int i = 1; i < npts; ++i) { line(cv, px[i - 1], py[i - 1], px[i], py[i], cls, w); ++st.drawn_lines; }
        }
    }
};

Renderer::Renderer() : m_(nullptr) {}
Renderer::~Renderer() { cancel(); delete m_; }
bool Renderer::active() const { return m_ && m_->run; }
void Renderer::cancel() { if (m_) { m_->close_cell(); m_->run = false; } }
void Renderer::stats(RenderStats *s) const { memset(s, 0, sizeof(*s)); if (m_) *s = m_->st; }
uint32_t Renderer::cells_total() const { return m_ ? m_->ncells : 0; }
uint32_t Renderer::cells_done() const { return m_ ? m_->cursor : 0; }

bool Renderer::begin(FileIo *io, const char *cache_dir, const Meta &meta, const geo::View &view, Canvas canvas, uint16_t layers) {
    if (!m_) { m_ = new (std::nothrow) Impl(); if (!m_) return false; }
    Impl &m = *m_;
    m.close_cell();
    m.io = io; snprintf(m.dir, sizeof(m.dir), "%s", cache_dir); m.meta = meta; m.view = view; m.cv = canvas; m.layers = layers;
    m.st = RenderStats{}; m.cursor = 0; m.ncells = 0;
    // geographic extent of the canvas (all four corners and edge midpoints: valid for rotated views too)
    double lat_min = 1e9, lat_max = -1e9, lon_min = 1e9, lon_max = -1e9;
    for (int i = 0; i < 9; ++i) {
        double la, lo; geo::from_px(view, (float)(canvas.w * (i % 3) / 2.0), (float)(canvas.h * (i / 3) / 2.0), &la, &lo);
        if (la < lat_min) lat_min = la;
        if (la > lat_max) lat_max = la;
        if (lo < lon_min) lon_min = lo;
        if (lo > lon_max) lon_max = lo;
    }
    const int32_t dlat = (int32_t)(((int64_t)meta.max_lat_e7 - meta.min_lat_e7) / kGrid + 1), dlon = (int32_t)(((int64_t)meta.max_lon_e7 - meta.min_lon_e7) / kGrid + 1);
    auto row = [&](double lat) { int64_t r = (int64_t)floor((lat * 1e7 - meta.min_lat_e7) / dlat); return r; };
    auto col = [&](double lon) { int64_t c = (int64_t)floor((lon * 1e7 - meta.min_lon_e7) / dlon); return c; };
    int64_t r0 = row(lat_min), r1 = row(lat_max), c0 = col(lon_min), c1 = col(lon_max);
    if (r1 < 0 || c1 < 0 || r0 >= (int64_t)kGrid || c0 >= (int64_t)kGrid) { m.run = false; return true; }     // view entirely outside the map data
    if (r0 < 0) r0 = 0;
    if (c0 < 0) c0 = 0;
    if (r1 >= (int64_t)kGrid) r1 = kGrid - 1;
    if (c1 >= (int64_t)kGrid) c1 = kGrid - 1;
    // nearest the view centre first, so a partial render is useful
    const int64_t cr = row(view.lat), cc = col(view.lon);
    struct Cand { int64_t d; uint16_t c; } cand[kGrid * kGrid]; uint32_t n = 0;
    for (int64_t r = r0; r <= r1; ++r) for (int64_t c = c0; c <= c1; ++c) cand[n++] = {(r - cr) * (r - cr) + (c - cc) * (c - cc), (uint16_t)(r * kGrid + c)};
    for (uint32_t i = 1; i < n; ++i) { Cand v = cand[i]; int j = (int)i - 1; while (j >= 0 && cand[j].d > v.d) { cand[j + 1] = cand[j]; --j; } cand[j + 1] = v; }
    for (uint32_t i = 0; i < n; ++i) m.cells[i] = cand[i].c;
    m.ncells = n; m.run = n > 0;
    return true;
}

bool Renderer::step(unsigned max_records) {
    if (!m_ || !m_->run) return false;
    Impl &m = *m_;
    while (max_records) {
        if (!m.file) {
            if (m.cursor >= m.ncells) { m.run = false; return false; }
            char p[96]; const unsigned c = m.cells[m.cursor]; cell_path(p, sizeof(p), m.dir, c / kGrid, c % kGrid);
            m.file = m.io->open_read(p);
            ++m.cursor; ++m.st.cells;
            if (!m.file) continue;                                             // empty cell: no file was written
            m.have = m.pos = 0; m.eof = false;
        }
        if (!m.ensure(kRecHdr)) { m.close_cell(); continue; }
        const uint8_t cls = m.buf[m.pos], flags = m.buf[m.pos + 1]; uint16_t np; memcpy(&np, m.buf + m.pos + 2, 2);
        int32_t lat0, lon0; memcpy(&lat0, m.buf + m.pos + 4, 4); memcpy(&lon0, m.buf + m.pos + 8, 4);
        if (cls == C_NONE || cls >= C_COUNT || np < 2 || np > kRecPts) { m.close_cell(); continue; }      // corrupt cell: stop reading it
        const size_t need = kRecHdr + 4 * (size_t)(np - 1);
        if (!m.ensure(need)) { m.close_cell(); continue; }
        m.draw_record(cls, flags & 1, np, lat0, lon0, m.buf + m.pos + kRecHdr);
        m.pos += need; ++m.st.records; --max_records;
    }
    return true;
}
}  // namespace marauder::osm
