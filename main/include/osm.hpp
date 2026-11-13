#pragma once
// Offline OpenStreetMap support: streaming .osm XML indexer + cell-cache renderer. No whole-file loads.
//
//  /maps/regional_map.osm            input (standard sorted OSM XML: nodes, then ways, then relations)
//  /map_cache/<name>/meta.bin        cache header (bounds, counts, source size)
//  /map_cache/<name>/cXXYY.bin       one file per grid cell: compact polylines
//  /map_cache/<name>/nodes.tmp       temporary node table, deleted when indexing finishes
//
// Indexing is a single sequential pass, run in small time slices by the caller (no task of its own).
// Rendering streams only the cells in view, into a caller-owned 4-bit canvas.
#include <stddef.h>
#include <stdint.h>
#include "geo.hpp"

namespace marauder::osm {

// Storage abstraction: firmware implements it on the SD card, the host tests on stdio.
class FileIo {
public:
    virtual ~FileIo() {}
    virtual void *open_read(const char *path) = 0;
    virtual void *open_write(const char *path, bool append) = 0;
    virtual size_t read(void *h, void *buf, size_t n) = 0;
    virtual bool write(void *h, const void *buf, size_t n) = 0;
    virtual bool seek(void *h, size_t offset) = 0;
    virtual void close(void *h) = 0;
    virtual size_t size(const char *path) = 0;          // 0 when missing
    virtual bool remove(const char *path) = 0;
    virtual bool mkdir(const char *path) = 0;           // ok if it already exists
};

enum Class : uint8_t { C_NONE = 0, C_GREEN, C_WATER, C_BUILD_FILL, C_BUILD, C_RAIL, C_PATH, C_ROAD_MINOR, C_ROAD_MAJOR, C_COUNT };
// z-order == enum order: a pixel is only overwritten by an equal or higher class.

enum Layer : uint16_t { L_ROADS = 1, L_PATHS = 2, L_BUILDINGS = 4, L_WATER = 8, L_GREEN = 16, L_RAIL = 32, L_ALL = 63 };

constexpr unsigned kGrid = 8;                  // kGrid x kGrid cells over the data bounds
constexpr unsigned kMaxNodes = 1000000;        // node table limit (index memory is bounded, see README)
constexpr unsigned kMaxRefs = 384;             // nodes per way kept (OSM allows 2000; extras are dropped)
constexpr unsigned kRecPts = 200;              // points per stored record (longer ways are split)

struct Meta {
    uint32_t magic, version;
    uint32_t src_size;
    int32_t min_lat_e7, min_lon_e7, max_lat_e7, max_lon_e7;
    uint32_t nodes, ways_seen, features, missing_refs, truncated_ways, bytes_out;
    uint32_t reserved[4];
};

enum class State : uint8_t { IDLE, NODES, WAYS, DONE, ERROR };
enum class Error : uint8_t { NONE, NO_FILE, EMPTY, OPEN_FAILED, NO_MEMORY, TOO_MANY_NODES, NODES_UNSORTED, NODES_AFTER_WAYS, WRITE_FAILED, NO_DATA };
const char *error_text(Error e);

struct IndexProgress {
    State state; Error error;
    uint32_t bytes_done, bytes_total;
    uint32_t nodes, ways, features, missing_refs, skipped_tags;
};

class Indexer {
public:
    Indexer();
    ~Indexer();
    // Start indexing src_path (e.g. "maps/regional_map.osm") into cache_dir (e.g. "map_cache/regional_map").
    bool start(FileIo *io, const char *src_path, const char *cache_dir);
    // Process up to max_bytes of input. Returns false when finished or failed (see progress()).
    bool step(size_t max_bytes);
    void cancel();
    void progress(IndexProgress *p) const;
    size_t heap_estimate() const;       // bytes the indexer will allocate at its peak
private:
    struct Impl;
    Impl *m_;
};

// ---- cache reader / renderer ----
bool load_meta(FileIo *io, const char *cache_dir, const char *src_path, Meta *out);   // false: missing, bad, or source changed

struct Canvas { uint8_t *px; int w, h; };        // 4 bits per pixel, packed two per byte (even x = low nibble)
inline uint8_t canvas_get(const Canvas &c, int x, int y) { const uint8_t b = c.px[((size_t)y * c.w + x) >> 1]; return (x & 1) ? b >> 4 : b & 15; }
void canvas_clear(Canvas &c);
size_t canvas_bytes(int w, int h);

struct RenderStats { uint32_t cells, records, bytes, drawn_lines; };

class Renderer {
public:
    Renderer();
    ~Renderer();
    // Prepare a render pass of the cache for this view into the canvas. Does not draw yet.
    bool begin(FileIo *io, const char *cache_dir, const Meta &meta, const geo::View &view, Canvas canvas, uint16_t layers);
    // Draw up to max_records stored records. Returns true while more work remains.
    bool step(unsigned max_records);
    void cancel();
    bool active() const;
    void stats(RenderStats *s) const;
    uint32_t cells_total() const;
    uint32_t cells_done() const;
private:
    struct Impl;
    Impl *m_;
};

// Level-of-detail: which layers are drawn at a scale (metres per pixel).
uint16_t layers_for_scale(double m_per_px, uint16_t enabled);
}
