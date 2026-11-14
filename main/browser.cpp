#include "include/browser.hpp"
#include "include/dir_window.hpp"
#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <new>

namespace marauder {
namespace {
const char *ext_of(const char *n) { const char *d = strrchr(n, '.'); return d && d != n ? d + 1 : ""; }
bool in_list(const char *e, const char *const *list) { for (; *list; ++list) if (!strcasecmp(e, *list)) return true; return false; }
}

FileKind browser_kind(const char *name, bool is_dir) {
    if (is_dir) return FileKind::FOLDER;
    static const char *const img[] = {"jpg", "jpeg", "png", nullptr};
    static const char *const vid[] = {"mp4", nullptr};
    static const char *const txt[] = {"txt", "log", "csv", "json", "meta", "gpx", "xml", "md", "ini", "cfg", "rfp", "rrc", "app", "trk", "nmea", nullptr};
    const char *e = ext_of(name);
    if (in_list(e, img)) return FileKind::IMAGE;
    if (in_list(e, vid)) return FileKind::VIDEO;
    if (!strcasecmp(e, "c8")) return FileKind::CAPTURE;
    if (!strcasecmp(e, "osm")) return FileKind::MAP;
    if (in_list(e, txt)) return FileKind::TEXT;
    return FileKind::OTHER;
}
const char *browser_kind_label(FileKind k) {
    switch (k) { case FileKind::FOLDER: return "FOLDER"; case FileKind::IMAGE: return "IMAGE"; case FileKind::VIDEO: return "VIDEO (NO DECODER)";
                 case FileKind::TEXT: return "TEXT"; case FileKind::CAPTURE: return "IQ8 CAPTURE"; case FileKind::MAP: return "OSM MAP"; default: return "FILE"; }
}
bool browser_deletable(const char *name, bool is_dir) {
    if (is_dir) return false;
    static const char *const ok[] = {"c8", "meta", "jpg", "jpeg", "png", "txt", "log", "csv", "gpx", "trk", "nmea", "rrc", "rfp", "bin", nullptr};
    return in_list(ext_of(name), ok);
}
void browser_format_size(uint32_t b, char out[10]) {
    if (b < 1000) snprintf(out, 10, "%u B", (unsigned)b);
    else if (b < 1000u * 1024) snprintf(out, 10, "%u.%u KB", (unsigned)(b / 1024), (unsigned)((b % 1024) * 10 / 1024));
    else if (b < 100u * 1024 * 1024) snprintf(out, 10, "%u.%u MB", (unsigned)(b >> 20), (unsigned)(((b & 0xFFFFF) * 10) >> 20));
    else snprintf(out, 10, "%u MB", (unsigned)(b >> 20));
    }
void browser_fit_name(const char *name, size_t cols, char *out, size_t out_size) {
    if (!out_size) return;
    if (cols >= out_size) cols = out_size - 1;
    const size_t n = strlen(name);
    if (n <= cols) { memcpy(out, name, n + 1); return; }
    const char *dot = strrchr(name, '.');
    size_t ext = dot && dot != name ? strlen(dot) : 0;
    if (ext > 5 || cols < ext + 4) ext = 0;
    const size_t head = cols - ext - 2;
    memcpy(out, name, head); out[head] = '.'; out[head + 1] = '.';
    if (ext) memcpy(out + head + 2, name + n - ext, ext);
    out[cols] = 0;
}
bool browser_join(const char *dir, const char *name, char *out, size_t out_size) {
    const int n = (dir[0] == '.' && !dir[1]) ? snprintf(out, out_size, "%s", name) : snprintf(out, out_size, "%s/%s", dir, name);
    return n > 0 && (size_t)n < out_size && n <= 78;
}

void BrowserModel::init(PageFn fn) { fn_ = fn; }
bool BrowserModel::load(const DirEntry *anchor, bool before) {
    size_t total = 0;
    DirEntry tmp[kRows];
    const size_t n = fn_(dir_, anchor, before, tmp, kRows, &total);
    total_ = total;
    if (!n && anchor) return false;
    memcpy(win_, tmp, n * sizeof(win_[0]));
    count_ = n; ++rev_;
    return true;
}
void BrowserModel::release_history() {
    for (size_t i = 0; i < kHist; ++i) { delete hist_[i]; hist_[i] = nullptr; }
    hist_n_ = 0;
}
void BrowserModel::push_snapshot() {
    Snap *s = new (std::nothrow) Snap();             // no memory: Back simply reads the card as before
    if (!s) return;
    snprintf(s->dir, sizeof(s->dir), "%s", dir_);
    memcpy(s->win, win_, sizeof(win_));
    s->count = count_; s->cursor = cursor_; s->start = start_; s->total = total_;
    if (hist_n_ == kHist) {                          // full: drop the oldest level
        delete hist_[0];
        for (size_t i = 1; i < kHist; ++i) hist_[i - 1] = hist_[i];
        hist_[--hist_n_] = nullptr;
    }
    hist_[hist_n_++] = s;
}
bool BrowserModel::restore_snapshot(const char *parent) {
    if (!hist_n_ || strcmp(hist_[hist_n_ - 1]->dir, parent)) return false;
    Snap *s = hist_[--hist_n_]; hist_[hist_n_] = nullptr;
    snprintf(dir_, sizeof(dir_), "%s", s->dir);
    memcpy(win_, s->win, sizeof(win_));
    count_ = s->count; cursor_ = s->cursor; start_ = s->start; total_ = s->total;
    ++rev_;
    delete s;
    return true;
}
bool BrowserModel::open_dir(const char *dir) {
    if (!keep_hist_) release_history();               // a fresh jump (not a descend) invalidates the cache
    char keep[80]; memcpy(keep, dir_, sizeof(keep));
    snprintf(dir_, sizeof(dir_), "%s", dir);
    size_t total = 0; DirEntry probe[1];
    // an unreadable folder is distinguished from an empty one by the call itself failing to find the directory
    (void)probe;
    if (!load(nullptr, false)) { memcpy(dir_, keep, sizeof(keep)); return false; }
    (void)total;
    cursor_ = 0; start_ = 0;
    return true;
}
bool BrowserModel::reload() {
    release_history();                                // the folder changed under us: cached parents may be stale too
    char name[64] = {0}; bool isdir = false;
    if (count_) { snprintf(name, sizeof(name), "%s", win_[cursor_].name); isdir = win_[cursor_].is_dir; }
    const size_t oldpos = start_ + cursor_;
    DirEntry anchor{}; snprintf(anchor.name, sizeof(anchor.name), "%s", name); anchor.is_dir = isdir;
    load(nullptr, false);
    cursor_ = 0; start_ = 0;
    if (!name[0]) return true;
    // walk forward to the previous selection (or the nearest entry after it)
    for (size_t guard = 0; guard < 100000; ++guard) {
        for (size_t i = 0; i < count_; ++i) if (!dir_entry_less(win_[i].is_dir != 0, win_[i].name, anchor.is_dir != 0, anchor.name)) { cursor_ = i; return true; }
        if (start_ + count_ >= total_) break;
        DirEntry last = win_[count_ - 1];
        if (!load(&last, false)) break;
        start_ += kRows;
    }
    (void)oldpos;
    cursor_ = count_ ? count_ - 1 : 0;
    return true;
}
void BrowserModel::move(int delta) {
    if (!count_) return;
    if (delta > 0) {
        if (cursor_ + 1 < count_) { ++cursor_; ++rev_; return; }
        if (start_ + count_ < total_) { DirEntry first = win_[0]; if (load(&first, false) ) { ++start_; cursor_ = count_ - 1; return; } }
        // wrap to the first page
        load(nullptr, false); start_ = 0; cursor_ = 0;
    } else {
        if (cursor_ > 0) { --cursor_; ++rev_; return; }
        if (start_ > 0) { DirEntry last = win_[count_ - 1]; if (load(&last, true)) { --start_; cursor_ = 0; return; } }
        // wrap to the last page
        load(nullptr, true); start_ = total_ > count_ ? total_ - count_ : 0; cursor_ = count_ ? count_ - 1 : 0;
    }
}
void BrowserModel::page(int direction) {
    if (!count_) return;
    if (direction > 0) {
        if (start_ + count_ < total_) { DirEntry last = win_[count_ - 1]; const size_t old = count_; if (load(&last, false)) { start_ += old; cursor_ = 0; return; } }
        if (cursor_ + 1 < count_) { cursor_ = count_ - 1; ++rev_; return; }
        load(nullptr, false); start_ = 0; cursor_ = 0;
    } else {
        if (start_ > 0) { DirEntry first = win_[0]; if (load(&first, true)) { start_ = start_ >= count_ ? start_ - count_ : 0; cursor_ = count_ ? count_ - 1 : 0; return; } }
        if (cursor_ > 0) { cursor_ = 0; ++rev_; return; }
        load(nullptr, true); start_ = total_ > count_ ? total_ - count_ : 0; cursor_ = count_ ? count_ - 1 : 0;
    }
}
bool BrowserModel::enter() {
    if (!count_ || !win_[cursor_].is_dir) return false;
    char next[80];
    if (!browser_join(dir_, win_[cursor_].name, next, sizeof(next))) return false;
    push_snapshot();
    const size_t depth = hist_n_;
    keep_hist_ = true;
    const bool ok = open_dir(next);
    keep_hist_ = false;
    if (!ok && hist_n_ == depth && hist_n_) { delete hist_[--hist_n_]; hist_[hist_n_] = nullptr; }   // could not enter: undo
    return ok;
}
bool BrowserModel::up() {
    if (at_root()) return false;
    char parent[80]; snprintf(parent, sizeof(parent), "%s", dir_);
    char *slash = strrchr(parent, '/');
    char left[80];
    if (slash) { snprintf(left, sizeof(left), "%s", slash + 1); *slash = 0; } else { snprintf(left, sizeof(left), "%s", parent); snprintf(parent, sizeof(parent), "."); }
    if (restore_snapshot(parent)) return true;       // instant: no card access
    if (!open_dir(parent)) return false;
    // select the folder we just left
    for (size_t guard = 0; guard < 100000; ++guard) {
        for (size_t i = 0; i < count_; ++i) if (win_[i].is_dir && !strcmp(win_[i].name, left)) { cursor_ = i; return true; }
        if (start_ + count_ >= total_) return true;
        DirEntry last = win_[count_ - 1]; const size_t old = count_;
        if (!load(&last, false)) return true;
        start_ += old; cursor_ = 0;
    }
    return true;
}
bool BrowserModel::step_kind(FileKind kind, int direction, DirEntry *out) {
    // Walk the folder in order from the selected entry, wrapping once, to the next/previous entry of this kind.
    if (!count_) return false;
    const size_t save_start = start_, save_cursor = cursor_;
    const DirEntry origin = win_[cursor_];
    bool found = false;
    for (size_t i = 0; i < total_ + 1; ++i) {
        move(direction > 0 ? 1 : -1);
        const DirEntry &e = win_[cursor_];
        if (!strcmp(e.name, origin.name) && e.is_dir == origin.is_dir) break;          // came all the way round
        if (browser_kind(e.name, e.is_dir) == kind) { *out = e; found = true; break; }
    }
    (void)save_start; (void)save_cursor;
    return found;
}
}
