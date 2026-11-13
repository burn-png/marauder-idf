#pragma once
// SD file browser model: folder navigation over bounded-RAM pages, plus classification / formatting helpers.
// Pure logic (the paging source is injected) so it is host-tested; the screen code lives in app_main.cpp.
#include <stddef.h>
#include <stdint.h>
#include "storage.hpp"

namespace marauder {

enum class FileKind : uint8_t { FOLDER, IMAGE, VIDEO, TEXT, CAPTURE, MAP, OTHER };
FileKind browser_kind(const char *name, bool is_dir);
const char *browser_kind_label(FileKind k);
bool browser_deletable(const char *name, bool is_dir);                 // user-data files only; folders and system files are protected
void browser_format_size(uint32_t bytes, char out[10]);                 // "812 B", "1.4 KB", "23 MB"
// Shorten `name` to at most `cols` characters, keeping the extension: "IMG_20240101_1234.jpg" -> "IMG_2024010..jpg".
void browser_fit_name(const char *name, size_t cols, char *out, size_t out_size);
bool browser_join(const char *dir, const char *name, char *out, size_t out_size);   // "." + "A" -> "A"; "A" + "b.jpg" -> "A/b.jpg"

class BrowserModel {
public:
    using PageFn = size_t (*)(const char *dir, const DirEntry *anchor, bool before, DirEntry *out, size_t cap, size_t *total);
    static constexpr size_t kRows = 21;
    void init(PageFn fn);
    bool open_dir(const char *dir);              // loads the first page; false if the folder cannot be read
    bool reload();                               // refresh in place, keeping the selection by name
    void move(int delta);                        // +1 / -1: scrolls one row at a time, wraps at the ends
    void page(int direction);                    // +1 / -1: one screenful
    bool enter();                                // selected entry is a folder: descend
    bool up();                                   // parent folder; the folder we left stays selected. false at the root
    const char *dir() const { return dir_; }
    bool at_root() const { return dir_[0] == '.' && !dir_[1]; }
    size_t count() const { return count_; }
    size_t cursor() const { return cursor_; }
    const DirEntry &entry(size_t i) const { return win_[i]; }
    const DirEntry *selected() const { return count_ ? &win_[cursor_] : nullptr; }
    size_t total() const { return total_; }
    size_t position() const { return count_ ? start_ + cursor_ + 1 : 0; }     // 1-based index within the folder
    uint32_t revision() const { return rev_; }   // bumps on every change of window or cursor
    // neighbouring files of a kind, for next/previous image: returns false when there is none (wraps through the folder)
    bool step_kind(FileKind kind, int direction, DirEntry *out);
private:
    bool load(const DirEntry *anchor, bool before);
    PageFn fn_ = nullptr;
    char dir_[80] = ".";
    DirEntry win_[kRows];
    size_t count_ = 0, cursor_ = 0, start_ = 0, total_ = 0;
    uint32_t rev_ = 0;
};
}
