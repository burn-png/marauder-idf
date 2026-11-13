#pragma once
// Ordered window selection for bounded-RAM directory paging (pure logic, host-tested in tests/browser_host).
// Order: folders first, then case-insensitive name, ties broken by exact name.
#include <string.h>
#include <strings.h>
#include <stddef.h>
#include <stdint.h>
#include "storage.hpp"

namespace marauder {
inline bool dir_entry_less(bool ad, const char *an, bool bd, const char *bn) {
    if (ad != bd) return ad;
    const int c = strcasecmp(an, bn);
    return c ? c < 0 : strcmp(an, bn) < 0;
}

// Offer one directory entry to the window `out[0..count)` (ascending). anchor==nullptr: no bound.
// before=false keeps the `capacity` smallest entries after the anchor; before=true keeps the largest before it.
inline void dir_window_offer(DirEntry *out, size_t &count, size_t capacity, const DirEntry *anchor, bool before,
                             bool is_dir, const char *name) {
    if (anchor) {
        const bool less = dir_entry_less(is_dir, name, anchor->is_dir != 0, anchor->name);
        const bool greater = dir_entry_less(anchor->is_dir != 0, anchor->name, is_dir, name);
        if (before ? !less : !greater) return;
    }
    size_t pos = count;
    while (pos && dir_entry_less(is_dir, name, out[pos - 1].is_dir != 0, out[pos - 1].name)) --pos;
    if (count == capacity) {
        if (!before) {
            if (pos == count) return;                              // larger than everything kept
            for (size_t i = capacity - 1; i > pos; --i) out[i] = out[i - 1];
        } else {
            if (pos == 0) return;                                  // smaller than everything kept
            memmove(&out[0], &out[1], (pos - 1) * sizeof(out[0]));
            --pos;
        }
    } else {
        for (size_t i = count; i > pos; --i) out[i] = out[i - 1];
        ++count;
    }
    memset(&out[pos], 0, sizeof(out[pos]));
    { size_t l = strnlen(name, sizeof(out[pos].name) - 1); memcpy(out[pos].name, name, l); out[pos].name[l] = 0; }
    out[pos].is_dir = is_dir ? 1 : 0;
}
}
