#include "../../main/include/browser.hpp"
#include "../../main/include/dir_window.hpp"
#include <algorithm>
#include <assert.h>
#include <stdio.h>
#include <string>
#include <vector>
using namespace marauder;
struct Ent { std::string name; bool dir; };
static std::vector<std::pair<std::string, std::vector<Ent>>> g_fs;
static std::vector<Ent> *find(const char *d) { for (auto &p : g_fs) if (p.first == d) return &p.second; return nullptr; }
static size_t page(const char *dir, const DirEntry *anchor, bool before, DirEntry *out, size_t cap, size_t *total) {
    auto *v = find(dir); if (!v) { *total = 0; return 0; }
    DirEntry a; if (anchor) { a = *anchor; anchor = &a; }
    size_t n = 0;
    for (auto &e : *v) dir_window_offer(out, n, cap, anchor, before, e.dir, e.name.c_str());
    *total = v->size();
    return n;
}
static std::vector<Ent> sorted(std::vector<Ent> v) {
    std::sort(v.begin(), v.end(), [](const Ent &a, const Ent &b) { return dir_entry_less(a.dir, a.name.c_str(), b.dir, b.name.c_str()); });
    return v;
}
int main() {
    std::vector<Ent> root = {{"zeta", true}, {"Alpha", true}, {"maps", true}};
    for (int i = 0; i < 70; ++i) { char b[32]; snprintf(b, sizeof b, "%s%02d.%s", i % 3 ? "img" : "IMG", (i * 37) % 70, i % 5 ? "jpg" : "txt"); root.push_back({b, false}); }
    g_fs.push_back({".", root});
    g_fs.push_back({"maps", {{"regional_map.osm", false}, {"cache", true}}});
    g_fs.push_back({"maps/cache", {}});
    auto ref = sorted(root);
    BrowserModel m; m.init(page);
    assert(m.open_dir(".")); assert(m.total() == root.size());
    // forward walk visits everything in order, then wraps
    for (size_t i = 0; i < ref.size(); ++i) {
        assert(m.selected()->name == ref[i].name && (bool)m.selected()->is_dir == ref[i].dir);
        assert(m.position() == i + 1);
        m.move(1);
    }
    assert(m.position() == 1);                       // wrapped
    // backward from the first wraps to the last, then reverse walk
    m.move(-1); assert(m.selected()->name == ref.back().name && m.position() == ref.size());
    for (size_t i = ref.size(); i-- > 0;) { assert(m.selected()->name == ref[i].name); m.move(-1); }
    assert(m.position() == ref.size());
    // paging
    m.open_dir("."); m.page(1); assert(m.position() == BrowserModel::kRows + 1);
    assert(m.selected()->name == ref[BrowserModel::kRows].name);
    m.page(-1); assert(m.selected()->name == ref[BrowserModel::kRows - 1].name);
    // folders first
    m.open_dir("."); for (int i = 0; i < 3; ++i) { assert(m.selected()->is_dir); m.move(1); } assert(!m.selected()->is_dir);
    // enter / up keeps the folder selected
    m.open_dir("."); while (std::string(m.selected()->name) != "maps") m.move(1);
    assert(m.enter()); assert(std::string(m.dir()) == "maps"); assert(m.count() == 2);
    assert(m.enter());  /* cache is first (folder) */ assert(std::string(m.dir()) == "maps/cache"); assert(m.count() == 0 && !m.selected());
    m.move(1); m.page(1);                           // no crash on empty
    assert(m.up()); assert(std::string(m.dir()) == "maps" && std::string(m.selected()->name) == "cache");
    assert(m.up()); assert(m.at_root() && std::string(m.selected()->name) == "maps");
    assert(!m.up());
    // reload keeps selection
    m.reload(); assert(std::string(m.selected()->name) == "maps");
    // step_kind finds the next image and wraps
    m.open_dir("."); DirEntry e; size_t seen = 0; std::string first;
    while (m.step_kind(FileKind::IMAGE, 1, &e)) { if (!seen) first = e.name; ++seen; if (seen > 200) break; if (seen == 60) break; }
    assert(seen == 60);
    // helpers
    assert(browser_kind("a.JPG", false) == FileKind::IMAGE && browser_kind("x", true) == FileKind::FOLDER && browser_kind("a.c8", false) == FileKind::CAPTURE);
    assert(browser_deletable("a.c8", false) && !browser_deletable("a.c8", true) && !browser_deletable("a.bin2", false) && !browser_deletable("noext", false));
    char s[10]; browser_format_size(812, s); assert(!strcmp(s, "812 B")); browser_format_size(1500, s); assert(!strcmp(s, "1.4 KB"));
    browser_format_size(23u << 20, s); assert(!strcmp(s, "23.0 MB"));
    char n[32]; browser_fit_name("IMG_20240101_123456.jpg", 16, n, sizeof n); assert(strlen(n) == 16 && !strcmp(n + 12, ".jpg")); printf("fit: %s\n", n);
    browser_fit_name("short.txt", 16, n, sizeof n); assert(!strcmp(n, "short.txt"));
    char j[80]; assert(browser_join(".", "a.jpg", j, sizeof j) && !strcmp(j, "a.jpg")); assert(browser_join("A/B", "c", j, sizeof j) && !strcmp(j, "A/B/c"));
    puts("ALL PASS");
}
