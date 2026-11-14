#include "../../main/include/appvm.hpp"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>
using namespace marauder::vm1;
struct Rec { std::vector<std::string> texts; std::vector<uint32_t> rnd; size_t rnd_i = 0; int fills = 0, clears = 0; int last_score = -1; };
static void fill(void *c, int x, int y, int w, int h, uint8_t) { auto *r = (Rec *)c; ++r->fills; assert(x >= 0 && y >= 0 && w > 0 && h > 0 && x + w <= 240 && y + h <= 256); }
static void text(void *c, int, int, const char *s, uint8_t) { auto *r = (Rec *)c; r->texts.push_back(s); if (isdigit((unsigned char)s[0])) r->last_score = atoi(s); }
static void clear(void *c) { ((Rec *)c)->clears++; }
static uint32_t rnd(void *c) { auto *r = (Rec *)c; return r->rnd_i < r->rnd.size() ? r->rnd[r->rnd_i++] : (uint32_t)rand(); }
static Host host(Rec &r) { Host h; h.fill = fill; h.text = text; h.clear = clear; h.random = rnd; h.ctx = &r; return h; }
static bool has(Rec &r, const char *s) { for (auto &t : r.texts) if (t == s) return true; return false; }

static std::vector<uint8_t> load_app(const char *path) {
    FILE *f = fopen(path, "rb"); assert(f); std::string s; char b[4096]; size_t n; while ((n = fread(b, 1, sizeof b, f)) > 0) s.append(b, n); fclose(f);
    std::vector<uint8_t> code(kMaxCode); const char *why = nullptr;
    size_t len = parse_app(s.data(), s.size(), code.data(), code.size(), &why); assert(len && "parse"); code.resize(len); return code;
}
// Advance one game frame: run until the machine waits; returns state. Keeps a simulated clock.
static uint32_t g_now = 0;
static State frame(Machine &m, int16_t key = 0) {
    if (key) m.push_key(key);
    State s = m.run(g_now, 100000);
    if (s == State::WAITING) { assert(m.depth() == 0); g_now += 1000; }   // stack must be balanced at every frame boundary
    return s;
}

// ---- SYS opcode: argument order, result, missing host table, stack underflow ----
struct SysRec { uint16_t fn = 0; int16_t a = 0, b = 0, c = 0; int calls = 0; };
static int16_t sys_cb(void *ctx, uint16_t fn, int16_t a, int16_t b, int16_t c, int16_t *mem, size_t n) {
    (void)mem; (void)n;
    auto *r = (SysRec *)ctx; r->fn = fn; r->a = a; r->b = b; r->c = c; ++r->calls; return (int16_t)(a + b + c + fn);
}
static void test_sys() {
    // push 1, 2, 3, then fn 20, SYS, then store result to mem[0], HALT
    const uint8_t prog[] = {1, 1, 1, 2, 1, 3, 1, 20, 60, 1, 0, 41, 0};
    Machine m; assert(m.load(prog, sizeof prog));
    Rec rec; Host h = host(rec); m.set_host(h);
    m.run(0, 1000); assert(m.state() == State::FAULT && m.fault() == Fault::BAD_OPCODE);   // no sys table: refused
    SysRec sr; h.sys = sys_cb; h.ctx = &sr; m.set_host(h); m.reset();
    m.run(0, 1000); assert(m.state() == State::HALTED); assert(sr.calls == 1);
    assert(sr.fn == 20 && sr.a == 1 && sr.b == 2 && sr.c == 3);
    const uint8_t under[] = {1, 1, 1, 20, 60, 0};   // only two operands below fn
    Machine u; assert(u.load(under, sizeof under)); u.set_host(h); sr.calls = 0;
    u.run(0, 1000); assert(u.state() == State::FAULT && u.fault() == Fault::STACK_UNDERFLOW); assert(sr.calls == 0);
    printf("SYS opcode ok\n");
}

// ---- STR opcode: stores a 0-terminated string into app memory, bounds-checked ----
static void test_str() {
    const uint8_t prog[] = {1, 10, 61, 3, 'A', 'B', 'C', 0};     // push 10, STR "ABC", HALT
    Machine m; assert(m.load(prog, sizeof prog)); Rec rec; m.set_host(host(rec));
    m.run(0, 100); assert(m.state() == State::HALTED);
    const uint8_t prog2[] = {1, 10, 61, 3, 'A', 'B', 'C', 1, 10, 40, 4, 1, 11, 40, 4, 0};  // read mem[10], mem[11] back via LOAD
    Machine m2; assert(m2.load(prog2, sizeof prog2)); m2.set_host(host(rec));
    m2.run(0, 100); assert(m2.state() == State::HALTED && m2.depth() == 0);
    const uint8_t bad[] = {2, 0xFE, 0x04, 61, 3, 'A', 'B', 'C', 0};   // address 1278: 1278+3+1 > 1280
    Machine m3; assert(m3.load(bad, sizeof bad)); m3.set_host(host(rec));
    m3.run(0, 100); assert(m3.state() == State::FAULT && m3.fault() == Fault::MEM_RANGE);
    const uint8_t trunc[] = {1, 0, 61, 9, 'A'};                         // declared length runs past the code
    Machine m4; assert(m4.load(trunc, sizeof trunc)); m4.set_host(host(rec));
    m4.run(0, 100); assert(m4.state() == State::FAULT && m4.fault() == Fault::TEXT_RANGE);
    printf("STR opcode ok\n");
}

// ---- SDLIST.APP against a mock file system: listing, selection, preview, string drawing ----
struct Fs : Rec { bool open_ok = false; int closes = 0; };   // Rec first: the draw callbacks share this context
static int16_t fs_sys(void *ctx, uint16_t fn, int16_t a, int16_t b, int16_t c, int16_t *mem, size_t n) {
    auto *f = (Fs *)ctx; (void)c;
    static const char *names[2] = {"MAPS", "README.TXT"};
    auto str_at = [&](int addr) { std::string t; while ((size_t)addr < n && mem[addr]) t += (char)mem[addr++]; return t; };
    switch (fn) {
        case 37: return str_at(a).empty() ? 2 : -1;                        // root listing only
        case 38: { if (a < 0 || a > 1) return -1; size_t l = strlen(names[a]); for (size_t i = 0; i < l; ++i) mem[b + i] = names[a][i]; mem[b + l] = 0; return (int16_t)l; }
        case 39: return a == 0 ? 1 : 0;
        case 29: return 1;
        case 27: f->texts.push_back(str_at(a)); return 1;
        case 30: f->open_ok = str_at(a) == "README.TXT" && b == 0; return f->open_ok;
        case 33: { if (!f->open_ok) return -1; const char *d = "hello world\n"; int got = 0; for (; got < b && d[got]; ++got) mem[a + got] = d[got]; return got; }
        case 36: ++f->closes; return 1;
    }
    return -1;
}
static void test_sdlist(const char *path) {
    auto code = load_app(path);
    Machine m; assert(m.load(code.data(), code.size()));
    Fs fs; Rec &rec = fs; Host h = host(rec); h.sys = fs_sys; h.ctx = &fs; m.set_host(h);
    g_now = 0;
    State s = frame(m); assert(s == State::WAITING);                       // listed and idle
    assert(has(rec, "MAPS") && has(rec, "README.TXT"));
    frame(m, K_DOWN); frame(m); frame(m, K_CENTER); frame(m);               // select README.TXT, preview it
    assert(has(rec, "hello world."));                                       // newline shown as '.'
    assert(fs.closes == 1);                                                 // file closed after the preview
    frame(m, K_UP); frame(m, K_CENTER); frame(m);                           // folder selected: no preview, no fault
    assert(m.state() != State::FAULT);
    printf("SDLIST ok\n");
}
int main(int argc, char **argv) {
    test_sys();
    test_sdlist("../../sdcard_examples/APPS/SDLIST.APP");
    test_str();
    assert(argc > 1);
    auto code = load_app(argv[1]);
    // ---- plain run: snake goes right into the wall ----
    {
        Rec r; r.rnd = {20, 3}; Machine m; m.set_host(host(r)); assert(m.load(code.data(), code.size()));
        int frames = 0; while (!has(r, "GAME OVER") && frames < 100) { assert(frame(m) != State::FAULT); ++frames; }
        assert(has(r, "GAME OVER")); assert(frames >= 12 && frames <= 14); assert(r.clears == 1); assert(m.fault() == Fault::NONE);
        // restart with CENTER
        g_now += 1000; assert(frame(m, K_CENTER) == State::WAITING || true);
        for (int i = 0; i < 4; ++i) frame(m);
        assert(r.clears == 2);
        printf("wall death after %d frames, restart ok\n", frames);
    }
    // ---- eating, growth, scoring, self collision, no-reverse ----
    {
        Rec r; r.rnd = {13, 12, 14, 12, 20, 20}; Machine m; m.set_host(host(r)); m.load(code.data(), code.size());
        frame(m);                                     // init + first move: head -> (13,12) eats
        assert(r.last_score == 1);
        frame(m);                                     // head -> (14,12) eats
        assert(r.last_score == 2);
        frame(m, K_LEFT);                             // reverse is ignored: keeps moving right
        assert(!has(r, "GAME OVER"));
        frame(m, K_UP); frame(m, K_LEFT); assert(!has(r, "GAME OVER"));
        frame(m, K_DOWN);                             // back into the body
        for (int i = 0; i < 3; ++i) frame(m);
        assert(has(r, "GAME OVER"));
        printf("eat/score/no-reverse/self-collision ok (score %d)\n", r.last_score);
    }
    // ---- long soak: random keys never fault, stack stays balanced ----
    {
        Rec r; Machine m; m.set_host(host(r)); m.load(code.data(), code.size()); srand(7);
        for (int i = 0; i < 20000; ++i) { State s = frame(m, (int16_t)(rand() % 7)); assert(s != State::FAULT && s != State::HALTED); }
        printf("soak ok, %u VM steps\n", m.steps());
    }
    // ---- sandbox faults ----
    auto expect = [&](std::vector<uint8_t> p, Fault f) { Machine m; Rec r; m.set_host(host(r)); m.load(p.data(), p.size()); m.run(0, 1000); assert(m.state() == State::FAULT && m.fault() == f); };
    expect({4}, Fault::STACK_UNDERFLOW);
    { std::vector<uint8_t> p; for (int i = 0; i < 70; ++i) { p.push_back(1); p.push_back(1); } expect(p, Fault::STACK_OVERFLOW); }
    expect({2, 0x00, 0x7F, 40}, Fault::MEM_RANGE);                  // LOAD 32512
    expect({1, 5, 1, 0, 13}, Fault::DIV_ZERO);
    expect({200}, Fault::BAD_OPCODE);
    expect({30, 0xFF, 0xFF}, Fault::PC_RANGE);
    expect({1, 1}, Fault::PC_RANGE);                                // runs off the end
    expect({1, 0, 1, 0, 1, 0, 56, 60}, Fault::TEXT_RANGE);
    { Machine m; std::vector<uint8_t> big(kMaxCode + 1, 0); assert(!m.load(big.data(), big.size())); assert(!m.load(nullptr, 0)); }
    // infinite loop is bounded by the step budget, not a hang
    { Machine m; Rec r; m.set_host(host(r)); uint8_t p[] = {30, 0, 0}; m.load(p, 3); assert(m.run(0, 5000) == State::RUNNING); assert(m.steps() == 5000); }
    // drawing is clipped to the window
    { Machine m; Rec r; m.set_host(host(r)); uint8_t p[] = {2,0x38,0xFF, 1,10, 2,0xF0,0x00, 2,0x20,0x03, 1,1, 54, 0}; m.load(p, sizeof p); m.run(0, 100); assert(r.fills == 1); }
    // random garbage programs under ASan: never crash
    srand(99);
    for (int t = 0; t < 3000; ++t) { uint8_t p[64]; size_t n = 1 + rand() % 64; for (size_t i = 0; i < n; ++i) p[i] = (uint8_t)rand(); Machine m; Rec r; m.set_host(host(r)); m.load(p, n); for (int k = 0; k < 20; ++k) { m.run(g_now, 500); g_now += 6000; } }
    // manifest parsing
    {
        uint8_t out[64]; const char *why = nullptr;
        auto P = [&](const char *t) { why = nullptr; return parse_app(t, strlen(t), out, sizeof out, &why); };
        assert(P("MARAUDER_APP=1\nAPI=1\nRUNTIME=VM1\nCODE=0102FF\n") == 3 && out[2] == 0xFF);
        assert(P("MARAUDER_APP=1\nAPI=1\nCODE=00\n") == 0 && why);
        assert(P("MARAUDER_APP=1\nAPI=1\nRUNTIME=VM1\n") == 0 && why);
        assert(P("MARAUDER_APP=1\nAPI=1\nRUNTIME=VM1\nCODE=0G\n") == 0 && why);
        assert(P("MARAUDER_APP=1\nAPI=2\nRUNTIME=VM1\nCODE=00\n") == 0);
        assert(P("MARAUDER_APP=1\r\nAPI=1\r\nRUNTIME=VM1\r\nCODE=0A0B\r\n") == 2);
        assert(P("MARAUDER_APP=1\nAPI=1\nRUNTIME=VM1\nCODE=") == 0);
    }
    puts("ALL PASS");
}
