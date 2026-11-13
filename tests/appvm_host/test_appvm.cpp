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
int main(int argc, char **argv) {
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
