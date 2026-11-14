#pragma once
// VM1: a tiny sandboxed bytecode machine for SD-card apps (APPS/*.APP with RUNTIME=VM1 and a CODE= hex line).
// An app can compute, read the joystick, wait, and draw rectangles / text / numbers inside a 240x256 window.
// It cannot touch memory outside its own 1280-word array, the SD card or GPIO. The radio, GPS, Wi-Fi scan and fox
// tone are reachable only through the SYS opcode, which calls the host's table (see APP_API.md).
// Hard limits: code <= 4096 bytes, 64-entry stack, bounded step budget per call, every access bounds-checked.
#include <stddef.h>
#include <stdint.h>

namespace marauder {
namespace vm1 {

constexpr size_t kMaxCode = 4096, kStack = 64, kMem = 1280, kWinW = 240, kWinH = 256, kMaxText = 40;

enum Op : uint8_t {
    HALT = 0, PUSH8 = 1, PUSH16 = 2, DUP = 3, DROP = 4, SWAP = 5, OVER = 6,
    ADD = 10, SUB = 11, MUL = 12, DIV = 13, MOD = 14, AND = 15, OR = 16, XOR = 17, NEG = 18, NOT = 19,
    LT = 20, GT = 21, EQ = 22, NE = 23, LE = 24, GE = 25,
    JMP = 30, JZ = 31, JNZ = 32,
    LOAD = 40, STORE = 41,
    RND = 50, KEY = 51, FRAME = 52, CLS = 53, RECT = 54, NUM = 55, TEXT = 56, SYS = 60, STR = 61,
};
enum Key : int16_t { K_NONE = 0, K_UP = 1, K_DOWN = 2, K_LEFT = 3, K_RIGHT = 4, K_CENTER = 5 };
enum class State : uint8_t { EMPTY, RUNNING, WAITING, HALTED, FAULT };
enum class Fault : uint8_t { NONE, BAD_OPCODE, PC_RANGE, STACK_UNDERFLOW, STACK_OVERFLOW, MEM_RANGE, DIV_ZERO, TEXT_RANGE, TOO_LARGE };

// Colour indices 0..7: black, white, green, red, cyan, amber, gray, blue.
struct Host {
    void (*fill)(void *ctx, int x, int y, int w, int h, uint8_t color) = nullptr;   // window coordinates, already clipped
    void (*text)(void *ctx, int x, int y, const char *s, uint8_t color) = nullptr;
    void (*clear)(void *ctx) = nullptr;
    uint32_t (*random)(void *ctx) = nullptr;
    // SYS opcode; null = faults. `mem` is the app's own word memory so calls can move data in and out; the host must
    // bounds-check every range it touches against mem_words.
    int16_t (*sys)(void *ctx, uint16_t fn, int16_t a, int16_t b, int16_t c, int16_t *mem, size_t mem_words) = nullptr;
    void *ctx = nullptr;
};

class Machine {
public:
    bool load(const uint8_t *code, size_t len);                 // copies the program; false if too large / empty
    uint8_t *code_buffer() { return code_; }                    // parse_app may write straight here (kMaxCode bytes)
    bool commit(size_t len);                                    // adopt `len` bytes already in code_buffer()
    void reset();
    void set_host(const Host &h) { host_ = h; }
    void push_key(int16_t k) { key_ = k; }
    // Runs until FRAME (waits `ms`), HALT, a fault, or max_steps. now_ms is the caller's clock.
    State run(uint32_t now_ms, uint32_t max_steps);
    State state() const { return state_; }
    Fault fault() const { return fault_; }
    static const char *fault_text(Fault f);
    uint32_t steps() const { return total_steps_; }
    size_t depth() const { return sp_; }
private:
    bool push(int32_t v); bool pop(int32_t *v); void fail(Fault f) { fault_ = f; state_ = State::FAULT; }
    uint8_t code_[kMaxCode]; size_t len_ = 0; size_t pc_ = 0;
    int16_t stack_[kStack]; size_t sp_ = 0;
    int16_t mem_[kMem];
    int16_t key_ = 0; uint32_t wake_ms_ = 0; uint32_t total_steps_ = 0;
    State state_ = State::EMPTY; Fault fault_ = Fault::NONE; Host host_;
};

// Parses an .APP text file (as read from the SD card): requires MARAUDER_APP=1, API=1, RUNTIME=VM1 and a CODE=<hex> line.
// Returns the number of code bytes written to `out`, or 0 with `*why` set.
size_t parse_app(const char *text, size_t text_len, uint8_t *out, size_t out_cap, const char **why);

}  // namespace vm1
}  // namespace marauder
