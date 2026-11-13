#include "include/appvm.hpp"
#include <stdio.h>
#include <string.h>

namespace marauder {
namespace vm1 {

const char *Machine::fault_text(Fault f) {
    switch (f) {
        case Fault::BAD_OPCODE: return "BAD OPCODE"; case Fault::PC_RANGE: return "PC OUT OF RANGE";
        case Fault::STACK_UNDERFLOW: return "STACK UNDERFLOW"; case Fault::STACK_OVERFLOW: return "STACK OVERFLOW";
        case Fault::MEM_RANGE: return "MEMORY RANGE"; case Fault::DIV_ZERO: return "DIVIDE BY ZERO";
        case Fault::TEXT_RANGE: return "TEXT RANGE"; case Fault::TOO_LARGE: return "PROGRAM TOO LARGE"; default: return "NONE";
    }
}
bool Machine::load(const uint8_t *code, size_t len) {
    if (!code || !len || len > kMaxCode) { fail(Fault::TOO_LARGE); state_ = State::EMPTY; return false; }
    memcpy(code_, code, len); len_ = len; reset(); return true;
}
bool Machine::commit(size_t len) {
    if (!len || len > kMaxCode) { fail(Fault::TOO_LARGE); state_ = State::EMPTY; return false; }
    len_ = len; reset(); return true;
}
void Machine::reset() {
    pc_ = 0; sp_ = 0; memset(mem_, 0, sizeof(mem_)); key_ = 0; wake_ms_ = 0; total_steps_ = 0;
    fault_ = Fault::NONE; state_ = len_ ? State::RUNNING : State::EMPTY;
}
bool Machine::push(int32_t v) { if (sp_ >= kStack) { fail(Fault::STACK_OVERFLOW); return false; } stack_[sp_++] = (int16_t)v; return true; }
bool Machine::pop(int32_t *v) { if (!sp_) { fail(Fault::STACK_UNDERFLOW); return false; } *v = stack_[--sp_]; return true; }

State Machine::run(uint32_t now_ms, uint32_t max_steps) {
    if (state_ == State::WAITING) { if ((int32_t)(now_ms - wake_ms_) < 0) return state_; state_ = State::RUNNING; }
    if (state_ != State::RUNNING) return state_;
    int32_t a, b, c, d, e;
    for (uint32_t step = 0; step < max_steps && state_ == State::RUNNING; ++step) {
        if (pc_ >= len_) { fail(Fault::PC_RANGE); break; }
        const uint8_t op = code_[pc_++]; ++total_steps_;
        switch (op) {
            case HALT: state_ = State::HALTED; break;
            case PUSH8: if (pc_ >= len_) { fail(Fault::PC_RANGE); break; } push((int8_t)code_[pc_++]); break;
            case PUSH16: if (pc_ + 1 >= len_) { fail(Fault::PC_RANGE); break; } push((int16_t)(code_[pc_] | (code_[pc_ + 1] << 8))); pc_ += 2; break;
            case DUP: if (!pop(&a)) break; push(a); push(a); break;
            case DROP: pop(&a); break;
            case SWAP: if (!pop(&a) || !pop(&b)) break; push(a); push(b); break;
            case OVER: if (!pop(&a) || !pop(&b)) break; push(b); push(a); push(b); break;
            case ADD: case SUB: case MUL: case DIV: case MOD: case AND: case OR: case XOR:
            case LT: case GT: case EQ: case NE: case LE: case GE: {
                if (!pop(&b) || !pop(&a)) break;
                int32_t r = 0;
                switch (op) {
                    case ADD: r = a + b; break; case SUB: r = a - b; break; case MUL: r = a * b; break;
                    case DIV: if (!b) { fail(Fault::DIV_ZERO); break; } r = a / b; break;
                    case MOD: if (!b) { fail(Fault::DIV_ZERO); break; } r = a % b; break;
                    case AND: r = a & b; break; case OR: r = a | b; break; case XOR: r = a ^ b; break;
                    case LT: r = a < b; break; case GT: r = a > b; break; case EQ: r = a == b; break;
                    case NE: r = a != b; break; case LE: r = a <= b; break; case GE: r = a >= b; break;
                }
                if (state_ == State::RUNNING) push(r);
                break; }
            case NEG: if (pop(&a)) push(-a); break;
            case NOT: if (pop(&a)) push(a == 0); break;
            case JMP: case JZ: case JNZ: {
                if (pc_ + 1 >= len_) { fail(Fault::PC_RANGE); break; }
                const size_t target = code_[pc_] | (code_[pc_ + 1] << 8); pc_ += 2;
                bool take = true;
                if (op != JMP) { if (!pop(&a)) break; take = op == JZ ? a == 0 : a != 0; }
                if (take) { if (target >= len_) { fail(Fault::PC_RANGE); break; } pc_ = target; }
                break; }
            case LOAD: if (!pop(&a)) break; if (a < 0 || a >= (int32_t)kMem) { fail(Fault::MEM_RANGE); break; } push(mem_[a]); break;
            case STORE: if (!pop(&a) || !pop(&b)) break; if (a < 0 || a >= (int32_t)kMem) { fail(Fault::MEM_RANGE); break; } mem_[a] = (int16_t)b; break;
            case RND: if (!pop(&a)) break; push(a > 0 && host_.random ? (int32_t)(host_.random(host_.ctx) % (uint32_t)a) : 0); break;
            case KEY: push(key_); key_ = 0; break;
            case FRAME: if (!pop(&a)) break; if (a < 0) a = 0; if (a > 5000) a = 5000; wake_ms_ = now_ms + (uint32_t)a; state_ = State::WAITING; break;
            case CLS: if (host_.clear) host_.clear(host_.ctx); break;
            case RECT: {   // x y w h color
                if (!pop(&e) || !pop(&d) || !pop(&c) || !pop(&b) || !pop(&a)) break;
                int x = a, y = b, w = c, h = d;
                if (x < 0) { w += x; x = 0; }
                if (y < 0) { h += y; y = 0; }
                if (x + w > (int)kWinW) { w = (int)kWinW - x; }
                if (y + h > (int)kWinH) { h = (int)kWinH - y; }
                if (w > 0 && h > 0 && host_.fill) host_.fill(host_.ctx, x, y, w, h, (uint8_t)(e & 7));
                break; }
            case NUM: {    // x y value color
                if (!pop(&d) || !pop(&c) || !pop(&b) || !pop(&a)) break;
                char t[12]; snprintf(t, sizeof(t), "%d", (int)c);
                if (a >= 0 && a < (int32_t)kWinW && b >= 0 && b < (int32_t)kWinH && host_.text) host_.text(host_.ctx, a, b, t, (uint8_t)(d & 7));
                break; }
            case TEXT: {   // inline: len, chars; stack: x y color
                if (pc_ >= len_) { fail(Fault::PC_RANGE); break; }
                const size_t n = code_[pc_++];
                if (n > kMaxText || pc_ + n > len_) { fail(Fault::TEXT_RANGE); break; }
                char t[kMaxText + 1]; memcpy(t, code_ + pc_, n); t[n] = 0; pc_ += n;
                if (!pop(&c) || !pop(&b) || !pop(&a)) break;
                if (a >= 0 && a < (int32_t)kWinW && b >= 0 && b < (int32_t)kWinH && host_.text) host_.text(host_.ctx, a, b, t, (uint8_t)(c & 7));
                break; }
            default: fail(Fault::BAD_OPCODE); break;
        }
    }
    return state_;
}

static int hexv(char c) { if (c >= '0' && c <= '9') return c - '0'; if (c >= 'a' && c <= 'f') return c - 'a' + 10; if (c >= 'A' && c <= 'F') return c - 'A' + 10; return -1; }
size_t parse_app(const char *text, size_t n, uint8_t *out, size_t cap, const char **why) {
    static const char *kNoMagic = "NOT A MARAUDER APP", *kRuntime = "RUNTIME NOT VM1", *kNoCode = "NO CODE= LINE", *kBad = "BAD HEX IN CODE", *kBig = "PROGRAM TOO LARGE";
    auto find = [&](const char *key) -> const char * {
        const size_t kl = strlen(key);
        for (size_t i = 0; i + kl <= n; ++i) if ((i == 0 || text[i - 1] == '\n') && !memcmp(text + i, key, kl)) return text + i + kl;
        return nullptr;
    };
    if (!find("MARAUDER_APP=1") || !find("API=1")) { *why = kNoMagic; return 0; }
    const char *rt = find("RUNTIME=");
    if (!rt || n - (size_t)(rt - text) < 3 || memcmp(rt, "VM1", 3)) { *why = kRuntime; return 0; }
    const char *code = find("CODE=");
    if (!code) { *why = kNoCode; return 0; }
    size_t len = 0; const char *end = text + n;
    while (code + 1 < end) {
        if (*code == '\r' || *code == '\n') break;
        const int h = hexv(code[0]), l = hexv(code[1]);
        if (h < 0 || l < 0) { *why = kBad; return 0; }
        if (len >= cap || len >= kMaxCode) { *why = kBig; return 0; }
        out[len++] = (uint8_t)(h * 16 + l); code += 2;
    }
    if (!len) { *why = kNoCode; return 0; }
    return len;
}
}  // namespace vm1
}  // namespace marauder
