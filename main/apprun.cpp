#include "include/apprun.hpp"
#include "include/appvm.hpp"
#include "include/display.hpp"
#include "include/storage.hpp"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_random.h"
#include <new>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace marauder {
namespace {
constexpr char kTag[] = "apprun";
constexpr int kTop = 36;                                  // window origin on screen
constexpr size_t kMaxFile = 6144;
vm1::Machine *s_vm = nullptr;
char s_name[24] = {};
bool s_faulted = false, s_halted = false;

const uint16_t kColor[8] = {0x0000, 0xFFFF, 0x07E0, 0xF800, 0x07FF, 0xFEA0, 0x7BEF, 0x001F};
void h_fill(void *, int x, int y, int w, int h, uint8_t c) { display_fill_rect(x, kTop + y, w, h, kColor[c & 7]); }
void h_text(void *, int x, int y, const char *s, uint8_t c) { display_text(x, kTop + y, s, kColor[c & 7], 0x0000); }
void h_clear(void *) { display_fill_rect(0, kTop, 240, 256, 0x0000); }
uint32_t h_random(void *) { return esp_random(); }
const AppSysApi *s_sys = nullptr;
uint8_t s_text_color = 1;                                // colour for fn 26 / 27, set by fn 29
int16_t h_sys(void *, uint16_t fn, int16_t a, int16_t b, int16_t c, int16_t *mem, size_t mem_words) {
    if (!s_sys || !s_sys->call) return -1;
    if (fn == 26) {   // draw string result at window (b, c): the SSID of Wi-Fi scan entry a
        const char *t = s_sys->text ? s_sys->text(fn, a) : nullptr;
        if (!t || !t[0]) return 0;
        if (b < 0 || b >= (int)vm1::kWinW || c < 0 || c >= (int)vm1::kWinH) return 0;
        char buf[33]; snprintf(buf, sizeof(buf), "%.32s", t);
        h_text(nullptr, b, c, buf, s_text_color);
        return (int16_t)strlen(buf);
    }
    if (fn == 27) {   // draw the 0-terminated string stored in the app's memory at address a, at window (b, c)
        if (b < 0 || b >= (int)vm1::kWinW || c < 0 || c >= (int)vm1::kWinH || a < 0 || (size_t)a >= mem_words) return 0;
        char buf[41]; size_t i = 0;
        while (i < 40 && (size_t)a + i < mem_words && mem[a + i] != 0) { buf[i] = (char)(mem[a + i] & 0x7F); ++i; }
        buf[i] = 0;
        h_text(nullptr, b, c, buf, s_text_color);
        return (int16_t)i;
    }
    if (fn == 29) { s_text_color = (uint8_t)(a & 7); return 1; }   // text colour for 26 / 27
    return s_sys->call(fn, a, b, c, mem, mem_words);
}
}
void app_run_set_sys(const AppSysApi *api) { s_sys = api; }

bool app_run_start(const char *rel, char *why, size_t why_size) {
    app_run_stop();
    auto fail = [&](const char *m) { snprintf(why, why_size, "%s", m); return false; };
    const size_t sz = storage_file_size(rel);
    if (!sz) return fail("EMPTY OR MISSING FILE");
    if (sz > kMaxFile) return fail("APP FILE OVER 6 KB");
    char *text = (char *)malloc(sz + 1);
    s_vm = new (std::nothrow) vm1::Machine();
    if (!text || !s_vm) {
        const size_t need = sz + 1 + sizeof(vm1::Machine);
        free(text); delete s_vm; s_vm = nullptr;
        snprintf(why, why_size, "NO MEMORY: NEED %uK HAVE %uK", (unsigned)((need + 1023) / 1024),
                 (unsigned)(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT) / 1024));
        return false;
    }
    const size_t got = storage_read_file(rel, 0, text, sz);
    text[got] = 0;
    const char *reason = "BAD APP";
    const size_t len = vm1::parse_app(text, got, s_vm->code_buffer(), vm1::kMaxCode, &reason);
    s_name[0] = 0;
    if (len) {
        const char *n = strstr(text, "NAME=");
        if (n) { n += 5; size_t z = strcspn(n, "\r\n"); if (z >= sizeof(s_name)) z = sizeof(s_name) - 1; memcpy(s_name, n, z); s_name[z] = 0; }
    }
    free(text);
    if (!len || !s_vm->commit(len)) { delete s_vm; s_vm = nullptr; return fail(len ? "PROGRAM REJECTED" : reason); }
    vm1::Host h; h.fill = h_fill; h.text = h_text; h.clear = h_clear; h.random = h_random; h.sys = h_sys;
    s_vm->set_host(h);
    s_faulted = s_halted = false;
    ESP_LOGI(kTag, "started %s (%u bytes of code)", s_name, (unsigned)len);
    return true;
}
void app_run_stop() {
    const bool was = s_vm != nullptr;
    delete s_vm; s_vm = nullptr; s_faulted = s_halted = false;
    if (was && s_sys && s_sys->on_stop) s_sys->on_stop();
}
void app_run_restart() { if (s_vm) { s_vm->reset(); s_faulted = s_halted = false; } }
void app_run_key(int16_t k) { if (s_vm) s_vm->push_key(k); }
bool app_run_active() { return s_vm != nullptr; }
const char *app_run_name() { return s_name[0] ? s_name : "APP"; }
void app_run_tick(uint32_t now_ms) {
    if (!s_vm) return;
    const vm1::State st = s_vm->run(now_ms, 4000);
    if (st == vm1::State::FAULT && !s_faulted) {
        s_faulted = true;
        char l[40]; snprintf(l, sizeof(l), "APP FAULT: %s", vm1::Machine::fault_text(s_vm->fault()));
        display_text(0, kTop + 120, l, 0xF800, 0x0000);
    } else if (st == vm1::State::HALTED && !s_halted) {
        s_halted = true;
        display_text(0, kTop + 120, "PROGRAM ENDED", 0xFEA0, 0x0000);
    }
}
}
