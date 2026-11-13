#include "include/transmitter.hpp"
#include "include/storage.hpp"
#include "radio_backend.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include <atomic>
#include <cstdio>
#include <cstring>

namespace marauder {
namespace {
constexpr char kTag[] = "foxtx";
constexpr uint8_t kVersion = 2;

struct HopSet { const char *name; uint8_t count; uint16_t mhz[24]; };
constexpr HopSet kHopSets[] = {
    {"1-6-11", 3, {2412, 2437, 2462}},
    {"1-4-7-10-13", 5, {2412, 2427, 2442, 2457, 2472}},
    {"ALL 13", 13, {2412, 2447, 2422, 2457, 2432, 2467, 2442, 2417, 2452, 2427, 2462, 2437, 2472}},
    {"WIDE 100 MHZ", 11, {1802, 1902, 2002, 2102, 2202, 2302, 2402, 2502, 2602, 2702, 2802}},
    {"WIDE 50 MHZ", 21, {1802, 1852, 1902, 1952, 2002, 2052, 2102, 2152, 2202, 2252, 2302, 2352, 2402, 2452, 2502, 2552, 2602, 2652, 2702, 2752, 2802}},
};
constexpr unsigned kHopSetCount = sizeof(kHopSets) / sizeof(*kHopSets);

std::atomic<bool> s_stop{false};
std::atomic<bool> s_running{false};
std::atomic<bool> s_carrier_on{false};
std::atomic<uint16_t> s_freq{2437};
std::atomic<uint32_t> s_hops{0}, s_ids{0};
std::atomic<int32_t> s_last_err{ESP_OK};
std::atomic<int64_t> s_start_us{0};
TaskHandle_t s_task{};          // published by the task itself, cleared under s_cfg_lock
std::atomic<bool> s_starting{false};
portMUX_TYPE s_cfg_lock = portMUX_INITIALIZER_UNLOCKED;
FoxConfig s_live{};
char s_manual_call[kFoxTextMax + 1]; // copy of the live manual callsign (read without the lock)

// Files read from the SD card.
char s_messages[kFoxMaxMessages][kFoxMessageMax + 1];
unsigned s_message_count = 0;
char s_file_call[kFoxTextMax + 1];

FoxConfig live() {
    FoxConfig c;
    portENTER_CRITICAL(&s_cfg_lock);
    c = s_live;
    portEXIT_CRITICAL(&s_cfg_lock);
    return c;
}

uint64_t now_ms() { return (uint64_t)(esp_timer_get_time() / 1000); }
uint32_t jitter(uint32_t x) { return (uint32_t)(((uint64_t)x * (70u + esp_random() % 61u)) / 100u); }

bool duration_expired(const FoxConfig &c) {
    if (!c.duration_min || c.mode == (uint8_t)FoxMode::STEADY) return false;
    return (esp_timer_get_time() - s_start_us.load()) >= (int64_t)c.duration_min * 60ll * 1000000ll;
}

bool should_stop() {
    if (s_stop.load()) return true;
    if (duration_expired(live())) { s_stop.store(true); return true; }
    return false;
}

// Interruptible wait (a stop request or a setting change wakes it). Returns true if stopping.
bool wait_ms(uint32_t ms) {
    while (ms) {
        const uint32_t slice = ms > 100u ? 100u : ms;
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(slice) ? pdMS_TO_TICKS(slice) : 1);
        if (should_stop()) return true;
        ms -= slice;
    }
    return should_stop();
}

struct Ctx { unsigned freq; unsigned backoff; bool on; };

bool apply(Ctx &c, bool on) {
    const esp_err_t e = marauder_radio_tx_cw_set(on, fox_freq_channel(c.freq), c.backoff);
    if (e != ESP_OK) { s_last_err.store(e); return false; }
    c.on = on;
    s_carrier_on.store(on);
    return true;
}

bool carrier(Ctx &c, bool on) { return c.on == on ? true : apply(c, on); }

// Move to a new frequency / power. Re-keys the carrier if it is on.
bool tune(Ctx &c, unsigned freq, unsigned backoff) {
    if (c.freq == freq && c.backoff == backoff) return true;
    const bool was = c.on;
    if (c.freq != freq) { s_hops.fetch_add(1); s_freq.store((uint16_t)freq); }
    c.freq = freq;
    c.backoff = backoff;
    return was ? apply(c, true) : true; // marauder_radio_tx_cw_set stops the old tone first
}

const char *morse_code(char ch) {
    static const char *const letters[26] = {".-","-...","-.-.","-..",".","..-.","--.","....","..",".---","-.-",".-..","--","-.","---",".--.","--.-",".-.","...","-","..-","...-",".--","-..-","-.--","--.."};
    static const char *const digits[10] = {"-----",".----","..---","...--","....-",".....","-....","--...","---..","----."};
    if (ch >= 'a' && ch <= 'z') ch = (char)(ch - 'a' + 'A');
    if (ch >= 'A' && ch <= 'Z') return letters[ch - 'A'];
    if (ch >= '0' && ch <= '9') return digits[ch - '0'];
    switch (ch) { case '/': return "-..-."; case '.': return ".-.-.-"; case ',': return "--..--"; case '?': return "..--.."; case '=': return "-...-"; case '-': return "-....-"; default: return nullptr; }
}

// Uppercase and keep only characters Morse can send (and single spaces). Returns the new length.
size_t clean_text(char *dst, size_t cap, const char *src) {
    size_t n = 0;
    bool space = false;
    for (const char *p = src; *p && n + 1 < cap; ++p) {
        char ch = *p;
        if (ch >= 'a' && ch <= 'z') ch = (char)(ch - 'a' + 'A');
        if (ch == ' ' || ch == '\t' || ch == '\r' || ch == '\n') { space = n > 0; continue; }
        if (!morse_code(ch) || ch == ',') continue; // ',' separates messages in morse.txt, never sent
        if (space) { dst[n++] = ' '; space = false; if (n + 1 >= cap) break; }
        dst[n++] = ch;
    }
    dst[n] = 0;
    return n;
}

// Re-read the live settings and apply tuning at a safe point (carrier off).
void sync_tuning(Ctx &c) {
    const FoxConfig cfg = live();
    if (cfg.mode == (uint8_t)FoxMode::HOP) return; // hop owns the frequency
    (void)tune(c, cfg.freq_mhz, fox_backoff_quarter_db(cfg.power));
}

// Key a text in Morse. Standard timing: dit = 1200 / wpm ms, dah = 3, element gap 1, letter gap 3, word gap 7.
// Returns true if a stop was requested part way.
bool send_text(Ctx &c, const char *text, bool restore_on) {
    (void)carrier(c, false);
    for (const char *s = text; *s; ++s) {
        const uint32_t dit = 1200u / (live().wpm ? live().wpm : 15u);
        if (*s == ' ') { if (wait_ms(4 * dit)) return true; continue; }
        const char *code = morse_code(*s);
        if (!code) continue;
        for (const char *e = code; *e; ++e) {
            sync_tuning(c); // settings changed while sending take effect between elements
            if (!carrier(c, true)) return true;
            if (wait_ms(*e == '.' ? dit : 3 * dit)) return true;
            if (!carrier(c, false)) return true;
            if (wait_ms(dit)) return true;
        }
        if (wait_ms(2 * dit)) return true;
    }
    if (restore_on) (void)carrier(c, true);
    return false;
}

void tx_task(void *) {
    portENTER_CRITICAL(&s_cfg_lock);
    s_task = xTaskGetCurrentTaskHandle();
    portEXIT_CRITICAL(&s_cfg_lock);
    s_starting.store(false);
    FoxConfig cfg = live();
    Ctx c{cfg.freq_mhz, fox_backoff_quarter_db(cfg.power), false};
    if (cfg.mode == (uint8_t)FoxMode::HOP) c.freq = kHopSets[cfg.hop_set % kHopSetCount].mhz[0];
    esp_err_t err = marauder_radio_tx_cw_begin();
    if (err == ESP_OK && cfg.mode != (uint8_t)FoxMode::MORSE) {
        err = marauder_radio_tx_cw_set(true, fox_freq_channel(c.freq), c.backoff);
        c.on = (err == ESP_OK);
    }
    if (err != ESP_OK) {
        s_last_err.store(err);
        marauder_radio_tx_cw_end();
        s_running.store(false);
        portENTER_CRITICAL(&s_cfg_lock);
        s_task = nullptr;
        portEXIT_CRITICAL(&s_cfg_lock);
        vTaskDelete(nullptr);
        return;
    }
    s_last_err.store(ESP_OK);
    s_freq.store((uint16_t)c.freq);
    s_carrier_on.store(c.on);
    s_start_us.store(esp_timer_get_time());
    s_running.store(true);
    ESP_LOGI(kTag, "TX start: %u MHz (channel %d)", c.freq, fox_freq_channel(c.freq));

    uint8_t last_mode = cfg.mode;
    uint64_t next_id = now_ms() + 1500;
    uint64_t next_step = now_ms() + cfg.interval_ms;
    bool burst_on = true;
    unsigned hop_idx = 0;
    int msg_idx = -1;

    while (!should_stop()) {
        cfg = live();
        const unsigned backoff = fox_backoff_quarter_db(cfg.power);
        const HopSet &hop = kHopSets[cfg.hop_set % kHopSetCount];
        const bool morse = cfg.mode == (uint8_t)FoxMode::MORSE;

        if (cfg.mode != last_mode) { // mode changed on the fly: restart that mode's state
            last_mode = cfg.mode;
            burst_on = true;
            hop_idx = 0;
            next_step = now_ms() + cfg.interval_ms;
            if (!morse) (void)carrier(c, true);
        }
        // Frequency / power follow the settings (hop picks its own frequency).
        const unsigned want_freq = cfg.mode == (uint8_t)FoxMode::HOP ? hop.mhz[hop_idx % hop.count] : cfg.freq_mhz;
        if (!tune(c, want_freq, backoff)) break;

        if (morse) {
            const char *text = nullptr;
            if (s_message_count) {
                msg_idx = cfg.random_order ? (int)(esp_random() % s_message_count) : (msg_idx + 1) % (int)s_message_count;
                text = s_messages[msg_idx];
            } else if (cfg.text[0]) {
                text = cfg.text;
            }
            if (text) {
                if (send_text(c, text, false)) break;
                s_ids.fetch_add(1);
            }
            if (wait_ms(cfg.interval_ms)) break;
            continue;
        }

        const char *call = fox_callsign();
        const bool id_on = call[0] && cfg.id_every_s;
        const uint64_t t = now_ms();
        if (id_on && t >= next_id) {
            const bool restore = burst_on || cfg.mode != (uint8_t)FoxMode::BURST;
            if (send_text(c, call, restore)) break;
            s_ids.fetch_add(1);
            next_id = now_ms() + (uint64_t)cfg.id_every_s * 1000u;
            continue;
        }
        if (cfg.mode == (uint8_t)FoxMode::HOP && t >= next_step) {
            hop_idx = (hop_idx + 1) % hop.count;
            if (!tune(c, hop.mhz[hop_idx], backoff)) break;
            next_step = now_ms() + cfg.interval_ms;
        } else if (cfg.mode == (uint8_t)FoxMode::BURST && t >= next_step) {
            const uint32_t on_ms = cfg.interval_ms * cfg.duty_pct / 100u;
            const uint32_t off_ms = cfg.interval_ms - on_ms;
            burst_on = !burst_on;
            if (!carrier(c, burst_on)) break;
            const uint32_t d = burst_on ? on_ms : off_ms;
            next_step = now_ms() + (d < 20u ? 20u : jitter(d));
        } else if (cfg.mode == (uint8_t)FoxMode::STEADY || cfg.mode == (uint8_t)FoxMode::HOP) {
            if (!c.on && !carrier(c, true)) break;
        }
        uint64_t deadline = UINT64_MAX;
        if (cfg.mode == (uint8_t)FoxMode::HOP || cfg.mode == (uint8_t)FoxMode::BURST) deadline = next_step;
        if (id_on && next_id < deadline) deadline = next_id;
        const uint64_t n = now_ms();
        uint32_t w = deadline == UINT64_MAX ? 250u : (deadline > n ? (uint32_t)(deadline - n) : 1u);
        if (w > 250u) w = 250u;
        if (wait_ms(w)) break;
    }

    marauder_radio_tx_cw_end();
    s_carrier_on.store(false);
    s_running.store(false);
    ESP_LOGI(kTag, "TX stopped");
    portENTER_CRITICAL(&s_cfg_lock);
    s_task = nullptr;
    portEXIT_CRITICAL(&s_cfg_lock);
    vTaskDelete(nullptr);
}

// Read a small text file from the SD card into buf. Returns the length (0 if missing).
size_t read_small_file(const char *path, char *buf, size_t cap) {
    if (!storage_accessible() || !storage_file_exists(path)) return 0;
    const size_t n = storage_read_file(path, 0, buf, cap - 1);
    buf[n] = 0;
    return n;
}
}

FoxConfig fox_config_defaults() {
    FoxConfig c{};
    c.mode = (uint8_t)FoxMode::STEADY;
    c.power = 0;
    c.hop_set = 0;
    c.duty_pct = 20;
    c.wpm = 15;
    c.random_order = 0;
    c.freq_mhz = 2437;
    c.interval_ms = 1500;
    c.id_every_s = 0;
    c.duration_min = 0;
    return c; // no callsign: it comes from callsign.txt or the user
}

void fox_config_sanitize(FoxConfig &c) {
    if (c.mode >= (uint8_t)FoxMode::COUNT) c.mode = (uint8_t)FoxMode::STEADY;
    if (c.power > kFoxPowerMax) c.power = 0;
    if (c.hop_set >= kHopSetCount) c.hop_set = 0;
    if (c.duty_pct < 10 || c.duty_pct > 90) c.duty_pct = 20;
    if (c.wpm < 5 || c.wpm > 30) c.wpm = 15;
    c.random_order = c.random_order ? 1 : 0;
    if (c.freq_mhz < kFoxFreqMin) c.freq_mhz = kFoxFreqMin;
    if (c.freq_mhz > kFoxFreqMax) c.freq_mhz = kFoxFreqMax;
    c.freq_mhz = (uint16_t)(kFoxFreqMin + ((c.freq_mhz - kFoxFreqMin + 2) / 5) * 5); // snap to the 5 MHz grid
    if (c.freq_mhz > kFoxFreqMax) c.freq_mhz = kFoxFreqMax;
    if (c.interval_ms < 200 || c.interval_ms > 20000) c.interval_ms = 1500;
    if (c.id_every_s > 3600) c.id_every_s = 0;
    if (c.duration_min > 720) c.duration_min = 0;
    c.text[kFoxTextMax] = 0;
    c.call[kFoxTextMax] = 0;
    char clean[kFoxTextMax + 1];
    clean_text(clean, sizeof(clean), c.text);
    memset(c.text, 0, sizeof(c.text));
    memcpy(c.text, clean, strlen(clean));
    clean_text(clean, sizeof(clean), c.call);
    memset(c.call, 0, sizeof(c.call));
    memcpy(c.call, clean, strlen(clean));
}

void fox_config_load(FoxConfig *config) {
    *config = fox_config_defaults();
    nvs_handle_t h;
    if (nvs_open("foxtx", NVS_READONLY, &h) != ESP_OK) return;
    uint8_t ver = 0; FoxConfig tmp{}; size_t len = sizeof(tmp);
    if (nvs_get_u8(h, "ver", &ver) == ESP_OK && ver == kVersion && nvs_get_blob(h, "cfg", &tmp, &len) == ESP_OK && len == sizeof(tmp)) {
        fox_config_sanitize(tmp);
        *config = tmp;
    }
    nvs_close(h);
}

void fox_config_save(const FoxConfig &config) {
    nvs_handle_t h;
    if (nvs_open("foxtx", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u8(h, "ver", kVersion);
    nvs_set_blob(h, "cfg", &config, sizeof(config));
    nvs_commit(h);
    nvs_close(h);
}

const char *fox_mode_name(uint8_t mode) {
    switch (mode) { case (uint8_t)FoxMode::STEADY: return "STEADY"; case (uint8_t)FoxMode::BURST: return "BURST"; case (uint8_t)FoxMode::HOP: return "HOP"; case (uint8_t)FoxMode::MORSE: return "MORSE"; default: return "?"; }
}
unsigned fox_hop_set_count() { return kHopSetCount; }
const char *fox_hop_set_name(unsigned i) { return kHopSets[i % kHopSetCount].name; }
unsigned fox_backoff_quarter_db(unsigned power) {
    // power 32 = +20 dBm nominal = no attenuation; each step down is 1 dB (4 quarter-dB units), floor 31.75 dB.
    const int b = 4 * ((int)kFoxPowerMax - (int)power);
    return b > 127 ? 127u : (unsigned)b;
}
int fox_power_dbm(unsigned power) { return (int)power - 12; }
int fox_freq_channel(unsigned freq_mhz) {
    if (freq_mhz == 2484) return 14;
    return ((int)freq_mhz - 2407) / 5;
}

void fox_files_load() {
    static char buf[1536]; // static: keep the UI task stack small
    s_message_count = 0;
    s_file_call[0] = 0;
    if (read_small_file("callsign.txt", buf, sizeof(buf)) > 0) {
        char first[kFoxTextMax + 1];
        const char *line_end = buf + strcspn(buf, "\r\n,");
        char tmp[kFoxTextMax * 2 + 2];
        const size_t take = (size_t)(line_end - buf) < sizeof(tmp) - 1 ? (size_t)(line_end - buf) : sizeof(tmp) - 1;
        memcpy(tmp, buf, take);
        tmp[take] = 0;
        clean_text(first, sizeof(first), tmp);
        snprintf(s_file_call, sizeof(s_file_call), "%s", first);
    }
    if (read_small_file("morse.txt", buf, sizeof(buf)) > 0) {
        char *save = nullptr;
        for (char *tok = strtok_r(buf, ",", &save); tok && s_message_count < kFoxMaxMessages; tok = strtok_r(nullptr, ",", &save)) {
            if (clean_text(s_messages[s_message_count], sizeof(s_messages[0]), tok) > 0) ++s_message_count;
        }
    }
    ESP_LOGI(kTag, "files: %u messages, callsign %s", s_message_count, s_file_call[0] ? s_file_call : "(none)");
}
unsigned fox_file_message_count() { return s_message_count; }
bool fox_callsign_from_file() { return s_file_call[0] != 0; }
const char *fox_callsign() {
    if (s_file_call[0]) return s_file_call;
    return s_manual_call; // manual; may be empty
}

esp_err_t fox_start(const FoxConfig &config) {
    if (s_task || s_starting.load()) return ESP_ERR_INVALID_STATE;
    FoxConfig c = config;
    fox_config_sanitize(c);
    portENTER_CRITICAL(&s_cfg_lock);
    s_live = c;
    memcpy(s_manual_call, c.call, sizeof(s_manual_call));
    portEXIT_CRITICAL(&s_cfg_lock);
    s_stop.store(false);
    s_hops.store(0);
    s_ids.store(0);
    s_last_err.store(ESP_OK);
    s_carrier_on.store(false);
    s_running.store(true); // visible to the UI at once; the task clears it on failure
    // 8 KiB: the PHY and Wi-Fi calls made when the session ends need far more than 4 KiB.
    s_starting.store(true);
    if (xTaskCreatePinnedToCore(tx_task, "fox_tx", 12288, nullptr, 6, nullptr, 0) != pdPASS) {
        s_starting.store(false);
        s_running.store(false);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void fox_update(const FoxConfig &config) {
    FoxConfig c = config;
    fox_config_sanitize(c);
    portENTER_CRITICAL(&s_cfg_lock);
    s_live = c;
    memcpy(s_manual_call, c.call, sizeof(s_manual_call));
    portEXIT_CRITICAL(&s_cfg_lock);
    portENTER_CRITICAL(&s_cfg_lock);
    TaskHandle_t t = s_task;
    if (t) xTaskNotifyGive(t);
    portEXIT_CRITICAL(&s_cfg_lock);
}

void fox_stop() {
    for (int i = 0; i < 100 && s_starting.load(); ++i) vTaskDelay(pdMS_TO_TICKS(5)); // let a starting task publish itself
    s_stop.store(true);
    portENTER_CRITICAL(&s_cfg_lock);
    TaskHandle_t t = s_task;
    if (t) xTaskNotifyGive(t);
    portEXIT_CRITICAL(&s_cfg_lock);
    if (!t) return;
    for (int i = 0; i < 300; ++i) {
        portENTER_CRITICAL(&s_cfg_lock);
        const bool alive = s_task != nullptr;
        portEXIT_CRITICAL(&s_cfg_lock);
        if (!alive) break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

FoxStatus fox_status() {
    FoxStatus st{};
    st.running = s_running.load();
    st.carrier_on = s_carrier_on.load();
    st.freq_mhz = s_freq.load();
    st.hops = s_hops.load();
    st.ids = s_ids.load();
    st.last_err = (esp_err_t)s_last_err.load();
    st.elapsed_s = st.running ? (uint32_t)((esp_timer_get_time() - s_start_us.load()) / 1000000) : 0;
    return st;
}

}
