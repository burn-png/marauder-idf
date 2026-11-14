#pragma once
#include <stdint.h>
#include "esp_err.h"

namespace marauder {

// Fox beacon on the original ESP32. The only transmit path this chip offers is Espressif's PHY
// test-mode carrier, which sits on a "channel" at 2407 + 5 * channel MHz. Channels 1-14 are the Wi-Fi
// channels the SDK documents; the beacon also passes other channel numbers straight through to reach
// 1802..2802 MHz (on a 5 MHz grid that includes 2412, 2437, ...). The silicon may ignore or reject
// those; that is untested.
enum class FoxMode : uint8_t { STEADY, BURST, HOP, MORSE, MULTI, COUNT }; // MULTI: hop through your own list of frequencies

inline constexpr unsigned kFoxTextMax = 16;
inline constexpr unsigned kFoxPowerMax = 32;        // power 0..32 = -12 .. +20 dBm (nominal)
inline constexpr unsigned kFoxFreqMin = 1802;       // MHz, on the 2407 + 5n grid
inline constexpr unsigned kFoxFreqMax = 2802;
inline constexpr unsigned kFoxMaxMessages = 24;
inline constexpr unsigned kFoxMessageMax = 80;

struct FoxConfig {
    uint8_t mode;         // FoxMode
    uint8_t power;        // 0..32  (-12 .. +20 dBm nominal)
    uint8_t hop_set;      // index into the hop sets
    uint8_t duty_pct;     // burst on-time, 10..90
    uint8_t wpm;          // Morse speed
    uint8_t random_order; // 1 = pick messages at random instead of in order
    uint16_t freq_mhz;    // carrier, kFoxFreqMin..kFoxFreqMax, 5 MHz grid
    uint16_t interval_ms; // burst period / hop dwell / Morse pause
    uint16_t id_every_s;  // CW ID period for steady/burst/hop, 0 = off
    uint16_t duration_min;// stop by itself after this long, 0 = until stopped (steady ignores it)
    uint16_t multi_mhz[12]; // MULTI mode: your frequency list, 1 MHz resolution
    uint16_t multi_on;      // MULTI mode: bit i set = multi_mhz[i] is enabled
    uint8_t multi_count;    // MULTI mode: entries in use, 1..kFoxMultiMax
    char text[kFoxTextMax + 1]; // manual Morse message (used when morse.txt is absent)
    char call[kFoxTextMax + 1]; // manual callsign (used when callsign.txt is absent); never defaulted
};

struct FoxStatus {
    bool running;
    bool carrier_on;
    uint16_t freq_mhz;
    uint32_t elapsed_s;
    uint32_t hops;
    uint32_t ids;
    esp_err_t last_err;
};

inline constexpr unsigned kFoxMultiMax = 12;        // entries in the MULTI list
unsigned fox_multi_enabled_count(const FoxConfig &config);
bool fox_multi_toggle(FoxConfig &config, unsigned index);              // false if it would leave none enabled
bool fox_multi_add(FoxConfig &config, unsigned mhz);                   // appended enabled; false if full
bool fox_multi_remove(FoxConfig &config, unsigned index);              // false if it is the only entry
void fox_multi_set(FoxConfig &config, unsigned index, unsigned mhz);   // clamped to the allowed range

FoxConfig fox_config_defaults();
void fox_config_sanitize(FoxConfig &config);
void fox_config_load(FoxConfig *config);
void fox_config_save(const FoxConfig &config);

const char *fox_mode_name(uint8_t mode);
unsigned fox_hop_set_count();
const char *fox_hop_set_name(unsigned index);
unsigned fox_backoff_quarter_db(unsigned power);
int fox_power_dbm(unsigned power);
int fox_freq_channel(unsigned freq_mhz); // the SDK "channel" number for a frequency

// SD files: morse.txt (messages separated by commas) and callsign.txt (one callsign).
// Re-read every time the fox page is opened.
void fox_files_load();
unsigned fox_file_message_count();
const char *fox_callsign();          // callsign.txt, else the manual one, else ""
bool fox_callsign_from_file();

// The receive task must be paused by the caller before fox_start() and may be
// resumed after fox_stop() (or after the status reports running == false).
esp_err_t fox_start(const FoxConfig &config);
void fox_update(const FoxConfig &config); // apply changed settings to a running transmitter
void fox_stop();
const char *fox_stage();     // last step the transmit task reached (diagnostics)
unsigned fox_stack_bytes();  // stack the transmit task needs
unsigned fox_stack_free_bytes(); // lowest unused stack seen since the last start (0 before the first start)
FoxStatus fox_status();

}
