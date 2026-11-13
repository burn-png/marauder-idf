#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#define MARAUDER_RADIO_FREQ_MIN_MHZ 1842u
#define MARAUDER_RADIO_FREQ_MAX_MHZ 2800u
#define MARAUDER_RADIO_FREQ_BAND_MAX_MHZ 2484u

#ifdef __cplusplus
extern "C" {
#endif

// Receive-only adapter for the ESP-SDR original-ESP32 dump-engine backend.
esp_err_t marauder_radio_init(void);
esp_err_t marauder_radio_capture_iq8(uint32_t center_mhz, size_t count, int8_t *iq_interleaved);
esp_err_t marauder_radio_set_rx_gain(bool automatic, unsigned index);
bool marauder_radio_rx_gain_is_auto(void);
unsigned marauder_radio_rx_gain_index(void);
esp_err_t marauder_radio_bench_tx_once(void);

// Carrier-wave (CW) transmit through Espressif's PHY test mode: a single
// unmodulated carrier at 2407 + 5 * channel MHz. Channels 1-14 are the Wi-Fi channels the SDK
// documents; other channel numbers (negative down to -127, and above 14) are passed to the SDK
// unchecked to reach 1800-2800 MHz, and the silicon may ignore or reject them. The original
// ESP32 has no DAC replay path, so this is the only TX it offers.
// backoff_quarter_db is the SDK power attenuation in 0.25 dB units (0..127;
// 127 = 31.75 dB, the weakest). Call begin, then set() to key/retune, then end.
// The caller must keep the receive task paused for the whole session.
esp_err_t marauder_radio_tx_cw_begin(void);
esp_err_t marauder_radio_tx_cw_set(bool on, int wifi_channel, unsigned backoff_quarter_db);
esp_err_t marauder_radio_tx_cw_end(void);
bool marauder_radio_tx_cw_active(void);

// Passive Wi-Fi monitor (management frames only, nothing is transmitted). The caller keeps the receive task
// paused. begin() installs the callback and tunes the channel; end() restores the SDR receive state.
#include "esp_wifi_types.h"
typedef void (*marauder_promisc_cb_t)(void *buf, wifi_promiscuous_pkt_type_t type);
esp_err_t marauder_radio_monitor_begin(marauder_promisc_cb_t cb, uint8_t channel);
esp_err_t marauder_radio_monitor_channel(uint8_t channel);
esp_err_t marauder_radio_monitor_end(void);
bool marauder_radio_monitor_active(void);

#ifdef __cplusplus
}
#endif
