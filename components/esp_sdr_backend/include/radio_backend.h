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

#ifdef __cplusplus
}
#endif
