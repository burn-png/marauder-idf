/*
 * Receive-only adapter based on ESPARGOS/esp-sdr's original ESP32 backend.
 * Register sequence and SRAM reservation remain under the upstream GPL-3.0-or-later.
 */
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "radio_backend.h"
#include "rx_recalibration.h"
#include "rx_lo.h"
#include "rx_tuning.h"
#include "rx_bandwidth.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_phy_cert_test.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_err.h"
#include "esp_check.h"
#include "nvs_flash.h"
#include "soc/soc.h"
#include "soc/rtc.h"
#include "soc/dport_reg.h"
#include "soc/dport_access.h"
#include "heap_memory_layout.h"

/* The dump engine overwrites this entire 64 KiB SRAM aperture. The linker
 * fragment keeps code, static data, and heap outside it. */
SOC_RESERVE_MEMORY_REGION(0x3ffe8000, 0x3fff8000, esp32_rf_dump);
#define DUMP_CTRL   0x60033d90u
#define DUMP_STATUS 0x60033d94u
#define DUMP_BYTES  0x60033dc4u
#define RX_GAIN     0x3ff5c02cu
#define CAPACITY    16384u
#define MAX_SAMPLES (CAPACITY - 4)
#define SENTINEL    0xa55aa55au

static uint32_t *const s_samples = (void *)0x3ffe8000;
static unsigned s_frequency_mhz = 2412;
static int s_rx_filter = -1;
static bool s_rx_ready;
static unsigned s_gain_code = 40;
static bool s_hardware_agc = true;
static SemaphoreHandle_t s_radio_mutex;

extern unsigned rom_chip_i2c_readReg(unsigned, unsigned, unsigned);
extern void rom_chip_i2c_writeReg(unsigned, unsigned, unsigned, unsigned);
extern void rom_pbus_workmode(void);
extern void rom_pbus_xpd_rx_on(int);
extern void rom_pbus_xpd_tx_off(void);
extern void rom_set_rxclk_en(int);
extern void set_chanfreq(unsigned, unsigned);
extern void rom_set_rf_freq_offset(unsigned, unsigned, int);

static unsigned s_filter_saved[2];

static void filter_apply(void) {
    for (unsigned j = 0; j < 2; ++j) {
        s_filter_saved[j] = rom_chip_i2c_readReg(0x67, 1, 1 + j);
        if (s_rx_filter >= 0) {
            rom_chip_i2c_writeReg(0x67, 1, 1 + j,
                (s_filter_saved[j] & ~127u) | (unsigned)s_rx_filter);
        }
    }
    esp_rom_delay_us(10);
}

static void filter_restore(void) {
    if (s_rx_filter >= 0) {
        for (unsigned j = 0; j < 2; ++j) {
            rom_chip_i2c_writeReg(0x67, 1, 1 + j, s_filter_saved[j]);
        }
    }
}

static void apply_gain(void) {
    uint32_t gain = REG_READ(RX_GAIN);
    if (s_hardware_agc) {
        REG_CLR_BIT(RX_GAIN, BIT(23));
    } else {
        REG_WRITE(RX_GAIN, (gain & 0x7fffff) | (s_gain_code << 24) | BIT(23));
    }
}
static void prepare_rx(void);

esp_err_t marauder_radio_set_rx_gain(bool automatic, unsigned index) {
    if (index > 127) return ESP_ERR_INVALID_ARG;
    if (!s_radio_mutex || xSemaphoreTake(s_radio_mutex, portMAX_DELAY) != pdTRUE)
        return ESP_ERR_TIMEOUT;
    s_hardware_agc = automatic;
    s_gain_code = index;
    apply_gain();
    xSemaphoreGive(s_radio_mutex);
    nvs_handle_t nvs;
    if (nvs_open("sdr_rx", NVS_READWRITE, &nvs) == ESP_OK) {
        esp_err_t e = nvs_set_u8(nvs, "agc", automatic ? 1 : 0);
        if (e == ESP_OK) e = nvs_set_u8(nvs, "gain", (uint8_t)index);
        if (e == ESP_OK) e = nvs_commit(nvs);
        nvs_close(nvs);
        return e;
    }
    return ESP_OK;
}

bool marauder_radio_rx_gain_is_auto(void) { return s_hardware_agc; }
unsigned marauder_radio_rx_gain_index(void) { return s_gain_code; }

/* Deliberately narrow certification probe: a single fixed 802.11 test frame,
 * channel 1, 1 Mb/s, 32-byte PSDU, and maximum documented attenuation. It is
 * not an IQ/replay path and accepts no caller-supplied waveform or payload. */
esp_err_t marauder_radio_bench_tx_once(void) {
    if (!s_radio_mutex || xSemaphoreTake(s_radio_mutex, portMAX_DELAY) != pdTRUE)
        return ESP_ERR_TIMEOUT;
    esp_phy_rftest_config(1);
    esp_phy_rftest_init();
    esp_phy_test_start_stop(3);
    esp_phy_tx_contin_en(false);
    esp_phy_wifi_tx(1, PHY_RATE_1M, 127, 32, 1000, 1);
    esp_rom_delay_us(5000);
    esp_phy_test_start_stop(0);
    esp_phy_rftest_config(0);
    s_rx_ready = false;
    prepare_rx();
    xSemaphoreGive(s_radio_mutex);
    return ESP_OK;
}

static void tune_rx(unsigned mhz) {
    static unsigned calibrated_mhz;
    if (calibrated_mhz != mhz || rx_recalibration_stale()) {
        rx_recalibrate(mhz);
        calibrated_mhz = mhz;
    }
    rx_lo_plan_t plan = rx_lo_plan(mhz);
    (void)rx_lo_select(false);
    bool channel = (mhz >= 2412 && mhz <= 2472 && (mhz - 2412) % 5 == 0) || mhz == 2484;
    set_chanfreq(channel ? mhz : 2412, 0);
    if (!channel) {
        unsigned xtal = rtc_clk_xtal_freq_get();
        unsigned cap = ram_chip_i2c_readReg(0x62, 1, 0);
        ram_chip_i2c_writeReg(0x62, 1, 0, cap & ~0x80u);
        int offset = (plan.offset_khz * 1024 + 500) / 1000;
        rom_set_rf_freq_offset(xtal == 26 ? 1 : xtal == 24 ? 2 : 0,
                               plan.mhz, offset);
    }
}

static void prepare_rx(void) {
    if (!s_rx_ready || rx_recalibration_stale()) {
        tune_rx(s_frequency_mhz);
        s_rx_ready = true;
    }
    rom_pbus_workmode();
    rom_pbus_xpd_tx_off();
    rom_pbus_xpd_rx_on(1);
    rom_set_rxclk_en(1);
    apply_gain();
    if (rx_lo_select(rx_lo_plan(s_frequency_mhz).alternate)) {
        esp_rom_delay_us(3000);
    }
}

/* Original ESP32 RF dump block. The CPU does not touch s_samples while the
 * hardware owns the SRAM aperture; ownership is restored before returning. */
static bool acquire_iq(unsigned n, unsigned *elapsed_us) {
    if (n < 256 || n > MAX_SAMPLES) return false;
    if (rx_recalibration_stale()) prepare_rx();
    REG_WRITE(DUMP_CTRL, 0);
    for (unsigned j = 0; j < CAPACITY; ++j) s_samples[j] = SENTINEL;
    filter_apply();
    uint32_t owner = DPORT_REG_READ(DPORT_IRAM_DRAM_AHB_SEL_REG);
    uint32_t byte_select = REG_READ(DUMP_BYTES);
    DPORT_REG_WRITE(DPORT_IRAM_DRAM_AHB_SEL_REG,
        (owner & ~DPORT_MAC_DUMP_MODE_M) | (3 << DPORT_MAC_DUMP_MODE_S));
    REG_WRITE(DUMP_BYTES, 0x03020100);
    /* 16 MS/s dump clock, I/Q source lane. */
    REG_WRITE(DUMP_CTRL, n | (1u << 31) | (1u << 16));
    int64_t start = esp_timer_get_time();
    REG_SET_BIT(DUMP_CTRL, BIT(19));
    REG_CLR_BIT(DUMP_CTRL, BIT(19));
    while (!(REG_READ(DUMP_CTRL) & BIT(18)) && esp_timer_get_time() - start < 20000) { }
    uint32_t result = REG_READ(DUMP_CTRL);
    uint32_t status = REG_READ(DUMP_STATUS);
    *elapsed_us = (unsigned)(esp_timer_get_time() - start);
    REG_WRITE(DUMP_CTRL, 0);
    esp_rom_delay_us(10);
    DPORT_REG_WRITE(DPORT_IRAM_DRAM_AHB_SEL_REG, owner);
    REG_WRITE(DUMP_BYTES, byte_select);
    filter_restore();
    if (!(result & BIT(18)) || (status & 0x7fff) != n) return false;
    for (unsigned j = 0; j < CAPACITY; ++j) {
        if ((j < n && s_samples[j] == SENTINEL) ||
            (j >= n && s_samples[j] != SENTINEL)) return false;
    }
    return true;
}

esp_err_t marauder_radio_init(void) {
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), "marauder_sdr", "erase incompatible NVS");
        e = nvs_flash_init();
    }
    if (e != ESP_OK) return e;
    e = esp_event_loop_create_default();
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE) return e;
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    e = esp_wifi_init(&cfg);
    if (e != ESP_OK && e != ESP_ERR_WIFI_INIT_STATE) return e;
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), "marauder_sdr", "wifi storage");
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_NULL), "marauder_sdr", "wifi mode");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), "marauder_sdr", "wifi start");
    ESP_RETURN_ON_ERROR(esp_wifi_set_ps(WIFI_PS_NONE), "marauder_sdr", "wifi ps");
    ESP_RETURN_ON_ERROR(esp_wifi_set_promiscuous(true), "marauder_sdr", "wifi promiscuous");
    ESP_RETURN_ON_ERROR(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE), "marauder_sdr", "wifi channel");
    s_radio_mutex = xSemaphoreCreateMutex();
    if (!s_radio_mutex) return ESP_ERR_NO_MEM;
    nvs_handle_t rx_nvs;
    if (nvs_open("sdr_rx", NVS_READONLY, &rx_nvs) == ESP_OK) {
        uint8_t value;
        if (nvs_get_u8(rx_nvs, "agc", &value) == ESP_OK) s_hardware_agc = value != 0;
        if (nvs_get_u8(rx_nvs, "gain", &value) == ESP_OK && value <= 127) s_gain_code = value;
        nvs_close(rx_nvs);
    }
    prepare_rx();
    return ESP_OK;
}

esp_err_t marauder_radio_capture_iq8(uint32_t center_mhz, size_t count, int8_t *iq) {
    if (!iq || count < 256 || count > MAX_SAMPLES) return ESP_ERR_INVALID_ARG;
    /* Frequencies above the 2.4 GHz band remain direct-PLL tune attempts, not
     * a claim of calibrated reception. Requests through 2800 MHz are accepted
     * so the hardware behavior can be evaluated; >2651 MHz is beyond the
     * conservative envelope documented by rx_lo.h. */
    if (center_mhz < MARAUDER_RADIO_FREQ_MIN_MHZ ||
        center_mhz > MARAUDER_RADIO_FREQ_MAX_MHZ) return ESP_ERR_NOT_SUPPORTED;
    if (!s_radio_mutex || xSemaphoreTake(s_radio_mutex, portMAX_DELAY) != pdTRUE)
        return ESP_ERR_TIMEOUT;
    bool changed = center_mhz != s_frequency_mhz;
    s_frequency_mhz = center_mhz;
    if (changed) {
        s_rx_ready = false;
        prepare_rx();
    }
    unsigned elapsed_us = 0;
    bool ok = acquire_iq((unsigned)count, &elapsed_us);
    if (ok) {
        for (size_t k = 0; k < count; ++k) {
            uint32_t word = s_samples[k];
            iq[2 * k] = (int8_t)((word >> 2) & 0xff);
            iq[2 * k + 1] = (int8_t)((word >> 12) & 0xff);
        }
    }
    xSemaphoreGive(s_radio_mutex);
    return ok ? ESP_OK : ESP_ERR_TIMEOUT;
}
