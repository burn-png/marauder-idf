# Dominion transmit research on ESP32 Marauder V7 hardware

## Finding

Marble's current `esp-sdr-zephyr` work demonstrates transmit paths on supported ESP32 family chips, but its published TX implementation does not provide an original ESP32 target. Its public library documents the DAC burst and loop TX calls for ESP32-S3, and separate CPU-paced TX paths for ESP32-C5/C6. The project lists ESP32-S3, C5, and C6 for its applications; the original ESP32 is not in that TX support list. The core `ESPARGOS/esp-sdr` project describes itself as receive-only.

The Marauder V7 uses the original ESP32. Its present receiver adapter uses that chip's DPORT dump aperture, reserves the `0x3ffe8000` SRAM region, and captures 256-sample IQ8 bursts at a fixed nominal 16 MS/s. The receiver registers and SRAM dump path do not establish that the transmitter's DAC path, timing, RF calibration, or output power controls work on this chip.

ESP-SDR's `rx_tuning.h` labels 100–6000 MHz as software attempt limits, not a measured hardware coverage guarantee. The local LO helper describes 1842–2209 MHz as mapping into a conservative 2210–2651 MHz PLL envelope and uses direct-PLL tuning attempts outside that range. The original ESP32 Marauder V7's specified radio band ends at 2484 MHz. The backend accepts direct-PLL receive requests through 2800 MHz, but frequencies above 2484 MHz are explicitly marked unverified/out-of-band in the UI, and requests above 2651 MHz also exceed the conservative PLL envelope. This enables evaluation of direct tuning without asserting calibrated or supported reception.

ESP-IDF v6.1 also ships `esp_phy_cert_test.h` for the original ESP32 when `CONFIG_ESP_PHY_ENABLE_CERT_TEST=y`. That is a separate Wi-Fi certification/test-mode API, not Marble's ESP-SDR raw-IQ engine. This patch uses only its finite packet function for a lab probe.

## Work completed in this patch

- Added working receive gain controls: AGC/manual mode, manual index 0–127, and NVS persistence. The backend serializes gain writes with the receiver capture mutex.
- Made the spectrum header show center frequency in GHz and mark max hold (`MH`).
- Added left/center/right frequency labels for the plotted span.
- Increased the spectrum trace height so the red max-hold trace is easier to distinguish.
- Normalized the ILI9341 scroll origin before clearing the display. This prevents a previous waterfall scroll position from offsetting menus or leaving black patches after leaving a scan/spectrum view.
- Kept the actual receiver limits visible: 16 MS/s nominal sampling, 256 complex IQ8 samples per sparse snapshot. The other sample rates in the supplied ESP-SDR web UI are not exposed as options because this V7 backend programs a fixed 16 MS/s dump clock.
- Set the receiver direct-PLL tune-request ceiling to 2800 MHz in the backend, UI, and settings validation. Frequencies above 2484 MHz are red/unverified in the UI; values above 2651 MHz are beyond the conservative PLL envelope.
- Added a separate TX Bench application that issues one fixed 802.11 test frame on channel 1 at 1 Mb/s, with 32-byte PSDU, maximum SDK power backoff (31.75 dB), continuous mode off, and a finite packet count of one. It requires two Center presses, stops PHY test mode, and reapplies the RX setup. There is no arbitrary payload, modulation input, raw-IQ playback, or repetition control.

## What this does not implement

The one-packet test is not raw-IQ transmission, RF replay, or a port of Marble's SDR TX engine. Copying the S3 DAC path or C5/C6 paced paths into the original-ESP32 target would still require a chip-specific implementation. This project's bench packet hook builds, but its RF output and receive restoration have not been physically measured. Use only with a shielded enclosure or a conducted setup with appropriate attenuation and measurement equipment; the V7 antenna can radiate over the air.

## Sources

- [Marble's ESP-SDR Zephyr repository](https://github.com/cyber-murmel/esp-sdr-zephyr)
- [Marble's TX API and supported-chip guards](https://github.com/cyber-murmel/esp-sdr-zephyr/blob/main/include/esp_sdr/esp_sdr_tx.h)
- [Marble's ESP-SDR CMake target selection](https://github.com/cyber-murmel/esp-sdr-zephyr/blob/main/lib/esp_sdr/CMakeLists.txt)
- [ESPARGOS ESP-SDR repository](https://github.com/ESPARGOS/esp-sdr)
- [ESP-SDR software tuning limits](https://github.com/ESPARGOS/esp-sdr/blob/main/main/common/rx_tuning.h)
- [ESP-SDR receive LO planning and documented PLL envelope](https://github.com/ESPARGOS/esp-sdr/blob/main/main/common/rx_lo.h)
- [ESP-IDF v6.1 RF certification API](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/RF_calibration.html)
