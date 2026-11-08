idf_component_register(SRCS "app_main.cpp" "display.cpp" "gps.cpp" "input.cpp" "battery.cpp" "dsp.cpp" "storage.cpp" "settings.cpp" "spi_gate.cpp" "media_player.cpp"
    INCLUDE_DIRS "include" REQUIRES esp_sdr_backend driver esp_wifi esp_event nvs_flash fatfs sdmmc
        esp_driver_spi esp_driver_sdspi esp_driver_uart esp_driver_gpio esp_driver_i2c esp_timer log esp_common
        jpeg-roi-decoder libpng)
