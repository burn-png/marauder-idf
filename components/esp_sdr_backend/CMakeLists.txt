set(SDR_ROOT "${CMAKE_CURRENT_LIST_DIR}/upstream")
idf_component_register(
    SRCS "src/radio_backend.c"
         "${SDR_ROOT}/main/common/rx_recalibration.c"
         "${SDR_ROOT}/main/common/rx_calibration_state.c"
    INCLUDE_DIRS "include" "${SDR_ROOT}/main/common" "${SDR_ROOT}/main/targets/esp32"
    REQUIRES esp_wifi esp_event esp_phy nvs_flash esp_timer esp_hw_support heap)
target_link_options(${COMPONENT_LIB} INTERFACE
    "-T${SDR_ROOT}/main/targets/esp32/sram_guard.ld"
    "-Wl,--wrap=chip_v7_set_chan_ana")
