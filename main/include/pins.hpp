#pragma once
#include "driver/gpio.h"

namespace marauder::pins {
inline constexpr gpio_num_t tft_mosi = GPIO_NUM_23;
inline constexpr gpio_num_t spi_miso = GPIO_NUM_19;
inline constexpr gpio_num_t tft_sclk = GPIO_NUM_18;
inline constexpr gpio_num_t tft_cs = GPIO_NUM_17;
inline constexpr gpio_num_t tft_dc = GPIO_NUM_26;
inline constexpr gpio_num_t backlight = GPIO_NUM_32;
inline constexpr gpio_num_t sd_cs = GPIO_NUM_4;
// Marauder V7 pin map from the upstream JustCallMeKoko ESP32Marauder configs.
// Five active-low switch inputs; GPS uses UART2 with MCU RX/TX on GPIO 21/22.
inline constexpr gpio_num_t joy_left = GPIO_NUM_13;
inline constexpr gpio_num_t joy_center = GPIO_NUM_34;
inline constexpr gpio_num_t joy_up = GPIO_NUM_36;
inline constexpr gpio_num_t joy_right = GPIO_NUM_39;
inline constexpr gpio_num_t joy_down = GPIO_NUM_35;
// Upstream V7 uses GPS_TX=21/GPS_RX=22 as MCU RX/TX pin arguments.
inline constexpr gpio_num_t gps_rx = GPIO_NUM_21;
inline constexpr gpio_num_t gps_tx = GPIO_NUM_22;
inline constexpr gpio_num_t led_a = GPIO_NUM_15;
inline constexpr int gps_baud = 9600;
}
