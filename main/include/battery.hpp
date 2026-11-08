#pragma once

namespace marauder {
// Reads the optional MAX17048 installed on some V7 boards. Returns -1 when
// that gauge is not present or a read fails; it never estimates from ESP32 ADC.
void battery_init();
int battery_percent();
void battery_update();
}
