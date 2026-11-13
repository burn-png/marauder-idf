#pragma once

namespace marauder {
// Reads the optional V7 MAX17048 gauge on GPIO33/16. Returns -1 when no
// gauge responds; it never estimates battery level from ESP32 ADC.
void battery_init();
int battery_percent();
int battery_scl_pin(); // SCL GPIO the gauge answered on, or -1
void battery_update();
}
