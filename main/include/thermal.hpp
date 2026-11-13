#pragma once

namespace marauder {
// Rough die temperature and CPU load for the status bar.
// The original ESP32 has no supported temperature driver; this reads the legacy
// on-die sensor, which is a coarse (several degrees F) relative indicator.
void thermal_init();
// Call about once a second. Returns false until a first reading exists.
bool thermal_update();
int die_temp_f();        // degrees Fahrenheit
unsigned cpu_load_pct(); // 0..100 over the last update interval
}
