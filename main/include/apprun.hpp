#pragma once
// Runs a VM1 .APP from the SD card inside the app window (240x256 at y=36). See appvm.hpp for the sandbox rules.
#include <stddef.h>
#include <stdint.h>

namespace marauder {
bool app_run_start(const char *relative_path, char *why, size_t why_size);   // loads, validates and starts; false with a reason
void app_run_stop();                                                         // frees everything
void app_run_restart();                                                      // restart the program from the top (after the screen was cleared)
void app_run_key(int16_t vm_key);                                            // 1 up, 2 down, 3 left, 4 right, 5 center
void app_run_tick(uint32_t now_ms);                                          // non-blocking slice; call every UI loop
bool app_run_active();
const char *app_run_name();
}
