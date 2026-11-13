#pragma once
// Map / navigation / Remote ID screens. The module draws only inside the content area (y 32..295) and owns no task:
// map_ui_tick() is called from the UI loop and time-slices indexing and rendering in small steps.
#include "input.hpp"

namespace marauder {
enum class MapScreen : uint8_t { MAP, NAV, RID };

void map_ui_set_radio_hook(void (*pause_sdr)(bool pause));   // pause / resume the SDR receive task around Wi-Fi monitoring
void map_ui_open(MapScreen screen, bool imperial_units);
void map_ui_close();                                          // stops Remote ID scanning, cancels indexing, frees buffers
bool map_ui_event(const InputEvent &e, bool move, bool confirm);   // true: leave the screen
void map_ui_tick();                                           // non-blocking work slice
void map_ui_draw(bool screen_changed);                        // call from the periodic redraw
const char *map_ui_title();
const char *map_ui_footer();
}
