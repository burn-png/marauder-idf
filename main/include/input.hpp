#pragma once
#include <stdint.h>
#include "esp_err.h"

namespace marauder {
enum class InputKey : uint8_t { NONE, UP, DOWN, LEFT, RIGHT, CENTER };
enum class InputKind : uint8_t { NONE, PRESS, REPEAT, SHORT_RELEASE, LONG_PRESS };
struct InputEvent { InputKey key; InputKind kind; uint16_t repeat_count; };
struct InputConfig { uint16_t repeat_delay_ms; uint16_t repeat_period_ms; uint16_t long_press_ms; uint16_t debounce_ms; uint8_t acceleration; bool invert_vertical; bool invert_horizontal; };
esp_err_t input_init();
InputEvent input_next_event(const InputConfig &config);
}
