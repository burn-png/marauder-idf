#include "include/thermal.hpp"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <algorithm>
#include <atomic>
#include <cstdint>

// Legacy on-die sensor, present in the PHY library. Returns the raw reading in degrees Fahrenheit.
extern "C" uint8_t temprature_sens_read(void);

namespace marauder {
namespace {
std::atomic<int> s_temp_f{-1000};
std::atomic<unsigned> s_cpu{0};
int64_t s_prev_us{};
uint32_t s_prev_idle[2]{};
bool s_have_prev{};
bool s_have_temp{};
float s_filtered{};

uint8_t median3(uint8_t a, uint8_t b, uint8_t c) { return std::max(std::min(a, b), std::min(std::max(a, b), c)); }
}

void thermal_init() { (void)thermal_update(); }

bool thermal_update() {
    // Temperature: median of three reads, then a light smoothing filter so the alarm does not chatter.
    const uint8_t raw = median3(temprature_sens_read(), temprature_sens_read(), temprature_sens_read());
    if (raw != 0 && raw != 0xff) {
        s_filtered = s_have_temp ? s_filtered * 0.7f + (float)raw * 0.3f : (float)raw;
        s_have_temp = true;
        s_temp_f.store((int)(s_filtered + 0.5f));
    }

    // CPU load: share of the last interval the two idle tasks did NOT run.
    const int64_t now = esp_timer_get_time();
    uint32_t idle[2]{};
    for (int core = 0; core < 2; ++core) {
        TaskStatus_t st{};
        vTaskGetInfo(xTaskGetIdleTaskHandleForCore(core), &st, pdFALSE, eRunning);
        idle[core] = st.ulRunTimeCounter;
    }
    if (s_have_prev && now > s_prev_us) {
        const uint64_t elapsed = (uint64_t)(now - s_prev_us) * 2u; // two cores, counter in microseconds
        const uint64_t idle_delta = (uint64_t)(uint32_t)(idle[0] - s_prev_idle[0]) + (uint64_t)(uint32_t)(idle[1] - s_prev_idle[1]);
        const unsigned busy = idle_delta >= elapsed ? 0u : (unsigned)(100u - idle_delta * 100u / elapsed);
        s_cpu.store(std::min(busy, 100u));
    }
    s_prev_us = now;
    s_prev_idle[0] = idle[0];
    s_prev_idle[1] = idle[1];
    s_have_prev = true;
    return s_have_temp;
}

int die_temp_f() { return s_temp_f.load(); }
unsigned cpu_load_pct() { return s_cpu.load(); }
}
