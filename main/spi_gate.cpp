#include "include/spi_gate.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_err.h"

namespace { SemaphoreHandle_t s_gate; }
namespace marauder {
void spi_gate_init() { s_gate = xSemaphoreCreateMutex(); ESP_ERROR_CHECK(s_gate ? ESP_OK : ESP_ERR_NO_MEM); }
void spi_gate_lock() { if (s_gate) xSemaphoreTake(s_gate, portMAX_DELAY); }
void spi_gate_unlock() { if (s_gate) xSemaphoreGive(s_gate); }
}
