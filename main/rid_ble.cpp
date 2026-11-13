// BLE 4 legacy Remote ID receiver (HCI scanner over VHCI). Compiled to a real scanner only when the Bluetooth
// controller is enabled in sdkconfig (CONFIG_BT_ENABLED + CONFIG_BT_CONTROLLER_ENABLED); this firmware's
// default sdkconfig has it OFF, so by default these are inert stubs and Remote ID is Wi-Fi only.
// BLE 5 extended advertising (used for ASTM message packs over BLE) cannot be received on the original ESP32.
#include "include/rid.hpp"
#if defined(CONFIG_BT_ENABLED) && defined(CONFIG_BT_CONTROLLER_ENABLED)
#include "esp_bt.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#endif

namespace marauder {
#if defined(CONFIG_BT_ENABLED) && defined(CONFIG_BT_CONTROLLER_ENABLED)
namespace {
bool s_ble_on;
const char kTag[] = "rid_ble";
void send_cmd(uint16_t opcode, const uint8_t *params, uint8_t n) {
    uint8_t buf[4 + 16];
    buf[0] = 0x01; buf[1] = opcode & 0xFF; buf[2] = opcode >> 8; buf[3] = n;
    if (n) memcpy(buf + 4, params, n);
    uint32_t tries = 0;
    while (!esp_vhci_host_check_send_available() && tries++ < 100) vTaskDelay(1);
    esp_vhci_host_send_packet(buf, 4 + n);
}
void notify_send_available() {}
int host_recv(uint8_t *data, uint16_t len) {
    // HCI event: 0x04, 0x3E (LE meta), plen, subevent 0x02 (advertising report), num reports, then packed reports
    if (len < 7 || data[0] != 0x04 || data[1] != 0x3E || data[3] != 0x02) return 0;
    const uint8_t reports = data[4];
    size_t off = 5;
    // Legacy report layout when num_reports == 1 is event_type, addr_type, addr[6], data_len, data[], rssi. With several
    // reports the fields are stored column-wise; only the (typical) single-report case is handled and the rest skipped.
    if (reports != 1) return 0;
    if (off + 9 > len) return 0;
    const uint8_t *addr = data + off + 2;                    // little endian
    const uint8_t dlen = data[off + 8];
    if (off + 9 + dlen + 1 > len) return 0;
    const int8_t rssi = (int8_t)data[off + 9 + dlen];
    uint8_t mac[6]; for (int k = 0; k < 6; ++k) mac[k] = addr[5 - k];
    rid_feed_ble_adv(data + off + 9, dlen, mac, rssi, true, (uint32_t)(esp_timer_get_time() / 1000));
    return 0;
}
esp_vhci_host_callback_t s_cb = {notify_send_available, host_recv};
}  // namespace
bool rid_ble_available() { return true; }
bool rid_ble_start() {
    if (s_ble_on) return true;
    esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if (esp_bt_controller_init(&cfg) != ESP_OK) { ESP_LOGW(kTag, "controller init failed"); return false; }
    if (esp_bt_controller_enable(ESP_BT_MODE_BLE) != ESP_OK) { esp_bt_controller_deinit(); ESP_LOGW(kTag, "controller enable failed"); return false; }
    esp_vhci_host_register_callback(&s_cb);
    send_cmd(0x0C03, nullptr, 0);                                        // HCI_Reset
    vTaskDelay(pdMS_TO_TICKS(50));
    const uint8_t evmask[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20};   // LE Meta event only
    send_cmd(0x0C01, evmask, 8);
    const uint8_t lemask[8] = {0x01, 0, 0, 0, 0, 0, 0, 0};               // LE Advertising Report
    send_cmd(0x2001, lemask, 8);
    const uint8_t params[7] = {0x00, 0x85, 0x00, 0x40, 0x00, 0x00, 0x00};  // passive, 83 ms interval / 40 ms window (as Marauder)
    send_cmd(0x200B, params, 7);
    const uint8_t enable[2] = {0x01, 0x00};                              // enable, duplicate filter off
    send_cmd(0x200C, enable, 2);
    s_ble_on = true;
    return true;
}
void rid_ble_stop() {
    if (!s_ble_on) return;
    const uint8_t disable[2] = {0x00, 0x00};
    send_cmd(0x200C, disable, 2);
    vTaskDelay(pdMS_TO_TICKS(20));
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
    s_ble_on = false;
}
#else
bool rid_ble_available() { return false; }
bool rid_ble_start() { return false; }
void rid_ble_stop() {}
#endif
}
