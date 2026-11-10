#pragma once
#include "Arduino.h"
enum esp_spp_cb_event_t { ESP_SPP_SRV_OPEN_EVT, ESP_SPP_CLOSE_EVT };
constexpr int ESP_SPP_SUCCESS = 0;
struct esp_spp_cb_param_t { struct { int status = ESP_SPP_SUCCESS; } srv_open; };
struct BluetoothSerial : FakeSerial {
  bool connected = false;
  std::string name;
  std::string pin;
  void (*callback)(esp_spp_cb_event_t, esp_spp_cb_param_t*) = nullptr;
  bool begin(const char* value) { name = value; return true; }
  void setPin(const char* value) { pin = value; }
  void register_callback(void (*value)(esp_spp_cb_event_t, esp_spp_cb_param_t*)) { callback = value; }
  bool hasClient() { return connected; }
  void connection(bool value) {
    connected = value;
    esp_spp_cb_param_t param;
    callback(value ? ESP_SPP_SRV_OPEN_EVT : ESP_SPP_CLOSE_EVT, &param);
  }
};
