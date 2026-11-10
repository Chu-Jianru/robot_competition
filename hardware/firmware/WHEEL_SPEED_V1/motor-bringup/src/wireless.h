#pragma once
#include <BluetoothSerial.h>

// 命令解析与 GPIO 操作只在 Arduino loop 任务执行，蓝牙回调只记录连接状态。
// 控制权与应答端口不同：controlOwner 决定谁能控制，replyPort 决定本次答复发给谁。
// epoch 标记连接代次；重连后丢弃旧连接的发送包，不能重放旧会话消息。
enum class ControlPort { None, Usb, Bluetooth };
ControlPort replyPort = ControlPort::Usb;
ControlPort controlOwner = ControlPort::None;
BluetoothSerial SerialBT;
constexpr const char* kBluetoothName = "PushRobot-ESP32";
constexpr const char* kBluetoothPin = "271038";
portMUX_TYPE bluetoothMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t bluetoothEpoch = 0;
volatile bool bluetoothConnected = false;
volatile bool bluetoothTxFailed = false;
bool bluetoothReady = false;

struct BluetoothState { uint32_t epoch; bool connected; bool txFailed; };
BluetoothState bluetoothState() {
  portENTER_CRITICAL(&bluetoothMux);
  BluetoothState state{bluetoothEpoch, bluetoothConnected, bluetoothTxFailed};
  portEXIT_CRITICAL(&bluetoothMux);
  return state;
}

void bluetoothEvent(esp_spp_cb_event_t event, esp_spp_cb_param_t* param) {
  // A rejected second client must not revoke the actual controlling client.
  const bool open = event == ESP_SPP_SRV_OPEN_EVT && param->srv_open.status == ESP_SPP_SUCCESS;
  const bool close = event == ESP_SPP_CLOSE_EVT && !SerialBT.hasClient();
  if (!open && !close) return;
  portENTER_CRITICAL(&bluetoothMux);
  if (open && bluetoothConnected) { portEXIT_CRITICAL(&bluetoothMux); return; }
  ++bluetoothEpoch;
  bluetoothConnected = open;
  bluetoothTxFailed = false;
  portEXIT_CRITICAL(&bluetoothMux);
}

#ifdef ARDUINO_ARCH_ESP32
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
struct BluetoothPacket { uint32_t epoch; char text[384]; };
QueueHandle_t bluetoothTxQueue = nullptr;

void bluetoothWriter(void*) {
  BluetoothPacket packet;
  for (;;) {
    if (xQueueReceive(bluetoothTxQueue, &packet, portMAX_DELAY) != pdTRUE) continue;
    const BluetoothState state = bluetoothState();
    if (!state.connected || packet.epoch != state.epoch) continue;
    // BluetoothSerial::write can block on its internal queue. Keep it off the
    // motor loop so a stalled receiver cannot delay the motor stop deadline.
    if (SerialBT.write(reinterpret_cast<const uint8_t*>(packet.text), strlen(packet.text)) == 0) {
      portENTER_CRITICAL(&bluetoothMux);
      if (packet.epoch == bluetoothEpoch) bluetoothTxFailed = true;
      portEXIT_CRITICAL(&bluetoothMux);
    }
  }
}
#endif

void bluetoothSend(const char* text) {
  const BluetoothState state = bluetoothState();
  if (!state.connected || state.txFailed) return;
#ifdef ARDUINO_ARCH_ESP32
  BluetoothPacket packet{};
  packet.epoch = state.epoch;
  snprintf(packet.text, sizeof(packet.text), "%s", text);
  if (xQueueSend(bluetoothTxQueue, &packet, 0) != pdTRUE) {
    portENTER_CRITICAL(&bluetoothMux);
    bluetoothTxFailed = true;
    portEXIT_CRITICAL(&bluetoothMux);
  }
#else
  SerialBT.printf("%s", text);
#endif
}

struct CommandReply {
  void println(const char* text) {
    printf("%s\r\n", text);
  }
  template<typename... Args> void printf(const char* format, Args... args) {
    if (replyPort == ControlPort::Usb) { Serial.printf(format, args...); return; }
    if (replyPort != ControlPort::Bluetooth) return;
    char text[384];
    snprintf(text, sizeof(text), format, args...);
    bluetoothSend(text);
  }
} reply;

void beginBluetooth() {
#ifdef ARDUINO_ARCH_ESP32
  bluetoothTxQueue = xQueueCreate(16, sizeof(BluetoothPacket));
  if (!bluetoothTxQueue) { Serial.println("BLUETOOTH_ERROR tx_queue"); return; }
  if (xTaskCreate(bluetoothWriter, "robot_bt_tx", 3072, nullptr, 1, nullptr) != pdPASS) {
    Serial.println("BLUETOOTH_ERROR tx_task");
    return;
  }
#endif
  SerialBT.register_callback(bluetoothEvent);
  SerialBT.setPin(kBluetoothPin);
  bluetoothReady = SerialBT.begin(kBluetoothName);
  Serial.println(bluetoothReady ? "BLUETOOTH_READY SPP NAME=PushRobot-ESP32" : "BLUETOOTH_ERROR begin");
}
