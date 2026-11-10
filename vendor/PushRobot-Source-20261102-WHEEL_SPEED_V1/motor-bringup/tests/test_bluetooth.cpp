#include <cassert>
#include <iostream>
#include "../src/main.cpp"

void stopped() {
  assert(!running && levels[kStandby] == LOW);
  for (int i = 0; i < 8; ++i) assert(pwmValues[i] == 0);
}
void usb(const char* text, uint32_t now) {
  fakeNow = now;
  Serial.inject(text);
  loop();
}
void bt(const char* text, uint32_t now) {
  fakeNow = now;
  SerialBT.inject(text);
  loop();
}
void connectBt(uint32_t now) {
  fakeNow = now;
  SerialBT.connection(true);
  loop();
}
int main() {
  setup();
  stopped();
  assert(bluetoothReady && SerialBT.name == "PushRobot-ESP32");
  assert(SerialBT.pin == "271038");
  connectBt(0);
  assert(controlOwner == ControlPort::None);
  Serial.output.clear(); SerialBT.output.clear();
  bt("WHO\nCAPS\n", 1);
  assert(SerialBT.output.find("UNIFIED_ROBOT 1 mode=DRY_RUN") != std::string::npos);
  assert(SerialBT.output.find("CAPS MOTORS=ABCD PINS=FOUR_V1") != std::string::npos);
  assert(Serial.output.empty());
  bt("MECANUM 50 0 0 100\n", 2); stopped();
  bt("MODE CHASSIS\n", 3); stopped();
  assert(controlOwner == ControlPort::Bluetooth && !layoutConfigured);
  bt("MECANUM 50 0 0 100\n", 4); stopped();
  bt("LAYOUT CBAD 1 1 1 1\n", 5);
  assert(layoutConfigured && wheelLayout.channel[0] == 2);
  // Two partial lines cannot combine into a motion command.
  bt("MECANUM 50 ", 6);
  usb("0 0 5000\n", 7); stopped();
  bt("0 0 5000\n", 8);
  assert(running);
  const uint32_t start = startedAt;
  usb("MODE MOTOR_A_TEST\nPING\nSTATUS\n", 1000);
  assert(running && runtimeMode == RuntimeMode::Chassis);
  assert(Serial.output.find("ERROR control_owned_by_other_port") != std::string::npos);
  assert(lastContactAt == start && ownerContactAt == 3);
  // Even malformed traffic from the other port cannot interfere.
  usb("this is a deliberately overlong USB line that is longer than sixty three bytes...........\n", 1001);
  assert(running);
  // Other-port heartbeat cannot prevent owner loss. Queued motion is discarded.
  fakeNow = 1503;
  SerialBT.inject("PING\nWHEELS 50 50 50 50 100\n");
  loop(); stopped();
  assert(controlOwner == ControlPort::None && !layoutConfigured);
  assert(runtimeMode == RuntimeMode::DryRun && SerialBT.input.empty());
  assert(SerialBT.output.find("UNIFIED_BOOT 1 default=DRY_RUN outputs=STOPPED") != std::string::npos);
  bt("MECANUM 50 0 0 100\n", 1504); stopped();
  bt("MODE CHASSIS\nLAYOUT CBAD 1 1 1 1\n", 2600);
  bt("WHEELS 50 50 50 50 5000\n", 2601); assert(running);
  const uint32_t epoch = bluetoothEpoch;
  SerialBT.connection(true);  // rejected second client's open/close callbacks
  esp_spp_cb_param_t rejected;
  bluetoothEvent(ESP_SPP_CLOSE_EVT, &rejected);
  fakeNow = 2602; loop();
  assert(running && bluetoothEpoch == epoch);
  bt("PING\n", 3600); assert(running);
  bt("STATUS\n", 4600); assert(running);
  // STOP always works from the non-owner and notifies both connections.
  SerialBT.output.clear();
  usb("STOP\n", 4601); stopped();
  assert(SerialBT.output.find("STOPPED command") != std::string::npos);
  assert(controlOwner == ControlPort::Bluetooth);
  bt("PING\n", 5600);
  bt("WHEELS 20 20 20 20 100\n", 5602); assert(running);
  Serial.output.clear(); SerialBT.output.clear();
  fakeNow = 5702; loop(); stopped();
  assert(SerialBT.output.find("STOPPED timer") != std::string::npos);
  assert(Serial.output.empty());
  bt("PING\n", 6600);
  bt("WHEELS 20 20 20 20 5000\nMECANUM ", 6800); assert(running);
  // Rapid disconnect/reconnect is still detected by the generation counter.
  SerialBT.connection(false);
  SerialBT.connection(true);
  SerialBT.inject("50 0 0 5000\n");
  fakeNow = 6801; loop(); stopped();
  assert(!layoutConfigured && controlOwner == ControlPort::None);
  bt("MODE CHASSIS\n", 7900);
  bt("MECANUM 50 0 0 100\n", 7901); stopped();
  bt("LAYOUT CBAD 1 1 1 1\nWHEELS 50 50 50 50 5000\n", 7902); assert(running);
  bt("WHEELS 1 ", 7903);
  fakeNow = 9400; loop(); stopped();  // half line cannot delay timeout
  assert(!layoutConfigured && controlOwner == ControlPort::None);
  usb("MODE CHASSIS\nLAYOUT CBAD 1 1 1 1\n", 10500);
  usb("WHEELS 50 50 50 50 5000\n", 10501); assert(running);
  SerialBT.connection(false);
  fakeNow = 10502; loop();
  assert(running && controlOwner == ControlPort::Usb && layoutConfigured);
  usb("STOP\n", 10503); stopped();
  fakeNow = 12000; loop();
  assert(controlOwner == ControlPort::None);
  connectBt(13000);
  bt("MODE MOTOR_A_TEST\nJOG B 50 5000\n", 13001); assert(running);
  bluetoothTxFailed = true;
  fakeNow = 13002; loop(); stopped();
  assert(controlOwner == ControlPort::None && runtimeMode == RuntimeMode::DryRun);
  // New session clears a blocked writer condition and requires MODE again.
  SerialBT.connection(false); connectBt(UINT32_MAX-49);
  bt("MODE CHASSIS\nLAYOUT CBAD 1 1 1 1\n", UINT32_MAX-49);
  hasStopped = false;
  bt("WHEELS 50 50 50 50 5000\n", UINT32_MAX-48); assert(running);
  fakeNow = 1449; loop(); assert(running);
  fakeNow = 1450; loop(); stopped();
  assert(controlOwner == ControlPort::None && !layoutConfigured);
  std::cout << "PASS: Bluetooth/USB protocol routing, independent parsers, control ownership, "
               "other-port STOP, heartbeat isolation, timeout/wrap, stale RX disposal, "
               "rapid reconnect, fresh layout gate, TX failure and USB unaffected by BT disconnect\n";
}
