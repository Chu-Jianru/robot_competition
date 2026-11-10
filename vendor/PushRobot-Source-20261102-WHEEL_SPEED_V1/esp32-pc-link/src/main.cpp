#include <Arduino.h>
#include <esp_random.h>

#include "link_core.h"

namespace {

drylink::LinkCore g_core;

uint32_t randomU32(void* /*ctx*/) { return esp_random(); }

void emitLine(void* /*ctx*/, const char* line) { Serial.println(line); }

}  // namespace

void setup() {
  Serial.begin(115200);

  // DRY_RUN firmware: intentionally no pinMode(), no motor pins, no STBY.
  // Commands are parsed and reported only; nothing here can drive a motor.
  uint32_t boot_id = esp_random();
  if (boot_id == 0) {
    boot_id = 1;
  }
  uint32_t arm_token = esp_random();
  if (arm_token == 0) {
    arm_token = 1;
  }

  drylink::LinkCore::Config config;
  config.boot_id = boot_id;
  config.arm_token = arm_token;
  config.random = &randomU32;
  config.random_ctx = nullptr;
  config.emit = &emitLine;
  config.emit_ctx = nullptr;
  g_core.begin(millis(), config);
}

void loop() {
  uint8_t buffer[64];
  size_t count = 0;
  while (count < sizeof(buffer) && Serial.available() > 0) {
    const int value = Serial.read();
    if (value < 0) {
      break;
    }
    buffer[count++] = static_cast<uint8_t>(value);
  }
  if (count > 0) {
    g_core.feed(buffer, count, millis());
  }
  g_core.tick(millis());
}
