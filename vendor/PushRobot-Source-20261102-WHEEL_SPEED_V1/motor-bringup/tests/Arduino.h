#pragma once
#include <cstdarg>
#include <cstdio>
#include <deque>
#include <string>
#include <cstdint>

constexpr int LOW = 0;
constexpr int HIGH = 1;
constexpr int OUTPUT = 1;
constexpr int INPUT = 0;
constexpr int CHANGE = 3;
#define IRAM_ATTR
#define DRAM_ATTR
using portMUX_TYPE = int;
#define portMUX_INITIALIZER_UNLOCKED 0
inline void portENTER_CRITICAL(portMUX_TYPE*) {}
inline void portEXIT_CRITICAL(portMUX_TYPE*) {}
inline void portENTER_CRITICAL_ISR(portMUX_TYPE*) {}
inline void portEXIT_CRITICAL_ISR(portMUX_TYPE*) {}
inline uint32_t fakeNow = 0;
inline uint32_t fakeMicros = 0;
inline int levels[40] = {};
inline int pwmValues[16] = {};
inline int pinModes[40] = {};
inline int pinConfigured[40] = {};
inline int attachedPins[16] = {};
inline int interrupts[40] = {};
inline uint32_t millis() { return fakeNow; }
inline uint32_t micros() { return fakeMicros; }
inline void digitalWrite(int pin, int value) { levels[pin] = value; }
inline void pinMode(int pin, int mode) { pinModes[pin] = mode; ++pinConfigured[pin]; }
inline int digitalRead(int pin) { return levels[pin]; }
inline int digitalPinToInterrupt(int pin) { return pin; }
inline void attachInterrupt(int pin, void (*)(), int) { ++interrupts[pin]; }
inline void ledcSetup(int, int, int) {}
inline void ledcAttachPin(int pin, int channel) { attachedPins[channel] = pin; }
inline void ledcWrite(int channel, int value) { pwmValues[channel] = value; }
inline uint32_t ledcRead(int channel) { return pwmValues[channel]; }
struct FakeSerial {
  std::deque<unsigned char> input;
  std::string output;
  // 可设置的 USB 发送缓冲剩余空间；默认足够大，测试可设为 0 验证遥测跳过。
  int writeSpace = 1024;
  void begin(int) {}
  void println(const char* text) { output += std::string(text) + "\n"; }
  void printf(const char* format, ...) {
    char text[256];
    va_list args;
    va_start(args, format);
    vsnprintf(text, sizeof(text), format, args);
    va_end(args);
    output += text;
  }
  int availableForWrite() { return writeSpace; }
  int available() { return static_cast<int>(input.size()); }
  int read() {
    if (input.empty()) return -1;
    int byte = input.front();
    input.pop_front();
    return byte;
  }
  void inject(const std::string& text) {
    for (unsigned char byte : text) input.push_back(byte);
  }
};
inline FakeSerial Serial;
