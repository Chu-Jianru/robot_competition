#pragma once

#include <stddef.h>
#include <stdint.h>

namespace drylink {

// Protocol v1, DRY_RUN only. This core is pure C++ and performs no GPIO access.
constexpr uint16_t kProtocolVersion = 1;
constexpr uint32_t kWatchdogMs = 300;
constexpr size_t kMaxLineLen = 120;      // command characters, excluding CR/LF
constexpr size_t kMaxOutputLen = 512;
constexpr int32_t kMaxLinearMmS = 300;   // vx, vy
constexpr int32_t kMaxAngularDegS = 180; // w

constexpr const char* kFirmwareVersion = "0.1.0";
constexpr const char* kMode = "DRY_RUN";

enum class State : uint8_t {
  Disarmed = 0,
  Armed = 1,
};

enum class StopReason : uint8_t {
  None = 0,
  Boot,
  Stop,
  Disarm,
  Watchdog,
};

enum class Error : uint8_t {
  None = 0,
  BadLine,
  TooLong,
  BadArgs,
  UnknownCmd,
  NotArmed,
  AlreadyArmed,
  StaleArmToken,
  SessionReused,
  SessionMismatch,
  StaleSeq,
  Range,
};

class LinkCore {
 public:
  using EmitFn = void (*)(void* ctx, const char* line);
  using RandomFn = uint32_t (*)(void* ctx);

  struct Config {
    uint32_t boot_id;
    uint32_t arm_token;
    RandomFn random;
    void* random_ctx;
    EmitFn emit;
    void* emit_ctx;
  };

  LinkCore();

  void begin(uint32_t now_ms, const Config& config);
  void feed(const uint8_t* data, size_t len, uint32_t now_ms);
  void tick(uint32_t now_ms);

  State state() const { return state_; }
  uint32_t bootId() const { return boot_id_; }
  uint32_t armToken() const { return arm_token_; }
  uint32_t session() const { return session_; }
  uint32_t lastSession() const { return last_session_; }
  uint32_t lastSeq() const { return last_seq_; }
  int32_t vx() const { return vx_; }
  int32_t vy() const { return vy_; }
  int32_t w() const { return w_; }
  StopReason reason() const { return reason_; }
  uint32_t uptimeMs() const { return now_ms_; }

 private:
  void resetParser();
  void consumeByte(uint8_t byte, uint32_t now_ms);
  void finishLine(uint32_t now_ms);
  void processLine(uint32_t now_ms);
  void updateWatchdog(uint32_t now_ms);
  void enterArmed(uint32_t session, uint32_t now_ms);
  void enterDisarmed(StopReason reason, uint32_t now_ms, const char* ack_cmd);
  void rotateToken();
  void emitLine(const char* line);
  void emitHello();
  void emitStatus();
  void emitPong();
  void emitWatchdogEvent();
  void emitAck(const char* cmd, Error error);
  const char* stateString() const;
  const char* reasonString() const;
  static const char* errorString(Error error);

  State state_;
  StopReason reason_;
  uint32_t boot_id_;
  uint32_t arm_token_;
  uint32_t token_state_;
  uint32_t session_;
  uint32_t last_session_;
  uint32_t last_seq_;
  int32_t vx_;
  int32_t vy_;
  int32_t w_;
  uint32_t last_drive_ms_;
  uint32_t now_ms_;

  RandomFn random_;
  void* random_ctx_;
  EmitFn emit_;
  void* emit_ctx_;

  char line_[kMaxLineLen + 1];
  size_t line_len_;
  bool too_long_;
  bool bad_char_;
};

}  // namespace drylink
