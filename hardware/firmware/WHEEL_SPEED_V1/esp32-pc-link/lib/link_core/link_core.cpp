#include "link_core.h"

#include <stdio.h>
#include <string.h>

namespace drylink {
namespace {

bool parseU32(const char* text, uint32_t& value) {
  if (text == nullptr || *text == '\0') {
    return false;
  }
  uint32_t result = 0;
  for (const char* p = text; *p != '\0'; ++p) {
    if (*p < '0' || *p > '9') {
      return false;
    }
    const uint32_t digit = static_cast<uint32_t>(*p - '0');
    if (result > (0xFFFFFFFFu - digit) / 10u) {
      return false;
    }
    result = result * 10u + digit;
  }
  value = result;
  return true;
}

bool parseI32(const char* text, int32_t& value) {
  if (text == nullptr || *text == '\0') {
    return false;
  }
  bool negative = false;
  if (*text == '-') {
    negative = true;
    ++text;
    if (*text == '\0') {
      return false;
    }
  }
  uint64_t magnitude = 0;
  for (const char* p = text; *p != '\0'; ++p) {
    if (*p < '0' || *p > '9') {
      return false;
    }
    magnitude = magnitude * 10u + static_cast<uint64_t>(*p - '0');
    if (magnitude > 2147483648ULL) {
      return false;
    }
  }
  if (negative) {
    if (magnitude == 2147483648ULL) {
      value = INT32_MIN;
    } else {
      value = -static_cast<int32_t>(magnitude);
    }
  } else {
    if (magnitude > 2147483647ULL) {
      return false;
    }
    value = static_cast<int32_t>(magnitude);
  }
  return true;
}

}  // namespace

LinkCore::LinkCore()
    : state_(State::Disarmed),
      reason_(StopReason::Boot),
      boot_id_(1),
      arm_token_(1),
      token_state_(0),
      session_(0),
      last_session_(0),
      last_seq_(0),
      vx_(0),
      vy_(0),
      w_(0),
      last_drive_ms_(0),
      now_ms_(0),
      random_(nullptr),
      random_ctx_(nullptr),
      emit_(nullptr),
      emit_ctx_(nullptr),
      line_len_(0),
      too_long_(false),
      bad_char_(false) {
  line_[0] = '\0';
}

void LinkCore::begin(uint32_t now_ms, const Config& config) {
  random_ = config.random;
  random_ctx_ = config.random_ctx;
  emit_ = config.emit;
  emit_ctx_ = config.emit_ctx;

  boot_id_ = (config.boot_id == 0) ? 1u : config.boot_id;
  arm_token_ = (config.arm_token == 0) ? 1u : config.arm_token;
  token_state_ = boot_id_ ^ arm_token_ ^ 0xA5A5A5A5u;

  state_ = State::Disarmed;
  reason_ = StopReason::Boot;
  session_ = 0;
  last_session_ = 0;
  last_seq_ = 0;
  vx_ = 0;
  vy_ = 0;
  w_ = 0;
  last_drive_ms_ = now_ms;
  now_ms_ = now_ms;
  resetParser();
  emitHello();
}

void LinkCore::resetParser() {
  line_len_ = 0;
  too_long_ = false;
  bad_char_ = false;
  line_[0] = '\0';
}

void LinkCore::feed(const uint8_t* data, size_t len, uint32_t now_ms) {
  // 先检查看门狗，再解析本批字节，过期会话不能被迟到命令重新激活。
  now_ms_ = now_ms;
  // Update first so a command arriving exactly at the 300 ms boundary cannot
  // revive an already-expired motion command.
  updateWatchdog(now_ms);
  for (size_t i = 0; i < len; ++i) {
    consumeByte(data[i], now_ms);
  }
}

void LinkCore::tick(uint32_t now_ms) {
  now_ms_ = now_ms;
  updateWatchdog(now_ms);
}

void LinkCore::consumeByte(uint8_t byte, uint32_t now_ms) {
  if (byte == '\r') {
    return;
  }
  if (byte == '\n') {
    finishLine(now_ms);
    return;
  }
  if (too_long_) {
    return;
  }
  if (line_len_ >= kMaxLineLen) {
    too_long_ = true;
    return;
  }
  if ((byte < 0x20 && byte != '\t') || byte > 0x7E) {
    bad_char_ = true;
  }
  line_[line_len_++] = static_cast<char>(byte);
}

void LinkCore::finishLine(uint32_t now_ms) {
  line_[line_len_] = '\0';
  if (too_long_) {
    resetParser();
    emitAck("", Error::TooLong);
    return;
  }
  if (bad_char_) {
    resetParser();
    emitAck("", Error::BadLine);
    return;
  }
  if (line_len_ == 0) {
    resetParser();
    return;
  }
  processLine(now_ms);
  resetParser();
}

void LinkCore::processLine(uint32_t now_ms) {
  // 按有界 token 解析，协议命令与中文界面显示分离，串口只传 ASCII。
  char* tokens[8];
  size_t token_count = 0;
  bool extra_tokens = false;
  char* p = line_;
  while (*p != '\0') {
    while (*p == ' ' || *p == '\t') {
      ++p;
    }
    if (*p == '\0') {
      break;
    }
    if (token_count < 8) {
      tokens[token_count++] = p;
    } else {
      extra_tokens = true;
    }
    while (*p != '\0' && *p != ' ' && *p != '\t') {
      ++p;
    }
    if (*p != '\0') {
      *p++ = '\0';
    }
  }
  if (token_count == 0) {
    return;
  }

  if (strcmp(tokens[0], "STOP") == 0) {
    enterDisarmed(StopReason::Stop, now_ms, "STOP");
    return;
  }
  if (strcmp(tokens[0], "DISARM") == 0) {
    enterDisarmed(StopReason::Disarm, now_ms, "DISARM");
    return;
  }
  if (extra_tokens) {
    emitAck(tokens[0], Error::BadArgs);
    return;
  }

  if (strcmp(tokens[0], "PING") == 0) {
    if (token_count != 1) {
      emitAck("PING", Error::BadArgs);
      return;
    }
    emitPong();
    return;
  }

  if (strcmp(tokens[0], "STATUS") == 0) {
    if (token_count != 1) {
      emitAck("STATUS", Error::BadArgs);
      return;
    }
    emitStatus();
    return;
  }

  if (strcmp(tokens[0], "ARM") == 0) {
    if (token_count != 3) {
      emitAck("ARM", Error::BadArgs);
      return;
    }
    uint32_t new_session = 0;
    uint32_t token = 0;
    if (!parseU32(tokens[1], new_session) || !parseU32(tokens[2], token) ||
        new_session == 0 || token == 0) {
      emitAck("ARM", Error::BadArgs);
      return;
    }
    if (state_ == State::Armed) {
      emitAck("ARM", Error::AlreadyArmed);
      return;
    }
    if (token != arm_token_) {
      emitAck("ARM", Error::StaleArmToken);
      return;
    }
    if (new_session == last_session_) {
      emitAck("ARM", Error::SessionReused);
      return;
    }
    enterArmed(new_session, now_ms);
    return;
  }

  if (strcmp(tokens[0], "DRIVE") == 0) {
    if (token_count != 6) {
      emitAck("DRIVE", Error::BadArgs);
      return;
    }
    uint32_t drive_session = 0;
    uint32_t seq = 0;
    int32_t vx = 0;
    int32_t vy = 0;
    int32_t w = 0;
    if (!parseU32(tokens[1], drive_session) || !parseU32(tokens[2], seq) ||
        !parseI32(tokens[3], vx) || !parseI32(tokens[4], vy) ||
        !parseI32(tokens[5], w) || drive_session == 0 || seq == 0) {
      emitAck("DRIVE", Error::BadArgs);
      return;
    }
    if (state_ != State::Armed) {
      emitAck("DRIVE", Error::NotArmed);
      return;
    }
    if (drive_session != session_) {
      emitAck("DRIVE", Error::SessionMismatch);
      return;
    }
    if (seq <= last_seq_) {
      emitAck("DRIVE", Error::StaleSeq);
      return;
    }
    if (vx < -kMaxLinearMmS || vx > kMaxLinearMmS || vy < -kMaxLinearMmS ||
        vy > kMaxLinearMmS || w < -kMaxAngularDegS || w > kMaxAngularDegS) {
      emitAck("DRIVE", Error::Range);
      return;
    }
    vx_ = vx;
    vy_ = vy;
    w_ = w;
    last_seq_ = seq;
    last_drive_ms_ = now_ms;
    emitAck("DRIVE", Error::None);
    return;
  }

  emitAck("", Error::UnknownCmd);
}

void LinkCore::updateWatchdog(uint32_t now_ms) {
  if (state_ != State::Armed) {
    return;
  }
  const uint32_t elapsed = now_ms - last_drive_ms_;  // wrap-safe unsigned diff
  if (elapsed >= kWatchdogMs) {
    state_ = State::Disarmed;
    session_ = 0;
    vx_ = 0;
    vy_ = 0;
    w_ = 0;
    last_drive_ms_ = now_ms;
    reason_ = StopReason::Watchdog;
    rotateToken();
    emitWatchdogEvent();
  }
}

void LinkCore::enterArmed(uint32_t session, uint32_t now_ms) {
  state_ = State::Armed;
  reason_ = StopReason::None;
  session_ = session;
  last_session_ = session;
  last_seq_ = 0;
  vx_ = 0;
  vy_ = 0;
  w_ = 0;
  last_drive_ms_ = now_ms;
  emitAck("ARM", Error::None);
}

void LinkCore::enterDisarmed(StopReason reason, uint32_t now_ms,
                             const char* ack_cmd) {
  state_ = State::Disarmed;
  session_ = 0;
  vx_ = 0;
  vy_ = 0;
  w_ = 0;
  last_drive_ms_ = now_ms;
  reason_ = reason;
  rotateToken();
  if (ack_cmd != nullptr) {
    emitAck(ack_cmd, Error::None);
  }
}

void LinkCore::rotateToken() {
  uint32_t next = arm_token_;
  for (int attempt = 0; attempt < 8; ++attempt) {
    if (random_ != nullptr) {
      next = random_(random_ctx_);
    } else {
      token_state_ = token_state_ * 1664525u + 1013904223u;
      next = token_state_;
    }
    if (next != 0 && next != arm_token_) {
      break;
    }
  }
  if (next == 0 || next == arm_token_) {
    next = arm_token_ ^ 0x9E3779B9u;
  }
  if (next == 0) {
    next = 1;
  }
  arm_token_ = next;
}

void LinkCore::emitLine(const char* line) {
  if (emit_ != nullptr) {
    emit_(emit_ctx_, line);
  }
}

void LinkCore::emitHello() {
  char buffer[kMaxOutputLen];
  const int written = snprintf(
      buffer, sizeof(buffer),
      "{\"v\":%u,\"type\":\"hello\",\"fw\":\"%s\",\"mode\":\"%s\","
      "\"protocol\":%u,\"boot_id\":%lu,\"arm_token\":%lu,"
      "\"watchdog_ms\":%lu,\"state\":\"%s\",\"uptime_ms\":%lu}",
      static_cast<unsigned>(kProtocolVersion), kFirmwareVersion, kMode,
      static_cast<unsigned>(kProtocolVersion),
      static_cast<unsigned long>(boot_id_),
      static_cast<unsigned long>(arm_token_),
      static_cast<unsigned long>(kWatchdogMs), stateString(),
      static_cast<unsigned long>(now_ms_));
  if (written > 0 && static_cast<size_t>(written) < sizeof(buffer)) {
    emitLine(buffer);
  }
}

void LinkCore::emitStatus() {
  char buffer[kMaxOutputLen];
  char age_text[24];
  if (state_ == State::Armed) {
    snprintf(age_text, sizeof(age_text), "%lu",
             static_cast<unsigned long>(now_ms_ - last_drive_ms_));
  } else {
    snprintf(age_text, sizeof(age_text), "-1");
  }
  const int written = snprintf(
      buffer, sizeof(buffer),
      "{\"v\":%u,\"type\":\"status\",\"fw\":\"%s\",\"mode\":\"%s\","
      "\"protocol\":%u,\"boot_id\":%lu,\"arm_token\":%lu,\"state\":\"%s\","
      "\"session\":%lu,\"last_session\":%lu,\"last_seq\":%lu,"
      "\"vx\":%ld,\"vy\":%ld,\"w\":%ld,\"age_ms\":%s,"
      "\"watchdog_ms\":%lu,\"reason\":\"%s\",\"uptime_ms\":%lu,"
      "\"dry_run\":true}",
      static_cast<unsigned>(kProtocolVersion), kFirmwareVersion, kMode,
      static_cast<unsigned>(kProtocolVersion),
      static_cast<unsigned long>(boot_id_),
      static_cast<unsigned long>(arm_token_), stateString(),
      static_cast<unsigned long>(session_),
      static_cast<unsigned long>(last_session_),
      static_cast<unsigned long>(last_seq_), static_cast<long>(vx_),
      static_cast<long>(vy_), static_cast<long>(w_), age_text,
      static_cast<unsigned long>(kWatchdogMs), reasonString(),
      static_cast<unsigned long>(now_ms_));
  if (written > 0 && static_cast<size_t>(written) < sizeof(buffer)) {
    emitLine(buffer);
  }
}

void LinkCore::emitPong() {
  char buffer[kMaxOutputLen];
  const int written = snprintf(
      buffer, sizeof(buffer),
      "{\"v\":%u,\"type\":\"pong\",\"boot_id\":%lu,\"arm_token\":%lu,"
      "\"state\":\"%s\",\"uptime_ms\":%lu}",
      static_cast<unsigned>(kProtocolVersion),
      static_cast<unsigned long>(boot_id_),
      static_cast<unsigned long>(arm_token_), stateString(),
      static_cast<unsigned long>(now_ms_));
  if (written > 0 && static_cast<size_t>(written) < sizeof(buffer)) {
    emitLine(buffer);
  }
}

void LinkCore::emitWatchdogEvent() {
  char buffer[kMaxOutputLen];
  const int written = snprintf(
      buffer, sizeof(buffer),
      "{\"v\":%u,\"type\":\"event\",\"event\":\"WATCHDOG_STOP\","
      "\"state\":\"%s\",\"boot_id\":%lu,\"arm_token\":%lu,"
      "\"session\":0,\"last_session\":%lu,\"last_seq\":%lu,"
      "\"reason\":\"%s\",\"uptime_ms\":%lu}",
      static_cast<unsigned>(kProtocolVersion), stateString(),
      static_cast<unsigned long>(boot_id_),
      static_cast<unsigned long>(arm_token_),
      static_cast<unsigned long>(last_session_),
      static_cast<unsigned long>(last_seq_), reasonString(),
      static_cast<unsigned long>(now_ms_));
  if (written > 0 && static_cast<size_t>(written) < sizeof(buffer)) {
    emitLine(buffer);
  }
}

void LinkCore::emitAck(const char* cmd, Error error) {
  char buffer[kMaxOutputLen];
  const int written = snprintf(
      buffer, sizeof(buffer),
      "{\"v\":%u,\"type\":\"ack\",\"cmd\":\"%s\",\"ok\":%s,"
      "\"error\":\"%s\",\"state\":\"%s\",\"boot_id\":%lu,"
      "\"arm_token\":%lu,\"session\":%lu,\"last_session\":%lu,"
      "\"last_seq\":%lu,\"vx\":%ld,\"vy\":%ld,\"w\":%ld,"
      "\"reason\":\"%s\",\"uptime_ms\":%lu}",
      static_cast<unsigned>(kProtocolVersion), cmd,
      (error == Error::None) ? "true" : "false", errorString(error),
      stateString(), static_cast<unsigned long>(boot_id_),
      static_cast<unsigned long>(arm_token_),
      static_cast<unsigned long>(session_),
      static_cast<unsigned long>(last_session_),
      static_cast<unsigned long>(last_seq_), static_cast<long>(vx_),
      static_cast<long>(vy_), static_cast<long>(w_), reasonString(),
      static_cast<unsigned long>(now_ms_));
  if (written > 0 && static_cast<size_t>(written) < sizeof(buffer)) {
    emitLine(buffer);
  }
}

const char* LinkCore::stateString() const {
  return (state_ == State::Armed) ? "ARMED" : "DISARMED";
}

const char* LinkCore::reasonString() const {
  switch (reason_) {
    case StopReason::Boot:
      return "BOOT";
    case StopReason::Stop:
      return "STOP";
    case StopReason::Disarm:
      return "DISARM";
    case StopReason::Watchdog:
      return "WATCHDOG";
    case StopReason::None:
    default:
      return "NONE";
  }
}

const char* LinkCore::errorString(Error error) {
  switch (error) {
    case Error::BadLine:
      return "BAD_LINE";
    case Error::TooLong:
      return "TOO_LONG";
    case Error::BadArgs:
      return "BAD_ARGS";
    case Error::UnknownCmd:
      return "UNKNOWN_CMD";
    case Error::NotArmed:
      return "NOT_ARMED";
    case Error::AlreadyArmed:
      return "ALREADY_ARMED";
    case Error::StaleArmToken:
      return "STALE_ARM_TOKEN";
    case Error::SessionReused:
      return "SESSION_REUSED";
    case Error::SessionMismatch:
      return "SESSION_MISMATCH";
    case Error::StaleSeq:
      return "STALE_SEQ";
    case Error::Range:
      return "RANGE";
    case Error::None:
    default:
      return "NONE";
  }
}

}  // namespace drylink
