// 固件入口与调试导航（详细排查步骤见 ../DEBUGGING.md）：
// 接收：receiveByte -> dispatchCommand（控制权）-> handleCommand -> handleChassis。
// 闭环：SPEED/SPEEDW -> startClosedLoop -> serviceControl -> setMotorForwardPercent。
// 标定：startCalibration -> serviceCalibration；中断/失败统一走 abortCalibration。
// 所有四轮状态数组均按 A/B/C/D 排列；FL/FR/RL/RR 只在 mecanum::mix 内映射。
// 编码器 ISR 只累计原始计数；loop 负责标定、控制、故障与协议，不在 ISR 中打印。
#include <Arduino.h>
#include <esp_random.h>
#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "wireless.h"
#include "calibration.h"
#include "config_store.h"
#include "mecanum.h"
#include "motor_profile.h"
#include "wheel_speed.h"
#include "../../esp32-pc-link/lib/link_core/link_core.h"

// ---- 1. 硬件、节拍与运行状态 ----
// 上电模拟；台架逐路、底盘四路联动，两种真实模式均需电脑显式选择。
enum class RuntimeMode { DryRun, MotorA, Chassis };
RuntimeMode runtimeMode = RuntimeMode::DryRun;
drylink::LinkCore dryCore;
uint32_t physicalBootId = 1;
constexpr int kStandby = 13;
constexpr uint32_t kCooldownMs = 1000;
constexpr uint32_t kLinkTimeoutMs = 1500;
// 调度使用真实经过时间，不靠 delay；修改周期时同时检查测速窗口与故障测试。
constexpr uint32_t kControlPeriodUs = 10000;
constexpr uint32_t kControlMaxGapUs = 50000;
constexpr uint32_t kTelemetryPeriodMs = 50;
constexpr uint32_t kPwmFrequencyHz = 10000;
constexpr int kPwmResolutionBits = 8;
// 标定先等待稳定，再取末段计数。60% 不转时仅重试一次 75%。
constexpr uint32_t kCalSettleMs = 200;
constexpr uint32_t kCalWheelMeasureStartMs = 700;
constexpr uint32_t kCalWheelRunMs = 1200;
constexpr uint32_t kCalSimMeasureStartMs = 1000;
constexpr uint32_t kCalSimRunMs = 1500;
constexpr uint32_t kCalMeasureWindowMs = 500;
struct MotorPins { int in1, in2, encA, encB, pwm1, pwm2; };
// 字段顺序为 IN1、IN2、编码器 A/B、LEDC 通道；接线表按 IN2 在前，不能抄反。
// 当前版本将 PWM 输出接在 IN1/IN2；驱动板 PWMA/B/C/D 固定接其自身 5V。
DRAM_ATTR const MotorPins kMotors[4] = {
    {21, 19, 22, 23, 0, 1}, {16, 4, 17, 18, 2, 3},
    {27, 14, 26, 25, 4, 5}, {32, 33, 35, 34, 6, 7}};
int activeChannel = -1;  // -1=停止；0..3=单路 A..D；-2=四轮联动。
mecanum::Layout wheelLayout;
bool layoutConfigured = false;

// setup 从 NVS 装载标定与 PI 参数；未标定时只拒绝闭环，原开环仍可使用。
// 电机电气极性在 motor_profile.h；编码器反馈符号在 calibrationData.wheel[i].sign。
// 两者分别作用于输出与测量，不能通过改布局方向重复补偿。
calibration::Data calibrationData = {};
wheel_speed::Gains closedGains;
wheel_speed::SpeedEstimator speedEstimators[4];
wheel_speed::WheelPid wheelPids[4];
// vref/ramp/measured 均为 counts/s，appliedDuty 为逻辑占空比百分比。
bool closedLoopActive = false;
bool calibrating = false;
// 标定状态机：逐轮 60/75% 与 100% 两点测速，再做四轮同时 100% 的系统测速。
// 标定期间直接下发开环占空比，不经过 PI，也不置 running。
enum class CalPhase { Idle, Settle, Wheel60, Wheel100, SimSettle, Sim, Done };
struct CalibrationRun {
  CalPhase phase;
  int wheel;
  int duty;
  int tries;
  uint32_t phaseStart;
  uint32_t measureAt;
  int32_t measureCount;
  int8_t signs[4];
  float v60[4];
  float v100[4];
  float vsim[4];
};
CalibrationRun calRun;
// 四轮同时测速需要保存四路测量起点；单轮阶段使用 calRun.measureCount。
int32_t calSimCounts[4] = {};
float goalTarget[4] = {};    // 指令目标，counts/s = 混控百分比 / 100 * Vmax。
float rampedTarget[4] = {};  // 经过加速度限制的目标，counts/s；TLM 显示此值。
float measuredSpeed[4] = {};  // 方向归一化后的实测轮速，counts/s。
float appliedDuty[4] = {};  // PI 输出百分比；写 PWM 前四舍五入，并非目标速度百分比。
int32_t lastEncoderCount[4] = {};
uint32_t lastControlUs = 0;
// FaultMonitor 每个实例只保存一套（三项条件的）持续时间计时，因此必须一轮一个：
// 四轮共用一个实例会把计时按 4 倍推进，且任一正常轮都会把故障轮的计时清零。
// faultLatched 是跨运行的闩锁，只有 STOP/切换模式/撤销控制才清除。
wheel_speed::FaultMonitor faultMonitor[4];
bool faultLatched = false;
const char* lastFaultReason = "none";
// 20 Hz 遥测开关。模式切换、断链、撤销控制权都会关闭，避免向已失权的端口继续发送。
bool telemEnabled = false;
uint32_t lastTelemMs = 0;

// running 仅覆盖定时开环/闭环动作；标定独立使用 calibrating。
// 标定期间的失联保护由 serviceConnections 的 ownerContactAt 执行。
bool running = false;
bool hasStopped = false;
uint32_t startedAt = 0;
uint32_t durationMs = 0;
uint32_t stoppedAt = 0;
uint32_t lastContactAt = 0;
struct CommandLine {
  char buffer[64] = {};
  size_t length = 0;
  bool dropping = false;
};
CommandLine usbLine, bluetoothLine;
uint32_t observedBluetoothEpoch = 0;
uint32_t ownerContactAt = 0;

portMUX_TYPE encoderMux = portMUX_INITIALIZER_UNLOCKED;
struct EncoderState { int32_t count; uint32_t invalid; uint8_t previous; };
volatile EncoderState encoders[4] = {};
// ---- 2. 编码器采集与输出底层 ----
// A/B 两相四边沿计数：查表索引为 (previousAB << 2) | currentAB。
// invalid 统计两位同时变化等非法跃迁；增长时先排查信号/接线/中断负载。
DRAM_ATTR const int8_t encoderSteps[16] = {
    0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

void IRAM_ATTR encoderEdgeFor(int channel) {
  // 中断中只读电平和累计计数，不打印串口；正负号取决于编码器线序。
  portENTER_CRITICAL_ISR(&encoderMux);
  volatile EncoderState& encoder = encoders[channel];
  const uint8_t current = (digitalRead(kMotors[channel].encA) << 1) | digitalRead(kMotors[channel].encB);
  const int8_t step = encoderSteps[(encoder.previous << 2) | current];
  if (step > 0 && encoder.count < INT32_MAX) ++encoder.count;
  if (step < 0 && encoder.count > INT32_MIN) --encoder.count;
  if (step == 0 && current != encoder.previous) ++encoder.invalid;
  encoder.previous = current;
  portEXIT_CRITICAL_ISR(&encoderMux);
}
void IRAM_ATTR encoderEdge() { encoderEdgeFor(0); }
void IRAM_ATTR encoderEdgeB() { encoderEdgeFor(1); }
void IRAM_ATTR encoderEdgeC() { encoderEdgeFor(2); }
void IRAM_ATTR encoderEdgeD() { encoderEdgeFor(3); }

void printEncoder(int channel = -1) {
  if (channel < 0) {
    for (int i = 0; i < 4; ++i) printEncoder(i);
    return;
  }
  portENTER_CRITICAL(&encoderMux);
  const int32_t count = encoders[channel].count;
  const uint32_t invalid = encoders[channel].invalid;
  portEXIT_CRITICAL(&encoderMux);
  reply.printf("ENC %c ticks=%ld invalid=%lu A=%d B=%d\r\n",
                'A' + channel, static_cast<long>(count), static_cast<unsigned long>(invalid),
                digitalRead(kMotors[channel].encA), digitalRead(kMotors[channel].encB));
}

// 一个临界区内取四路计数，避免四个控制轮读到不同时刻的快照。
void snapshotEncoders(int32_t out[4]) {
  portENTER_CRITICAL(&encoderMux);
  for (int i = 0; i < 4; ++i) out[i] = encoders[i].count;
  portEXIT_CRITICAL(&encoderMux);
}

// 停车/启动前清空闭环状态；故障计时器一并清零，避免上一次运行的累积跨运行泄漏。
// 故障闩锁本身由 clearFault() 管理，停一次车不能悄悄解除闭锁。
void resetClosedLoop() {
  closedLoopActive = false;
  for (int i = 0; i < 4; ++i) {
    wheelPids[i].reset();
    speedEstimators[i].reset();
    goalTarget[i] = 0.0f;
    rampedTarget[i] = 0.0f;
    measuredSpeed[i] = 0.0f;
    appliedDuty[i] = 0.0f;
    lastEncoderCount[i] = 0;
  }
  for (int i = 0; i < 4; ++i) faultMonitor[i].reset();
}

// 解除故障闭锁：状态、原因与计时器全部回到初始值。仅 STOP/切换模式/撤销控制调用。
void clearFault() {
  faultLatched = false;
  lastFaultReason = "none";
  for (int i = 0; i < 4; ++i) faultMonitor[i].reset();
}

void stopMotor(uint32_t now) {
  resetClosedLoop();
  // STBY 为四路共用：先撤销使能，再把所有控制输出清零，避免换向时带电。
  digitalWrite(kStandby, LOW);
  for (int i = 0; i < 4; ++i) {
    ledcWrite(kMotors[i].pwm1, 0);
    ledcWrite(kMotors[i].pwm2, 0);
  }
  if (running) {
    hasStopped = true;
    stoppedAt = now;
  }
  running = false;
  activeChannel = -1;
}

void setMotorForwardPercent(int channel, int forwardPercent) {
  // 台架、四轮直控、麦轮混控均走同一入口，杜绝各层重复取反。
  // 调用者保证 channel=0..3、输出在 ±100；这里仅做极性转换与 8 位 PWM 映射。
  const int electrical = motorprofile::electricalPercent(channel, forwardPercent);
  const int magnitude = electrical < 0 ? -electrical : electrical;
  const int pwm = (magnitude*255 + 50)/100;
  ledcWrite(kMotors[channel].pwm1, electrical > 0 ? pwm : 0);
  ledcWrite(kMotors[channel].pwm2, electrical < 0 ? pwm : 0);
}

// ---- 3. 自动标定（直接输出开环 PWM，必须四轮架空）----
// 标定失败/中断的唯一收尾路径：停车、作废本次标定、落盘、回报、回到 Idle。
// 只有 calibrating 时才动作，因此 STOP/切换模式/撤销控制可无条件调用。
void abortCalibration(const char* reason) {
  if (!calibrating) return;
  // 标定输出也是真实电机运动：中止后按普通停车建立 1 s 冷却。
  stopMotor(millis());
  hasStopped = true;
  stoppedAt = millis();
  calibrating = false;
  calibrationData.valid = false;
  // 返回值无需检查：CAL 启动时已成功写入 valid=0；即使此处落盘失败，
  // NVS 也不会残留可用的标定，闭环保持禁用。
  config_store::saveCalibration(calibrationData);
  reply.printf("CAL FAIL %s\r\n", reason);
  calRun.phase = CalPhase::Idle;
}

// CAL?：回读有效性、量程与四轮参数（DEADBAND=a，SLOPE=b）。
void printCalibration() {
  reply.printf("CAL VALID=%d VMAX=%.1f LIMIT=%d\r\n", calibrationData.valid ? 1 : 0,
               calibrationData.vmax, static_cast<int>(lroundf(closedGains.limit)));
  for (int i = 0; i < 4; ++i) {
    const calibration::Wheel& wheel = calibrationData.wheel[i];
    reply.printf("CALW %c SIGN=%d V100=%.1f DEADBAND=%.1f SLOPE=%.4f\r\n",
                 'A' + i, static_cast<int>(wheel.sign), wheel.v100, wheel.a, wheel.b);
  }
}

void startCalibration(uint32_t now) {
  // 所有前置条件先于任何状态修改：被拒绝的 CAL 不得让旧标定失效或让电机掉使能。
  if (runtimeMode != RuntimeMode::Chassis) {
    reply.println("ERROR requires MODE CHASSIS");
    return;
  }
  if (faultLatched) {
    reply.println("ERROR fault latched; send STOP");
    return;
  }
  if (!layoutConfigured) {
    reply.println("ERROR confirm X roller layout and LAYOUT first");
    return;
  }
  if (running) {
    reply.println("ERROR busy; stop before calibration");
    return;
  }
  if (calibrating) {
    reply.println("ERROR busy; calibration already running");
    return;
  }
  if (controlOwner == ControlPort::None) {
    reply.println("ERROR select MODE before control");
    return;
  }
  if (hasStopped && static_cast<uint32_t>(now - stoppedAt) < kCooldownMs) {
    reply.println("ERROR wait 1 second after stop");
    return;
  }

  // 旧标定在标定开始即失效（RAM 与 NVS）：失败或中断后闭环保持禁用。
  calibrationData.valid = false;
  if (!config_store::saveCalibration(calibrationData)) {
    reply.println("CAL FAIL nvs");
    return;
  }

  calRun = CalibrationRun{};
  calibrating = true;
  stopMotor(now);
  calRun.phase = CalPhase::Settle;
  calRun.phaseStart = now;
  calRun.wheel = 0;
  calRun.duty = 60;
  calRun.tries = 0;
  reply.println("CAL START");
}

void serviceCalibration(uint32_t now) {
  if (!calibrating) return;
  const uint32_t elapsed = now - calRun.phaseStart;

  switch (calRun.phase) {
    case CalPhase::Settle: {
      // 停车等待 200 ms 后只给本轮输出；驱动板使能在输出前统一打开。
      if (elapsed < kCalSettleMs) return;
      digitalWrite(kStandby, HIGH);
      setMotorForwardPercent(calRun.wheel, calRun.duty);
      calRun.phase = CalPhase::Wheel60;
      calRun.phaseStart = now;
      calRun.measureAt = 0;
      reply.printf("CAL PROGRESS run %d %d\r\n", calRun.wheel, calRun.duty);
      return;
    }
    case CalPhase::Wheel60: {
      int32_t counts[4];
      snapshotEncoders(counts);
      // 1.2 s 测量窗的最后 500 ms：700 ms 处记录起点。
      if (calRun.measureAt == 0 && elapsed >= kCalWheelMeasureStartMs) {
        calRun.measureAt = now;
        calRun.measureCount = counts[calRun.wheel];
      }
      if (elapsed < kCalWheelRunMs || calRun.measureAt == 0) return;
      const uint32_t window = now - calRun.measureAt;
      if (window < kCalMeasureWindowMs) return;  // 大间隔跳 tick 时保证窗口完整。
      const float v = static_cast<float>(counts[calRun.wheel] - calRun.measureCount) /
                      (static_cast<float>(window) / 1000.0f);
      const float speed = fabsf(v);
      if (speed < 50.0f) {
        if (calRun.tries == 0) {
          // 60% 不转：停输出、等待后换 75% 重试一次。
          calRun.tries = 1;
          calRun.duty = 75;
          setMotorForwardPercent(calRun.wheel, 0);
          calRun.phase = CalPhase::Settle;
          calRun.phaseStart = now;
          calRun.measureAt = 0;
          return;
        }
        char reason[32];
        snprintf(reason, sizeof(reason), "no_feedback %d", calRun.wheel);
        abortCalibration(reason);
        return;
      }
      calRun.signs[calRun.wheel] = (v > 0.0f) ? 1 : -1;
      calRun.v60[calRun.wheel] = speed;
      reply.printf("CAL PROGRESS speed %d %.1f\r\n", calRun.wheel, speed);
      calRun.duty = 100;
      setMotorForwardPercent(calRun.wheel, 100);
      calRun.phase = CalPhase::Wheel100;
      calRun.phaseStart = now;
      calRun.measureAt = 0;
      return;
    }
    case CalPhase::Wheel100: {
      int32_t counts[4];
      snapshotEncoders(counts);
      if (calRun.measureAt == 0 && elapsed >= kCalWheelMeasureStartMs) {
        calRun.measureAt = now;
        calRun.measureCount = counts[calRun.wheel];
      }
      if (elapsed < kCalWheelRunMs || calRun.measureAt == 0) return;
      const uint32_t window = now - calRun.measureAt;
      if (window < kCalMeasureWindowMs) return;
      const float v100 = fabsf(static_cast<float>(counts[calRun.wheel] - calRun.measureCount) /
                               (static_cast<float>(window) / 1000.0f));
      const float d1 = (calRun.tries == 0) ? 60.0f : 75.0f;
      calibration::Wheel& target = calibrationData.wheel[calRun.wheel];
      if (!calibration::computeLine(d1, calRun.v60[calRun.wheel], v100, target)) {
        char reason[32];
        snprintf(reason, sizeof(reason), "bad_line %d", calRun.wheel);
        abortCalibration(reason);
        return;
      }
      // 失败时不改动旧参数；只有成功才写入 sign（computeLine 已写 a/b/v100）。
      target.sign = calRun.signs[calRun.wheel];
      calRun.v100[calRun.wheel] = v100;
      reply.printf("CAL PROGRESS line %d a=%.1f b=%.4f\r\n", calRun.wheel, target.a,
                   target.b);
      setMotorForwardPercent(calRun.wheel, 0);
      ++calRun.wheel;
      calRun.measureAt = 0;
      calRun.tries = 0;
      calRun.phaseStart = now;
      if (calRun.wheel < 4) {
        calRun.phase = CalPhase::Settle;
        calRun.duty = 60;
      } else {
        calRun.phase = CalPhase::SimSettle;
        calRun.duty = 100;
      }
      return;
    }
    case CalPhase::SimSettle: {
      if (elapsed < kCalSettleMs) return;
      digitalWrite(kStandby, HIGH);
      for (int i = 0; i < 4; ++i) setMotorForwardPercent(i, 100);
      calRun.phase = CalPhase::Sim;
      calRun.phaseStart = now;
      calRun.measureAt = 0;
      return;
    }
    case CalPhase::Sim: {
      int32_t counts[4];
      snapshotEncoders(counts);
      // 1.5 s 测量窗的最后 500 ms：1000 ms 处记录四路起点。
      if (calRun.measureAt == 0 && elapsed >= kCalSimMeasureStartMs) {
        calRun.measureAt = now;
        for (int i = 0; i < 4; ++i) calSimCounts[i] = counts[i];
      }
      if (elapsed < kCalSimRunMs || calRun.measureAt == 0) return;
      const uint32_t window = now - calRun.measureAt;
      if (window < kCalMeasureWindowMs) return;
      float minSpeed = 0.0f;
      int minWheel = 0;
      for (int i = 0; i < 4; ++i) {
        const float speed = fabsf(static_cast<float>(counts[i] - calSimCounts[i]) /
                                  (static_cast<float>(window) / 1000.0f));
        // 先只写本次运行；校验通过后才整体复制，失败标定不能把新 vsim
        // 与旧的 a/b/v100 混存。
        calRun.vsim[i] = speed;
        if (i == 0 || speed < minSpeed) {
          minSpeed = speed;
          minWheel = i;
        }
      }
      if (minSpeed < 200.0f) {
        char reason[32];
        snprintf(reason, sizeof(reason), "bad_sim %d", minWheel);
        abortCalibration(reason);
        return;
      }
      for (int i = 0; i < 4; ++i) calibrationData.wheel[i].vsim = calRun.vsim[i];
      calibrationData.vmax =
          calibration::computeVmax(calibrationData.wheel, closedGains.limit);
      calibrationData.valid = true;
      // 先停车再写 NVS：Flash 写入不能发生在车轮仍在驱动时（设计 §5.7）。
      // 标定成功同样以一次真实停车结束，对外部运动命令建立 1 s 冷却。
      stopMotor(millis());
      hasStopped = true;
      stoppedAt = millis();
      if (!config_store::saveCalibration(calibrationData)) {
        // 落盘失败：标定作废（RAM/NVS 都不得留下 valid=1），不走 CAL DONE。
        calibrationData.valid = false;
        abortCalibration("nvs");
        return;
      }
      reply.printf("CAL DONE VALID=1 VMAX=%.1f\r\n", calibrationData.vmax);
      calibrating = false;
      calRun.phase = CalPhase::Done;
      return;
    }
    case CalPhase::Idle:
    case CalPhase::Done:
      return;
  }
}

// ---- 4. 定时动作、模式与协议基础 ----
void serviceDeadline(uint32_t now) {
  // 无符号时间差可跨 millis() 回绕；定时停止不依赖电脑界面是否正常。
  if (!running) return;
  const char* reason = nullptr;
  if (static_cast<uint32_t>(now - startedAt) >= durationMs) {
    reason = "STOPPED timer";
  } else if (static_cast<uint32_t>(now - lastContactAt) >= kLinkTimeoutMs) {
    reason = "STOPPED link_timeout";
  }
  if (reason != nullptr) {
    stopMotor(now);
    reply.println(reason);
    printEncoder();
  }
}

uint32_t randomU32(void*) { return esp_random(); }

void emitDryLine(void*, const char* line) {
  if (runtimeMode == RuntimeMode::DryRun) reply.println(line);
}

const char* modeName() {
  if (runtimeMode == RuntimeMode::DryRun) return "DRY_RUN";
  return runtimeMode == RuntimeMode::Chassis ? "CHASSIS" : "MOTOR_A_TEST";
}

void initializeDry(uint32_t now) {
  drylink::LinkCore::Config config{};
  config.boot_id = physicalBootId;
  config.arm_token = esp_random();
  if (config.arm_token == 0) config.arm_token = 1;
  config.random = &randomU32;
  config.emit = &emitDryLine;
  dryCore.begin(now, config);
}

void selectMode(RuntimeMode mode, uint32_t now) {
  // A mode switch interrupts calibration before any output/state rotation.
  abortCalibration("stopped");
  // Always disable physical output before rotating the simulation session.
  stopMotor(now);
  clearFault();  // 切换模式同样解除闭锁，与设计 §5.6 一致。
  telemEnabled = false;  // 模式切换后必须由新会话重新开启遥测。
  runtimeMode = mode;
  layoutConfigured = false;  // 重连/切换后必须重新确认轮位，不自动复用上一位操作者的设置。
  reply.printf("MODE_OK %s STOPPED\r\n", modeName());
  // begin() emits the original core's hello after the mode acknowledgement.
  // There is no HELLO request in protocol v1. Motor mode suppresses this hello.
  initializeDry(now);
  if (mode == RuntimeMode::MotorA) {
    reply.println("MOTOR_A_TEST: four-channel bench FOUR_V1; stopped by default.");
  } else if (mode == RuntimeMode::Chassis) {
    reply.println("CHASSIS: MECANUM_V2 unified forward; stopped; configure LAYOUT before MECANUM.");
  }
}

// 有界严格整数解析：拒绝小数、溢出、缺参和多余参数，不使用 atoi 静默截断。
bool parseNumbers(const char* text, int32_t* values, int count) {
  for (int i = 0; i < count; ++i) {
    while (*text == ' ') ++text;
    const char* digits = text + ((*text == '-' || *text == '+') ? 1 : 0);
    if (!isdigit(static_cast<unsigned char>(*digits))) return false;
    errno = 0;
    char* end = nullptr;
    const long value = strtol(text, &end, 10);
    if (errno == ERANGE || value < INT32_MIN || value > INT32_MAX ||
        (*end != '\0' && *end != ' ')) return false;
    values[i] = static_cast<int32_t>(value);
    text = end;
  }
  while (*text == ' ') ++text;
  return *text == '\0';
}

void printLayout() {
  reply.printf("LAYOUT CONFIGURED=%d ORDER=%c%c%c%c SIGNS=%d,%d,%d,%d TYPE=X\r\n",
                layoutConfigured ? 1 : 0, 'A'+wheelLayout.channel[0], 'A'+wheelLayout.channel[1],
                'A'+wheelLayout.channel[2], 'A'+wheelLayout.channel[3], wheelLayout.sign[0],
                wheelLayout.sign[1], wheelLayout.sign[2], wheelLayout.sign[3]);
}

// ---- 5. 轮速闭环与遥测 ----
// 20 Hz 遥测：单行固定 13 个数值字段，全部取整供面板按整数解析。
// TLM <ESP32毫秒> <目标A..D counts/s> <实测A..D counts/s> <输出A..D %>。
// 闭环未运行也发送（STOP 后发送的是 resetClosedLoop 清零值，不是停车瞬态），
// 但标定中不发送；输出端口由 replyPort 决定，loop/dispatch 始终指向当前控制端口。
// USB 直写可能阻塞：先格式化，只有 availableForWrite() 容得下整行才写；
// 空间不足时跳过本轮，下一控制 tick 再试。蓝牙仍走现有非阻塞发送队列。
void maybeSendTelemetry(uint32_t nowMs) {
  // 纵深防御：selectMode 已清除 telemEnabled，这里再要求 Chassis 模式。
  if (runtimeMode != RuntimeMode::Chassis) return;
  if (!telemEnabled || controlOwner == ControlPort::None || calibrating) return;
  if (static_cast<uint32_t>(nowMs - lastTelemMs) < kTelemetryPeriodMs) return;
  char line[160];
  const int length = snprintf(
      line, sizeof(line),
      "TLM %lu %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld\r\n",
      static_cast<unsigned long>(nowMs),
      static_cast<long>(lroundf(rampedTarget[0])),
      static_cast<long>(lroundf(rampedTarget[1])),
      static_cast<long>(lroundf(rampedTarget[2])),
      static_cast<long>(lroundf(rampedTarget[3])),
      static_cast<long>(lroundf(measuredSpeed[0])),
      static_cast<long>(lroundf(measuredSpeed[1])),
      static_cast<long>(lroundf(measuredSpeed[2])),
      static_cast<long>(lroundf(measuredSpeed[3])),
      static_cast<long>(lroundf(appliedDuty[0])),
      static_cast<long>(lroundf(appliedDuty[1])),
      static_cast<long>(lroundf(appliedDuty[2])),
      static_cast<long>(lroundf(appliedDuty[3])));
  if (length <= 0 || length >= static_cast<int>(sizeof(line))) return;
  if (replyPort == ControlPort::Usb) {
    if (Serial.availableForWrite() < length) return;  // 跳过本轮，不阻塞控制 tick
    Serial.printf("%s", line);
  } else {
    reply.printf("%s", line);  // 蓝牙：现有 16 包队列，不在此阻塞
  }
  lastTelemMs = nowMs;
}

// 10 ms 控制 tick：计数快照 → 测速 → 目标斜坡 → PI → 逻辑占空比 → PWM。
void serviceControl(uint32_t nowUs) {
  maybeSendTelemetry(millis());  // 无论闭环是否 active 都按 20 Hz 节拍发送。
  if (!closedLoopActive || calibrating) {
    lastControlUs = nowUs;  // 空闲时不积累 dt；重新启动由 startClosedLoop 重置。
    return;
  }
  const uint32_t dtUs = nowUs - lastControlUs;
  if (dtUs < kControlPeriodUs) return;  // 累积到控制周期再服务，按实际 dt 运算。
  if (dtUs > kControlMaxGapUs) {
    // 超过 50 ms 的停顿不能把陈旧积分和测速窗口带进下一次输出。
    // 同时立即清零 8 路 PWM：间隙期间不能让上一次占空比留在电机上，
    // 但保留 running/STBY 状态，下一正常 tick 可以继续闭环。
    for (int i = 0; i < 4; ++i) {
      speedEstimators[i].reset();
      wheelPids[i].reset();
      measuredSpeed[i] = 0.0f;
      appliedDuty[i] = 0.0f;
      setMotorForwardPercent(i, 0);
    }
    snapshotEncoders(lastEncoderCount);
    lastControlUs = nowUs;
    return;
  }

  int32_t counts[4];
  snapshotEncoders(counts);
  const float dt = static_cast<float>(dtUs) * 1e-6f;
  const float vmax = calibrationData.vmax;
  const float maxStep = (closedGains.amax / 100.0f) * vmax * dt;
  wheel_speed::Fault fault = wheel_speed::Fault::None;
  for (int i = 0; i < 4; ++i) {
    // 计数快照始终为原始值；标定符号只作用于速度增量。
    const int32_t delta =
        (counts[i] - lastEncoderCount[i]) * calibrationData.wheel[i].sign;
    lastEncoderCount[i] = counts[i];
    measuredSpeed[i] = speedEstimators[i].update(delta, dtUs);

    // 目标斜坡：AMAX 是每秒可变化的目标百分比量程。
    const float difference = goalTarget[i] - rampedTarget[i];
    if (difference > maxStep) rampedTarget[i] += maxStep;
    else if (difference < -maxStep) rampedTarget[i] -= maxStep;
    else rampedTarget[i] = goalTarget[i];

    appliedDuty[i] = wheelPids[i].step(rampedTarget[i], measuredSpeed[i], vmax,
                                       calibrationData.wheel[i], closedGains, dt);
    setMotorForwardPercent(i, static_cast<int>(lroundf(appliedDuty[i])));

    // 判据用本周期实际占空比；任一路非 None 就整体停车闭锁，故障原因取第一路。
    const wheel_speed::Fault wheelFault = faultMonitor[i].update(
        rampedTarget[i], measuredSpeed[i], appliedDuty[i], vmax, dt);
    if (wheelFault != wheel_speed::Fault::None && fault == wheel_speed::Fault::None)
      fault = wheelFault;
  }
  lastControlUs = nowUs;
  if (fault != wheel_speed::Fault::None) {
    lastFaultReason = wheel_speed::faultName(fault);
    faultLatched = true;
    stopMotor(millis());  // 清 STBY、8 路 PWM 与闭环状态（含故障计时器）。
    closedLoopActive = false;
    reply.printf("FAULT %s\r\n", lastFaultReason);
  }
}

// SPEED/SPEEDW 的公共启动路径；percent 为已混控或直控的四轮百分比。
bool startClosedLoop(const int32_t percent[4], uint32_t duration, uint32_t now) {
  // 所有校验先于任何掉电/状态清理：被拒绝的命令不得让驱动器退出使能。
  if (faultLatched) {
    reply.println("ERROR fault latched; send STOP");
    return false;
  }
  if (!calibrationData.valid || !(calibrationData.vmax > 0.0f)) {
    reply.println("ERROR calibration required");
    return false;
  }
  if (duration == 0) {
    reply.println("ERROR time must be 1..2147483647 ms");
    return false;
  }
  bool anyNonZero = false;
  for (int i = 0; i < 4; ++i) {
    if (percent[i] == 0) continue;
    anyNonZero = true;
    const int32_t magnitude = percent[i] < 0 ? -percent[i] : percent[i];
    if (magnitude < 5) {  // 低速计数量化差、死区附近易抖动，v1 不承诺可控。
      reply.println("ERROR closed-loop target too low");
      return false;
    }
  }
  if (!anyNonZero) {
    // 命令层把全零当 STOP；这里仍拒绝，避免其他调用者无目标上电。
    reply.println("ERROR closed-loop target too low");
    return false;
  }

  stopMotor(now);  // 只有通过全部校验后才清零输出并重新装载闭环状态。
  for (int i = 0; i < 4; ++i)
    goalTarget[i] = (static_cast<float>(percent[i]) / 100.0f) * calibrationData.vmax;
  snapshotEncoders(lastEncoderCount);
  lastControlUs = micros();
  closedLoopActive = true;
  running = true;
  activeChannel = -2;  // 与开环四路联动使用同一状态值。
  startedAt = lastContactAt = now;
  durationMs = duration;
  digitalWrite(kStandby, HIGH);  // 初始占空比为 0，四路准备好后再统一使能。
  reply.printf("SPEED_RUN A=%.0f B=%.0f C=%.0f D=%.0f duration=%lu\r\n",
               goalTarget[0], goalTarget[1], goalTarget[2], goalTarget[3],
               static_cast<unsigned long>(durationMs));
  return true;
}

// ---- 6. 命令解析（先校验，后修改输出状态）----
void handleChassis(const char* line, uint32_t now) {
  if (strcmp(line, "LAYOUT?") == 0) { printLayout(); return; }
  if (strncmp(line, "LAYOUT ", 7) == 0) {
    if (running) { reply.println("ERROR stop before changing layout"); return; }
    const char* order = line + 7;
    int32_t signs[4];
    if (strlen(order) < 6 || order[4] != ' ' || !parseNumbers(order+5, signs, 4)) {
      reply.println("ERROR use LAYOUT <FL FR RL RR channels e.g. ABCD> <four signs +/-1>");
      return;
    }
    mecanum::Layout candidate;
    for (int i = 0; i < 4; ++i) { candidate.channel[i] = order[i]-'A'; candidate.sign[i] = signs[i]; }
    if (!mecanum::valid(candidate)) { reply.println("ERROR layout: unique ABCD channels and signs +/-1 required"); return; }
    for (int i = 0; i < 4; ++i) {
      if (candidate.sign[i] != 1) {
        reply.println("ERROR MECANUM_V2 forward is normalized; all LAYOUT signs must be +1");
        return;
      }
    }
    wheelLayout = candidate;
    layoutConfigured = true;
    printLayout();
    return;
  }
  // 闭环命令优先识别，避免与开环 MECANUM/WHEELS 的解析混在一起。
  const bool speedMix = strncmp(line, "SPEED ", 6) == 0;
  const bool speedDirect = strncmp(line, "SPEEDW ", 7) == 0;
  if (speedMix || speedDirect) {
    int32_t values[5] = {};
    const int count = speedMix ? 4 : 5;
    if (!parseNumbers(line + (speedMix ? 6 : 7), values, count)) {
      reply.println("ERROR use SPEED <x> <y> <z> <ms> or SPEEDW <A> <B> <C> <D> <ms>");
      return;
    }
    for (int i = 0; i < count-1; ++i) {
      if (values[i] < -100 || values[i] > 100) { reply.println("ERROR chassis percent must be -100..100"); return; }
    }
    if (values[count-1] < 1) { reply.println("ERROR time must be 1..2147483647 ms"); return; }
    bool allZero = true;
    for (int i = 0; i < count-1; ++i) allZero = allZero && values[i] == 0;
    if (allZero) {  // 全零只停车，不要求标定/布局；与显式 STOP 不同，不清故障闩锁。
      stopMotor(now);
      reply.println("STOPPED command");
      printEncoder();
      return;
    }
    // 故障闭锁优先于布局/标定/冷却答复：闭锁期间先要求显式 STOP，重发/双击不能重新上电。
    if (faultLatched) {
      reply.println("ERROR fault latched; send STOP");
      return;
    }
    if (speedMix && !layoutConfigured) { reply.println("ERROR confirm X roller layout and LAYOUT first"); return; }
    if (!calibrationData.valid || !(calibrationData.vmax > 0.0f)) {
      reply.println("ERROR calibration required");
      return;
    }
    if (running) { reply.println("ERROR busy; STOP before next chassis motion"); return; }
    if (hasStopped && static_cast<uint32_t>(now-stoppedAt) < kCooldownMs) {
      reply.println("ERROR wait 1 second after stop");
      return;
    }
    int32_t percent[4] = {};
    if (speedMix) mecanum::mix(values[0], values[1], values[2], wheelLayout, percent);
    else for (int i = 0; i < 4; ++i) percent[i] = values[i];
    // startClosedLoop 在全部校验通过后才停输出，拒绝的命令不会掉 STBY。
    startClosedLoop(percent, static_cast<uint32_t>(values[count-1]), now);
    return;
  }
  const bool mix = strncmp(line, "MECANUM ", 8) == 0;
  const bool direct = strncmp(line, "WHEELS ", 7) == 0;
  int32_t values[5] = {};
  const int count = mix ? 4 : 5;
  if ((!mix && !direct) || !parseNumbers(line + (mix ? 8 : 7), values, count)) {
    reply.println("ERROR use MECANUM <forward> <left> <ccw> <ms> or WHEELS <A> <B> <C> <D> <ms>");
    return;
  }
  for (int i = 0; i < count-1; ++i) {
    if (values[i] < -100 || values[i] > 100) { reply.println("ERROR chassis percent must be -100..100"); return; }
  }
  if (values[count-1] < 1) { reply.println("ERROR time must be 1..2147483647 ms"); return; }
  if (mix && !layoutConfigured) { reply.println("ERROR confirm X roller layout and LAYOUT first"); return; }
  int output[4];
  if (mix) mecanum::mix(values[0], values[1], values[2], wheelLayout, output);
  else for (int i = 0; i < 4; ++i) output[i] = values[i];
  if (output[0] == 0 && output[1] == 0 && output[2] == 0 && output[3] == 0) {
    stopMotor(now);
    reply.println("STOPPED command");
    printEncoder();
    return;
  }
  // 这是定时动作接口，不把重复按钮排成长队，也不在运行中突然反向。
  if (running) { reply.println("ERROR busy; STOP before next chassis motion"); return; }
  if (hasStopped && static_cast<uint32_t>(now-stoppedAt) < kCooldownMs) {
    reply.println("ERROR wait 1 second after stop"); return;
  }
  stopMotor(now);
  for (int i = 0; i < 4; ++i) setMotorForwardPercent(i, output[i]);
  activeChannel = -2;  // 四路联动；与单电机 A/B/C/D 的状态区分。
  startedAt = lastContactAt = now;
  durationMs = static_cast<uint32_t>(values[count-1]);
  running = true;
  digitalWrite(kStandby, HIGH);  // 八个输出全部准备好后再统一打开驱动。
  reply.printf("CHASSIS_RUN A=%d B=%d C=%d D=%d duration=%lu\r\n", output[0], output[1], output[2], output[3], static_cast<unsigned long>(durationMs));
}

void handleCommand(char* line, uint32_t now) {
  serviceDeadline(now);
  if (strcmp(line, "WHO") == 0) {
    reply.printf("UNIFIED_ROBOT 1 mode=%s\r\n", modeName());
    return;
  }
  if (strcmp(line, "CAPS") == 0) {
    reply.println("CAPS MOTORS=ABCD PINS=FOUR_V1");
    reply.println("CHASSIS_CAPS API=MECANUM_V2 MIX=X UNITS=PERCENT FORWARD=UNIFIED");
    reply.println("CHASSIS_CAPS2 API=WHEEL_SPEED_V1 CLOSED=1 CAL=1 TELEM=1 PID=MILLI");
    reply.printf("LINK_CAPS USB=1 BT=%s OWNER=EXCLUSIVE NAME=PushRobot-ESP32\r\n", bluetoothReady ? "SPP" : "OFF");
    return;
  }
  if (strcmp(line, "POLARITY") == 0) {
    reply.printf("MOTOR_FORWARD A=%d B=%d C=%d D=%d\r\n", motorprofile::kForwardPolarity[0],
                  motorprofile::kForwardPolarity[1], motorprofile::kForwardPolarity[2], motorprofile::kForwardPolarity[3]);
    return;
  }
  if (strcmp(line, "MODE DRY_RUN") == 0 || strcmp(line, "MODE MOTOR_A_TEST") == 0 || strcmp(line, "MODE CHASSIS") == 0) {
    selectMode(strcmp(line, "MODE DRY_RUN") == 0 ? RuntimeMode::DryRun :
               (strcmp(line, "MODE CHASSIS") == 0 ? RuntimeMode::Chassis : RuntimeMode::MotorA), now);
    return;
  }
  if (strncmp(line, "MODE", 4) == 0) {
    // 无效 MODE 也是一次外部停止请求：标定中必须先走统一中止路径，
    // 否则 Sim 测量窗被四轮同时停车截断，可能以偏小的 vsim 通过校验。
    abortCalibration("stopped");
    stopMotor(now);
    reply.println("ERROR use MODE DRY_RUN, MODE MOTOR_A_TEST or MODE CHASSIS");
    return;
  }
  // 闭环调参命令只允许在 MODE CHASSIS 下使用（设计 §5.9）。
  // 在其它模式下先统一拒绝，绝不能落到开环底盘解析或 JOG 路径。
  const bool closedLoopCommand = strcmp(line, "PID?") == 0 ||
                                 strncmp(line, "PID ", 4) == 0 ||
                                 strncmp(line, "TELEM", 5) == 0 ||
                                 strcmp(line, "SAVE") == 0;
  const bool calibrationCommand = strcmp(line, "CAL") == 0 ||
                                  strcmp(line, "CAL?") == 0 ||
                                  strcmp(line, "CAL RESET") == 0;
  if ((closedLoopCommand || calibrationCommand) && runtimeMode != RuntimeMode::Chassis) {
    reply.println("ERROR requires MODE CHASSIS");
    return;
  }
  // 闭环参数：kp/ki 以千分之一为单位传入，limit 同时决定 Vmax 量程。
  if (strcmp(line, "PID?") == 0) {
    reply.printf("PID_OK KP=%.3f KI=%.3f LIMIT=%d\r\n", closedGains.kp,
                 closedGains.ki, static_cast<int>(lroundf(closedGains.limit)));
    return;
  }
  if (strncmp(line, "PID ", 4) == 0) {
    int32_t values[3];
    if (!parseNumbers(line + 4, values, 3)) {
      reply.println("ERROR use PID <kp_milli 0..10000> <ki_milli 0..10000> <limit 10..100>");
      return;
    }
    if (values[0] < 0 || values[0] > 10000 || values[1] < 0 || values[1] > 10000) {
      reply.println("ERROR PID kp/ki must be 0..10000 (milli)");
      return;
    }
    if (values[2] < 10 || values[2] > 100) {
      reply.println("ERROR PID limit must be 10..100");
      return;
    }
    if (running || calibrating) {
      reply.println("ERROR busy; stop before changing PID");
      return;
    }
    closedGains.kp = static_cast<float>(values[0]) / 1000.0f;
    closedGains.ki = static_cast<float>(values[1]) / 1000.0f;
    closedGains.limit = static_cast<float>(values[2]);
    // limit 参与 Vmax；标定有效时必须立即按原始 vsim 重算，后续 SPEED 才用新量程。
    if (calibrationData.valid)
      calibrationData.vmax =
          calibration::computeVmax(calibrationData.wheel, closedGains.limit);
    reply.printf("PID_OK KP=%.3f KI=%.3f LIMIT=%d\r\n", closedGains.kp,
                 closedGains.ki, static_cast<int>(lroundf(closedGains.limit)));
    return;
  }
  if (strncmp(line, "TELEM", 5) == 0) {
    if (strcmp(line, "TELEM 1") == 0) {
      telemEnabled = true;
      lastTelemMs = millis();  // 第一行约 50 ms 后出现。
      reply.println("TELEM ON");
      return;
    }
    if (strcmp(line, "TELEM 0") == 0) {
      telemEnabled = false;
      reply.println("TELEM OFF");
      return;
    }
    reply.println("ERROR use TELEM 0 or TELEM 1");
    return;
  }
  if (strcmp(line, "SAVE") == 0) {
    if (running || calibrating) {
      reply.println("ERROR busy; stop before SAVE");
      return;
    }
    const bool savedCalibration = config_store::saveCalibration(calibrationData);
    const bool savedGains = config_store::saveGains(closedGains);
    if (savedCalibration && savedGains) reply.println("SAVED");
    else reply.println("ERROR NVS save failed");
    return;
  }
  if (strcmp(line, "CAL") == 0) {
    startCalibration(now);
    return;
  }
  // 查询命令：任何时刻都可读（含故障闭锁、运行中），不改变电机状态。
  if (strcmp(line, "CAL?") == 0) {
    printCalibration();
    return;
  }
  if (strcmp(line, "CAL RESET") == 0) {
    // 设计只要求“停止时”允许；标定后的冷却不适用于本命令。
    // 只清 valid（保留数值供诊断），不解除故障闭锁；落盘失败必须回报。
    if (running || calibrating) {
      reply.println("ERROR busy; stop before CAL RESET");
      return;
    }
    if (faultLatched) {
      reply.println("ERROR fault latched; send STOP");
      return;
    }
    stopMotor(now);
    calibrationData.valid = false;
    // 落盘失败时不回滚 RAM：本机必须立即保持失效（fail-safe），
    // 只提示 NVS 写入失败，闭环禁用不依赖这次写入。
    if (!config_store::saveCalibration(calibrationData)) {
      reply.println("ERROR NVS save failed");
      return;
    }
    reply.println("CAL RESET");
    return;
  }
  if (strcmp(line, "IO") == 0) {
    reply.printf("IO mode=%s standby=%d", modeName(), digitalRead(kStandby));
    for (int i = 0; i < 4; ++i)
      reply.printf(" %c=%lu,%lu", 'A'+i, static_cast<unsigned long>(ledcRead(kMotors[i].pwm1)),
                    static_cast<unsigned long>(ledcRead(kMotors[i].pwm2)));
    reply.println("");
    return;
  }
  if (strcmp(line, "HELP") == 0) {
    reply.println("UNIFIED_ROBOT: WHO | CAPS | MODE DRY_RUN | MODE MOTOR_A_TEST | MODE CHASSIS | IO");
    reply.println("MOTOR_A_TEST limits: duty +/-1..100 percent, time 1..2147483647 ms");
    reply.println("Link timeout: 1500 ms without PING/STATUS; panel maintains heartbeat automatically.");
    reply.println("DRY_RUN: PING STATUS ARM DRIVE STOP DISARM; no physical output.");
    reply.println("MOTOR_A_TEST: JOG <A|B|C|D> <signed_percent> <ms>; one motor at a time");
    reply.println("ENC [channel] | ENCZERO [channel|ALL] | STOP (all) | PING | STATUS");
    reply.println("JOG/WHEELS forward normalized: positive=forward, negative=backward; POLARITY reads electrical mapping");
    reply.println("CHASSIS: LAYOUT <FL/FR/RL/RR e.g.ABCD> <signs> | LAYOUT? | WHEELS A B C D ms | MECANUM forward left ccw ms");
    reply.println("CHASSIS closed loop (needs calibration): SPEED x y z ms | SPEEDW A B C D ms");
    reply.println("CHASSIS tuning: PID <kp_milli 0..10000> <ki_milli 0..10000> <limit 10..100> | PID? | SAVE");
    reply.println("CHASSIS telemetry: TELEM 0|1 (20 Hz TLM) | calibration: CAL | CAL? | CAL RESET");
    return;
  }
  // 标定期间拒绝一切外部运动，且不得调用 stopMotor/改目标，否则会破坏
  // 正在进行的测量窗口。STOP 仍可中止标定（在下面单独处理）。
  const bool motionCommand =
      strncmp(line, "SPEED ", 6) == 0 || strncmp(line, "SPEEDW ", 7) == 0 ||
      strncmp(line, "MECANUM ", 8) == 0 || strncmp(line, "WHEELS ", 7) == 0 ||
      strcmp(line, "JOG") == 0 || strncmp(line, "JOG ", 4) == 0;
  if (calibrating && motionCommand) {
    reply.println("ERROR busy; calibration running");
    return;
  }
  if (runtimeMode == RuntimeMode::DryRun) {
    // 模拟核心没有 GPIO 回调：即使接收 ARM/DRIVE，也只能更改模拟目标。
    stopMotor(now);
    if (strncmp(line, "JOG", 3) == 0) {
      reply.println("ERROR JOG requires MODE MOTOR_A_TEST");
      return;
    }
    dryCore.feed(reinterpret_cast<const uint8_t*>(line), strlen(line), now);
    const uint8_t newline = '\n';
    dryCore.feed(&newline, 1, now);
    return;
  }
  if (strcmp(line, "STOP") == 0) {
    abortCalibration("stopped");  // 标定中 STOP 先中止标定并停车。
    stopMotor(now);
    clearFault();  // 显式 STOP 是解除故障闭锁的唯一常规途径。
    reply.println("STOPPED command");
    printEncoder();
    return;
  }
  if (strcmp(line, "STATUS") == 0) {
    lastContactAt = now;
    if (calibrating) reply.println("RUNNING CAL");
    else if (running && runtimeMode == RuntimeMode::Chassis) reply.println("RUNNING CHASSIS");
    else if (running) reply.printf("RUNNING %c\r\n", 'A' + activeChannel);
    else reply.println("STOPPED ALL");
    printEncoder();
    // 保持旧面板兼容：状态行格式不变，故障另起一行。
    if (faultLatched) reply.printf("FAULT last=%s\r\n", lastFaultReason);
    else reply.println("FAULT none");
    return;
  }
  if (strcmp(line, "PING") == 0) {
    lastContactAt = now;
    reply.printf("PONG %s\r\n", modeName());
    return;
  }
  if (strcmp(line, "ENC") == 0 || (strlen(line) == 5 && strncmp(line, "ENC ", 4) == 0 && line[4] >= 'A' && line[4] <= 'D')) {
    printEncoder(strlen(line) == 3 ? -1 : line[4] - 'A');
    return;
  }
  if (strcmp(line, "ENCZERO") == 0 || strcmp(line, "ENCZERO ALL") == 0 ||
      (strlen(line) == 9 && strncmp(line, "ENCZERO ", 8) == 0 && line[8] >= 'A' && line[8] <= 'D')) {
    if (calibrating) {
      // 标定窗口依赖原始计数；中途清零会让本次测速变成负数/大幅跳变。
      reply.println("ERROR busy; calibration running");
      return;
    }
    if (running) {
      reply.println("ERROR stop motor before resetting encoder");
      return;
    }
    const int selected = strcmp(line, "ENCZERO ALL") == 0 ? -1 : (strlen(line) == 7 ? 0 : line[8] - 'A');
    portENTER_CRITICAL(&encoderMux);
    for (int i = 0; i < 4; ++i) {
      if (selected >= 0 && selected != i) continue;
      encoders[i].count = 0;
      encoders[i].invalid = 0;
      encoders[i].previous = (digitalRead(kMotors[i].encA) << 1) | digitalRead(kMotors[i].encB);
    }
    portEXIT_CRITICAL(&encoderMux);
    if (selected < 0) reply.println("ZEROED ALL");
    else reply.printf("ZEROED %c\r\n", 'A' + selected);
    printEncoder(selected);
    return;
  }

  if (runtimeMode == RuntimeMode::Chassis) { handleChassis(line, now); return; }

  // A bounded line and bounded digit tokens prevent integer-overflow parsing.
  int duty = 0;
  char extra = 0;
  char dutyText[5] = {};
  char timeText[11] = {};
  int selected = 0;
  const char* arguments = line;
  if (strncmp(line, "JOG ", 4) == 0) {
    arguments = line + 4;
    if (arguments[0] >= 'A' && arguments[0] <= 'D' && arguments[1] == ' ') {
      selected = arguments[0] - 'A';
      arguments += 2;
    }
  }
  if (strncmp(line, "JOG ", 4) != 0 ||
      sscanf(arguments, "%4s %10s %c", dutyText, timeText, &extra) != 2) {
    reply.println("ERROR use JOG <A|B|C|D> <signed_percent> <ms> or STOP");
    return;
  }
  for (size_t i = (dutyText[0] == '-' || dutyText[0] == '+') ? 1 : 0;
       i < strlen(dutyText); ++i) {
    if (!isdigit(static_cast<unsigned char>(dutyText[i]))) {
      reply.println("ERROR number");
      return;
    }
  }
  for (size_t i = 0; i < strlen(timeText); ++i) {
    if (!isdigit(static_cast<unsigned char>(timeText[i]))) {
      reply.println("ERROR number");
      return;
    }
  }
  sscanf(dutyText, "%d", &duty);
  errno = 0;
  const unsigned long milliseconds = strtoul(timeText, nullptr, 10);
  if (duty == 0 || duty < -100 || duty > 100 ||
      errno == ERANGE || milliseconds == 0 || milliseconds > INT32_MAX) {
    reply.println("ERROR limits: duty +/-1..100 percent, time 1..2147483647 ms");
    return;
  }
  if (running) {
    reply.println("ERROR busy; existing pulse is not extended");
    return;
  }
  if (hasStopped && static_cast<uint32_t>(now - stoppedAt) < kCooldownMs) {
    reply.println("ERROR wait 1 second after stop");
    return;
  }

  stopMotor(now);
  setMotorForwardPercent(selected, duty);
  startedAt = now;
  lastContactAt = now;
  durationMs = static_cast<uint32_t>(milliseconds);
  running = true;
  activeChannel = selected;
  digitalWrite(kStandby, HIGH);
  reply.printf("JOG %c %d percent %lu ms; automatic stop enabled\r\n",
                'A' + selected, duty, milliseconds);
}

// ---- 7. 上电初始化与 USB/蓝牙协作调度 ----
void setup() {
  digitalWrite(kStandby, LOW);
  pinMode(kStandby, OUTPUT);
  for (int i = 0; i < 4; ++i) {
    const MotorPins& pins = kMotors[i];
    digitalWrite(pins.in1, LOW);
    digitalWrite(pins.in2, LOW);
    pinMode(pins.in1, OUTPUT);
    pinMode(pins.in2, OUTPUT);
    ledcSetup(pins.pwm1, kPwmFrequencyHz, kPwmResolutionBits);
    ledcSetup(pins.pwm2, kPwmFrequencyHz, kPwmResolutionBits);
    ledcAttachPin(pins.in1, pins.pwm1);
    ledcAttachPin(pins.in2, pins.pwm2);
  }
  stopMotor(millis());
  // 编码器由 D24A 的 3.3V 供电；GPIO34/35 仅输入且无内部上拉，不接 5V 信号。
  void (*const handlers[4])() = {encoderEdge, encoderEdgeB, encoderEdgeC, encoderEdgeD};
  for (int i = 0; i < 4; ++i) {
    pinMode(kMotors[i].encA, INPUT);
    pinMode(kMotors[i].encB, INPUT);
    encoders[i].previous = (digitalRead(kMotors[i].encA) << 1) | digitalRead(kMotors[i].encB);
    attachInterrupt(digitalPinToInterrupt(kMotors[i].encA), handlers[i], CHANGE);
    attachInterrupt(digitalPinToInterrupt(kMotors[i].encB), handlers[i], CHANGE);
  }
  // 上电装载标定与增益。失败/缺省时 load 保持默认值，标定保持无效，闭环禁用。
  config_store::load(calibrationData, closedGains);
  Serial.begin(115200);
  beginBluetooth();
  physicalBootId = esp_random();
  if (physicalBootId == 0) physicalBootId = 1;
  runtimeMode = RuntimeMode::DryRun;
  reply.println("UNIFIED_BOOT 1 default=DRY_RUN outputs=STOPPED");
  initializeDry(millis());
}

// Only an explicit MODE command acquires control. Identity and STOP remain
// available from either port; another port cannot keep this owner's lease alive.
void dispatchCommand(char* line, ControlPort source, uint32_t now) {
  const ControlPort previousReply = replyPort;
  replyPort = source;
  const bool identity = strcmp(line, "WHO") == 0 || strcmp(line, "CAPS") == 0 ||
                        strcmp(line, "HELP") == 0 || strcmp(line, "POLARITY") == 0;
  const bool stop = strcmp(line, "STOP") == 0;
  const bool mode = strcmp(line, "MODE DRY_RUN") == 0 ||
                    strcmp(line, "MODE MOTOR_A_TEST") == 0 || strcmp(line, "MODE CHASSIS") == 0;
  if (controlOwner != ControlPort::None && controlOwner != source && !identity && !stop) {
    reply.println("ERROR control_owned_by_other_port; disconnect other panel first");
  } else if (controlOwner == ControlPort::None && !mode && !identity && !stop &&
             strcmp(line, "PING") != 0 && strcmp(line, "STATUS") != 0 && strcmp(line, "IO") != 0 &&
             strcmp(line, "ENC") != 0 && strcmp(line, "LAYOUT?") != 0) {
    reply.println("ERROR select MODE before control");
  } else {
    if (mode) controlOwner = source;
    if (controlOwner == source && (mode || strcmp(line, "PING") == 0 || strcmp(line, "STATUS") == 0))
      ownerContactAt = now;
    handleCommand(line, now);
    // Tell the owner about a stop requested through the other connection too.
    if (stop && controlOwner != ControlPort::None && controlOwner != source) {
      replyPort = controlOwner;
      reply.println("STOPPED command");
    }
  }
  replyPort = previousReply;
}

void revokeControl(uint32_t now, const char* reason) {
  replyPort = controlOwner;
  abortCalibration("stopped");  // 断链/超时/撤销控制权立即中止标定。
  stopMotor(now);
  clearFault();  // 断链/超时撤销控制权后不得保留故障闭锁，重连可重新选择模式。
  telemEnabled = false;  // 断链/超时后不得继续向旧控制端口发送遥测。
  runtimeMode = RuntimeMode::DryRun;
  layoutConfigured = false;
  reply.println(reason);
  // The normal boot marker makes existing PC clients clear pending commands,
  // re-identify the mode and require a fresh layout acknowledgement.
  reply.println("UNIFIED_BOOT 1 default=DRY_RUN outputs=STOPPED");
  initializeDry(now);
  controlOwner = ControlPort::None;
  replyPort = ControlPort::Usb;
  usbLine = CommandLine{};
  bluetoothLine = CommandLine{};
  // Discard already queued commands when ownership expires. Never replay them.
  for (int i = 0; i < 1024 && Serial.available() > 0; ++i) Serial.read();
  if (bluetoothReady)
    for (int i = 0; i < 1024 && SerialBT.available() > 0; ++i) SerialBT.read();
}

void serviceConnections(uint32_t now) {
  const BluetoothState state = bluetoothState();
  if (state.epoch != observedBluetoothEpoch) {
    observedBluetoothEpoch = state.epoch;
    bluetoothLine = CommandLine{};
    if (controlOwner == ControlPort::Bluetooth) revokeControl(now, "STOPPED bluetooth_disconnect");
    if (bluetoothReady)
      for (int i = 0; i < 1024 && SerialBT.available() > 0; ++i) SerialBT.read();
  }
  if (controlOwner == ControlPort::Bluetooth && state.txFailed) {
    revokeControl(now, "STOPPED bluetooth_tx_failed");
  }
  if (controlOwner != ControlPort::None && static_cast<uint32_t>(now-ownerContactAt) >= kLinkTimeoutMs) {
    revokeControl(now, "STOPPED link_timeout");
  }
}

void receiveByte(CommandLine& input, int byte, ControlPort source) {
  if (byte < 0 || byte == '\r') return;
  if (byte == '\n') {
    if (!input.dropping && input.length > 0) {
      input.buffer[input.length] = '\0';
      dispatchCommand(input.buffer, source, millis());
    }
    input = CommandLine{};
  } else if (!input.dropping) {
    if (byte < 32 || byte > 126 || input.length >= sizeof(input.buffer)-1) {
      input.dropping = true;
      input.length = 0;
      // Malformed traffic on the non-owning port must not interfere with it.
      const bool shouldStop = controlOwner == source || controlOwner == ControlPort::None;
      // 畸形行同样属于外部停止：标定中先中止状态机，避免 Sim 测量窗被
      // 四轮同时停车截断后仍以偏小的 vsim 判定成功。
      if (shouldStop) {
        abortCalibration("stopped");
        stopMotor(millis());
      }
      const ControlPort previousReply = replyPort;
      replyPort = source;
      reply.println(shouldStop ? "ERROR invalid/long line; stopped" : "ERROR invalid/long line; discarded");
      replyPort = previousReply;
    } else {
      input.buffer[input.length++] = static_cast<char>(byte);
    }
  }
}

void loop() {
  // 先处理失联与停车，再控制/标定，最后收字节；不要在此添加 delay 或阻塞打印。
  // 收包预算循环中重复服务这些任务，避免连续串口流量饿死控制与停车逻辑。
  serviceConnections(millis());
  replyPort = controlOwner == ControlPort::None ? ControlPort::Usb : controlOwner;
  serviceDeadline(millis());
  serviceControl(micros());
  serviceCalibration(millis());
  if (runtimeMode == RuntimeMode::DryRun) dryCore.tick(millis());
  // Each port has its own bounded parser. A half line cannot block the motor
  // deadline, and bytes from USB and Bluetooth can never form a shared command.
  for (int budget = 0; budget < 64; ++budget) {
    serviceConnections(millis());
    replyPort = controlOwner == ControlPort::None ? ControlPort::Usb : controlOwner;
    serviceDeadline(millis());
    serviceControl(micros());
    serviceCalibration(millis());
    bool received = false;
    if (Serial.available() > 0) {
      receiveByte(usbLine, Serial.read(), ControlPort::Usb);
      received = true;
    }
    if (bluetoothReady && bluetoothState().connected && !bluetoothState().txFailed && SerialBT.available() > 0) {
      receiveByte(bluetoothLine, SerialBT.read(), ControlPort::Bluetooth);
      received = true;
    }
    if (!received) break;
  }
}
