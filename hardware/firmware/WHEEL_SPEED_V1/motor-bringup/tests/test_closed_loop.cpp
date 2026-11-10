#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include "Preferences.h"
#include "../src/config_store.h"
#include "../src/main.cpp"

static void command(const char* text, uint32_t now) {
  fakeNow = now; fakeMicros = now*1000;
  char copy[120]; snprintf(copy,sizeof(copy),"%s",text);
  handleCommand(copy, now);
}
static void tickUs(uint32_t us) { fakeMicros += us; fakeNow = fakeMicros/1000; serviceControl(fakeMicros); }
static void installCalibration() {
  calibrationData = calibration::Data{};
  calibrationData.valid = true; calibrationData.vmax = 4000;
  for (int i=0;i<4;++i) {
    calibrationData.wheel[i].sign = 1;
    calibrationData.wheel[i].a = 40; calibrationData.wheel[i].b = 0.01f;
    calibrationData.wheel[i].v100 = 6000; calibrationData.wheel[i].vsim = 5000;
  }
}
static void stoppedAndClear() {
  assert(!running && levels[kStandby] == LOW);
  for (int i=0;i<8;++i) assert(pwmValues[i] == 0);
}
// Task 8 calibration: every wheel vsim=4000, so Vmax = limit% x 4000.
static void installCalibrationVsim4000() {
  calibrationData = calibration::Data{};
  calibrationData.valid = true;
  for (int i=0;i<4;++i) {
    calibrationData.wheel[i].sign = 1;
    calibrationData.wheel[i].a = 40; calibrationData.wheel[i].b = 0.01f;
    calibrationData.wheel[i].v100 = 5000; calibrationData.wheel[i].vsim = 4000;
  }
  calibrationData.vmax =
      calibration::computeVmax(calibrationData.wheel, closedGains.limit);
}
// 故障/间隙测试用：确认 8 路 PWM 全部为 0。
static void assertAllPwmZero() {
  for (int i = 0; i < 8; ++i) assert(pwmValues[i] == 0);
}
// 真实标定参数：a=32.5、b=0.016，vsim=5000，默认 LIMIT=80 -> Vmax=4000。
static void installRealisticCalibration() {
  calibrationData = calibration::Data{};
  calibrationData.valid = true;
  for (int i = 0; i < 4; ++i) {
    calibrationData.wheel[i].sign = 1;
    calibrationData.wheel[i].a = 32.5f;
    calibrationData.wheel[i].b = 0.016f;
    calibrationData.wheel[i].v100 = 5000.0f;
    calibrationData.wheel[i].vsim = 5000.0f;
  }
  calibrationData.vmax =
      calibration::computeVmax(calibrationData.wheel, closedGains.limit);
}
static int countTelemetryLines(const std::string& text) {
  int count = 0;
  for (size_t pos = text.find("TLM "); pos != std::string::npos;
       pos = text.find("TLM ", pos + 4)) ++count;
  return count;
}

// Task 9 calibration helper: every 10 ms, advance time and inject encoder
// counts derived from the PWM duty currently applied to each wheel.
// 60% -> 2000 counts/s, 75% -> 2500, 100% -> 5000; when all four wheels run
// together (the sim stage) each produces 4500 counts/s.
static int8_t calInjectSign[4] = {1, 1, 1, 1};
// Task 10 failure tests override the injected rate per stage; -1 keeps the
// normal 2000/2500/5000/4500 mapping.
static int32_t calLowRate[4] = {-1, -1, -1, -1};
static int32_t calHighRate[4] = {-1, -1, -1, -1};
static int32_t calSimRate[4] = {-1, -1, -1, -1};
// Counts are integer, so sub-count rates are carried in thousandths.
static int32_t calCountRemainder[4] = {};

static void calSetRates(int32_t low, int32_t high, int32_t sim) {
  for (int i = 0; i < 4; ++i) {
    calLowRate[i] = low;
    calHighRate[i] = high;
    calSimRate[i] = sim;
    calCountRemainder[i] = 0;
  }
}
static void calClearRates() { calSetRates(-1, -1, -1); }

static int32_t calRateFor(int wheel) {
  const int pwm1 = pwmValues[kMotors[wheel].pwm1];
  const int pwm2 = pwmValues[kMotors[wheel].pwm2];
  const int pwm = pwm1 > pwm2 ? pwm1 : pwm2;
  if (pwm == 0) return 0;
  int driven = 0;
  for (int i = 0; i < 4; ++i) {
    if (pwmValues[kMotors[i].pwm1] != 0 || pwmValues[kMotors[i].pwm2] != 0) ++driven;
  }
  if (driven == 4) return calSimRate[wheel] >= 0 ? calSimRate[wheel] : 4500;
  if (pwm >= 250) return calHighRate[wheel] >= 0 ? calHighRate[wheel] : 5000;
  if (pwm >= 180) return 2500;
  return calLowRate[wheel] >= 0 ? calLowRate[wheel] : 2000;
}

// inject=false advances the state machine with a stationary encoder (all zero).
static void calTick(int ms, bool inject = true) {
  for (int t = 0; t < ms; t += 10) {
    fakeNow += 10;
    fakeMicros = fakeNow * 1000;
    for (int i = 0; i < 4; ++i) {
      if (!inject) {
        calCountRemainder[i] = 0;
        continue;
      }
      calCountRemainder[i] += calRateFor(i) * 10;
      const int32_t step = calCountRemainder[i] / 1000;
      calCountRemainder[i] %= 1000;
      encoders[i].count += calInjectSign[i] * step;
    }
    serviceCalibration(fakeNow);
  }
}

int main() {
  setup();
  stoppedAndClear();
  command("MODE CHASSIS", 0);
  command("LAYOUT CBAD 1 1 1 1", 1);
  assert(layoutConfigured);
  installCalibration();
  closedGains = wheel_speed::Gains{};

  // 2. SPEED mixes to four full-scale targets, starts with a zero ramp, and reports.
  Serial.output.clear();
  command("SPEED 100 0 0 1000", 1000);
  assert(running);
  assert(activeChannel == -2);
  assert(levels[kStandby] == HIGH);
  assert(closedLoopActive);
  for (int i=0;i<4;++i) {
    assert(std::fabs(goalTarget[i] - 4000.0f) < 0.01f);
    assert(rampedTarget[i] == 0.0f);
    assert(measuredSpeed[i] == 0.0f);
    assert(appliedDuty[i] == 0.0f);
  }
  assert(Serial.output.find("SPEED_RUN") != std::string::npos);

  // 3. Target ramps by AMAX/100 x Vmax x dt = 300/100 x 4000 x 0.01 = 120 per tick,
  //    reaching the 4000 goal on the 34th tick.
  // 4. Encoder counts follow the current ramp; after the speed window fills the
  //    estimator matches the target and the duty settles near the 80% feedforward.
  for (int t = 1; t <= 40; ++t) {
    for (int i=0;i<4;++i)
      encoders[i].count += static_cast<int32_t>(std::lround(rampedTarget[i] * 0.01f));
    tickUs(10000);
    if (t <= 33) {
      for (int i=0;i<4;++i)
        assert(std::fabs(rampedTarget[i] - 120.0f * t) < 0.5f);
    } else if (t == 34) {
      for (int i=0;i<4;++i) assert(std::fabs(rampedTarget[i] - 4000.0f) < 0.01f);
    }
  }
  for (int i=0;i<4;++i) {
    assert(std::fabs(measuredSpeed[i] - 4000.0f) < 1.0f);
    assert(appliedDuty[i] > 78.0f && appliedDuty[i] < 86.0f);
    // Logical +80% maps through the -1 electrical polarity onto the second PWM pin.
    assert(pwmValues[kMotors[i].pwm1] == 0);
    assert(std::abs(pwmValues[kMotors[i].pwm2] - 204) <= 12);
  }

  // 5. Timer deadline stops and clears the whole closed-loop state.
  serviceDeadline(2000);
  stoppedAndClear();
  assert(!closedLoopActive);
  for (int i=0;i<4;++i) {
    assert(goalTarget[i] == 0.0f && rampedTarget[i] == 0.0f);
    assert(measuredSpeed[i] == 0.0f && appliedDuty[i] == 0.0f);
    assert(wheelPids[i].integral() == 0.0f);
    assert(speedEstimators[i].speed() == 0.0f);
  }

  // 6. SPEEDW is direct per-wheel and does not require a confirmed layout.
  layoutConfigured = false;
  Serial.output.clear();
  command("SPEEDW 50 0 50 0 500", 3000);
  assert(running && activeChannel == -2 && closedLoopActive);
  assert(!layoutConfigured);
  assert(std::fabs(goalTarget[0] - 2000.0f) < 0.01f);
  assert(goalTarget[1] == 0.0f);
  assert(std::fabs(goalTarget[2] - 2000.0f) < 0.01f);
  assert(goalTarget[3] == 0.0f);
  assert(Serial.output.find("SPEED_RUN") != std::string::npos);
  command("STOP", 3001);
  stoppedAndClear();
  command("LAYOUT CBAD 1 1 1 1", 3002);
  assert(layoutConfigured);

  // 7a. Nonzero targets below 5% Vmax are rejected as a whole.
  Serial.output.clear();
  command("SPEED 4 0 0 500", 5000);
  stoppedAndClear();
  assert(Serial.output.find("closed-loop target too low") != std::string::npos);

  // 7b. All-zero closed-loop target is STOP, even while running.
  command("SPEED 50 0 0 500", 5001);
  assert(running && closedLoopActive);
  Serial.output.clear();
  command("SPEED 0 0 0 500", 5002);
  stoppedAndClear();
  assert(Serial.output.find("STOPPED command") != std::string::npos);

  // 7c. Calibration is required for both SPEED and SPEEDW.
  calibrationData.valid = false;
  Serial.output.clear();
  command("SPEED 50 0 0 500", 8000);
  stoppedAndClear();
  assert(Serial.output.find("calibration required") != std::string::npos);
  Serial.output.clear();
  command("SPEEDW 50 50 50 50 500", 8001);
  stoppedAndClear();
  assert(Serial.output.find("calibration required") != std::string::npos);
  installCalibration();

  // 7d. SPEED needs a confirmed layout.
  layoutConfigured = false;
  Serial.output.clear();
  command("SPEED 50 0 0 500", 8002);
  stoppedAndClear();
  assert(Serial.output.find("LAYOUT first") != std::string::npos);
  command("LAYOUT CBAD 1 1 1 1", 8003);
  assert(layoutConfigured);

  // 7e. Running commands are refused without disturbing the active run.
  command("SPEED 50 0 0 1000", 9000);
  assert(running && closedLoopActive);
  const uint32_t started = startedAt;
  Serial.output.clear();
  command("SPEED 50 0 0 500", 9001);
  assert(running && startedAt == started);
  assert(Serial.output.find("busy") != std::string::npos);
  command("STOP", 9002);
  stoppedAndClear();

  // 7f. Cooldown after a stop is enforced.
  Serial.output.clear();
  command("SPEED 50 0 0 500", 9500);
  stoppedAndClear();
  assert(Serial.output.find("wait 1 second") != std::string::npos);

  // 7g. Strict parsing/range for both closed-loop commands; low targets on either
  //     command are rejected without ever entering a run.
  for (const char* bad : {
      "SPEED 50.5 0 0 500", "SPEED 101 0 0 500", "SPEED -101 0 0 500",
      "SPEED 0 0 0 0", "SPEED 50 0 0 0", "SPEED 50 0 0 500 extra", "SPEED 50 0 0",
      "SPEED 50 0 0 99999999999999999999",
      "SPEEDW 50.5 0 0 0 500", "SPEEDW 0 0 0 101 500", "SPEEDW 50 0 0 0 0",
      "SPEEDW 50 0 0 0 500 extra", "SPEEDW 50 0 0 0",
      "SPEEDW 50 0 0 0 99999999999999999999",
      "SPEEDW 4 0 0 0 500", "SPEED -4 0 0 500"}) {
    Serial.output.clear();
    command(bad, 11000);
    stoppedAndClear();
    assert(!closedLoopActive && !running);
  }
  Serial.output.clear();
  command("SPEED 50 0 0 0", 11001);
  stoppedAndClear();
  assert(Serial.output.find("ERROR time") != std::string::npos);
  Serial.output.clear();
  command("SPEEDW 4 0 0 0 500", 11002);
  stoppedAndClear();
  assert(Serial.output.find("closed-loop target too low") != std::string::npos);
  // Direct callers are protected by startClosedLoop's own duration/target validation.
  {
    const int32_t direct[4] = {50, 0, 0, 0};
    Serial.output.clear();
    assert(!startClosedLoop(direct, 0, 11003));
    stoppedAndClear();
    assert(Serial.output.find("ERROR time") != std::string::npos);
  }

  // 7h. SPEED_RUN reports the actual per-wheel vref targets (counts/s), not inputs.
  Serial.output.clear();
  command("SPEEDW 50 -25 10 100 500", 12000);
  assert(running && closedLoopActive);
  assert(Serial.output.find("SPEED_RUN A=2000 B=-1000 C=400 D=4000 duration=500") != std::string::npos);
  command("STOP", 12001);
  stoppedAndClear();

  // Calibrated encoder sign: wheel A raw counts run backwards while the logical
  // speed is positive, so measuredSpeed is +4000 and the wheel is driven forward
  // (positive logical duty, which the -1 electrical polarity maps to pwm2).
  installCalibration();
  calibrationData.wheel[0].sign = -1;
  closedGains = wheel_speed::Gains{};
  command("SPEED 100 0 0 5000", 15000);
  assert(running && closedLoopActive);
  const int32_t rawBefore = encoders[0].count;
  for (int t = 1; t <= 40; ++t) {
    const int32_t step = static_cast<int32_t>(std::lround(rampedTarget[0] * 0.01f));
    encoders[0].count -= step;  // raw counts run backwards
    for (int i=1;i<4;++i) encoders[i].count += step;
    tickUs(10000);
  }
  assert(encoders[0].count < rawBefore);
  assert(std::fabs(measuredSpeed[0] - 4000.0f) < 1.0f);
  assert(appliedDuty[0] > 0.0f);
  assert(pwmValues[kMotors[0].pwm1] == 0);
  assert(pwmValues[kMotors[0].pwm2] > 0);
  command("STOP", 15001);
  stoppedAndClear();
  installCalibration();  // restore sign=+1 for the remaining cases

  // 8. A control gap above 50 ms clears the integrals and the speed window, and the
  //    next normal tick recovers from a fresh count snapshot.
  closedGains = wheel_speed::Gains{};
  command("SPEED 100 0 0 5000", 20000);
  assert(running && closedLoopActive);
  for (int t=0;t<20;++t) {
    for (int i=0;i<4;++i) encoders[i].count += 40;
    tickUs(10000);
  }
  assert(wheelPids[0].integral() != 0.0f);
  assert(speedEstimators[0].speed() > 0.0f);
  fakeMicros += 60000;
  fakeNow = fakeMicros / 1000;
  serviceControl(fakeMicros);
  for (int i=0;i<4;++i) {
    assert(wheelPids[i].integral() == 0.0f);
    assert(speedEstimators[i].speed() == 0.0f);
    assert(measuredSpeed[i] == 0.0f);
    assert(appliedDuty[i] == 0.0f);
  }
  // A >50 ms control gap must not leave the stale duty on the motors.
  assertAllPwmZero();
  assert(levels[kStandby] == HIGH);  // run/STBY state is preserved by design
  assert(running && closedLoopActive);
  for (int i=0;i<4;++i) encoders[i].count += 40;
  tickUs(10000);
  for (int i=0;i<4;++i) assert(std::fabs(measuredSpeed[i] - 4000.0f) < 1.0f);
  assert(running && closedLoopActive);

  // Ticks shorter than the 10 ms control period are ignored (and accumulate);
  // calibrating suspends control.
  command("STOP", 30000);
  stoppedAndClear();
  command("SPEED 100 0 0 5000", 31000);
  assert(running && closedLoopActive);
  const float rampedBefore = rampedTarget[0];
  tickUs(4000);
  assert(rampedTarget[0] == rampedBefore);
  tickUs(5000);  // 9 ms since the last serviced tick: still below one period
  assert(rampedTarget[0] == rampedBefore);
  tickUs(1000);  // 10 ms since the last serviced tick: one control period
  assert(rampedTarget[0] > rampedBefore);
  calibrating = true;
  const float rampedAtCalibration = rampedTarget[0];
  tickUs(10000);
  assert(rampedTarget[0] == rampedAtCalibration);
  calibrating = false;
  command("STOP", 32000);
  stoppedAndClear();

  // Link-timeout stops go through stopMotor and clear the closed-loop state too.
  command("SPEED 100 0 0 10000", 40000);
  assert(running && closedLoopActive);
  Serial.output.clear();
  serviceDeadline(41500);  // 1500 ms without PING/STATUS
  stoppedAndClear();
  assert(!closedLoopActive);
  assert(Serial.output.find("link_timeout") != std::string::npos);

  // 9. Fault detection and latch (Task 7).
  installCalibration();
  closedGains = wheel_speed::Gains{};
  if (!layoutConfigured) command("LAYOUT CBAD 1 1 1 1", 49000);
  assert(layoutConfigured);

  // 9a. Positive target with encoder counts running backwards: encoder_sign after
  //     the 300 ms hold. The monitor must not fire before the hold elapses.
  Serial.output.clear();
  command("SPEED 50 0 0 1000", 50000);
  assert(running && closedLoopActive && !faultLatched);
  int signTicks = 0;
  for (int t = 1; t <= 35 && running; ++t) {
    for (int i=0;i<4;++i) encoders[i].count -= 40;  // logical -4000 counts/s
    tickUs(10000);
    ++signTicks;
  }
  assert(!running && !closedLoopActive);
  assert(faultLatched);
  assert(std::strcmp(lastFaultReason, "encoder_sign") == 0);
  assert(signTicks >= 30);
  stoppedAndClear();
  assert(Serial.output.find("FAULT encoder_sign") != std::string::npos);

  // 9b. The latch rejects new closed-loop commands (even inside the post-stop
  //     cooldown, where the fault message must win) until an explicit STOP.
  Serial.output.clear();
  command("SPEED 50 0 0 1000", 50500);
  assert(!running && !closedLoopActive);
  stoppedAndClear();
  assert(Serial.output.find("fault latched") != std::string::npos);
  Serial.output.clear();
  command("SPEEDW 50 0 0 0 500", 50501);
  stoppedAndClear();
  assert(Serial.output.find("fault latched") != std::string::npos);

  // 9c. STATUS reports the latched reason, and the clean state after STOP.
  Serial.output.clear();
  command("STATUS", 50502);
  assert(Serial.output.find("FAULT last=encoder_sign") != std::string::npos);
  Serial.output.clear();
  command("STOP", 50503);
  assert(!faultLatched);
  assert(std::strcmp(lastFaultReason, "none") == 0);
  assert(Serial.output.find("STOPPED command") != std::string::npos);
  Serial.output.clear();
  command("STATUS", 50504);
  assert(Serial.output.find("FAULT none") != std::string::npos);

  // 9d. After STOP a fresh good run is accepted and stays fault-free.
  Serial.output.clear();
  command("SPEED 50 0 0 5000", 52000);
  assert(running && closedLoopActive && !faultLatched);
  for (int t = 1; t <= 25; ++t) {
    for (int i=0;i<4;++i)
      encoders[i].count += static_cast<int32_t>(std::lround(rampedTarget[i]*0.01f));
    tickUs(10000);
  }
  assert(running && closedLoopActive && !faultLatched);
  assert(Serial.output.find("FAULT") == std::string::npos);
  command("STOP", 53000);
  stoppedAndClear();

  // 9e. A partial fault timer (0.2 s < 0.3 s hold) must not leak into the next
  //     run: STOP, then a good run must start and stay fault-free.
  Serial.output.clear();
  command("SPEED 50 0 0 5000", 55000);
  assert(running && closedLoopActive);
  for (int t = 1; t <= 20 && running; ++t) {
    for (int i=0;i<4;++i) encoders[i].count -= 40;
    tickUs(10000);
  }
  assert(running && closedLoopActive && !faultLatched);
  assert(Serial.output.find("FAULT") == std::string::npos);
  command("STOP", 56000);
  stoppedAndClear();
  Serial.output.clear();
  command("SPEED 50 0 0 5000", 58000);
  assert(running && closedLoopActive && !faultLatched);
  for (int t = 1; t <= 10; ++t) {
    for (int i=0;i<4;++i)
      encoders[i].count += static_cast<int32_t>(std::lround(rampedTarget[i]*0.01f));
    tickUs(10000);
  }
  assert(running && !faultLatched);
  assert(Serial.output.find("FAULT") == std::string::npos);
  command("STOP", 59000);
  stoppedAndClear();

  // 9f. The deadline-stop path also resets the monitor timers (resetClosedLoop),
  //     so a 0.2 s timer cannot carry into the next run and fault it early.
  Serial.output.clear();
  command("SPEED 50 0 0 2000", 62000);
  assert(running && closedLoopActive);
  for (int t = 1; t <= 20 && running; ++t) {
    for (int i=0;i<4;++i) encoders[i].count -= 40;
    tickUs(10000);
  }
  assert(running && !faultLatched);
  Serial.output.clear();
  serviceDeadline(64000);  // 2000 ms deadline -> stopMotor -> resetClosedLoop
  stoppedAndClear();
  assert(!closedLoopActive && !faultLatched);
  assert(Serial.output.find("STOPPED timer") != std::string::npos);

  Serial.output.clear();
  command("SPEED 50 0 0 5000", 66000);
  assert(running && closedLoopActive);
  for (int t = 1; t <= 20 && running; ++t) {
    for (int i=0;i<4;++i) encoders[i].count -= 40;
    tickUs(10000);
  }
  // A leaked 0.2 s timer would fault after ~10 ticks here.
  assert(running && closedLoopActive && !faultLatched);
  assert(Serial.output.find("FAULT") == std::string::npos);
  for (int t = 21; t <= 40 && running; ++t) {
    for (int i=0;i<4;++i) encoders[i].count -= 40;
    tickUs(10000);
  }
  assert(!running && faultLatched);
  assert(std::strcmp(lastFaultReason, "encoder_sign") == 0);
  assert(Serial.output.find("FAULT encoder_sign") != std::string::npos);
  command("STOP", 68000);
  stoppedAndClear();
  assert(!faultLatched);

  // 9g. Low-speed command with no encoder feedback: the PI pushes the duty past
  //     85% and no_feedback fires after its 800 ms hold (inside 3 s).
  Serial.output.clear();
  command("SPEED 10 0 0 4000", 70000);
  assert(running && closedLoopActive);
  for (int t = 1; t <= 300 && running; ++t) tickUs(10000);
  assert(!running && faultLatched);
  assert(std::strcmp(lastFaultReason, "no_feedback") == 0);
  stoppedAndClear();
  assert(Serial.output.find("FAULT no_feedback") != std::string::npos);
  command("STOP", 74000);
  stoppedAndClear();

  // 9h. High target, 500 counts/s feedback: above the no_feedback speed gate but
  //     far below 20% of the target -> stall after the 1000 ms hold.
  Serial.output.clear();
  command("SPEEDW 100 100 100 100 2000", 76000);
  assert(running && closedLoopActive);
  for (int t = 1; t <= 200 && running; ++t) {
    for (int i=0;i<4;++i) encoders[i].count += 5;  // 500 counts/s
    tickUs(10000);
  }
  assert(!running && faultLatched);
  assert(std::strcmp(lastFaultReason, "stall") == 0);
  stoppedAndClear();
  assert(Serial.output.find("FAULT stall") != std::string::npos);

  // 9i. STATUS keeps the last fault name readable while latched.
  Serial.output.clear();
  command("STATUS", 78000);
  assert(Serial.output.find("FAULT last=stall") != std::string::npos);
  command("STOP", 78001);
  assert(!faultLatched);

  // 10. PID set/get/validation, running rejection, and Vmax recompute (Task 8).
  command("STOP", 88000);
  stoppedAndClear();
  closedGains = wheel_speed::Gains{};
  installCalibrationVsim4000();
  assert(std::fabs(calibrationData.vmax - 3200.0f) < 0.01f);  // 80% x 4000

  Serial.output.clear();
  command("PID 2500 1200 75", 90000);
  assert(Serial.output.find("PID_OK KP=2.500 KI=1.200 LIMIT=75") != std::string::npos);
  assert(std::fabs(closedGains.kp - 2.5f) < 0.0005f);
  assert(std::fabs(closedGains.ki - 1.2f) < 0.0005f);
  assert(std::fabs(closedGains.limit - 75.0f) < 0.001f);
  // Changing limit recomputes Vmax immediately from the original vsim.
  assert(std::fabs(calibrationData.vmax - 3000.0f) < 0.01f);

  Serial.output.clear();
  command("PID?", 90001);
  assert(Serial.output.find("PID_OK KP=2.500 KI=1.200 LIMIT=75") != std::string::npos);

  // Both directions of the limit change recompute Vmax (60 -> 2400, 80 -> 3200).
  Serial.output.clear();
  command("PID 2500 1200 60", 90002);
  assert(std::fabs(calibrationData.vmax - 2400.0f) < 0.01f);
  command("PID 2500 1200 80", 90003);
  assert(std::fabs(calibrationData.vmax - 3200.0f) < 0.01f);
  assert(Serial.output.find("PID_OK KP=2.500 KI=1.200 LIMIT=80") != std::string::npos);

  // Invalid ranges and malformed arguments leave the gains and Vmax untouched.
  const float kpBefore = closedGains.kp;
  const float kiBefore = closedGains.ki;
  const float limitBefore = closedGains.limit;
  const float vmaxBefore = calibrationData.vmax;
  for (const char* bad : {
      "PID 10001 1200 80", "PID 2500 10001 80", "PID 2500 1200 9",
      "PID 2500 1200 101", "PID -1 1200 80", "PID 2500 -1 80",
      "PID 2.5 1200 80", "PID 2500 1200 80 extra", "PID 2500 1200",
      "PID 2500 99999999999999999999 80"}) {
    Serial.output.clear();
    command(bad, 90010);
    assert(Serial.output.find("PID_OK") == std::string::npos);
    assert(closedGains.kp == kpBefore && closedGains.ki == kiBefore);
    assert(closedGains.limit == limitBefore && calibrationData.vmax == vmaxBefore);
  }

  // PID is rejected while a closed-loop run is active; the run is undisturbed.
  Serial.output.clear();
  command("SPEEDW 50 0 0 0 1000", 90020);
  assert(running && closedLoopActive);
  Serial.output.clear();
  command("PID 2500 1200 60", 90021);
  assert(Serial.output.find("busy") != std::string::npos);
  assert(Serial.output.find("PID_OK") == std::string::npos);
  assert(closedGains.kp == kpBefore && closedGains.limit == limitBefore);
  assert(std::fabs(calibrationData.vmax - vmaxBefore) < 0.01f);
  assert(running && closedLoopActive);
  command("STOP", 90022);
  stoppedAndClear();

  // CAPS keeps the old line and advertises the v1 closed-loop capability.
  Serial.output.clear();
  command("CAPS", 91000);
  assert(Serial.output.find("CHASSIS_CAPS API=MECANUM_V2 MIX=X UNITS=PERCENT FORWARD=UNIFIED") !=
         std::string::npos);
  assert(Serial.output.find("CHASSIS_CAPS2 API=WHEEL_SPEED_V1 CLOSED=1 CAL=1 TELEM=1 PID=MILLI") !=
         std::string::npos);

  // HELP documents the new closed-loop commands.
  Serial.output.clear();
  command("HELP", 91001);
  assert(Serial.output.find("SPEED") != std::string::npos);
  assert(Serial.output.find("PID") != std::string::npos);
  assert(Serial.output.find("CAL") != std::string::npos);
  assert(Serial.output.find("TELEM") != std::string::npos);

  // SAVE persists calibration + gains; reload sees the same values.
  fakePreferencesReset();
  closedGains.kp = 2.25f; closedGains.ki = 1.5f;
  closedGains.limit = 60.0f; closedGains.amax = 250.0f;
  calibrationData.vmax = calibration::computeVmax(calibrationData.wheel, closedGains.limit);
  Serial.output.clear();
  command("SAVE", 92000);
  assert(Serial.output.find("SAVED") != std::string::npos);
  {
    calibration::Data loadedCal;
    wheel_speed::Gains loadedGains;
    assert(config_store::load(loadedCal, loadedGains));
    assert(loadedCal.valid);
    assert(std::fabs(loadedCal.vmax - 2400.0f) < 0.01f);
    assert(std::fabs(loadedGains.kp - 2.25f) < 0.001f);
    assert(std::fabs(loadedGains.ki - 1.5f) < 0.001f);
    assert(std::fabs(loadedGains.limit - 60.0f) < 0.001f);
    assert(std::fabs(loadedGains.amax - 250.0f) < 0.001f);
  }

  // SAVE is refused while running.
  Serial.output.clear();
  command("SPEEDW 50 0 0 0 1000", 92001);
  assert(running && closedLoopActive);
  Serial.output.clear();
  command("SAVE", 92002);
  assert(Serial.output.find("busy") != std::string::npos);
  assert(Serial.output.find("SAVED") == std::string::npos);
  command("STOP", 92003);
  stoppedAndClear();

  // A failed NVS write reports an error instead of SAVED.
  fakePreferencesFailOnKey("ver");
  Serial.output.clear();
  command("SAVE", 93000);
  assert(Serial.output.find("SAVED") == std::string::npos);
  assert(Serial.output.find("ERROR") != std::string::npos);
  fakePreferencesClearFailures();

  // 11. TELEM 20 Hz timing, 13-field format, off switch and owner gating (Task 8).
  controlOwner = ControlPort::Usb;
  replyPort = ControlPort::Usb;
  Serial.output.clear();
  command("TELEM 1", 100000);
  assert(telemEnabled);
  assert(Serial.output.find("TELEM ON") != std::string::npos);
  Serial.output.clear();
  tickUs(10000);                // +10 ms
  tickUs(10000);                // +20 ms: below the 50 ms period
  assert(Serial.output.find("TLM ") == std::string::npos);
  tickUs(10000);                // +30 ms
  tickUs(10000);                // +40 ms
  assert(Serial.output.find("TLM ") == std::string::npos);
  tickUs(10000);                // +50 ms: first line
  assert(countTelemetryLines(Serial.output) == 1);
  {
    // Fixed 13 numeric fields after the TLM token, all integers.
    const size_t start = Serial.output.find("TLM ") + 4;
    const size_t end = Serial.output.find("\r\n", start);
    assert(end != std::string::npos);
    const std::string line = Serial.output.substr(start, end - start);
    int fields = 0;
    size_t cursor = 0;
    while (cursor < line.size()) {
      const size_t tokenStart = line.find_first_not_of(' ', cursor);
      if (tokenStart == std::string::npos) break;
      const size_t tokenEnd = line.find(' ', tokenStart);
      const std::string token =
          line.substr(tokenStart, tokenEnd == std::string::npos ? std::string::npos
                                                               : tokenEnd - tokenStart);
      char* parseEnd = nullptr;
      const long value = strtol(token.c_str(), &parseEnd, 10);
      (void)value;
      assert(parseEnd != nullptr && *parseEnd == '\0');
      ++fields;
      if (tokenEnd == std::string::npos) break;
      cursor = tokenEnd + 1;
    }
    assert(fields == 13);
  }
  Serial.output.clear();
  tickUs(10000);                // +60 ms: only one line per period
  assert(Serial.output.find("TLM ") == std::string::npos);

  // USB 发送缓冲不足时整行跳过，不阻塞控制 tick；缓冲恢复后下一周期补发。
  Serial.output.clear();
  Serial.writeSpace = 0;
  tickUs(50000);
  assert(Serial.output.find("TLM ") == std::string::npos);
  Serial.writeSpace = 1024;
  tickUs(50000);
  assert(countTelemetryLines(Serial.output) == 1);

  Serial.output.clear();
  command("TELEM 0", fakeNow);
  assert(!telemEnabled);
  assert(Serial.output.find("TELEM OFF") != std::string::npos);
  Serial.output.clear();
  for (int t = 0; t < 10; ++t) tickUs(10000);  // 100 ms without TLM
  assert(Serial.output.find("TLM ") == std::string::npos);

  // Telemetry is suppressed while calibrating.
  Serial.output.clear();
  command("TELEM 1", 101000);
  assert(telemEnabled);
  calibrating = true;
  Serial.output.clear();
  for (int t = 0; t < 10; ++t) tickUs(10000);
  assert(Serial.output.find("TLM ") == std::string::npos);
  calibrating = false;
  command("TELEM 0", 101200);
  assert(!telemEnabled);
  stoppedAndClear();

  // Mode switches and control revocation stop telemetry automatically.
  Serial.output.clear();
  command("TELEM 1", 102000);
  assert(telemEnabled);
  selectMode(RuntimeMode::Chassis, 102001);
  assert(!telemEnabled);
  command("TELEM 1", 102002);
  assert(telemEnabled);
  revokeControl(102003, "STOPPED test");
  assert(!telemEnabled);
  assert(controlOwner == ControlPort::None);

  // 12. Closed-loop tuning commands are Chassis-only (design §5.9). In any other
  //     mode they are rejected before the chassis parser or JOG path can see them.
  struct ModeCase { const char* command; RuntimeMode mode; };
  const ModeCase modeCases[] = {
      {"MODE MOTOR_A_TEST", RuntimeMode::MotorA},
      {"MODE DRY_RUN", RuntimeMode::DryRun}};
  uint32_t modeNow = 103000;
  const wheel_speed::Gains gainsBeforeMode = closedGains;
  const float vmaxBeforeMode = calibrationData.vmax;
  for (const ModeCase& modeCase : modeCases) {
    command(modeCase.command, modeNow++);
    assert(runtimeMode == modeCase.mode);
    assert(!telemEnabled);
    fakePreferencesReset();
    for (const char* text : {"PID 2500 1200 75", "PID?", "TELEM 1", "SAVE"}) {
      Serial.output.clear();
      command(text, modeNow++);
      assert(Serial.output.find("ERROR requires MODE CHASSIS") != std::string::npos);
      assert(Serial.output.find("PID_OK") == std::string::npos);
      assert(Serial.output.find("TELEM ON") == std::string::npos);
      assert(Serial.output.find("SAVED") == std::string::npos);
      assert(Serial.output.find("ERROR use JOG") == std::string::npos);
      assert(!telemEnabled);
      assert(closedGains.kp == gainsBeforeMode.kp && closedGains.ki == gainsBeforeMode.ki);
      assert(closedGains.limit == gainsBeforeMode.limit);
      assert(closedGains.amax == gainsBeforeMode.amax);
      assert(calibrationData.vmax == vmaxBeforeMode);
      assert(!fakePreferencesHasKey("chassis_cl", "ver"));
      assert(!fakePreferencesHasKey("chassis_cl", "kp"));
    }
  }

  // Back in MODE CHASSIS the same commands work again.
  command("MODE CHASSIS", modeNow++);
  assert(runtimeMode == RuntimeMode::Chassis);
  controlOwner = ControlPort::Usb;
  replyPort = ControlPort::Usb;
  Serial.output.clear();
  command("PID 2500 1200 75", modeNow++);
  assert(Serial.output.find("PID_OK KP=2.500 KI=1.200 LIMIT=75") != std::string::npos);
  Serial.output.clear();
  command("PID?", modeNow++);
  assert(Serial.output.find("PID_OK KP=2.500 KI=1.200 LIMIT=75") != std::string::npos);
  Serial.output.clear();
  command("TELEM 1", modeNow++);
  assert(telemEnabled);
  assert(Serial.output.find("TELEM ON") != std::string::npos);
  Serial.output.clear();
  for (int t = 0; t < 5; ++t) tickUs(10000);  // 50 ms -> one TLM line
  assert(countTelemetryLines(Serial.output) == 1);
  Serial.output.clear();
  command("TELEM 0", modeNow++);
  assert(!telemEnabled);
  fakePreferencesReset();
  Serial.output.clear();
  command("SAVE", modeNow++);
  assert(Serial.output.find("SAVED") != std::string::npos);
  assert(fakePreferencesHasKey("chassis_cl", "ver"));
  assert(fakePreferencesHasKey("chassis_cl", "kp"));

  // 13. Calibration state machine success path (Task 9).
  fakePreferencesReset();
  closedGains = wheel_speed::Gains{};
  calibrationData = calibration::Data{};
  faultLatched = false;
  controlOwner = ControlPort::Usb;
  replyPort = ControlPort::Usb;
  calibrating = false;
  command("MODE CHASSIS", 200000);
  command("LAYOUT CBAD 1 1 1 1", 200001);
  assert(layoutConfigured);
  assert(runtimeMode == RuntimeMode::Chassis);

  Serial.output.clear();
  command("CAL", 201000);
  assert(calibrating);
  assert(calRun.phase == CalPhase::Settle);
  assert(calRun.wheel == 0 && calRun.duty == 60 && calRun.tries == 0);
  assert(Serial.output.find("CAL START") != std::string::npos);
  // Starting a calibration invalidates the old result immediately, in RAM and NVS.
  assert(!calibrationData.valid);
  {
    calibration::Data stored;
    wheel_speed::Gains storedGains;
    assert(config_store::load(stored, storedGains));
    assert(!stored.valid);
  }

  // A/C are wired to count forward, B/D backwards: the state machine must
  // discover each wheel's sign. Absolute speed still follows the duty.
  calInjectSign[0] = 1;
  calInjectSign[1] = -1;
  calInjectSign[2] = 1;
  calInjectSign[3] = -1;
  bool statusChecked = false;
  while (calibrating && fakeNow < 240000) {
    calTick(10);
    if (!statusChecked && calibrating && calRun.phase == CalPhase::Wheel100) {
      Serial.output.clear();
      command("STATUS", fakeNow);
      assert(Serial.output.find("RUNNING CAL") != std::string::npos);
      int encLines = 0;
      for (size_t pos = Serial.output.find("ENC "); pos != std::string::npos;
           pos = Serial.output.find("ENC ", pos + 4)) ++encLines;
      assert(encLines == 4);
      statusChecked = true;
    }
  }
  assert(statusChecked);
  assert(!calibrating);
  assert(calRun.phase == CalPhase::Done);
  assert(Serial.output.find("CAL DONE VALID=1 VMAX=3600.0") != std::string::npos);
  assert(calibrationData.valid);
  assert(std::fabs(calibrationData.vmax - 3600.0f) < 0.05f);  // 80% x min(vsim=4500)
  for (int i = 0; i < 4; ++i) {
    assert(calRun.signs[i] == calInjectSign[i]);
    assert(calibrationData.wheel[i].sign == calInjectSign[i]);
    assert(calRun.v60[i] == 2000.0f);   // exact: 20 counts/tick over 500 ms
    assert(calRun.v100[i] == 5000.0f);  // exact: 50 counts/tick over 500 ms
    assert(calRun.vsim[i] == 4500.0f);  // exact: 45 counts/tick over 500 ms
    assert(calibrationData.wheel[i].vsim == 4500.0f);
  }
  stoppedAndClear();
  assert(levels[kStandby] == LOW);

  // A successful calibration ends with a real motor stop: external motion is
  // subject to the ordinary 1 s cooldown. Calibration's internal stage
  // transitions are not (the run above was continuous).
  const uint32_t calibrationDoneAt = fakeNow;
  assert(hasStopped);
  assert(stoppedAt == calibrationDoneAt);
  Serial.output.clear();
  command("SPEED 50 0 0 500", calibrationDoneAt + 10);
  assert(!running && !closedLoopActive);
  assert(Serial.output.find("wait 1 second after stop") != std::string::npos);
  stoppedAndClear();
  Serial.output.clear();
  command("SPEED 50 0 0 500", calibrationDoneAt + 1000);  // cooldown boundary
  assert(running && closedLoopActive);
  command("STOP", calibrationDoneAt + 1100);
  stoppedAndClear();

  // CAL? reports DEADBAND=a and SLOPE=b for all four wheels.
  Serial.output.clear();
  command("CAL?", 250000);
  assert(Serial.output.find("CAL VALID=1 VMAX=3600.0 LIMIT=80") != std::string::npos);
  int calwLines = 0;
  for (size_t pos = Serial.output.find("CALW "); pos != std::string::npos;
       pos = Serial.output.find("CALW ", pos + 5)) ++calwLines;
  assert(calwLines == 4);
  assert(Serial.output.find(
             "CALW A SIGN=1 V100=5000.0 DEADBAND=33.3 SLOPE=0.0133") !=
         std::string::npos);
  assert(Serial.output.find("CALW B SIGN=-1 V100=5000.0") != std::string::npos);
  assert(Serial.output.find("CALW D SIGN=-1 V100=5000.0") != std::string::npos);

  // The successful calibration was saved: a fresh load sees valid and Vmax.
  {
    calibration::Data loaded;
    wheel_speed::Gains loadedGains;
    assert(config_store::load(loaded, loadedGains));
    assert(loaded.valid);
    assert(std::fabs(loaded.vmax - 3600.0f) < 0.05f);
    assert(std::fabs(loadedGains.limit - 80.0f) < 0.001f);
    assert(loaded.wheel[1].sign == -1);
  }

  // CAL RESET invalidates RAM and NVS but preserves the diagnostic values.
  Serial.output.clear();
  command("CAL RESET", 251000);
  assert(!calibrationData.valid);
  assert(Serial.output.find("CAL RESET") != std::string::npos);
  {
    calibration::Data loaded;
    wheel_speed::Gains loadedGains;
    assert(config_store::load(loaded, loadedGains));
    assert(!loaded.valid);
    assert(std::fabs(loaded.wheel[0].v100 - 5000.0f) < 0.01f);
  }
  Serial.output.clear();
  command("CAL?", 251001);
  assert(Serial.output.find("CAL VALID=0 VMAX=3600.0 LIMIT=80") != std::string::npos);

  // CAL/CAL?/CAL RESET are Chassis-only.
  command("MODE DRY_RUN", 252000);
  for (const char* calCommand : {"CAL", "CAL?", "CAL RESET"}) {
    Serial.output.clear();
    command(calCommand, 252001);
    assert(Serial.output.find("ERROR requires MODE CHASSIS") != std::string::npos);
    assert(!calibrating);
  }
  command("MODE CHASSIS", 252002);
  command("LAYOUT CBAD 1 1 1 1", 252003);

  // An aborted calibration also ends with a real stop, so it establishes the
  // same cooldown for the next external motion command.
  Serial.output.clear();
  command("CAL", 260000);
  assert(calibrating);
  calTick(300);  // 200 ms settle + 100 ms driving wheel A at 60%
  assert(calibrating);
  assert(calRun.phase == CalPhase::Wheel60);
  Serial.output.clear();
  command("STOP", fakeNow);
  assert(!calibrating);
  assert(Serial.output.find("CAL FAIL stopped") != std::string::npos);
  assert(hasStopped);
  const uint32_t abortedAt = fakeNow;
  assert(stoppedAt == abortedAt);
  stoppedAndClear();
  Serial.output.clear();
  command("WHEELS 10 0 0 0 500", abortedAt + 10);
  assert(!running);
  assert(Serial.output.find("wait 1 second after stop") != std::string::npos);
  stoppedAndClear();
  Serial.output.clear();
  command("WHEELS 10 0 0 0 500", abortedAt + 1000);  // cooldown boundary
  assert(running);
  command("STOP", abortedAt + 1100);
  stoppedAndClear();

  // 14. Task 10: calibration failure/interruption matrix and safety gates.
  fakePreferencesReset();
  calClearRates();
  calibrationData = calibration::Data{};
  calibrating = false;
  faultLatched = false;
  controlOwner = ControlPort::Usb;
  replyPort = ControlPort::Usb;
  assert(runtimeMode == RuntimeMode::Chassis);
  assert(layoutConfigured);

  // 14a. While calibrating, all motion commands, ENCZERO, CAL and CAL RESET are
  //      refused with the existing busy error and must not disturb the run.
  Serial.output.clear();
  command("CAL", 270000);
  assert(calibrating);
  calTick(100);  // still inside the 200 ms settle: no wheel output yet
  const uint32_t gateNow = fakeNow;
  for (const char* motion : {"SPEED 50 0 0 500", "SPEEDW 50 0 0 0 500",
                             "MECANUM 10 0 0 500", "WHEELS 10 0 0 0 500",
                             "JOG A 10 500", "SPEED 0 0 0 500"}) {
    Serial.output.clear();
    command(motion, gateNow);
    assert(Serial.output.find("busy") != std::string::npos);
    assert(calibrating && !running);
    assert(calRun.phase == CalPhase::Settle);
    assert(levels[kStandby] == LOW);
    for (int i = 0; i < 8; ++i) assert(pwmValues[i] == 0);
  }
  const int32_t zeroBefore[4] = {encoders[0].count, encoders[1].count,
                                 encoders[2].count, encoders[3].count};
  Serial.output.clear();
  command("ENCZERO", gateNow);
  assert(Serial.output.find("busy") != std::string::npos);
  for (int i = 0; i < 4; ++i) assert(encoders[i].count == zeroBefore[i]);
  assert(calibrating);
  Serial.output.clear();
  command("CAL", gateNow);  // already calibrating
  assert(Serial.output.find("busy") != std::string::npos);
  assert(calibrating);
  Serial.output.clear();
  command("CAL RESET", gateNow);
  assert(Serial.output.find("busy") != std::string::npos);
  assert(calibrating);
  // Rejections preserved the measurement window: the run is still driving.
  calTick(200);
  assert(calibrating);
  assert(calRun.phase == CalPhase::Wheel60);
  assert(levels[kStandby] == HIGH);
  assert(pwmValues[kMotors[0].pwm1] != 0 || pwmValues[kMotors[0].pwm2] != 0);
  // Rejection also holds while a wheel is actively driven, and the drive
  // output must be unchanged by the rejected command.
  const int drivePwm1 = pwmValues[kMotors[0].pwm1];
  const int drivePwm2 = pwmValues[kMotors[0].pwm2];
  Serial.output.clear();
  command("SPEEDW 50 0 0 0 500", fakeNow);
  assert(Serial.output.find("busy") != std::string::npos);
  assert(calibrating && calRun.phase == CalPhase::Wheel60);
  assert(levels[kStandby] == HIGH);
  assert(pwmValues[kMotors[0].pwm1] == drivePwm1);
  assert(pwmValues[kMotors[0].pwm2] == drivePwm2);

  // STOP aborts the run: CAL FAIL, idle, valid=0 in RAM and NVS.
  Serial.output.clear();
  command("STOP", fakeNow);
  assert(!calibrating);
  assert(Serial.output.find("CAL FAIL stopped") != std::string::npos);
  assert(Serial.output.find("STOPPED command") != std::string::npos);
  assert(calRun.phase == CalPhase::Idle);
  stoppedAndClear();
  assert(!calibrationData.valid);
  {
    calibration::Data stored;
    wheel_speed::Gains storedGains;
    assert(config_store::load(stored, storedGains));
    assert(!stored.valid);
  }
  // abortCalibration is idempotent: a second call is a no-op.
  Serial.output.clear();
  abortCalibration("again");
  assert(!calibrating);
  assert(Serial.output.find("CAL FAIL again") == std::string::npos);
  const uint32_t gateAbortAt = fakeNow;
  Serial.output.clear();
  command("SPEED 50 0 0 500", gateAbortAt + 1000);
  assert(!running && !closedLoopActive);
  assert(Serial.output.find("calibration required") != std::string::npos);

  // 14b. No encoder feedback at 60% and again at the 75% retry -> no_feedback.
  calClearRates();
  const uint32_t nfStart = gateAbortAt + 1010;
  Serial.output.clear();
  command("CAL", nfStart);
  assert(calibrating);
  calTick(1500, false);  // settle + failed 60% window + retry settle at 75%
  assert(calibrating);
  assert(calRun.tries == 1);
  assert(calRun.duty == 75);
  assert(calRun.phase == CalPhase::Settle);
  calTick(1400, false);  // 200 ms settle + failed 75% window -> abort
  assert(!calibrating);
  assert(Serial.output.find("CAL FAIL no_feedback") != std::string::npos);
  assert(calRun.phase == CalPhase::Idle);
  stoppedAndClear();
  assert(!calibrationData.valid);
  {
    calibration::Data stored;
    wheel_speed::Gains storedGains;
    assert(config_store::load(stored, storedGains));
    assert(!stored.valid);
  }
  const uint32_t nfAbortAt = fakeNow;
  Serial.output.clear();
  command("SPEED 50 0 0 500", nfAbortAt + 1000);
  assert(!running && !closedLoopActive);
  assert(Serial.output.find("calibration required") != std::string::npos);

  // 14c. v60 and v100 nearly equal (60% -> 2000, 100% -> 2001 counts/s).
  calSetRates(-1, 2001, -1);
  const uint32_t blStart = nfAbortAt + 1010;
  Serial.output.clear();
  command("CAL", blStart);
  assert(calibrating);
  while (calibrating && fakeNow < blStart + 10000) calTick(10);
  assert(!calibrating);
  assert(Serial.output.find("CAL FAIL bad_line 0") != std::string::npos);
  assert(calRun.phase == CalPhase::Idle);
  assert(calibrationData.wheel[0].v100 == 0.0f);  // old diagnostic untouched
  stoppedAndClear();
  assert(!calibrationData.valid);
  {
    calibration::Data stored;
    wheel_speed::Gains storedGains;
    assert(config_store::load(stored, storedGains));
    assert(!stored.valid);
  }
  const uint32_t blAbortAt = fakeNow;
  calClearRates();

  // 14d. revokeControl (the link-timeout/TX-failure path) aborts calibration.
  const uint32_t rvStart = blAbortAt + 1010;
  Serial.output.clear();
  command("CAL", rvStart);
  assert(calibrating);
  calTick(300);
  assert(calibrating && calRun.phase == CalPhase::Wheel60);
  Serial.output.clear();
  revokeControl(fakeNow, "STOPPED test");
  assert(!calibrating);
  assert(Serial.output.find("CAL FAIL stopped") != std::string::npos);
  assert(calRun.phase == CalPhase::Idle);
  stoppedAndClear();
  assert(!calibrationData.valid);
  assert(runtimeMode == RuntimeMode::DryRun);
  assert(controlOwner == ControlPort::None);
  {
    calibration::Data stored;
    wheel_speed::Gains storedGains;
    assert(config_store::load(stored, storedGains));
    assert(!stored.valid);
  }
  const uint32_t rvAbortAt = fakeNow;
  command("MODE CHASSIS", rvAbortAt + 10);
  controlOwner = ControlPort::Usb;
  replyPort = ControlPort::Usb;
  command("LAYOUT CBAD 1 1 1 1", rvAbortAt + 20);
  assert(runtimeMode == RuntimeMode::Chassis && layoutConfigured);

  // 14e. A mode switch during calibration aborts it before rotating the mode.
  const uint32_t mdStart = rvAbortAt + 1010;
  Serial.output.clear();
  command("CAL", mdStart);
  assert(calibrating);
  calTick(300);
  assert(calibrating && calRun.phase == CalPhase::Wheel60);
  Serial.output.clear();
  command("MODE DRY_RUN", fakeNow);
  assert(!calibrating);
  assert(Serial.output.find("CAL FAIL stopped") != std::string::npos);
  assert(calRun.phase == CalPhase::Idle);
  stoppedAndClear();
  assert(!calibrationData.valid);
  assert(runtimeMode == RuntimeMode::DryRun);
  assert(!layoutConfigured);
  {
    calibration::Data stored;
    wheel_speed::Gains storedGains;
    assert(config_store::load(stored, storedGains));
    assert(!stored.valid);
  }
  const uint32_t mdAbortAt = fakeNow;
  command("MODE CHASSIS", mdAbortAt + 10);
  controlOwner = ControlPort::Usb;
  replyPort = ControlPort::Usb;
  command("LAYOUT CBAD 1 1 1 1", mdAbortAt + 20);
  assert(runtimeMode == RuntimeMode::Chassis && layoutConfigured);

  // 14f. A failed valid=0 save at CAL start aborts before any motion.
  const uint32_t nvsStart = mdAbortAt + 1010;
  calibrationData.valid = true;  // stale valid flag must be cleared in RAM
  fakePreferencesReset();
  fakePreferencesFailOnKey("valid", 1);
  Serial.output.clear();
  command("CAL", nvsStart);
  assert(!calibrating);
  assert(Serial.output.find("CAL FAIL nvs") != std::string::npos);
  assert(!calibrationData.valid);
  stoppedAndClear();
  {
    calibration::Data stored;
    wheel_speed::Gains storedGains;
    assert(!config_store::load(stored, storedGains));
  }
  fakePreferencesClearFailures();
  const uint32_t nvsAbortAt = fakeNow;
  assert(nvsAbortAt == nvsStart);  // no fake time passed, no motion

  // 14g. A successful calibration (valid=1), then a failed recalibration:
  //      valid goes back to 0 in RAM and NVS, and SPEED is rejected again.
  calClearRates();
  const uint32_t okStart = nvsAbortAt + 1010;
  Serial.output.clear();
  command("CAL", okStart);
  assert(calibrating);
  while (calibrating && fakeNow < okStart + 30000) calTick(10);
  assert(!calibrating);
  assert(Serial.output.find("CAL DONE VALID=1") != std::string::npos);
  assert(calibrationData.valid);
  const uint32_t okDoneAt = fakeNow;

  // Distinct old diagnostics: a bad_sim abort must not copy the new vsim over
  // the previous a/b/v100 record.
  const float oldVsim[4] = {1111.0f, 2222.0f, 3333.0f, 4444.0f};
  for (int i = 0; i < 4; ++i) calibrationData.wheel[i].vsim = oldVsim[i];
  calibrationData.vmax =
      calibration::computeVmax(calibrationData.wheel, closedGains.limit);
  calSimRate[1] = 150;  // wheel B under-speeds only in the four-wheel sim stage
  const uint32_t badStart = okDoneAt + 1010;
  Serial.output.clear();
  command("CAL", badStart);
  assert(calibrating);
  assert(Serial.output.find("CAL START") != std::string::npos);
  assert(!calibrationData.valid);  // starting invalidates the old result
  while (calibrating && fakeNow < badStart + 30000) calTick(10);
  assert(!calibrating);
  assert(Serial.output.find("CAL FAIL bad_sim 1") != std::string::npos);
  assert(calRun.phase == CalPhase::Idle);
  stoppedAndClear();
  assert(!calibrationData.valid);
  for (int i = 0; i < 4; ++i)
    assert(calibrationData.wheel[i].vsim == oldVsim[i]);
  assert(calRun.vsim[0] == 4500.0f);
  assert(calRun.vsim[1] == 150.0f);
  assert(calRun.vsim[2] == 4500.0f);
  assert(calRun.vsim[3] == 4500.0f);
  {
    calibration::Data stored;
    wheel_speed::Gains storedGains;
    assert(config_store::load(stored, storedGains));
    assert(!stored.valid);
    for (int i = 0; i < 4; ++i)
      assert(std::fabs(stored.wheel[i].vsim - oldVsim[i]) < 0.01f);
  }
  calSimRate[1] = -1;
  const uint32_t badAbortAt = fakeNow;

  // 14h. CAL RESET is allowed immediately after a failure (no cooldown gate).
  //      Keep fake time monotonic: both checks happen before the later SPEED.
  Serial.output.clear();
  command("CAL RESET", badAbortAt + 10);
  assert(!calibrationData.valid);
  assert(Serial.output.find("CAL RESET") != std::string::npos);
  assert(Serial.output.find("wait 1 second") == std::string::npos);
  {
    calibration::Data stored;
    wheel_speed::Gains storedGains;
    assert(config_store::load(stored, storedGains));
    assert(!stored.valid);
    for (int i = 0; i < 4; ++i)
      assert(std::fabs(stored.wheel[i].vsim - oldVsim[i]) < 0.01f);
  }
  // 14i. A failed persistence of valid=0 is reported, not swallowed.
  fakePreferencesFailOnKey("valid", 1);
  Serial.output.clear();
  command("CAL RESET", badAbortAt + 20);
  assert(!calibrationData.valid);
  assert(Serial.output.find("ERROR NVS save failed") != std::string::npos);
  assert(Serial.output.find("CAL RESET") == std::string::npos);
  fakePreferencesClearFailures();

  // The failed recalibration still rejects SPEED once the stop cooldown ends.
  Serial.output.clear();
  command("SPEED 50 0 0 500", badAbortAt + 1000);
  assert(!running && !closedLoopActive);
  assert(Serial.output.find("calibration required") != std::string::npos);
  assert(fakeNow == badAbortAt + 1000);  // monotonic from here on

  // 14j. An invalid MODE command is an external stop. If it lands inside the
  //      Sim measurement window it must abort calibration (not just stop the
  //      wheels), otherwise the truncated window can pass min(vsim)>=200.
  const uint32_t bogusModeStart = fakeNow + 10;
  Serial.output.clear();
  command("CAL", bogusModeStart);
  assert(calibrating);
  while (calRun.phase != CalPhase::Sim && calibrating &&
         fakeNow < bogusModeStart + 20000)
    calTick(10);
  assert(calibrating && calRun.phase == CalPhase::Sim);
  calTick(1100);  // past the 1000 ms mark: inside the last-500 ms window
  assert(calibrating);
  assert(calRun.measureAt != 0);
  Serial.output.clear();
  command("MODE BOGUS", fakeNow);
  assert(!calibrating);
  assert(Serial.output.find("CAL FAIL stopped") != std::string::npos);
  assert(Serial.output.find("ERROR use MODE") != std::string::npos);
  assert(Serial.output.find("CAL DONE") == std::string::npos);
  assert(calRun.phase == CalPhase::Idle);
  stoppedAndClear();
  assert(!calibrationData.valid);
  {
    calibration::Data stored;
    wheel_speed::Gains storedGains;
    assert(config_store::load(stored, storedGains));
    assert(!stored.valid);
  }
  const uint32_t bogusModeAbortAt = fakeNow;

  // 14k. Malformed/oversized input is also an external stop; inside the Sim
  //      window it must abort calibration with valid=0.
  const uint32_t malformedStart = bogusModeAbortAt + 1010;
  Serial.output.clear();
  command("CAL", malformedStart);
  assert(calibrating);
  while (calRun.phase != CalPhase::Sim && calibrating &&
         fakeNow < malformedStart + 20000)
    calTick(10);
  assert(calibrating && calRun.phase == CalPhase::Sim);
  calTick(1100);
  assert(calibrating);
  assert(calRun.measureAt != 0);
  Serial.output.clear();
  replyPort = ControlPort::Usb;
  CommandLine malformedLine;
  receiveByte(malformedLine, 1, ControlPort::Usb);  // control byte -> invalid line
  assert(!calibrating);
  assert(Serial.output.find("CAL FAIL stopped") != std::string::npos);
  assert(Serial.output.find("ERROR invalid/long line; stopped") != std::string::npos);
  assert(Serial.output.find("CAL DONE") == std::string::npos);
  assert(calRun.phase == CalPhase::Idle);
  stoppedAndClear();
  assert(!calibrationData.valid);
  {
    calibration::Data stored;
    wheel_speed::Gains storedGains;
    assert(config_store::load(stored, storedGains));
    assert(!stored.valid);
  }

  // 15. Single-wheel faults (I4). A/C/D track the target while only B is broken:
  //     the per-wheel monitor must not be reset by the healthy wheels and must
  //     report the exact reason, stop, clear closedLoopActive and latch.
  uint32_t singleNow = fakeNow + 1010;
  installCalibration();
  faultLatched = false;
  controlOwner = ControlPort::Usb;
  replyPort = ControlPort::Usb;
  assert(runtimeMode == RuntimeMode::Chassis && layoutConfigured);

  // 15a. Wheel B counts backwards while A/C/D follow the ramped target.
  closedGains = wheel_speed::Gains{};
  Serial.output.clear();
  command("SPEEDW 50 50 50 50 2000", singleNow);
  assert(running && closedLoopActive && !faultLatched);
  int singleTicks = 0;
  for (; singleTicks < 80 && running; ++singleTicks) {
    for (int i = 0; i < 4; ++i) {
      int32_t step = static_cast<int32_t>(std::lround(rampedTarget[i] * 0.01f));
      if (i == 1) step = -step;  // B reversed only
      encoders[i].count += step;
    }
    tickUs(10000);
  }
  assert(!running && !closedLoopActive && faultLatched);
  assert(std::strcmp(lastFaultReason, "encoder_sign") == 0);
  assert(singleTicks >= 30 && singleTicks <= 60);
  stoppedAndClear();
  assertAllPwmZero();
  assert(Serial.output.find("FAULT encoder_sign") != std::string::npos);
  command("STOP", fakeNow);
  assert(!faultLatched);

  // 15b. Wheel B has zero feedback while A/C/D are healthy. A healthy wheel
  //      must not reset B's timer; no_feedback fires only after the 800 ms hold.
  singleNow = fakeNow + 1010;
  installCalibration();
  closedGains = wheel_speed::Gains{};
  Serial.output.clear();
  command("SPEEDW 50 50 50 50 2000", singleNow);
  assert(running && closedLoopActive && !faultLatched);
  singleTicks = 0;
  for (; singleTicks < 160 && running; ++singleTicks) {
    for (int i = 0; i < 4; ++i) {
      if (i == 1) continue;  // B stays at zero counts
      encoders[i].count +=
          static_cast<int32_t>(std::lround(rampedTarget[i] * 0.01f));
    }
    tickUs(10000);
    if (singleTicks == 50) assert(running && !faultLatched);
  }
  assert(!running && !closedLoopActive && faultLatched);
  assert(std::strcmp(lastFaultReason, "no_feedback") == 0);
  assert(singleTicks >= 75 && singleTicks <= 130);
  stoppedAndClear();
  assertAllPwmZero();
  assert(Serial.output.find("FAULT no_feedback") != std::string::npos);
  command("STOP", fakeNow);

  // 15c. Negative target: B measures positive (reverse sign) -> encoder_sign.
  singleNow = fakeNow + 1010;
  installCalibration();
  closedGains = wheel_speed::Gains{};
  Serial.output.clear();
  command("SPEEDW -50 -50 -50 -50 2000", singleNow);
  assert(running && closedLoopActive && !faultLatched);
  singleTicks = 0;
  for (; singleTicks < 80 && running; ++singleTicks) {
    for (int i = 0; i < 4; ++i) {
      int32_t step = static_cast<int32_t>(std::lround(rampedTarget[i] * 0.01f));
      if (i == 1) step = -step;  // B counts opposite to the negative target
      encoders[i].count += step;
    }
    tickUs(10000);
  }
  assert(!running && !closedLoopActive && faultLatched);
  assert(std::strcmp(lastFaultReason, "encoder_sign") == 0);
  stoppedAndClear();
  assertAllPwmZero();
  command("STOP", fakeNow);

  // 16. Target-relative no_feedback at realistic calibration (C1): 5% and 10%
  //     targets with zero encoder counts must fault within ~1–2 s, while a
  //     healthy 5% run (measured ≈ target) must not fault.
  uint32_t lowNow = fakeNow + 1010;
  installRealisticCalibration();
  closedGains = wheel_speed::Gains{};
  assert(std::fabs(calibrationData.vmax - 4000.0f) < 0.01f);
  Serial.output.clear();
  command("SPEED 5 0 0 4000", lowNow);
  assert(running && closedLoopActive);
  int lowTicks = 0;
  for (; lowTicks < 260 && running; ++lowTicks) tickUs(10000);
  assert(!running && !closedLoopActive && faultLatched);
  assert(std::strcmp(lastFaultReason, "no_feedback") == 0);
  assert(lowTicks >= 80 && lowTicks <= 250);
  stoppedAndClear();
  command("STOP", fakeNow);

  lowNow = fakeNow + 1010;
  installRealisticCalibration();
  closedGains = wheel_speed::Gains{};
  Serial.output.clear();
  command("SPEED 10 0 0 4000", lowNow);
  assert(running && closedLoopActive);
  lowTicks = 0;
  for (; lowTicks < 200 && running; ++lowTicks) tickUs(10000);
  assert(!running && !closedLoopActive && faultLatched);
  assert(std::strcmp(lastFaultReason, "no_feedback") == 0);
  assert(lowTicks >= 60 && lowTicks <= 150);
  stoppedAndClear();
  command("STOP", fakeNow);

  lowNow = fakeNow + 1010;
  installRealisticCalibration();
  closedGains = wheel_speed::Gains{};
  Serial.output.clear();
  command("SPEED 5 0 0 4000", lowNow);
  assert(running && closedLoopActive);
  for (int t = 0; t < 250 && running; ++t) {
    for (int i = 0; i < 4; ++i)
      encoders[i].count +=
          static_cast<int32_t>(std::lround(rampedTarget[i] * 0.01f));
    tickUs(10000);
  }
  assert(running && closedLoopActive && !faultLatched);
  command("STOP", fakeNow);
  stoppedAndClear();

  // 17. NVS write ordering (SHOULD FIX): a successful calibration stops the
  //     outputs before saving, and no NVS write happens while any PWM is driven.
  fakePreferencesReset();
  calClearRates();
  uint32_t saveNow = fakeNow + 1010;
  fake_preferences::writesWhileDriven = 0;
  fake_preferences::drivenHook = []() {
    for (int i = 0; i < 8; ++i) {
      if (pwmValues[i] != 0) return true;
    }
    return false;
  };
  Serial.output.clear();
  command("CAL", saveNow);
  assert(calibrating);
  while (calibrating && fakeNow < saveNow + 30000) calTick(10);
  fake_preferences::drivenHook = nullptr;
  assert(!calibrating);
  assert(Serial.output.find("CAL DONE VALID=1") != std::string::npos);
  assert(calibrationData.valid);
  assert(fake_preferences::writesWhileDriven == 0);
  stoppedAndClear();
  assertAllPwmZero();

  // A failed final save must not emit CAL DONE and must leave valid=0 in
  // RAM and NVS; the rollback save also happens only after the stop.
  uint32_t failSaveNow = fakeNow + 1010;
  fakePreferencesReset();
  // saveCalibration writes valid twice (initial 0 + final), so CAL start uses
  // attempts 1-2; the successful calibration's final valid=1 is attempt 4.
  fakePreferencesFailOnKey("valid", 4);
  Serial.output.clear();
  command("CAL", failSaveNow);
  assert(calibrating);
  while (calibrating && fakeNow < failSaveNow + 30000) calTick(10);
  assert(!calibrating);
  assert(!calibrationData.valid);
  assert(Serial.output.find("CAL FAIL nvs") != std::string::npos);
  assert(Serial.output.find("CAL DONE") == std::string::npos);
  stoppedAndClear();
  assertAllPwmZero();
  fakePreferencesClearFailures();
  {
    calibration::Data stored;
    wheel_speed::Gains storedGains;
    assert(config_store::load(stored, storedGains));
    assert(!stored.valid);
  }

  // A pre-existing valid=1 record plus a failed first valid=0 write at CAL
  // start: the current boot stays invalid (RAM), but Flash can retain the old
  // valid record -- the documented "NVS completely unwritable" residual.
  uint32_t staleNow = fakeNow + 1010;
  calibration::Data good = calibrationData;
  good.valid = true;
  good.vmax = calibration::computeVmax(good.wheel, closedGains.limit);
  fakePreferencesReset();
  assert(config_store::saveCalibration(good));
  fakePreferencesFailOnKey("valid", 1);
  Serial.output.clear();
  command("CAL", staleNow);
  assert(!calibrating);
  assert(Serial.output.find("CAL FAIL nvs") != std::string::npos);
  assert(!calibrationData.valid);
  stoppedAndClear();
  {
    calibration::Data stored;
    wheel_speed::Gains storedGains;
    assert(config_store::load(stored, storedGains));
    assert(stored.valid);  // residual: old record survives a failed invalidation
  }
  fakePreferencesClearFailures();
  calibrationData.valid = false;
  config_store::saveCalibration(calibrationData);  // tidy the fake NVS

  std::cout << "PASS: SPEED/SPEEDW parsing/gating, 10 ms tick, target ramp, encoder sign, estimator/PI, duty to PWM, STOP/cooldown/busy, jitter reset, fault detect/latch/clear\n";
  std::cout << "PASS: PID set/get/range/running, Vmax recompute, TELEM 20 Hz/format/off, CAPS2, SAVE NVS roundtrip, HELP\n";
  std::cout << "PASS: PID/PID?/TELEM/SAVE are Chassis-only and restore after MODE CHASSIS\n";
  std::cout << "PASS: CAL success state machine, CAL?/CALW, NVS roundtrip, CAL RESET, Chassis-only, calibration-stop cooldown\n";
  std::cout << "PASS: CAL failure/interruption matrix, motion/ENCZERO/CAL gating, CAL RESET without cooldown and NVS failure, bad_sim preserves old vsim\n";
  std::cout << "PASS: invalid-MODE and malformed-line stops abort calibration inside the Sim window with valid=0\n";
  std::cout << "PASS: single-wheel B reverse/zero/negative faults, target-relative no_feedback at 5%/10%, healthy 5% no false fault\n";
  std::cout << "PASS: NVS saved only after stop, failed CAL save invalidates, stale valid record on failed CAL-start invalidation documented\n";
}
