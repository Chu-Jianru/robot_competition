# 麦轮轮速闭环（WHEEL_SPEED_V1）实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 给 ESP32 麦轮底盘增加基于编码器的四轮轮速闭环、悬空标定、串口调参/遥测和调试面板支持，同时保留现有开环行为。

**Architecture:** 在现有 `mecanum::mix()` 与 `setMotorForwardPercent()` 之间插入 10 ms 控制 tick。纯控制数学放进 `wheel_speed.h`/`calibration.h`，NVS 存取放进 `config_store.h`，`main.cpp` 只做集成、状态机和协议。面板通过新增 `SPEED`/`SPEEDW`/`PID`/`CAL`/`TELEM`/`SAVE` 命令使用闭环。

**Tech Stack:** ESP32-WROOM-32 / Arduino-ESP32 2.0.17 / PlatformIO `espressif32@7.1.3`；C++17 主机测试使用假 `Arduino.h`/`Preferences.h`；Python 3 + Tkinter 面板。

**Spec:** `docs/superpowers/specs/2026-10-31-mecanum-wheel-speed-closed-loop-design.md`

## Global Constraints

- 控制周期 10 ms；测速窗口 40 ms（4 个 tick）；转速单位 counts/s。
- 默认参数：`KP=2.0`、`KI=1.0`、`LIMIT=80`、`AMAX=300`（%/s）；闭环非零轮目标下限 5% Vmax。
- 协议参数全部用严格整数解析；`PID` 用千分位整数（`kp_milli`/`ki_milli`），禁止小数和多余参数。
- 目标速度 `Vmax = LIMIT/100 × min(vsim_i)`；`vsim_i` 是四轮同时 100% 的稳态速度。
- 故障闭锁：故障后 `SPEED`/`SPEEDW`/`CAL` 拒绝，必须显式 `STOP` 清除。
- `CAL` 开始即写 `valid=0`；只有整次成功才写 `valid=1`。
- 保留 `MECANUM`/`WHEELS` 开环语义和现有全部测试；不得削弱 STOP、1.5 s 超时、冷却、布局确认。
- 不改 GPIO、不改底层电气极性、不引入新库。
- NVS 命名空间 `chassis_cl`，结构版本 `1`。
- 遥测 20 Hz，格式 `TLM <ms> <rA> <rB> <rC> <rD> <sA> <sB> <sC> <sD> <uA> <uB> <uC> <uD>`。
- 工作区不是 git 仓库：不要执行 `git init`/`git commit`；每个任务结束时运行对应测试并记录结果。
- 中文注释和文档，串口协议保持 ASCII。

## Review Focus

- 控制 tick 被串口/蓝牙输出拖延超过 50 ms：必须清积分和测速窗口，不能把过期误差继续积分；Task 3/6 的测试覆盖。
- 标定被 STOP、断链、模式切换或控制权撤销打断：必须立即关输出、保持 `valid=0`；Task 10 覆盖。
- NVS 版本不匹配、字段损坏或未标定：闭环必须保持禁用，不能拿默认值运行；Task 5/9 覆盖。
- 遥测关闭、断开或非控制端口：不能继续发送 TLM，也不能把 TLM 写进面板日志；Task 8/12 覆盖。
- 麦轮混控后的边界：某轮目标为 0 是合法的，`0 < |p_i| < 5` 必须拒绝，全零等同 STOP；Task 6 覆盖。

---

### Task 1: 测速估计器 `SpeedEstimator`

**Files:**
- Create: `motor-bringup/src/wheel_speed.h`
- Create: `motor-bringup/tests/test_wheel_speed.cpp`

**Interfaces:**
- Produces:
  - `namespace wheel_speed { class SpeedEstimator { public: void reset(); float update(int32_t deltaCounts, uint32_t dtUs); float speed() const; }; }`
  - 常量：窗口 4 个样本；最小窗口 10000 µs；速度 `1e6 × sum(delta)/sum(dtUs)`。

- [ ] **Step 1: 写失败测试**

`tests/test_wheel_speed.cpp`：

```cpp
#include <cassert>
#include <cmath>
#include "../src/wheel_speed.h"

static bool near(float a, float b) { return std::fabs(a-b) < 0.5f; }

int main() {
  wheel_speed::SpeedEstimator e;
  e.reset();
  assert(near(e.speed(), 0.0f));

  float v = 0;
  for (int i = 0; i < 4; ++i) v = e.update(550, 10000);
  assert(near(v, 55000.0f));

  e.reset();
  v = e.update(400, 8000);       // 单样本不足 10 ms
  assert(near(v, 0.0f));         // 仍沿用上次（0）
  v = e.update(200, 4000);       // 合计 12000 µs
  assert(near(v, 50000.0f));     // 600 / 0.012

  e.reset();
  e.update(100, 10000);
  e.update(100, 10000);
  e.update(100, 10000);
  e.update(100, 10000);
  assert(near(e.speed(), 10000.0f));
  e.update(0, 0);                // 零 dt 不应改变速度
  assert(near(e.speed(), 10000.0f));

  e.reset();
  assert(near(e.speed(), 0.0f));
  return 0;
}
```

- [ ] **Step 2: 运行确认失败**

```powershell
cd D:\Users\1713810750\推物块机器人\motor-bringup
g++ -std=c++17 -I tests tests/test_wheel_speed.cpp -o tests/test_wheel_speed.exe
.\tests\test_wheel_speed.exe
```

Expected: 编译失败，`wheel_speed.h` 不存在或 `SpeedEstimator` 未定义。

- [ ] **Step 3: 实现最小 `SpeedEstimator`**

在 `src/wheel_speed.h` 中实现：4 项 `delta`/`dt` 环形缓冲、`reset()` 清零、`update()` 在 `sumDt >= 10000` 时更新速度，否则保持上次值；`dtUs == 0` 直接返回当前速度。

- [ ] **Step 4: 运行确认通过**

重复 Step 2。Expected: 退出码 0，无断言失败。

- [ ] **Step 5: 记录结果，不提交 git**

---

### Task 2: 标定数据与两点前馈线

**Files:**
- Create: `motor-bringup/src/calibration.h`
- Create: `motor-bringup/tests/test_calibration.cpp`

**Interfaces:**
- Produces:
  - `namespace calibration { struct Wheel { int8_t sign; float a; float b; float v100; float vsim; }; }`
  - `struct Data { uint32_t version; bool valid; Wheel wheel[4]; float vmax; };`
  - `bool computeLine(float d1, float v60, float v100, Wheel& wheel);`
  - `float computeVmax(const Wheel wheel[4], float limitPercent);`

- [ ] **Step 1: 写失败测试**

`tests/test_calibration.cpp`：

```cpp
#include <cassert>
#include <cmath>
#include "../src/calibration.h"

static bool near(float a, float b, float eps=0.01f) { return std::fabs(a-b) < eps; }

int main() {
  calibration::Wheel w;
  w.sign = 1;
  assert(calibration::computeLine(60, 2000, 5000, w));
  assert(near(w.b, 40.0f/3000.0f));
  assert(near(w.a, 60.0f - (40.0f/3000.0f)*2000.0f));
  assert(near(w.v100, 5000.0f));
  assert(w.sign == 1);

  calibration::Wheel bad;
  assert(!calibration::computeLine(60, 5000, 5000, bad)); // v100<=v60
  assert(!calibration::computeLine(60, 0, 100, bad));     // v100<200
  assert(!calibration::computeLine(60, 4000, 5000, bad)); // a<0
  assert(!calibration::computeLine(99, 1, 2000, bad));    // d1 非法

  calibration::Wheel wheels[4] = {};
  for (int i=0;i<4;++i) wheels[i].vsim = 4000 + i*500;
  assert(near(calibration::computeVmax(wheels, 80), 3200.0f));
  return 0;
}
```

- [ ] **Step 2: 运行确认失败**（编译失败，`calibration.h` 不存在）

- [ ] **Step 3: 实现 `calibration.h`**

`computeLine`：校验 `d1∈(0,100)`、`v100>v60`、`v100≥200`；计算 `b=(100-d1)/(v100-v60)`、`a=d1-b*v60`；若 `b<=0`、`a<0` 或 `a>=90` 返回 false；成功时写入 `wheel.a/b/v100` 并保留 `sign`。
`computeVmax`：返回 `limitPercent/100 × min(vsim_i)`。

- [ ] **Step 4: 运行确认通过**

- [ ] **Step 5: 记录结果，不提交 git**

---

### Task 3: 前馈 + PI + 抗积分饱和

**Files:**
- Modify: `motor-bringup/src/wheel_speed.h`
- Modify: `motor-bringup/tests/test_wheel_speed.cpp`

**Interfaces:**
- Consumes: `calibration::Wheel`。
- Produces:
  - `struct Gains { float kp; float ki; float limit; float amax; };`（默认 2.0/1.0/80/300）
  - `class WheelPid { public: void reset(); float step(float target, float measured, float vmax, const calibration::Wheel& wheel, const Gains& gains, float dt); float integral() const; };`

- [ ] **Step 1: 追加失败测试**

在 `test_wheel_speed.cpp` 增加：

```cpp
static calibration::Wheel wheel() {
  calibration::Wheel w; w.sign=1; w.a=33.333f; w.b=0.013333f; w.v100=5000; w.vsim=5000; return w;
}
static bool near2(float a, float b, float eps=0.02f) { return std::fabs(a-b) < eps; }

static void test_pid() {
  wheel_speed::Gains g; g.kp=2; g.ki=1; g.limit=80; g.amax=300;
  wheel_speed::WheelPid p; p.reset();
  // 零误差：只剩前馈
  assert(near2(p.step(2000, 2000, 5000, wheel(), g, 0.01f), 60.0f));
  // 误差 4%：+8% 占空比（用 ki=0 隔离纯 P，避免积分项 0.04 干扰断言）
  wheel_speed::Gains gp; gp.kp=2; gp.ki=0; gp.limit=80; gp.amax=300;
  p.reset();
  assert(near2(p.step(2000, 1800, 5000, wheel(), gp, 0.01f), 68.0f));
  // 正向饱和
  p.reset();
  assert(near2(p.step(5000, 0, 5000, wheel(), g, 0.01f), 100.0f));
  for (int i=0;i<100;++i) p.step(5000, 0, 5000, wheel(), g, 0.01f);
  assert(p.integral()*g.ki <= 30.01f);
  // 目标归零/重置
  p.reset();
  assert(near2(p.step(0, 0, 5000, wheel(), g, 0.01f), 0.0f));
  // 反向
  p.reset();
  assert(p.step(-2000, -2000, 5000, wheel(), g, 0.01f) < 0.0f);
}
```

在 `main()` 调用 `test_pid();`。

- [ ] **Step 2: 运行确认失败**（`Gains`/`WheelPid` 未定义）

- [ ] **Step 3: 实现 PI**

`step()`：
1. `uff = sign(target) × clamp(a + b×|target|, 0, 100)`。
2. `en = 100×(target-measured)/vmax`。
3. `integral += en×dt`，积分项 `ki×integral` 限幅 ±30。
4. `raw = uff + kp×en + ki×integral`。
5. 若 `raw` 饱和且 `en` 与饱和同号，回退本次积分增量；输出 clamp 到 ±100。
6. `target==0` 时清零积分并返回 0；目标符号变化时清零积分。

- [ ] **Step 4: 运行确认通过**

- [ ] **Step 5: 记录结果，不提交 git**

---

### Task 4: 故障监视器

**Files:**
- Modify: `motor-bringup/src/wheel_speed.h`
- Modify: `motor-bringup/tests/test_wheel_speed.cpp`

**Interfaces:**
- Produces:
  - `enum class Fault { None, EncoderSign, NoFeedback, Stall };`
  - `class FaultMonitor { public: void reset(); Fault update(float target, float measured, float duty, float vmax, float dt); };`
  - `const char* faultName(Fault fault);`

- [ ] **Step 1: 追加失败测试**

```cpp
static wheel_speed::Fault run_fault(wheel_speed::FaultMonitor& m, float target, float measured,
                                    float duty, float vmax, float seconds) {
  wheel_speed::Fault f = wheel_speed::Fault::None;
  for (float t=0; t<seconds; t+=0.01f) f = m.update(target, measured, duty, vmax, 0.01f);
  return f;
}

static void test_faults() {
  wheel_speed::FaultMonitor m;
  m.reset();
  assert(run_fault(m, 2000, 2000, 60, 5000, 2.0f) == wheel_speed::Fault::None);

  m.reset();
  assert(run_fault(m, 2000, -500, 60, 5000, 0.29f) == wheel_speed::Fault::None);
  assert(run_fault(m, 2000, -500, 60, 5000, 0.02f) == wheel_speed::Fault::EncoderSign);

  m.reset();  // 低速命令也能被占空比判据覆盖
  assert(run_fault(m, 500, 0, 90, 5000, 0.79f) == wheel_speed::Fault::None);
  assert(run_fault(m, 500, 0, 90, 5000, 0.02f) == wheel_speed::Fault::NoFeedback);

  m.reset();
  assert(run_fault(m, 500, 0, 40, 5000, 2.0f) == wheel_speed::Fault::None);

  m.reset();
  assert(run_fault(m, 4000, 500, 99, 5000, 0.99f) == wheel_speed::Fault::None);
  assert(run_fault(m, 4000, 500, 99, 5000, 0.02f) == wheel_speed::Fault::Stall);
}
```

在 `main()` 调用 `test_faults();`，并断言 `faultName(Fault::NoFeedback)` 包含 `"no_feedback"`。

- [ ] **Step 2: 运行确认失败**

- [ ] **Step 3: 实现判据**

- `encoder_sign`：`|duty|≥50`、`duty×measured<0`、`|measured|≥0.03×vmax`，持续 300 ms。
- `no_feedback`：`|duty|≥85`、`|measured|<0.02×vmax`，持续 800 ms。
- `stall`：`|target|≥0.20×vmax`、`|duty|≥98`、`|measured|<0.20×|target|`，持续 1000 ms。
- 条件不满足时对应计时清零；`reset()` 清全部计时。

- [ ] **Step 4: 运行确认通过**

- [ ] **Step 5: 记录结果，不提交 git**

---

### Task 5: NVS 存储与测试假件

**Files:**
- Create: `motor-bringup/src/config_store.h`
- Create: `motor-bringup/tests/Preferences.h`
- Modify: `motor-bringup/tests/Arduino.h`
- Create: `motor-bringup/tests/test_config_store.cpp`

**Interfaces:**
- Produces:
  - `namespace config_store { bool load(calibration::Data& cal, wheel_speed::Gains& gains); bool saveCalibration(const calibration::Data& cal); bool saveGains(const wheel_speed::Gains& gains); void clearCalibrationValidity(); }`
  - NVS 命名空间 `"chassis_cl"`，版本键 `ver=1`；键：`valid`、`s0..s3`、`a0..a3`、`b0..b3`、`v100_0..v100_3`、`vsim0..vsim3`、`vmax`、`kp`、`ki`、`limit`、`amax`。

- [ ] **Step 1: 写失败测试**

`tests/test_config_store.cpp`：

```cpp
#include <cassert>
#include <cmath>
#include "Arduino.h"
#include "Preferences.h"
#include "../src/config_store.h"

int main() {
  fakePreferencesReset();
  calibration::Data cal;
  cal.valid = true;
  cal.vmax = 3200;  // 80% × min(vsim=4000)
  for (int i=0;i<4;++i) {
    cal.wheel[i].sign = (i%2) ? -1 : 1;
    cal.wheel[i].a = 30+i; cal.wheel[i].b = 0.01f*(i+1);
    cal.wheel[i].v100 = 5000+i*100; cal.wheel[i].vsim = 4000+i*100;
  }
  wheel_speed::Gains g; g.kp=2.5f; g.ki=1.2f; g.limit=80; g.amax=250;
  assert(config_store::saveCalibration(cal));
  assert(config_store::saveGains(g));

  calibration::Data loaded;
  wheel_speed::Gains lg;
  assert(config_store::load(loaded, lg));
  assert(loaded.valid);
  assert(std::fabs(loaded.vmax-3200) < 0.01f);
  assert(loaded.wheel[1].sign == -1);
  assert(std::fabs(lg.kp-2.5f) < 0.001f);

  // 改 LIMIT 后重新加载：Vmax 必须由 vsim 和当前 limit 重新计算
  g.limit = 60;
  assert(config_store::saveGains(g));
  assert(config_store::load(loaded, lg));
  assert(std::fabs(loaded.vmax-2400) < 0.01f);  // 60% × 4000

  config_store::clearCalibrationValidity();
  assert(config_store::load(loaded, lg));
  assert(!loaded.valid);

  // 版本不匹配：闭环必须禁用
  fakePreferencesPutFloat("chassis_cl", "ver", 2);
  calibration::Data wrong;
  wheel_speed::Gains wg;
  assert(!config_store::load(wrong, wg));
  assert(!wrong.valid);
  return 0;
}
```

- [ ] **Step 2: 运行确认失败**

- [ ] **Step 3: 实现假 `Preferences` 与 `config_store.h`**

`tests/Preferences.h`：用静态 `std::map<std::string, std::map<std::string,double>>` 保存命名空间键值，实现 `begin/end/putUChar/getUChar/putChar/getChar/putFloat/getFloat/putUInt/getUInt/clear`；提供 `fakePreferencesReset()` 和 `fakePreferencesPutFloat(ns,key,value)`。
`tests/Arduino.h` 增加 `inline uint32_t fakeMicros = 0; inline uint32_t micros() { return fakeMicros; }`。
`config_store.h`：按上面的键读写；`load` 先读 `ver`，不等于 1 返回 false；`valid` 为 0 时 `cal.valid=false`；gains 缺省用默认值；加载后必须用 `calibration::computeVmax(cal.wheel, gains.limit)` 重新计算 `cal.vmax`（`vsim` 存的是原始四轮同时速度），不能只信 NVS 里的 `vmax` 字段。

- [ ] **Step 4: 运行确认通过**

- [ ] **Step 5: 记录结果，不提交 git**

---

### Task 6: 固件集成一：控制 tick + `SPEED`/`SPEEDW`

**Files:**
- Modify: `motor-bringup/src/main.cpp`
- Create: `motor-bringup/tests/test_closed_loop.cpp`

**Interfaces:**
- Produces（`main.cpp` 全局，测试可直接访问）：
  - `calibration::Data calibrationData;`
  - `wheel_speed::Gains closedGains;`
  - `wheel_speed::SpeedEstimator speedEstimators[4];`
  - `wheel_speed::WheelPid wheelPids[4];`
  - `bool closedLoopActive; bool calibrating; float goalTarget[4]; float rampedTarget[4]; float measuredSpeed[4]; float appliedDuty[4]; int32_t lastEncoderCount[4]; uint32_t lastControlUs;`
  - `void snapshotEncoders(int32_t out[4]);`
  - `void resetClosedLoop();`
  - `void serviceControl(uint32_t nowUs);`
  - `bool startClosedLoop(const int32_t percent[4], uint32_t durationMs, uint32_t now);`
- 命令：`SPEED <x> <y> <z> <ms>`、`SPEEDW <a> <b> <c> <d> <ms>`；响应 `SPEED_RUN A=.. B=.. C=.. D=.. duration=..`。

- [ ] **Step 1: 写失败测试**

`tests/test_closed_loop.cpp` 骨架：

```cpp
#include <cassert>
#include <cmath>
#include <iostream>
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
```

测试用例至少覆盖（每个用例开头先 `closedGains = wheel_speed::Gains{};`，避免前一个用例改过参数）：

1. `setup(); command("MODE CHASSIS",0); command("LAYOUT CBAD 1 1 1 1",1); installCalibration();`
2. `command("SPEED 100 0 0 1000",1000);` 断言 `running`、`activeChannel==-2`、`levels[kStandby]==HIGH`、四轮 `goalTarget` 都是 4000、`rampedTarget` 初始为 0、输出含 `SPEED_RUN`。
3. 连续 `tickUs(10000)`，`rampedTarget` 每 tick 增加约 120（`AMAX/100×4000×0.01`），34 tick 后到 4000。
4. 每个 tick 按当前 `rampedTarget` 增加编码器计数：`encoders[i].count += lround(rampedTarget[i]*0.01)`；窗口填满后 `measuredSpeed[i]` 接近目标，`appliedDuty[i]` 接近前馈 `40+0.01×4000=80`，逻辑 80% 对应 `pwm2=204`。
5. `serviceDeadline(2000)` 停车，PWM 全 0。
6. `SPEEDW 50 0 50 0 500` 的 `goalTarget` 为 `[2000,0,2000,0]`。
7. `SPEED 4 0 0 500` 被拒绝（最小 5%）；`SPEED 0 0 0 500` 等同 STOP；未标定、未布局、运行中、冷却中都拒绝。
8. `fakeMicros += 60000; serviceControl(...)` 后积分和测速窗口清零、`measuredSpeed/appliedDuty` 清零；下一次 tick 正常。子 10 ms 调用不触发控制，累计到 10 ms 才运行。
9. 编码器符号：把一路 `wheel.sign=-1`，原始计数反向增长，断言该路 `measuredSpeed≈+4000`、不产生反向驱动。
10. 严格解析与边界：`SPEED 1.5 0 0 100`、`SPEED 101 0 0 500`、`SPEED 0 0 0 0 extra`、`SPEEDW 4 0 0 0 500` 均拒绝；`SPEED_RUN` 的 A/B/C/D 目标值可断言。

- [ ] **Step 2: 运行确认失败**（`serviceControl` 等未定义）

- [ ] **Step 3: 集成到 `main.cpp`**

- `snapshotEncoders`：在 `encoderMux` 临界区内一次读四路计数。
- `resetClosedLoop`：清 `wheelPids`、`speedEstimators`、`goalTarget`、`rampedTarget`、`measuredSpeed`、`appliedDuty`，`closedLoopActive=false`。
- `stopMotor` 开头调用 `resetClosedLoop()`（不碰故障闭锁）。
- `serviceControl`：
  - `!closedLoopActive` 或 `calibrating` 时只更新 `lastControlUs` 返回。
  - `dtUs=nowUs-lastControlUs`；`<10000` 返回（控制周期 10 ms，保证 4 样本窗口=40 ms）；`>50000` 清估计器/PID/窗口、清零 `measuredSpeed`/`appliedDuty`、快照计数，返回。
  - 快照计数；对每轮 `delta=(count-last)×calibrationData.wheel[i].sign`（`lastCount` 存原始计数）、`estimator.update`、目标斜坡（`amax/100×vmax×dt`）、`pid.step`、`lroundf` 后 `setMotorForwardPercent`。
- `startClosedLoop`：先校验 `duration≥1`、`!(calibrationData.valid && calibrationData.vmax>0)`（NaN 安全写法）、每个非零百分比 `≥5`、至少一路非零；**这些校验必须在 `stopMotor()` 之前完成**，避免低目标命令无谓停车。然后写 `goalTarget`、重置、快照、`lastControlUs=micros()`、设置 `running/activeChannel=-2/startedAt/lastContactAt/durationMs`、`STBY HIGH`、回复。
- `handleChassis` 在 `MECANUM`/`WHEELS` 前识别 `SPEED ` 和 `SPEEDW `，复用 `parseNumbers`；`SPEED` 需要已确认布局，`SPEEDW` 不需要混控但仍需标定。
- `HELP` 增加 `SPEED`/`SPEEDW` 说明；`CHASSIS_CAPS2` 能力行放到 Task 8。
- `loop()` 和 64 字节预算循环内调用 `serviceControl(micros())`。

- [ ] **Step 4: 运行确认通过**

```powershell
g++ -std=c++17 -I tests tests/test_closed_loop.cpp src/dry_core.cpp -o tests/test_closed_loop.exe
.\tests\test_closed_loop.exe
```

- [ ] **Step 5: 回归现有 C++ 测试**

```powershell
g++ -std=c++17 -I tests tests/test_motor.cpp src/dry_core.cpp -o tests/test_motor.exe; .\tests\test_motor.exe
g++ -std=c++17 -I tests tests/test_chassis.cpp src/dry_core.cpp -o tests/test_chassis.exe; .\tests\test_chassis.exe
```

Expected: 全部退出码 0。`main.cpp` 实例化 `drylink::LinkCore`，所有包含 `main.cpp` 的测试都必须同时链接 `src/dry_core.cpp`。

---

### Task 7: 固件集成二：故障检测与闭锁

**Files:**
- Modify: `motor-bringup/src/main.cpp`
- Modify: `motor-bringup/tests/test_closed_loop.cpp`

**Interfaces:**
- Produces（`main.cpp` 全局）：
  - `wheel_speed::FaultMonitor faultMonitor[4];`（每轮独立计时器；单个实例会把四轮计时混在一起，导致 hold 加速 4 倍且无法检测单轮故障）
  - `bool faultLatched; const char* lastFaultReason;`
  - `void clearFault();`
- `STATUS` 追加 `FAULT last=<name>` 或 `FAULT none`。

- [ ] **Step 1: 追加失败测试**

1. 安装标定，`SPEED 50 0 0 1000`，让编码器计数反向增长，`tickUs` 直到 300 ms；断言 `!running`、`pwm` 全 0、输出含 `FAULT encoder_sign`。
2. 故障后 `SPEED 50 0 0 1000` 被拒绝，输出含 `fault latched`；`STOP` 后再发可运行。
3. 低速 `SPEED 10 0 0 4000`，编码器保持 0，`tickUs` 3 s；断言 `FAULT no_feedback`（PI 需要把占空比推到 85% 以上，约 2.1 s，所以给 3 s）。
4. `SPEEDW 100 100 100 100 2000`，编码器给 500 counts/s（约 12.5% 目标：高于 `no_feedback` 的 2% Vmax=80 阈值，低于 `stall` 的 20% 目标=800 阈值），断言 `FAULT stall`。
5. `STATUS` 输出含 `FAULT last=`。

- [ ] **Step 2: 运行确认失败**

- [ ] **Step 3: 集成**

- `serviceControl` 每轮调用 `faultMonitor[i].update(rampedTarget[i], measuredSpeed[i], appliedDuty[i], vmax, dt)`；任一非 `None` 时记录 `lastFaultReason`、`stopMotor`、`closedLoopActive=false`、`faultLatched=true`、回复 `FAULT <name>`。
- `clearFault()`：`faultLatched=false; lastFaultReason="none";` 并对四路 `faultMonitor[i].reset()`。
- `resetClosedLoop()`（Task 6）中对四路 `faultMonitor[i].reset()`，确保故障计时器不跨运行泄漏。
- `handleCommand("STOP")`、`selectMode`、`revokeControl` 调用 `clearFault()`。
- `startClosedLoop` 在 `faultLatched` 时返回错误 `ERROR fault latched; send STOP`。
- `STATUS` 追加故障行。

- [ ] **Step 4: 运行确认通过**（`test_closed_loop.exe` + 现有回归）

- [ ] **Step 5: 记录结果，不提交 git**

---

### Task 8: 固件集成三：`PID`、`TELEM`、`SAVE`、`CAPS2`

**Files:**
- Modify: `motor-bringup/src/main.cpp`
- Modify: `motor-bringup/tests/test_closed_loop.cpp`

**Interfaces:**
- Produces（`main.cpp` 全局）：`bool telemEnabled; uint32_t lastTelemMs; void maybeSendTelemetry(uint32_t nowMs);`
- Commands:
  - `PID <kp_milli> <ki_milli> <limit>`；`kp/ki ∈ 0..10000`，`limit ∈ 10..100`；仅停止时允许。
  - `PID?` → `PID_OK KP=%.3f KI=%.3f LIMIT=%d`。
  - `TELEM 0|1` → `TELEM ON|OFF`。
  - `SAVE` → `SAVED`（保存标定 + gains）。
- Telemetry: 20 Hz；内容使用 `rampedTarget/measuredSpeed/appliedDuty`。

- [ ] **Step 1: 追加失败测试**

1. `PID 2500 1200 75` → `PID_OK KP=2.500 KI=1.200 LIMIT=75`；`PID?` 回读一致；`PID 10001 ...`、`PID ... 9`、运行中发送均被拒绝。
2. 安装有效标定（`vsim=4000..`，默认 limit 80 → `Vmax=3200`）；`PID 2500 1200 60` 后 `calibrationData.vmax≈2400`；再 `PID 2500 1200 80` 后回到 3200。
3. 测试中先设 `controlOwner = ControlPort::Usb;`。`TELEM 1`；`tickUs(10000)` 两次（共 20 ms）不出现 TLM；到 50 ms 出现一行 `TLM `，字段数为 13；再 `tickUs(10000)` 不重复；`TELEM 0` 后 100 ms 无 TLM。
4. `CAPS` 含 `CHASSIS_CAPS2 API=WHEEL_SPEED_V1 CLOSED=1 CAL=1 TELEM=1 PID=MILLI`。
5. `SAVE` 后 `config_store::load` 读到的 gains 与设置一致。
6. `HELP` 含 `SPEED`、`PID`、`CAL`、`TELEM`。

- [ ] **Step 2: 运行确认失败**

- [ ] **Step 3: 实现**

- 在 `handleCommand` 的 `MODE`/`HELP` 附近加入 `PID`/`PID?`/`TELEM`/`SAVE` 分支。
- `PID`/`PID?`/`TELEM`/`SAVE` 仅在 `runtimeMode == RuntimeMode::Chassis` 时接受；其他模式回复 `ERROR requires MODE CHASSIS`。`maybeSendTelemetry` 也要求 Chassis（设计 §5.9）。
- `PID` 设置 `closedGains.limit` 后，如果 `calibrationData.valid`，立即用 `calibration::computeVmax(calibrationData.wheel, closedGains.limit)` 重算 `calibrationData.vmax`。
- `TELEM 1` 时把 `lastTelemMs=millis()`，保证第一行在约 50 ms 后出现。
- `serviceControl` 入口先调用 `maybeSendTelemetry(millis())`，**无论闭环是否 active 都调用**；标定中不发送。满足 `telemEnabled && controlOwner!=None && nowMs-lastTelemMs>=50` 时用 `reply.printf` 输出一行 `TLM ...\r\n`，更新 `lastTelemMs`。
- `selectMode` 和 `revokeControl` 把 `telemEnabled=false`；模式切换、断链、控制权撤销后不得继续发 TLM。
- `SAVE` 仅停止时允许：`config_store::saveCalibration(calibrationData); config_store::saveGains(closedGains);`。
- `CAPS` 增加能力行。

- [ ] **Step 4: 运行确认通过**

- [ ] **Step 5: 记录结果，不提交 git**

---

### Task 9: 标定状态机（成功路径）

**Files:**
- Modify: `motor-bringup/src/main.cpp`
- Modify: `motor-bringup/tests/test_closed_loop.cpp`

**Interfaces:**
- Consumes: `bool calibrating`、`stopMotor()`、`setMotorForwardPercent()`、`calibrationData`、`closedGains`（Task 6）。
- Produces（`main.cpp` 全局，测试可访问）：
  - `enum class CalPhase { Idle, Settle, Wheel60, Wheel100, SimSettle, Sim, Done };`
  - `struct CalibrationRun { CalPhase phase; int wheel; int duty; int tries; uint32_t phaseStart; uint32_t measureAt; int32_t measureCount; int8_t signs[4]; float v60[4]; float v100[4]; float vsim[4]; };`
  - `CalibrationRun calRun;`
  - `void startCalibration(uint32_t now); void serviceCalibration(uint32_t now); void abortCalibration(const char* reason);`
- Commands: `CAL`、`CAL?`、`CAL RESET`；`STATUS` 在标定时回复 `RUNNING CAL`。

- [ ] **Step 1: 写成功路径测试**

每个标定用例开始先 `fakePreferencesReset(); closedGains = wheel_speed::Gains{}; calibrationData = calibration::Data{}; faultLatched = false; controlOwner = ControlPort::Usb;`。

测试辅助：

```cpp
static void calTick(int ms, const int32_t rates[4]) {
  for (int t=0; t<ms; t+=10) {
    fakeNow += 10; fakeMicros = fakeNow*1000;
    for (int i=0;i<4;++i) encoders[i].count += (rates[i]*10)/1000;
    serviceCalibration(fakeNow);
  }
}
```

用例先 `command("MODE CHASSIS",0); command("LAYOUT CBAD 1 1 1 1",1); command("CAL",1000);`，然后循环 `calTick(10, rates)`。

测试用当前 PWM 决定 rates：某轮 60%（`pwm2≈153`）→ 2000 counts/s；75%（`pwm2≈191`）→ 2500；100%（`pwm2=255`）→ 5000；四轮同时 100% → 每轮 4500。测试循环直到 `!calibrating` 或超时 30 s。

断言：

- `CAL DONE VALID=1 VMAX=3600.0`（`80% × min(vsim=4500)`）。
- 四路 `sign` 与注入方向一致；`calibrationData.valid`；`calibrationData.vmax≈3600`。
- 结束时 `pwm` 全 0、`levels[kStandby]==LOW`。
- `CAL?` 输出包含 `CAL VALID=1` 和四行 `CALW`。
- `config_store::load` 能读回 `valid` 和 `vmax`。

- [ ] **Step 2: 运行确认失败**

- [ ] **Step 3: 实现状态机**

- `startCalibration`：前置条件（`MODE CHASSIS`、布局、未运行、未标定中、未故障闭锁、控制权、停后冷却已过）；`calibrationData.valid=false`，若 `config_store::saveCalibration` 失败则回复 `CAL FAIL nvs` 并返回；初始化 `calRun`；`calibrating=true`；`stopMotor`；`phase=Settle; phaseStart=now; wheel=0; duty=60; tries=0;`；回复 `CAL START`。
- `serviceCalibration`：
  - `Settle` 200 ms → 设置该轮输出，`phase=Wheel60`，`phaseStart=now`；到 700 ms 时记录 `measureAt=now` 和 `measureCount=count`；回复 `CAL PROGRESS run <wheel> <duty>`。
  - `Wheel60` 到 1200 ms：`v=(count-measureCount)/((now-measureAt)/1000.0)`；`|v|<50` 且未重试 → 75% 重试（`tries=1`，回到 `Settle`）；仍无 → `CAL FAIL no_feedback <i>`。
  - 有速度：`signs[i]=sign(v)`、`v60[i]=|v|`；回复 `CAL PROGRESS speed <wheel> %.1f`；切 100%，`phase=Wheel100`，重新计时/记录。
  - `Wheel100` 到 1200 ms 用同样方式取 `v100[i]`；`computeLine(d1, v60, v100, wheel)` 失败 → `CAL FAIL bad_line <i>`；成功 → 写入 `sign/a/b/v100`，回复 `CAL PROGRESS line <wheel> a=%.1f b=%.4f`；停输出，下一轮。
  - 四轮完成后 `SimSettle` 200 ms → 四轮 100%，`phase=Sim` 1500 ms；到 1000 ms 记录 `measureAt/measureCount`，1500 ms 取最后 500 ms `vsim`；`min(vsim)<200` → `CAL FAIL bad_sim <i>`；`calibrationData.vmax=computeVmax(wheel, closedGains.limit)`；`valid=true`；若保存失败 → `CAL FAIL nvs`；否则回复 `CAL DONE VALID=1 VMAX=%.1f`；停车；`calibrating=false`。
- `CAL?` 输出：`CAL VALID=0|1 VMAX=%.1f LIMIT=%d`，随后四行 `CALW <A-D> SIGN=%d V100=%.1f DEADBAND=%.1f SLOPE=%.4f`（`DEADBAND=a`，`SLOPE=b`）。
- `CAL RESET` 停止时清 `valid` 并保存；回复 `CAL RESET`。
- 标定成功或 `abortCalibration` 结束后，在 `stopMotor` 之后设置 `hasStopped=true; stoppedAt=millis();`，让后续外部运动命令走现有 1 s 冷却；标定内部阶段不受冷却限制（设计 §5.3）。
- `STATUS` 在 `calibrating` 时回复 `RUNNING CAL`，随后仍输出四路 ENC。
- `setup()` 在初始化编码器后调用 `config_store::load(calibrationData, closedGains)`；加载失败保持默认无效标定。
- `loop()` 调用 `serviceCalibration(millis())`。

- [ ] **Step 4: 运行确认通过**

- [ ] **Step 5: 记录结果，不提交 git**

---

### Task 10: 标定失败与中断

**Files:**
- Modify: `motor-bringup/src/main.cpp`
- Modify: `motor-bringup/tests/test_closed_loop.cpp`

**Interfaces:**
- Consumes: Task 9 的 `abortCalibration`。
- `STOP`、`MODE` 切换、`revokeControl`、通信超时都调用 `abortCalibration`。

- [ ] **Step 1: 追加失败测试**

1. 标定中所有编码器保持 0：60% 与 75% 都无速度 → `CAL FAIL no_feedback`，`calibrating=false`，PWM 全 0，`calibrationData.valid==false`，NVS `valid=0`。
2. 标定中发送 `STOP` → `CAL FAIL stopped`（或 `STOPPED command` + 标定中止），PWM 全 0，`valid=false`。
3. 标定中 `revokeControl`/`MODE DRY_RUN` → 同上。
4. 构造 `v60≈v100`（rates 2000 与 2001）→ `CAL FAIL bad_line`。
5. 先用成功路径建立 `valid=1`；再开始一次失败标定；结束后 `valid==false`，`SPEED` 被拒绝。

- [ ] **Step 2: 运行确认失败**

- [ ] **Step 3: 集成中止路径**

- `abortCalibration(reason)`：若 `calibrating`，停车、`calibrating=false`、`calibrationData.valid=false`、`config_store::saveCalibration`、回复 `CAL FAIL <reason>`、`phase=Idle`。
- `handleCommand("STOP")`、`selectMode`、`revokeControl` 在 `stopMotor` 前调用 `abortCalibration`。
- `serviceConnections` 的 1.5 s 超时走 `revokeControl`，自然覆盖。
- 标定期间拒绝 `SPEED`/`SPEEDW`/`MECANUM`/`WHEELS`/`JOG`（`calibrating` 加入 busy 判据）；`ENCZERO` 在标定期间也拒绝，避免清零破坏测量窗口；`CAL` 在标定中拒绝。
- `CAL RESET` 去掉不必要的停后冷却门控（设计只要求“停止时”）；用 `config_store::saveCalibration`（`valid=false`）检查返回值，失败回复 `ERROR NVS save failed`。
- `Sim` 阶段先写入 `calRun.vsim[i]`，全部通过 `min(vsim)≥200` 后才复制到 `calibrationData.wheel[i].vsim`，避免失败标定把新 `vsim` 与旧 `a/b/v100` 混存。

- [ ] **Step 4: 运行确认通过**

- [ ] **Step 5: 记录结果，不提交 git**

---

### Task 11: 面板协议客户端 `ChassisLink`

**Files:**
- Modify: `motor-bringup/chassis_link.py`
- Modify: `motor-bringup/tests/test_chassis_link.py`

**Interfaces:**
- Produces:
  - 属性：`closed_loop_supported: bool`、`cal_valid: bool`、`cal_vmax: float`、`cal_limit: int`、`cal_wheels: dict`、`pid_values: dict | None`。
  - 方法：`speed(values, milliseconds, *, mixed=True)`、`calibrate()`、`cal_reset()`、`request_cal()`、`set_pid(kp_milli, ki_milli, limit)`、`request_pid()`、`save()`、`telem(on)`。
  - 事件：`("capabilities2", True)`、`("cal_status", dict)`、`("cal_progress", str)`、`("cal_done", dict)`、`("cal_fail", str)`、`("pid", dict)`、`("telemetry", tuple[int,...])`、`("fault", str)`。

- [ ] **Step 1: 追加失败测试**

在 `test_chassis_link.py` 扩展 `ChassisPeer.write`：

- `CAPS` 回复旧行 + `CHASSIS_CAPS2 API=WHEEL_SPEED_V1 CLOSED=1 CAL=1 TELEM=1 PID=MILLI`。
- `SPEED ...` → `SPEED_RUN ...`；`CAL` → `CAL START`；`CAL?` → `CAL VALID=1 VMAX=3200.0 LIMIT=80` + 四行 `CALW`；`PID 2500 1200 75` → `PID_OK KP=2.500 KI=1.200 LIMIT=75`；`PID?` 同；`TELEM 1` → `TELEM ON`；`SAVE` → `SAVED`。

测试断言：

- 连接后 `closed_loop_supported` 为 True；`cal_valid` 为 True；`cal_wheels["A"]["sign"]==1`。
- `speed((50,0,0),500,mixed=True)` 排队 `SPEED 50 0 0 500`；`speed((50,0,50,0),500,mixed=False)` 排队 `SPEEDW 50 0 50 0 500`。
- 未标定/不支持时 `speed` 抛 `ValueError`。
- 收到 `TLM 1000 4000 ...` 产生 `("telemetry", (...))`，长度 13。
- 收到 `FAULT encoder_sign` 产生 `("fault", "encoder_sign")`。
- `close()` 前发送 `STOP` 和 `TELEM 0`（假串口 `writes` 中可见）。

- [ ] **Step 2: 运行确认失败**

```powershell
cd D:\Users\1713810750\推物块机器人\motor-bringup
D:\Users\1713810750\推物块机器人\esp32-pc-link\pc\.venv\Scripts\python.exe -m unittest discover -s tests -p "test_chassis_link.py" -v
```

- [ ] **Step 3: 实现**

- `connect`/`UNIFIED_BOOT`/`MODE_OK` 时重置闭环属性。
- `handle_response` 增加上述行解析；TLM 用 `split()` 转 int；`CALW` 的 `DEADBAND` 映射到 `a`、`SLOPE` 映射到 `b`。
- `speed` 镜像 `motion` 的参数校验和排队/冷却逻辑；混控用 `SPEED`，直控用 `SPEEDW`；`closed_loop_supported` 和 `cal_valid` 不满足时抛错。
- `calibrate` 也遵守 `_next_jog` 冷却；冷却中返回 False。
- `close`/`disconnect` 前发送 `TELEM 0` 和 `STOP`。
- `send_line` 白名单增加 `PID?`、`CAL?`、`SAVE`；运动仍只走方法。

- [ ] **Step 4: 运行确认通过**

- [ ] **Step 5: 记录结果，不提交 git**

---

### Task 12: 调试面板 UI

**Files:**
- Modify: `debug-workbench/chassis_app.py`
- Modify: `debug-workbench/tests/test_chassis_ui.py`

**Interfaces:**
- Consumes: Task 11 的 `ChassisLink` 属性/方法/事件。
- Produces:
  - `self.closed_loop: tk.BooleanVar`、`self.closed_status: tk.StringVar`、`self.kp/self.ki/self.limit: tk.StringVar`、`self.cal_progress: tk.StringVar`。
  - 新标签页“闭环调参”，含标定、PID、四轮目标/实际/占空比表格。
  - 闭环勾选时方向键/x/y/z 走 `speed(..., mixed=True)`；四路直控走 `speed(..., mixed=False)`。

- [ ] **Step 1: 追加失败测试**

在 `test_chassis_ui.py`：

1. 无 `CHASSIS_CAPS2` 时闭环复选框禁用；`cal_valid=False` 时勾选被拒绝。
2. 能力行 + `CAL VALID=1` 后可勾选；按“前进 ↑”发送 `SPEED 100 0 0 <ms>`，不勾选发送 `MECANUM`。
3. 闭环力度 4 时提示“闭环最低力度 5%”，不发送运动。
4. “开始标定”在 `messagebox.askyesno` 返回 True 时发送 `CAL`，期间运动按钮禁用；`CAL FAIL` 后显示原因，复选框保持禁用。
5. `PID 2500 1200 75` 后 `PID?` 回填 `Kp=2.500, Ki=1.200, 上限=75`；“应用 PID”和“保存参数”发送对应命令。
6. 注入 `TLM ...` 后表格显示四轮目标/实际/占空比，且日志文本框不含 `TLM `。
7. `FAULT ...` 后运动按钮禁用；`STOP` 后恢复。

- [ ] **Step 2: 运行确认失败**

```powershell
cd D:\Users\1713810750\推物块机器人\debug-workbench
D:\Users\1713810750\推物块机器人\esp32-pc-link\pc\.venv\Scripts\python.exe -m unittest discover -s tests -p "test_chassis_ui.py" -v
```

- [ ] **Step 3: 实现**

- 在 `_build` 的麦克纳姆页加入闭环复选框和状态标签；新增“闭环调参”页。
- `run_motion` 按 `closed_loop` 选择 `speed`/`motion`；方向键力度 <5 且闭环时提示并返回；`run_axes` 不预判混控后的轮目标（由固件拒绝）；`run_wheels` 闭环时检查每个非零 ≥5。
- 标定按钮仅在 `client.ready`、布局已确认、现场确认勾选、未运行、未标定时可用；点击后调用 `messagebox.askyesno`，确认后 `client.calibrate()`、禁用运动、显示进度。
- PID 输入校验整数和范围；应用/保存调用对应方法。
- 客户端在收到 `CHASSIS_CAPS2` 时已自动发 `CAL?`/`PID?`；面板**不要**在 `ready` 事件重复发送。
- `CAL DONE` 后发送 `request_cal()` 刷新每轮 `CALW`/`cal_vmax`；`PID_OK` 后也发一次 `request_cal()`（limit 改变会重算 Vmax）。
- `CAL RESET` 或收到 `ERROR` 后发送 `request_cal()` 同步 `cal_valid`；`CAL VALID=1` 才允许勾选闭环。
- `pump` 处理新事件；`TLM ` 行只更新表格，不写日志（断言用 `app.log_lines`）。
- `update_controls` 门控复选框、标定按钮、运动按钮；闭环无效时自动取消勾选。
- 遥测开关最终实现为“勾选闭环”**或**“闭环调参页激活”任一成立即 `TELEM 1`（有意偏离本计划最初的 AND 语义，避免离开复选框后失去调参遥测；与 `debug-workbench/README.md` 和 UI 测试一致）。
- v1 只做表格+数值刷新；4 线曲线留作后续，不作为本任务验收。
- `stop`/`disconnect`/`shutdown`/失焦发送 `telem(False)` 和 `STOP`。

- [ ] **Step 4: 运行确认通过**

- [ ] **Step 5: 记录结果，不提交 git**

---

### Task 13: 文档、全量回归与实机验收清单

**Files:**
- Modify: `motor-bringup/MECANUM.md`
- Modify: `motor-bringup/README.md`
- Modify: `debug-workbench/README.md`
- Create: `motor-bringup/CLOSED-LOOP-VERIFICATION.md`

- [ ] **Step 1: 更新文档**

- `MECANUM.md`：新增“轮速闭环”章节，列出 `SPEED`/`SPEEDW`/`PID`/`CAL`/`TELEM`/`SAVE`、标定步骤、5% 下限、打滑限制、故障闭锁。
- `README.md`：把“PID 尚未实现”改为“闭环已实现，需先悬空标定”；保留开环说明。
- `debug-workbench/README.md`：说明“闭环调参”页、标定确认和遥测。
- `CLOSED-LOOP-VERIFICATION.md`：实机验收 8 步（悬空标定、重启回读、低速跟随、落地直行对比、横移/斜移/旋转、拔编码器故障、断链停车、记录电压/起转占空比/稳态误差）。

- [ ] **Step 2: 全量 C++ 主机测试**

```powershell
cd D:\Users\1713810750\推物块机器人\motor-bringup
g++ -std=c++17 -I tests tests/test_wheel_speed.cpp -o tests/test_wheel_speed.exe; .\tests\test_wheel_speed.exe
g++ -std=c++17 -I tests tests/test_calibration.cpp -o tests/test_calibration.exe; .\tests\test_calibration.exe
g++ -std=c++17 -I tests tests/test_config_store.cpp -o tests/test_config_store.exe; .\tests\test_config_store.exe
g++ -std=c++17 -I tests tests/test_closed_loop.cpp src/dry_core.cpp -o tests/test_closed_loop.exe; .\tests\test_closed_loop.exe
g++ -std=c++17 -I tests tests/test_motor.cpp src/dry_core.cpp -o tests/test_motor.exe; .\tests\test_motor.exe
g++ -std=c++17 -I tests tests/test_chassis.cpp src/dry_core.cpp -o tests/test_chassis.exe; .\tests\test_chassis.exe
g++ -std=c++17 -I tests tests/test_bluetooth.cpp src/dry_core.cpp -o tests/test_bluetooth.exe; .\tests\test_bluetooth.exe
```

Expected: 全部退出码 0。

- [ ] **Step 3: 全量 Python 测试**

```powershell
cd D:\Users\1713810750\推物块机器人\motor-bringup
D:\Users\1713810750\推物块机器人\esp32-pc-link\pc\.venv\Scripts\python.exe -m unittest discover -s tests -p "test_*.py" -v
cd D:\Users\1713810750\推物块机器人\debug-workbench
D:\Users\1713810750\推物块机器人\esp32-pc-link\pc\.venv\Scripts\python.exe -m unittest discover -s tests -p "test_*.py" -v
```

Expected: 全部通过；Tk 测试需要可用桌面会话。

- [ ] **Step 4: PlatformIO 编译**

```powershell
cd D:\Users\1713810750\推物块机器人\motor-bringup
pio run
```

Expected: `SUCCESS`，无新增警告级别错误。

- [ ] **Step 5: 记录结果，不提交 git**

---

## 执行顺序与依赖

- Task 1–5 是纯模块，可先并行做，但 Task 3 依赖 Task 2，Task 4 依赖 Task 3 的 `Gains`。
- Task 6 依赖 Task 1/2/3；Task 7 依赖 Task 4/6；Task 8 依赖 Task 5/6。
- Task 9 依赖 Task 2/5/6；Task 10 依赖 Task 9。
- Task 11 依赖 Task 6/8/9 的协议；Task 12 依赖 Task 11。
- Task 13 最后做。

## 完成标准

- 所有新增主机测试通过，现有测试不回归。
- `pio run` 通过。
- 面板能完成：能力识别、闭环开关、标定、PID 调参、四轮目标/实际/占空比显示、故障提示。
- 实机验收按 `CLOSED-LOOP-VERIFICATION.md` 由用户执行；在用户确认前，不声称闭环性能已达标。
