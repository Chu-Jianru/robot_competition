#include <cassert>
#include <cmath>
#include <cstring>
#include "../src/wheel_speed.h"

static bool near(float a, float b) { return std::fabs(a-b) < 0.5f; }

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

// 积分状态限幅：ki>0 时状态必须保持在 ±(30/ki)，ki==0 时不做除法。
static void test_integral_state_clamp() {
  wheel_speed::WheelPid p;
  wheel_speed::Gains g;
  g.kp = 2.0f;
  g.ki = 5.0f;             // 状态上限 30/5 = 6
  g.limit = 80; g.amax = 300;
  // 大 vmax 让误差持续存在，但输出不到 100，从而积分能真正涨到上限。
  p.reset();
  for (int i = 0; i < 1200; ++i) p.step(1000.0f, 0.0f, 100000.0f, wheel(), g, 0.01f);
  assert(std::fabs(p.integral()) <= 30.0f / g.ki + 0.001f);
  assert(p.integral() > 30.0f / g.ki - 0.01f);

  g.ki = 1.0f;             // 状态上限 30
  p.reset();
  for (int i = 0; i < 4000; ++i) p.step(1000.0f, 0.0f, 100000.0f, wheel(), g, 0.01f);
  assert(std::fabs(p.integral()) <= 30.0f + 0.001f);
  assert(p.integral() > 29.9f);

  // ki == 0：积分不参与输出，也不能出现除零/NaN。
  g.ki = 0.0f;
  p.reset();
  for (int i = 0; i < 200; ++i) p.step(1000.0f, 0.0f, 100000.0f, wheel(), g, 0.01f);
  assert(std::isfinite(p.integral()));
}

// 简单被控对象：死区 + 一阶滞后 + 占空比饱和。
// 输入逻辑占空比 u，输出轮速 counts/s；用于验证 5%/10%/30% 目标下
// 积分有界、无持续极限环、能在合理时间内稳定。
struct Plant {
  float v = 0.0f;
  static constexpr float kDeadband = 32.5f;
  static constexpr float kMaxDuty = 100.0f;
  static constexpr float kMaxSpeed = 5000.0f;  // 100% 占空比稳态速度
  static constexpr float kTau = 0.08f;         // 一阶滞后 80 ms
  void step(float duty, float dt) {
    if (duty > kMaxDuty) duty = kMaxDuty;
    if (duty < -kMaxDuty) duty = -kMaxDuty;
    const float magnitude = std::fabs(duty);
    float target = 0.0f;
    if (magnitude > kDeadband) {
      target = (magnitude - kDeadband) / (kMaxDuty - kDeadband) * kMaxSpeed;
    }
    if (duty < 0.0f) target = -target;
    v += (target - v) * (dt / kTau);
  }
};

static void test_plant_model() {
  const float percents[] = {5.0f, 10.0f, 30.0f};
  for (float percent : percents) {
    wheel_speed::Gains g;
    wheel_speed::WheelPid pid;
    Plant plant;
    calibration::Wheel w;
    w.sign = 1; w.a = 32.5f; w.b = 0.016f; w.v100 = 5000; w.vsim = 5000;
    const float vmax = 4000.0f;  // LIMIT 80% × 5000
    const float target = percent / 100.0f * vmax;
    pid.reset();
    float minLate = 1e9f, maxLate = -1e9f;
    for (int i = 0; i < 600; ++i) {  // 6 s
      const float duty = pid.step(target, plant.v, vmax, w, g, 0.01f);
      plant.step(duty, 0.01f);
      assert(std::fabs(pid.integral()) <= 30.0f / g.ki + 0.001f);
      assert(duty >= -100.0f && duty <= 100.0f);
      if (i >= 500) {  // 最后 1 s：检查稳态误差与极限环
        minLate = std::fmin(minLate, plant.v);
        maxLate = std::fmax(maxLate, plant.v);
      }
    }
    assert(std::fabs(plant.v - target) < 0.05f * target);
    assert(maxLate - minLate < 0.05f * target);
  }
}

// 5%/10% 目标 + 断编码器（零计数）：目标相对 no_feedback 必须在 800 ms 后触发。
// 健康 5% 工况（测速≈目标）不得误报。
static int low_speed_fault_ticks(float percent, bool healthy) {
  wheel_speed::Gains g;
  wheel_speed::WheelPid pid;
  wheel_speed::FaultMonitor monitor;
  calibration::Wheel w;
  w.sign = 1; w.a = 32.5f; w.b = 0.016f; w.v100 = 5000; w.vsim = 5000;
  const float vmax = 4000.0f;
  const float target = percent / 100.0f * vmax;
  const float measured = healthy ? target : 0.0f;
  pid.reset();
  monitor.reset();
  for (int i = 1; i <= 400; ++i) {
    const float duty = pid.step(target, measured, vmax, w, g, 0.01f);
    const wheel_speed::Fault fault =
        monitor.update(target, measured, duty, vmax, 0.01f);
    if (fault != wheel_speed::Fault::None) {
      assert(fault == wheel_speed::Fault::NoFeedback);
      return i;
    }
  }
  return -1;
}

static void test_low_speed_no_feedback() {
  const int ticks5 = low_speed_fault_ticks(5.0f, false);
  const int ticks10 = low_speed_fault_ticks(10.0f, false);
  // 5% 时前馈+比例项约 46%，积分要再推 4% 才过 50% 门槛，
  // 因此约 1.5 s 触发；10% 约 0.8 s。都远快于旧版只靠 85% 绝对门槛。
  assert(ticks5 > 80 && ticks5 < 250);
  assert(ticks10 > 60 && ticks10 < 150);
  assert(low_speed_fault_ticks(5.0f, true) == -1);
  assert(low_speed_fault_ticks(10.0f, true) == -1);
}

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

  // 目标相对判据：5%/10% 目标下占空比 50–85%，旧绝对判据不会触发。
  m.reset();
  assert(run_fault(m, 250, 0, 75, 5000, 0.79f) == wheel_speed::Fault::None);
  assert(run_fault(m, 250, 0, 75, 5000, 0.02f) == wheel_speed::Fault::NoFeedback);
  m.reset();
  assert(run_fault(m, 500, 0, 80, 5000, 0.79f) == wheel_speed::Fault::None);
  assert(run_fault(m, 500, 0, 80, 5000, 0.02f) == wheel_speed::Fault::NoFeedback);

  m.reset();
  assert(run_fault(m, 4000, 500, 99, 5000, 0.99f) == wheel_speed::Fault::None);
  assert(run_fault(m, 4000, 500, 99, 5000, 0.02f) == wheel_speed::Fault::Stall);
}

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

  test_pid();
  test_integral_state_clamp();
  test_plant_model();
  assert(std::strstr(wheel_speed::faultName(wheel_speed::Fault::NoFeedback),
                     "no_feedback") != nullptr);
  test_faults();
  test_low_speed_no_feedback();
  return 0;
}
