#pragma once
#include <stdint.h>

#include "calibration.h"

// 轮速估计与轮速闭环（前馈 + PI + 抗积分饱和）。
// 纯计算，不依赖 Arduino；由 main.cpp 的 serviceControl 按实际 dt 调用。
// 调参入口：Gains；测速平滑：SpeedEstimator；停车阈值：FaultMonitor 私有常量。
// WheelPid 名称沿用协议 PID，算法只有 P/I，无 D 项；不要把 counts/s 当成 RPM。
namespace wheel_speed {

class SpeedEstimator {
 public:
  // 增大窗口可以减小计数量化波动，但会增加反馈延迟；需同步检查 PI 跟随表现。
  static constexpr int kWindow = 4;              // 4 个样本 = 40 ms @ 10 ms tick
  static constexpr uint32_t kMinWindowUs = 10000;  // 最小统计窗口 10 ms

  // 清零窗口与速度。
  inline void reset() {
    for (int i = 0; i < kWindow; ++i) {
      delta_[i] = 0;
      dtUs_[i] = 0;
    }
    index_ = 0;
    sumDelta_ = 0;
    sumDtUs_ = 0;
    speed_ = 0.0f;
  }

  // 输入本周期计数增量与周期长度，返回当前速度（counts/s）。
  // dtUs == 0 时不改变速度；累计窗口不足 kMinWindowUs 时沿用上次速度。
  inline float update(int32_t deltaCounts, uint32_t dtUs) {
    if (dtUs == 0) return speed_;

    // 环形缓冲：移除被覆盖的最旧样本，再写入新样本。
    sumDelta_ -= delta_[index_];
    sumDtUs_ -= dtUs_[index_];
    delta_[index_] = deltaCounts;
    dtUs_[index_] = dtUs;
    sumDelta_ += deltaCounts;
    sumDtUs_ += dtUs;
    index_ = (index_ + 1) % kWindow;

    if (sumDtUs_ >= kMinWindowUs) {
      speed_ = static_cast<float>(1e6 * static_cast<double>(sumDelta_) /
                                  static_cast<double>(sumDtUs_));
    }
    return speed_;
  }

  inline float speed() const { return speed_; }

 private:
  int32_t delta_[kWindow] = {};
  uint32_t dtUs_[kWindow] = {};
  int index_ = 0;
  int64_t sumDelta_ = 0;
  uint64_t sumDtUs_ = 0;
  float speed_ = 0.0f;
};

// PI 参数。limit 与 amax 供外部使用（limit 参与 Vmax 计算，amax 用于目标斜坡），
// step() 本身只关心 kp/ki。
struct Gains {
  float kp = 2.0f;     // 比例增益：作用于归一化误差，而非原始 counts/s。
  float ki = 1.0f;     // 积分增益：dt 以秒传入。
  float limit = 80.0f; // 目标速度量程百分比，非 PWM 限幅（输出仍可达 ±100%）。
  float amax = 300.0f; // 目标斜坡每秒最多改变 300% Vmax；不是轮子物理加速度。
};

// 单轮速度闭环：前馈 + PI + 抗积分饱和，输出逻辑占空比（-100..100）。
// 前馈由两点标定线 a + b*v 给出；PI 在归一化速度误差上工作（满量程 = 100）。
class WheelPid {
 public:
  // 清零积分与换向记录。
  inline void reset() {
    integral_ = 0.0f;
    lastTargetSign_ = 0;
  }

  // target/measured: counts/s；vmax: counts/s；dt: 秒。
  // 返回逻辑占空比百分比，已限幅到 ±100。
  inline float step(float target, float measured, float vmax,
                    const calibration::Wheel& wheel, const Gains& gains,
                    float dt) {
    // 目标为零：清零积分并直接输出 0（STOP 语义，不走 PI）。
    if (target == 0.0f) {
      integral_ = 0.0f;
      lastTargetSign_ = 0;
      return 0.0f;
    }

    // 目标符号变化：清零积分，避免换向时带着旧积分冲出去。
    const int targetSign = (target > 0.0f) ? 1 : -1;
    if (lastTargetSign_ != 0 && targetSign != lastTargetSign_) integral_ = 0.0f;
    lastTargetSign_ = targetSign;

    // 1. 前馈：uff = sign(target) × clamp(a + b×|target|, 0, 100)。
    const float absTarget = (target < 0.0f) ? -target : target;
    float ff = wheel.a + wheel.b * absTarget;
    if (ff < 0.0f) ff = 0.0f;
    if (ff > 100.0f) ff = 100.0f;
    const float uff = (targetSign < 0) ? -ff : ff;

    // 2. 归一化误差（满量程百分比）。vmax<=0 时视为无误差，避免除零。
    const float en =
        (vmax > 0.0f) ? (100.0f * (target - measured) / vmax) : 0.0f;

    // 3. 积分累加。积分状态本身限幅到 ±(30/ki)，使“限幅后的应用项”与
    //    内部状态一致：长时间堵转后积分项不会在输出退回线性区时仍停在 ±30，
    //    也不会因反复累加而无限增长。ki == 0 时积分不参与输出，不做除法。
    const float increment = en * dt;
    integral_ += increment;
    clampIntegralState(integral_, gains.ki);
    float integralTerm = clampIntegralTerm(gains.ki * integral_);

    // 4. 未限幅输出。
    float raw = uff + gains.kp * en + integralTerm;

    // 5. 抗积分饱和：输出已饱和且误差还在往饱和方向推时，回退本次积分增量。
    if ((raw > kOutputLimit && en > 0.0f) ||
        (raw < -kOutputLimit && en < 0.0f)) {
      integral_ -= increment;
      clampIntegralState(integral_, gains.ki);
      integralTerm = clampIntegralTerm(gains.ki * integral_);
      raw = uff + gains.kp * en + integralTerm;
    }

    // 输出限幅。
    if (raw > kOutputLimit) raw = kOutputLimit;
    if (raw < -kOutputLimit) raw = -kOutputLimit;
    return raw;
  }

  // 当前积分状态（单位：%·s），主要用于测试与遥测。
  inline float integral() const { return integral_; }

 private:
  static constexpr float kOutputLimit = 100.0f;
  static constexpr float kIntegralTermLimit = 30.0f;

  static inline float clampIntegralTerm(float term) {
    if (term > kIntegralTermLimit) return kIntegralTermLimit;
    if (term < -kIntegralTermLimit) return -kIntegralTermLimit;
    return term;
  }

  // 积分状态上限 = 应用项上限 / ki；ki 为 0（或负值防守）时不限幅。
  static inline void clampIntegralState(float& integral, float ki) {
    if (!(ki > 0.0f)) return;
    const float maxIntegral = kIntegralTermLimit / ki;
    if (integral > maxIntegral) integral = maxIntegral;
    else if (integral < -maxIntegral) integral = -maxIntegral;
  }

  float integral_ = 0.0f;
  int lastTargetSign_ = 0;
};

// 故障判据以输出占空比与实测轮速为主，覆盖低速命令。
// 无电流采样，NoFeedback/Stall 仅为运动学诊断，不能严格区分断线与机械卡死。
enum class Fault { None, EncoderSign, NoFeedback, Stall };

// 故障名（遥测/日志用），None 返回 "none"。
inline const char* faultName(Fault fault) {
  switch (fault) {
    case Fault::EncoderSign:
      return "encoder_sign";
    case Fault::NoFeedback:
      return "no_feedback";
    case Fault::Stall:
      return "stall";
    case Fault::None:
      break;
  }
  return "none";
}

// 故障监视器：条件持续满足达到各自时长才判故障；条件不满足时计时清零。
// target/measured: counts/s；duty: 逻辑占空比 %；vmax: counts/s；dt: 秒。
// 多个条件同时成立时按 EncoderSign > NoFeedback > Stall 顺序返回。
class FaultMonitor {
 public:
  // 清空全部持续时间计时。
  inline void reset() {
    encoderSignSec_ = 0.0f;
    noFeedbackSec_ = 0.0f;
    stallSec_ = 0.0f;
  }

  // 返回当前故障；未达判据返回 Fault::None。
  inline Fault update(float target, float measured, float duty, float vmax,
                      float dt) {
    const float step = (dt > 0.0f) ? dt : 0.0f;

    const float absDuty = absFloat(duty);
    const float absMeasured = absFloat(measured);
    const float absTarget = absFloat(target);
    const bool validVmax = (vmax > 0.0f);

    // encoder_sign：占空比与测速方向相反，且测速不可忽略。
    const bool encoderSign = validVmax && absDuty >= kEncoderSignMinDuty &&
                             duty * measured < 0.0f &&
                             absMeasured >= kEncoderSignMinSpeed * vmax;
    encoderSignSec_ = encoderSign ? encoderSignSec_ + step : 0.0f;

    // stall：目标与占空比都很大，但测速远低于目标。
    // 先算 stall，用于给目标相对 no_feedback 让路（见下）。
    const bool stall = validVmax && absTarget >= kStallMinTarget * vmax &&
                       absDuty >= kStallMinDuty &&
                       absMeasured < kStallMaxFraction * absTarget;
    stallSec_ = stall ? stallSec_ + step : 0.0f;

    // no_feedback 有两条判据：
    //  a) 绝对判据：高占空比但几乎无测速（原有行为）；
    //  b) 目标相对判据：任意非零目标下，占空比 ≥50% 且测速不足目标的 20%。
    //     低速目标（如 5% Vmax）时 PI 只会把占空比推到 75–80%，绝对判据的
    //     85% 门槛覆盖不到；相对判据让编码器断线/卡死在低速命令下也能停车。
    //     如果同时满足 stall（目标 ≥20% Vmax、占空比 ≥98%），则交给 1000 ms
    //     的 stall 判据，避免 800 ms 的相对判据抢在 stall 之前。
    const bool noFeedbackAbsolute = validVmax && absDuty >= kNoFeedbackMinDuty &&
                                    absMeasured < kNoFeedbackMaxSpeed * vmax;
    const bool noFeedbackRelative =
        absTarget > 0.0f && absDuty >= kNoFeedbackRelativeMinDuty &&
        absMeasured < kNoFeedbackRelativeMaxFraction * absTarget && !stall;
    const bool noFeedback = noFeedbackAbsolute || noFeedbackRelative;
    noFeedbackSec_ = noFeedback ? noFeedbackSec_ + step : 0.0f;

    if (encoderSignSec_ >= kEncoderSignHoldSec) return Fault::EncoderSign;
    if (noFeedbackSec_ >= kNoFeedbackHoldSec) return Fault::NoFeedback;
    if (stallSec_ >= kStallHoldSec) return Fault::Stall;
    return Fault::None;
  }

 private:
  static constexpr float kEncoderSignMinDuty = 50.0f;
  static constexpr float kEncoderSignMinSpeed = 0.03f;   // ×vmax
  static constexpr float kEncoderSignHoldSec = 0.3f;

  static constexpr float kNoFeedbackMinDuty = 85.0f;
  static constexpr float kNoFeedbackMaxSpeed = 0.02f;    // ×vmax
  static constexpr float kNoFeedbackRelativeMinDuty = 50.0f;
  static constexpr float kNoFeedbackRelativeMaxFraction = 0.20f;  // ×|target|
  static constexpr float kNoFeedbackHoldSec = 0.8f;

  static constexpr float kStallMinTarget = 0.20f;        // ×vmax
  static constexpr float kStallMinDuty = 98.0f;
  static constexpr float kStallMaxFraction = 0.20f;      // ×|target|
  static constexpr float kStallHoldSec = 1.0f;

  static inline float absFloat(float x) { return (x < 0.0f) ? -x : x; }

  float encoderSignSec_ = 0.0f;
  float noFeedbackSec_ = 0.0f;
  float stallSec_ = 0.0f;
};

}  // namespace wheel_speed
