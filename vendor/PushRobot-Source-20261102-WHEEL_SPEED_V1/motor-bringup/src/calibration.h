#pragma once
#include <stdint.h>

// 两点前馈标定：由低/高两个测速点拟合线性前馈 a + b*v，
// 并计算各轮统一限幅下的最大速度。纯计算，不依赖 Arduino。
namespace calibration {

struct Wheel {
  int8_t sign;  // 编码器反馈符号：逻辑正向转动时原始计数的符号，+1 / -1。
  float a;      // 前馈截距（占空比 %）；拟合量，不等于实测最低起转占空比。
  float b;      // 前馈斜率（% per count/s）
  float v100;   // 满占空比时的速度（counts/s）
  float vsim;   // 四轮同时 100% 输出时本轮实测速率（counts/s），不是模拟数据。
};

struct Data {
  uint32_t version;
  bool valid;
  Wheel wheel[4];  // A/B/C/D 通道顺序，与物理轮位通过 Layout 对应。
  float vmax;      // 所有轮共用的目标速度量程，counts/s。
};

// 由低/高两个测速点计算前馈线。
// d1 为 v60 对应的目标占空比（百分比）。
// 校验 d1∈(0,100)、v100>v60、v100≥200；
// b=(100-d1)/(v100-v60)、a=d1-b*v60。
// 若 b<=0、a<0 或 a>=90 则失败，且不修改 wheel；
// 成功时写入 a/b/v100 并保留 sign。
inline bool computeLine(float d1, float v60, float v100, Wheel& wheel) {
  if (d1 <= 0.0f || d1 >= 100.0f) return false;
  if (v100 <= v60) return false;
  if (v100 < 200.0f) return false;

  const float b = (100.0f - d1) / (v100 - v60);
  const float a = d1 - b * v60;

  if (b <= 0.0f) return false;
  if (a < 0.0f) return false;
  if (a >= 90.0f) return false;

  wheel.a = a;
  wheel.b = b;
  wheel.v100 = v100;
  return true;
}

// 所有轮统一的 Vmax：limitPercent/100 × min(vsim_i)。
// 使用最慢轮作为量程基准。limitPercent 限目标速度，不限制 PI 的 PWM 输出。
inline float computeVmax(const Wheel wheel[4], float limitPercent) {
  float minVsim = wheel[0].vsim;
  for (int i = 1; i < 4; ++i) {
    if (wheel[i].vsim < minVsim) minVsim = wheel[i].vsim;
  }
  return (limitPercent / 100.0f) * minVsim;
}

}  // namespace calibration
