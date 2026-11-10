#pragma once
#include <Preferences.h>
#include <cmath>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "calibration.h"
#include "wheel_speed.h"

// 轮速闭环参数的 NVS 持久化。
// 命名空间 "chassis_cl"，结构版本 1。只有 saveCalibration() 写版本键，
// 且版本键在记录完整写完后最后写入；任何一次写失败都不会留下可用的
// valid=1 记录，上电 load() 必须保持闭环禁用。
// 调试掉电丢标定：先查 load 返回值、ver、valid，再查逐轮字段合法性。
// 标定成功会自动 saveCalibration；界面 SAVE 才保存调整后的增益。
// Flash 写入在停止状态执行；不要将 save 调用放进 serviceControl 或编码器 ISR。
namespace config_store {

namespace detail {

constexpr const char* kNamespace = "chassis_cl";
constexpr float kStructureVersion = 1.0f;
constexpr int kWheelCount = 4;

// 生成 "prefix<index>" 形式的键名（如 "v100_3"）。
inline void formatKey(char* out, size_t outSize, const char* prefix, int index) {
  snprintf(out, outSize, "%s%d", prefix, index);
}

inline bool isFinite(float value) { return std::isfinite(value); }

// 增益合法区间：limit 10..100，kp/ki/amax 有限、非负且在合理上限内。
// PID 命令允许 kp/ki 到 10.000，amax 默认 300；超过这些量级说明记录损坏。
constexpr float kMaxKp = 10.0f;
constexpr float kMaxKi = 10.0f;
constexpr float kMaxAmax = 1000.0f;

inline bool gainsValid(const wheel_speed::Gains& gains) {
  if (!isFinite(gains.kp) || gains.kp < 0.0f || gains.kp > kMaxKp) return false;
  if (!isFinite(gains.ki) || gains.ki < 0.0f || gains.ki > kMaxKi) return false;
  if (!isFinite(gains.amax) || gains.amax < 0.0f || gains.amax > kMaxAmax) return false;
  if (!(gains.limit >= 10.0f && gains.limit <= 100.0f)) return false;
  return true;
}

// 单轮标定合法：sign 只能 ±1，vsim/v100/b 为正，a ∈ [0, 90)。
// vsim/v100 还必须 ≥200 counts/s：标定失败阈值保证低于此值的记录来自
// 损坏/中断写入，不能作为闭环量程。用 !(x > 0) 形式同时拒绝 NaN。
inline bool wheelValid(const calibration::Wheel& wheel) {
  if (wheel.sign != 1 && wheel.sign != -1) return false;
  if (!(wheel.vsim >= 200.0f)) return false;
  if (!(wheel.v100 >= 200.0f)) return false;
  if (!(wheel.b > 0.0f)) return false;
  if (!(wheel.a >= 0.0f && wheel.a < 90.0f)) return false;
  return true;
}

}  // namespace detail

// 读取标定与增益。
// - NVS 打开失败或 ver 不等于 1：cal 置为默认无效标定、gains 置为默认值，返回 false。
// - valid == 0：返回 true，但 cal.valid == false（数据仍可读，用于诊断）。
// - valid == 1 但字段损坏（sign 非 ±1、vsim/v100/b 非正、a 越界）：cal.valid = false。
// - gains 缺省（键不存在）：保留 wheel_speed::Gains 的默认值。
// - 增益越界（limit 不在 10..100，kp/ki/amax 非有限或为负）：整体回退默认值。
// - 成功后 cal.vmax 用标定时的原始 vsim 与当前 gains.limit 重新计算；
//   NVS 里的 vmax 字段不可信（limit 可能已改变）。
inline bool load(calibration::Data& cal, wheel_speed::Gains& gains) {
  cal = calibration::Data{};
  gains = wheel_speed::Gains{};

  Preferences prefs;
  if (!prefs.begin(detail::kNamespace, true)) return false;
  if (prefs.getFloat("ver", 0.0f) != detail::kStructureVersion) {
    prefs.end();
    return false;
  }

  cal.version = static_cast<uint32_t>(detail::kStructureVersion);
  // 只接受精确的 1；其它非零值（损坏/未来语义）一律视为无效。
  const bool flaggedValid = prefs.getUChar("valid", 0) == 1;

  char key[16];
  for (int i = 0; i < detail::kWheelCount; ++i) {
    calibration::Wheel& wheel = cal.wheel[i];
    detail::formatKey(key, sizeof(key), "s", i);
    wheel.sign = prefs.getChar(key, 0);  // 缺失即 0，validate 会判为无效
    detail::formatKey(key, sizeof(key), "a", i);
    wheel.a = prefs.getFloat(key, 0.0f);
    detail::formatKey(key, sizeof(key), "b", i);
    wheel.b = prefs.getFloat(key, 0.0f);
    detail::formatKey(key, sizeof(key), "v100_", i);
    wheel.v100 = prefs.getFloat(key, 0.0f);
    detail::formatKey(key, sizeof(key), "vsim", i);
    wheel.vsim = prefs.getFloat(key, 0.0f);
  }

  const wheel_speed::Gains defaults;
  gains.kp = prefs.getFloat("kp", defaults.kp);
  gains.ki = prefs.getFloat("ki", defaults.ki);
  gains.limit = prefs.getFloat("limit", defaults.limit);
  gains.amax = prefs.getFloat("amax", defaults.amax);
  if (!detail::gainsValid(gains)) gains = wheel_speed::Gains{};

  bool dataValid = true;
  for (int i = 0; i < detail::kWheelCount; ++i) {
    if (!detail::wheelValid(cal.wheel[i])) dataValid = false;
  }
  cal.valid = flaggedValid && dataValid;

  // 只信任由原始 vsim 与当前（可能已回退的）limit 推导出的 Vmax。
  cal.vmax = calibration::computeVmax(cal.wheel, gains.limit);

  prefs.end();
  return true;
}

// 保存标定数据。写入顺序保证中途失败也不会留下 valid=1 的残缺记录：
//   1. 先写 valid=0（失败则立即中止，不动其余数据）；
//   2. 写四轮字段与 vmax；
//   3. 仅当以上全部成功且 cal.valid 为真时写最终 valid=1，否则写 0；
//   4. 最后写 ver（版本键是记录完整性的最后一道标记）。
inline bool saveCalibration(const calibration::Data& cal) {
  Preferences prefs;
  if (!prefs.begin(detail::kNamespace, false)) return false;

  // 1. 先失效。失败则一个字节都不再写，旧记录保持原样。
  if (prefs.putUChar("valid", 0) == 0) {
    prefs.end();
    return false;
  }

  // 2. 轮询写入全部数据；保留首次失败的 ok 状态，最终 valid 会写 0。
  bool ok = true;
  char key[16];
  for (int i = 0; i < detail::kWheelCount; ++i) {
    detail::formatKey(key, sizeof(key), "s", i);
    ok = (prefs.putChar(key, cal.wheel[i].sign) > 0) && ok;
    detail::formatKey(key, sizeof(key), "a", i);
    ok = (prefs.putFloat(key, cal.wheel[i].a) > 0) && ok;
    detail::formatKey(key, sizeof(key), "b", i);
    ok = (prefs.putFloat(key, cal.wheel[i].b) > 0) && ok;
    detail::formatKey(key, sizeof(key), "v100_", i);
    ok = (prefs.putFloat(key, cal.wheel[i].v100) > 0) && ok;
    detail::formatKey(key, sizeof(key), "vsim", i);
    ok = (prefs.putFloat(key, cal.wheel[i].vsim) > 0) && ok;
  }
  ok = (prefs.putFloat("vmax", cal.vmax) > 0) && ok;

  // 3. 只有数据完整且调用方声明有效时才写 valid=1。
  const uint8_t finalValid = (ok && cal.valid) ? 1 : 0;
  ok = (prefs.putUChar("valid", finalValid) > 0) && ok;

  // 4. 版本键最后写。若此前有失败，记录已 valid=0，load 会保持闭环禁用；
  //    若本步失败且 NVS 中原无版本键，load 直接返回 false。
  ok = (prefs.putFloat("ver", detail::kStructureVersion) > 0) && ok;

  prefs.end();
  return ok;
}

// 保存闭环增益。不写 ver：版本键只由 saveCalibration() 建立，
// saveGains() 不能把外来/未来版本的记录误标成版本 1。
inline bool saveGains(const wheel_speed::Gains& gains) {
  Preferences prefs;
  if (!prefs.begin(detail::kNamespace, false)) return false;

  bool ok = true;
  ok = (prefs.putFloat("kp", gains.kp) > 0) && ok;
  ok = (prefs.putFloat("ki", gains.ki) > 0) && ok;
  ok = (prefs.putFloat("limit", gains.limit) > 0) && ok;
  ok = (prefs.putFloat("amax", gains.amax) > 0) && ok;

  prefs.end();
  return ok;
}

// 只清除 valid 标志（CAL RESET），保留标定数据供诊断。
inline void clearCalibrationValidity() {
  Preferences prefs;
  if (!prefs.begin(detail::kNamespace, false)) return;
  prefs.putUChar("valid", 0);
  prefs.end();
}

}  // namespace config_store
