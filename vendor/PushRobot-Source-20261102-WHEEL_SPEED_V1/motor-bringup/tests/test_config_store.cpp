#include <cassert>
#include <cmath>
#include <cstddef>
#include "Arduino.h"
#include "Preferences.h"
#include "../src/config_store.h"

static const char* const kNamespace = "chassis_cl";

// 与 brief 完全一致的往返/版本场景。
static void test_brief_scenario() {
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
}

static calibration::Data makeValidCalibration() {
  calibration::Data cal = calibration::Data{};
  cal.version = 1;
  cal.valid = true;
  cal.vmax = 3200;  // 80% × min(vsim=4000)
  for (int i=0;i<4;++i) {
    cal.wheel[i].sign = (i%2) ? -1 : 1;
    cal.wheel[i].a = 30+i; cal.wheel[i].b = 0.01f*(i+1);
    cal.wheel[i].v100 = 5000+i*100; cal.wheel[i].vsim = 4000+i*100;
  }
  return cal;
}

// 保存过程中任何一次 NVS 写失败，持久化记录都不得让闭环使能。
static void test_save_failure_keeps_loop_disabled() {
  const calibration::Data cal = makeValidCalibration();

  // (a) 最后一次 valid=1 写失败：记录仍可读，但 valid=0。
  fakePreferencesReset();
  fakePreferencesFailOnKey("valid", 2);  // 第 1 次是 valid=0，第 2 次是最终 valid
  assert(!config_store::saveCalibration(cal));
  fakePreferencesClearFailures();
  assert(fakePreferencesHasKey(kNamespace, "ver"));
  calibration::Data loaded;
  wheel_speed::Gains lg;
  assert(config_store::load(loaded, lg));
  assert(!loaded.valid);

  // (b) ver 写失败：没有版本键，load 必须失败并保持无效标定。
  fakePreferencesReset();
  fakePreferencesFailOnKey("ver");
  assert(!config_store::saveCalibration(cal));
  fakePreferencesClearFailures();
  assert(!fakePreferencesHasKey(kNamespace, "ver"));
  calibration::Data wrong;
  wheel_speed::Gains wg;
  assert(!config_store::load(wrong, wg));
  assert(!wrong.valid);

  // (c) 中途 wheel 字段写失败：记录保持 valid=0。
  fakePreferencesReset();
  fakePreferencesFailOnKey("vsim2");
  assert(!config_store::saveCalibration(cal));
  fakePreferencesClearFailures();
  assert(config_store::load(loaded, lg));
  assert(!loaded.valid);

  // (d) 第一次 valid=0 写失败：立即中止，不写版本键。
  fakePreferencesReset();
  fakePreferencesFailOnKey("valid", 1);
  assert(!config_store::saveCalibration(cal));
  fakePreferencesClearFailures();
  assert(!fakePreferencesHasKey(kNamespace, "ver"));
  assert(!config_store::load(wrong, wg));
  assert(!wrong.valid);
}

// 直接对照假件检查每轮键名拼写，并确认 saveGains 不写版本键。
static void test_key_spellings() {
  fakePreferencesReset();
  assert(config_store::saveCalibration(makeValidCalibration()));

  const char* const keys[] = {
      "s0", "s1", "s2", "s3",
      "a0", "a1", "a2", "a3",
      "b0", "b1", "b2", "b3",
      "v100_0", "v100_1", "v100_2", "v100_3",
      "vsim0", "vsim1", "vsim2", "vsim3",
  };
  for (size_t i = 0; i < sizeof(keys) / sizeof(keys[0]); ++i) {
    assert(fakePreferencesHasKey(kNamespace, keys[i]));
  }
  assert(std::fabs(fake_preferences::store().namespaces[kNamespace]["vsim3"] -
                   4300.0) < 0.01);

  // saveGains 不得写 ver：否则会把外来/未来版本块误标成版本 1。
  fakePreferencesReset();
  assert(config_store::saveGains(wheel_speed::Gains{}));
  assert(!fakePreferencesHasKey(kNamespace, "ver"));
  calibration::Data loaded;
  wheel_speed::Gains lg;
  assert(!config_store::load(loaded, lg));
  assert(!loaded.valid);
}

// sign 只接受 +1/-1；缺失键默认 0，必须使整条记录无效。
static void test_sign_validation() {
  calibration::Data cal = makeValidCalibration();
  cal.wheel[2].sign = 0;
  fakePreferencesReset();
  assert(config_store::saveCalibration(cal));
  calibration::Data loaded;
  wheel_speed::Gains lg;
  assert(config_store::load(loaded, lg));
  assert(!loaded.valid);

  cal = makeValidCalibration();
  cal.wheel[0].sign = 2;
  fakePreferencesReset();
  assert(config_store::saveCalibration(cal));
  assert(config_store::load(loaded, lg));
  assert(!loaded.valid);

  // 缺失 s2 键：默认 0（不是 +1），整条记录必须无效。
  fakePreferencesReset();
  assert(config_store::saveCalibration(makeValidCalibration()));
  fake_preferences::store().namespaces[kNamespace].erase("s2");
  assert(config_store::load(loaded, lg));
  assert(!loaded.valid);
}

// 增益越界必须回退默认值，并用默认 limit 重算 Vmax。
static void test_gain_validation() {
  const calibration::Data cal = makeValidCalibration();
  calibration::Data loaded;
  wheel_speed::Gains lg;

  // limit < 10：全部增益回退默认（limit=80），vmax 用 80% 重算。
  fakePreferencesReset();
  assert(config_store::saveCalibration(cal));
  wheel_speed::Gains bad;
  bad.kp = 5.0f; bad.ki = 4.0f; bad.limit = 5.0f; bad.amax = 100.0f;
  assert(config_store::saveGains(bad));
  assert(config_store::load(loaded, lg));
  assert(loaded.valid);
  assert(std::fabs(lg.kp-2.0f) < 0.001f);
  assert(std::fabs(lg.ki-1.0f) < 0.001f);
  assert(std::fabs(lg.limit-80.0f) < 0.001f);
  assert(std::fabs(lg.amax-300.0f) < 0.001f);
  assert(std::fabs(loaded.vmax-3200.0f) < 0.01f);  // 80% × 4000

  // limit > 100：同样回退。
  fakePreferencesReset();
  assert(config_store::saveCalibration(cal));
  bad.limit = 101.0f;
  assert(config_store::saveGains(bad));
  assert(config_store::load(loaded, lg));
  assert(std::fabs(lg.limit-80.0f) < 0.001f);

  // 负增益：回退。
  fakePreferencesReset();
  assert(config_store::saveCalibration(cal));
  bad.kp = -1.0f; bad.ki = 1.0f; bad.limit = 60.0f; bad.amax = 300.0f;
  assert(config_store::saveGains(bad));
  assert(config_store::load(loaded, lg));
  assert(std::fabs(lg.kp-2.0f) < 0.001f);
  assert(std::fabs(lg.limit-80.0f) < 0.001f);
}

// valid 标志只接受精确的 1；其它非零值不能启用闭环。
static void test_valid_flag_exact() {
  calibration::Data loaded;
  wheel_speed::Gains lg;
  fakePreferencesReset();
  assert(config_store::saveCalibration(makeValidCalibration()));
  fake_preferences::store().namespaces[kNamespace]["valid"] = 2;
  assert(config_store::load(loaded, lg));
  assert(!loaded.valid);
  fake_preferences::store().namespaces[kNamespace]["valid"] = 255;
  assert(config_store::load(loaded, lg));
  assert(!loaded.valid);
  fake_preferences::store().namespaces[kNamespace]["valid"] = 1;
  assert(config_store::load(loaded, lg));
  assert(loaded.valid);
}

// kp/ki/amax 的合理上限：越界整组回退默认值。
static void test_gain_upper_bounds() {
  const calibration::Data cal = makeValidCalibration();
  calibration::Data loaded;
  wheel_speed::Gains lg;
  struct BadGains { float kp, ki, amax; };
  const BadGains bads[] = {
      {10.5f, 1.0f, 300.0f},
      {2.0f, 10.5f, 300.0f},
      {2.0f, 1.0f, 1001.0f},
  };
  for (const BadGains& bad : bads) {
    fakePreferencesReset();
    assert(config_store::saveCalibration(cal));
    wheel_speed::Gains g;
    g.kp = bad.kp; g.ki = bad.ki; g.amax = bad.amax; g.limit = 80;
    assert(config_store::saveGains(g));
    assert(config_store::load(loaded, lg));
    assert(std::fabs(lg.kp - 2.0f) < 0.001f);
    assert(std::fabs(lg.ki - 1.0f) < 0.001f);
    assert(std::fabs(lg.amax - 300.0f) < 0.001f);
  }
}

// 每轮 v100/vsim 低于 200 counts/s 视为损坏记录，整条标定无效。
static void test_speed_floor_validation() {
  calibration::Data loaded;
  wheel_speed::Gains lg;

  calibration::Data cal = makeValidCalibration();
  cal.wheel[2].vsim = 150.0f;
  fakePreferencesReset();
  assert(config_store::saveCalibration(cal));
  assert(config_store::load(loaded, lg));
  assert(!loaded.valid);

  cal = makeValidCalibration();
  cal.wheel[1].v100 = 199.9f;
  fakePreferencesReset();
  assert(config_store::saveCalibration(cal));
  assert(config_store::load(loaded, lg));
  assert(!loaded.valid);

  // 200 是合法边界（标定失败阈值为 <200）。
  cal = makeValidCalibration();
  cal.wheel[0].vsim = 200.0f;
  cal.wheel[0].v100 = 200.0f;
  fakePreferencesReset();
  assert(config_store::saveCalibration(cal));
  assert(config_store::load(loaded, lg));
  assert(loaded.valid);
}

int main() {
  test_brief_scenario();
  test_save_failure_keeps_loop_disabled();
  test_key_spellings();
  test_sign_validation();
  test_gain_validation();
  test_valid_flag_exact();
  test_gain_upper_bounds();
  test_speed_floor_validation();
  return 0;
}
