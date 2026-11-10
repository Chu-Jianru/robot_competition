#pragma once
#include <stdint.h>

namespace mecanum {
// 标准 X 型、45 度滚轮；顺序为左前、右前、左后、右后。
// x=前进，y=向左，z=逆时针，均为归一化输出分量，不是 mm/s 或 rad/s。
// 所有轮子逻辑 +1 都表示向前；实际电气极性仅由 motor_profile.h 的底层驱动修正。
// Layout.sign 为库的数学选项；MECANUM_V2 协议固定为全 +1，避免重复取反。
struct Layout {
  int channel[4] = {0, 1, 2, 3};
  int sign[4] = {1, 1, 1, 1};
};

inline bool valid(const Layout& layout) {
  bool used[4] = {};
  for (int i = 0; i < 4; ++i) {
    if (layout.channel[i] < 0 || layout.channel[i] > 3 || used[layout.channel[i]] ||
        (layout.sign[i] != 1 && layout.sign[i] != -1)) return false;
    used[layout.channel[i]] = true;
  }
  return true;
}

inline void mix(int x, int y, int z, const Layout& layout, int output[4]) {
  const int wheel[4] = {x - y - z, x + y + z, x + y - z, x - y + z};
  int peak = 100;
  for (int value : wheel) {
    const int magnitude = value < 0 ? -value : value;
    if (magnitude > peak) peak = magnitude;
  }
  // 同比例缩放，避免对每轮分别截断后改变合成运动方向。
  for (int i = 0; i < 4; ++i) {
    const int scaled = (wheel[i] * 100 + (wheel[i] >= 0 ? peak / 2 : -peak / 2)) / peak;
    output[layout.channel[i]] = scaled * layout.sign[i];
  }
}
}  // namespace mecanum
