#pragma once

namespace motorprofile {
// 这台车实测的电气极性，按驱动板 A/B/C/D 排列（不是 FL/FR/RL/RR）。
// 只在最底层转换一次：逻辑 +1/+百分比统一向车头，-1/-百分比统一向车尾。
// 换线或换车时在此重新标定；面板四个方向保持 +1，不能再次补偿。
// 2026-10-27 再次逐路核对：B/C 逻辑正输出朝后，故只反转 B/C，A/D 保持。
constexpr int kForwardPolarity[4] = {-1, -1, -1, -1};

inline int electricalPercent(int channel, int forwardPercent) {
  return forwardPercent * kForwardPolarity[channel];
}
}  // namespace motorprofile
