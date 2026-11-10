#pragma once

namespace motorprofile {
// 这台车实测的电气极性，按驱动板 A/B/C/D 排列（不是 FL/FR/RL/RR）。
// 只在最底层转换一次：逻辑 +1/+百分比统一向车头，-1/-百分比统一向车尾。
// 换线或换车时在此重新标定；面板四个方向保持 +1，不能再次补偿。
//
// 2026-11-02 移植到本车：值取自 hardware/workbench（MECANUM_V2，已实车跑通前进/横移/转向）。
// 队友原包是 {-1,-1,-1,-1}，对应他自己那台车；本车 B/C 接线不同，保持 -1/+1/+1/-1，
// 否则 B/C 两轮会反向。轮位仍用运行期命令 LAYOUT ADBC 1 1 1 1，不写进固件。
constexpr int kForwardPolarity[4] = {-1, 1, 1, -1};

inline int electricalPercent(int channel, int forwardPercent) {
  return forwardPercent * kForwardPolarity[channel];
}
}  // namespace motorprofile
