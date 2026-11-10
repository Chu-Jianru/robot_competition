# 现用硬件与底盘

- [启动调试面板](workbench/启动调试面板.cmd) 或 [启动麦克纳姆底盘](workbench/启动麦克纳姆底盘.cmd)：原完整交付包整体移至 `workbench/`，需在 Windows 本地手动启动。
- [工作台使用说明](workbench/使用说明.md)（[HTML 版](workbench/使用说明.html)）。
- [接线表](ESP32与D24A当前接线表.xlsx)；[固件接线说明](workbench/source/motor-bringup/FOUR-MOTOR-WIRING.md)。
- [现用固件](workbench/source/motor-bringup/README.md)、[麦轮协议](workbench/source/motor-bringup/MECANUM.md)、[界面源码](workbench/source/debug-workbench/README.md)。
- [固件工作副本](firmware/README.md)：`vendor/` 中 WHEEL_SPEED_V1 的可写构建目录，已把四路极性改成本车 `-1/+1/+1/-1`，用于烧录蓝牙+轮速闭环版；`vendor/` 原件保持只读。
- 新版面板（2026-11-02，认识 `TLM`/`SPEED`/`CAL`）用 [启动新面板.cmd](../tools/启动新面板.cmd) 打开；依赖装在 `tools/panel-venv`，配置与记录写到 `tools/panel-data`，不往 `vendor/` 里写。旧版独立程序仍是 `workbench/app/RobotDebugWorkbench.exe`（2026-10-22，只认 MECANUM_V2）。

`workbench/app/` 为现有免安装程序及其运行依赖；`source/` 为开发源码；`third_party_licenses/` 保留依赖许可。内部目录和文件内容整体保留。源码修改后须重新构建才会更新 EXE。

`workbench/` 约 184 MB，只保留在本机，已在 `.gitignore` 排除；从仓库 clone 后没有这个目录，上面的工作台链接会失效，但 [固件工作副本](firmware/README.md) 和接线表仍在，可正常查看/烧录。需要工作台时从本机 `E:\robot_competition\hardware\workbench` 复制，或向队友索取封存包。

`source/esp32-pc-link/` 中的共享协议核心与界面模块仍被现用工程引用，不能整目录删除。交付包内的相同说明、启动脚本及发布模板是打包输入/输出，有清单约束，保留完整。

队友 2026-11-02 的源码包（含轮速闭环）在 [vendor](../vendor/README.md)，本目录与它的逐项对应、同名文件差异都记在那份说明里。本目录是 2026-10-22 封存交付包，两者轮位（ADBC/CBAD）与电气极性不同，**不要混用**。

旧 UDP 草稿已于 2026-11-03 删除（引脚与现用接线冲突、不可烧录，且无人调用）。当前状态与后续验收统一见 [交接说明](../交接说明.md)。本次整理没有启动程序或设备。
