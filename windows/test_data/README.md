# 测试与训练数据索引

本目录为开发与离线评测样本，不作为独立测试集，也不覆盖实机比赛场景。原图、标注和清单同时保留；检测器不读取文件名、预期块数或标签。

| 目录 | 内容 | 标注 | 用途 |
|---|---|---|---|
| `red_train_20261110/` | 红块训练原图，含取流帧、手机照片、带车和无车场地及桌面样本 | `frames/labels.json` | 训练与验证 ONNX 红块检测器；`sources.json` 记录来源 |
| `red_photos_20261110/` | 14 张手机照片，统一为“黄上蓝下” | `frames/labels.json`（试验标注） | 手机直拍样本，补充取流帧的尺度差异 |
| `field_photos_20261103/` | 13 张带车场地实拍 `field_01`–`field_13` | `annotations.json`：人工地面四角和 255 个红块中心点 | 场地检测、tag、离线评估；非每张 20 块的固定标签 |
| `field_no_robot_20261103/` | 7 张无车场地实拍 `no_robot_01`–`no_robot_07` | `annotations.json`：140 个红块中心点 | 无车条件下的误检与墙边聚堆回归 |
| `red_blocks/` | 9 张 2026-10-22 桌面截图，含红块、车轮和只有车的负例 | README 内逐图说明 | 红块与车轮形状回归 |
| `phone_tag_blocks/` | 7 张手机俯视照片，每张一枚 tag 和一个红块 | README 内块中心坐标 | tag 检出与红块毫米筛回归 |

## 使用

- 离线检测：`python windows/detect_field_batch.py --input windows/test_data/field_photos_20261103 --output log/<本次目录>`。
- 标注与补标：`windows/标注场地照片.cmd`、`windows/标注无车场地照片.cmd`、`windows/红块框选标注.cmd`。
- 首次克隆后，在仓库根目录导出 YOLO 训练集：`python windows/export_yolo.py windows/test_data/red_train_20261110 --val-every 5`。导出完成后即可本机训练或双击 `windows/打包训练集.cmd`。
- 训练与推理见 [training/README.md](../../training/README.md)、[windows/models/README.md](../models/README.md)。

## 约束

- `field_photos_20261103/` 的 255 个点为可见红块中心，不是每张固定 20 块的完整计数；`field_no_robot_20261103/` 的 140 个点同样只覆盖可见块。
- 带车批次参与过调参，相关离线分数仅作为开发回归，不作为独立测试或实机验收依据。
- `red_train_20261110/frames/` 中的原图和标注随仓库上传。导出的 `red_train_20261110/yolo/` 由 Git 忽略，`data.yaml` 的 `path` 使用本机绝对路径。标注更新或工作区搬迁后，训练前请重新导出。
