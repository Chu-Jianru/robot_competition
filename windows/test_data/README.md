# 测试与训练数据索引

这些是开发和离线评测用的样本，不是独立测试集，也不代表实机比赛场景已覆盖。原图、标注和清单同时保留；检测器不读取文件名、预期块数或标签。

| 目录 | 内容 | 标注 | 用途 |
|---|---|---|---|
| `red_train_20261110/` | 74 张红块训练帧（33 取流帧 + 14 手机照片 + 13 带车场地 + 7 无车场地 + 7 桌面），`yolo/` 已按 61/13 划分 | `frames/labels.json`，1326 个框，全部人工确认 | 训练/验证 ONNX 红块检测器；`sources.json` 记来源 |
| `red_photos_20261110/` | 14 张手机照片，统一成“黄上蓝下” | `frames/labels.json`（试验标注） | 手机直拍样本，补充取流帧的尺度差异 |
| `field_photos_20261103/` | 13 张带车场地实拍 `field_01`–`field_13` | `annotations.json`：人工地面四角和 255 个红块中心点 | 场地检测、tag、离线评估；**不是** 20 块/张的固定标签 |
| `field_no_robot_20261103/` | 7 张无车场地实拍 `no_robot_01`–`no_robot_07` | `annotations.json`：140 个红块中心点 | 无车条件下的误检/墙边聚堆回归 |
| `red_blocks/` | 9 张 2026-10-22 桌面截图，含红块、车轮和只有车的负例 | README 内逐图说明 | 红块/车轮形状回归 |
| `phone_tag_blocks/` | 7 张手机俯视照片，每张一枚 tag + 一个红块 | README 内块中心坐标 | tag 检出与红块毫米筛回归 |

## 怎么用

- 只跑离线检测：`python windows/detect_field_batch.py --input windows/test_data/field_photos_20261103 --output log/<本次目录>`。
- 看标注/补标：`windows/标注场地照片.cmd`、`windows/标注无车场地照片.cmd`、`windows/红块框选标注.cmd`。
- 重新导出 YOLO：`python windows/export_yolo.py windows/test_data/red_train_20261110 --val-every 5`。
- 训练与推理：见 [training/README.md](../../training/README.md)、[windows/models/README.md](../models/README.md)。

## 注意

- `field_photos_20261103/` 的 255 个点是“可见红块中心”，不是每张固定 20 块的完整计数；`field_no_robot_20261103/` 的 140 个点同样只覆盖可见块。
- 带车批次参与过调参，相关离线分数只能当开发回归，不能当独立测试或实机验收。
- 这些图片约 48 MB，随仓库上传；不要再用 `windows/captures/` 里的同名旧图覆盖它们，重复原图以本目录为准。
- `red_train_20261110/yolo/data.yaml` 的 `path` 是本机绝对路径（`E:/robot_competition/...`）。换机器或换目录后，先重跑 `python windows/export_yolo.py windows/test_data/red_train_20261110 --val-every 5` 再训练。
