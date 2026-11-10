# Windows 视觉客户端

运行方式和视觉模块说明集中维护在本文；统一状态见 [交接说明](../交接说明.md)。

日常复制执行的 PowerShell 命令见 [常用命令速查](../docs/常用命令.md)，包含取图、tag 自检、机位标定、两批照片标注、最新版离线识别/评估和底盘工具入口。

最简入口：双击 [日常工具.cmd](../tools/日常工具.cmd)，按数字选择拍图、tag、四角标定、定位、标注或查看结果，不用手输 Python 路径。

## Windows 自主清扫入口

插线、首次配置和日常运行的完整步骤见 [手机接入与自主清扫手册](../docs/手机接入与自主清扫手册.md)。[手机接入流程.cmd](../tools/手机接入流程.cmd) 会先检查 USB，再提供对应步骤。

首次打开 [日常工具](../tools/日常工具.cmd)，选 **1** 拍全场原图，再选 **4** 点地面四角。保持黄区在画面上、蓝区在下，使用相同机位、旋转和 1280 档取流。移动手机、变焦或改变分辨率后重新标定。

选 **11** 保存本车参数。输入蓝牙传出串口、实际确认的 FL/FR/RL/RR 轮位、前进与左移的毫米换算、逆时针的角度换算，以及实测可控的单轮输出上限。COM6 和 CBAD 来自上次连接和设备回读，实际轮位以本车接线和运动为准。换算使用各轴 **100 ms** 开环短脉冲的实测数据。将停稳后的位移（mm）或转角（°）除以指令百分数和 0.10 s。参数存入 `windows/task_drive.json`。设备的 `CAL VALID=1` 属于编码器轮速标定，无法替代车体运动换算。

之后接上手机、给底盘上电并释放底盘面板占用的串口。双击 [启动自主清扫.cmd](../tools/启动自主清扫.cmd) 或选日常工具 **10**。车放蓝区起跑位置，车头朝黄区。窗口显示 **SPACE to start** 后按空格，开始 90 秒任务。**Q / Esc** 或关闭窗口退出并请求停车。日常工具 **12** 可用相同取景运行策略预览，预览无需底盘配置。

程序从同一帧提取 ID 7 位姿、红块与障碍。它根据实拍颜色定位蓝黄分界，并用红块周围地面颜色判断归属。程序复用整排和局部清理策略，经蓝牙发出 100 ms 定时动作，等待停车和冷却后继续。**单帧 tag 丢失或分界缺失不再立刻结束本轮**：0.35 s 内沿用最后一帧有效场景，超过 0.35 s、位姿跳变、固件报错或比赛到时才结束并停车。红块经过合并/确认/平滑后再交给策略（`--no-track` 可关闭）。

首次实车使用还需核对车体外形与策略模型、现场取流、停车和推块效果。定时动作的冷却与开环驱动会限制速度和精度，90 秒清扫能力以实车为准。

## 场地照片离线分割（2026-11-03）

新增无小车 7 张照片在 [第二批样本目录](test_data/field_no_robot_20261103/README.md)，首轮冻结算法未调参结果在 `log/field_no_robot_20261103/frozen_detector/`。使用 [查看无车识别结果.cmd](查看无车识别结果.cmd) 查看，[标注无车场地照片.cmd](标注无车场地照片.cmd) 单独保存新批次标注。无标注通用推理用 `detect_field_batch.py --input ... --output ...`；该脚本不依据用户给出的总数量调整检测结果。

第二批标注完成后修复大聚堆整团被拒和蓝地面反射造成的偏紫漏检，最新查看入口打开 `optimized_v2/` 纯检测图。相对 140 保存点，首轮 118 匹配/22 漏/5 多余，修复后 135 匹配/5 漏/6 多余；01 因裁切和角点交叉继续按自动尺度只作像素对照，02–07 人工四角子集 115/120 匹配、2 多余。第一批回归为 249/255 匹配、9 多余，76 项离线检查通过。已保存候选的逐块评估用 `evaluate_field_batch.py --predictions ... --input ... --output ...`，冻结报告和新报告分目录保存，详见第二批说明；实验仍未接入实时相机或控制。

13 张用户标注实拍的实验入口为 [optimize_field_photos.py](optimize_field_photos.py)，执行 `python windows/optimize_field_photos.py`，或用 scrcpy 目录的现有 Python 环境执行同一脚本。读取照片与四角标定，将红块中心标签只用于评估；检测器不读取文件名、预期块数或红块标注。结果见 `log/field_photos_20261103/optimized/`，双击 [查看场地识别结果.cmd](查看场地识别结果.cmd) 打开纯检测总览。

[calibrated_blocks.py](calibrated_blocks.py) 用 HSV、按场地尺度的距离峰与面积估计拆分、绿色 PCB 和白色底盘区域排除车轮，并保守处理墙边。自主清扫入口使用该红块分割。`photo_tag.py` 的曝光补检仍用于离线照片。原算法基线可用 `analyze_field_photos.py` 和 `evaluate_field_labels.py` 重跑。

本批已保存标注的一对一匹配从 114/255 提升为 248/255，多余候选由 10 降为 9；tag 由 10/13 提升至 11/13。该批照片参与调参、使用人工四角、部分分割数量来自尺寸推断，仍需新照片验证；详情与匹配口径见 [样本说明](test_data/field_photos_20261103/README.md)。

## Windows USB 预览

完整 scrcpy 工具与已有 Python 环境保留在 `E:\scrcpy\scrcpy-win64-v5.0.1`。先关闭其他摄像头客户端，再执行：

```powershell
Set-Location 'E:\robot_competition\windows'
& 'E:\scrcpy\scrcpy-win64-v5.0.1\.venv\Scripts\python.exe' .\usb_camera.py --scrcpy-dir 'E:\scrcpy\scrcpy-win64-v5.0.1' --rotate 90
```

红块检测默认开启，不需要按F。B独立开关红块检测，F独立开关场地/车体定位；两者互不影响。启动时加 `--no-blocks` 可关闭红块检测，加 `--field` 可开启场地定位。

R旋转，C清除场地锁定，Q/Esc退出。S在 `windows/captures/` 同时保存同一帧的 `*_raw.png` 原图与 `*_annotated.png` 标注图，包含当前已开启模块的结果。关闭F后仍可保存红块边框。两张均为640×480，可能带黑边，不是传感器原分辨率。

先固定相机高度、角度与光照。K进入红块校准，拖框选择**真实红块顶面内部的红色区域**，避开灰色背景、阴影侧面和车轮，按Enter确认（Esc取消）。配置自动保存到 `windows/red_block_profile.json`，下次启动自动加载；`--red-profile` 可指定另一份配置。相机远近或光照明显变化后重新按K校准。

目前已用用户第一张原图中的真块顶面生成初始配置。校准学习实物饱和度、亮度范围及较宽松的像素面积下限，修补小缺口后再做形状筛选；没有额外色标或场地依赖。新增六张原图与原三张合计九张复核通过：八张含块图均仅检出真实红块，只有小车的一张检出0个。新版实时手机及K选框交互仍需重启后验收。

用户已提供三张桌面实拍；Windows预览已修正数字8形场地绿框，并用蓝黄地面占比检查拒绝假场地。红块模块独立按颜色及近方形轮廓筛选，显示绿色 `block?` 像素框，场地关闭或无效时仍正常检测；有效场地只为候选额外提供世界坐标与蓝黄归属。本轮三张实拍分别保留1、1、0个候选，排除了这些照片中的车轮误检。正式比赛还需完整蓝黄场地与可靠车体位姿；现有算法使用前品红、后青色标记，但用户暂不能贴标，无色标定位尚待实现与验收。

预览仍最多10Hz，尚未实施延迟优化；该入口不发送底盘速度。本次更新通过离线照片和测试检查，更新后的实时手机预览尚未验收。

## Windows 视觉模块

- [red_blocks.py](red_blocks.py)：独立红色及形状检测，仅依赖OpenCV，输入BGR图，输出像素中心、边框、面积、轮廓及形状指标；`RedBlockConfig` 单独配置颜色、像素面积、旋转矩形长宽比、矩形填充率与轮廓实心度。
- [field_vision.py](field_vision.py)：场地几何、蓝黄分界、车顶色标与障碍，不提取或统计红块。
- [scene_vision.py](scene_vision.py)：组合两路结果；只有有效场地才调用 `locate_red_blocks` 换算世界坐标与己方归属，像素候选保持不变。
- [usb_camera.py](usb_camera.py)：取图、独立开关、绘制与截图。

其他脚本可直接调用，不必初始化场地：

```python
from red_blocks import detect_red_blocks, load_red_config

config = load_red_config(r"E:\robot_competition\windows\red_block_profile.json")
detections = detect_red_blocks(bgr_image, config)
# 每项含 center_px、bbox_px、area_px、contour_px，没有场地坐标或己方标志。
```

没有配置文件时，基础形状阈值仍为短边/长边至少0.65、矩形填充率至少0.80、实心度至少0.90。使用实物校准配置后，增加饱和度上限、亮度下限和参考块面积下限，先做小缺口闭运算，形状阈值放宽到0.65/0.72/0.80，以减少光斑、暗侧面造成的漏检。筛选不使用物体在画面中的固定位置。

校准只适用于相应相机尺度和光照；粘连成条、严重遮挡、大角度侧视及外观相同的零件仍需另验。`RedBlockConfig(shape_filter=False)` 保留仅颜色诊断入口。没有实现车体遮罩或多帧跟踪，不能宣称所有车轮与物块均已可靠区分。

Windows场地模块已拆分，不再与 `src/sweep_robot/sweep_robot/field_vision.py` 逐字节一致；ROS正式模块和h15仿真源码未修改。本轮18项离线测试通过，九张原图在场地开关前后的结果一致。原图回归样本保存在 `windows/test_data/red_blocks/`，最新复核结果见 `log/red_calibration_20261022/`；上轮形状筛选证据保留在 `log/red_shape_20261022/`。

## 车顶定位 tag

车体位姿改用 ArUco 标记，替代现有品红/青色双色标（后者依赖颜色阈值，与红块/蓝黄地面互相干扰）。

- 当前编号：**ID 7**（DICT_4X4_50，定义在 `tag_detector.TAG_ID`）。台架、位姿、任务、诊断入口的 `--tag-id`/`--id` 默认值都取它；换码只改这一处，再重跑生成脚本重打图纸。

- 图纸：[aruco_4x4_50_50mm_A4.pdf](tags/aruco_4x4_50_50mm_A4.pdf)（一页 6 枚，**全部为 ID 7**，只有一枚贴车，其余是备用件）。生成脚本 [make_tags.py](tags/make_tags.py) 从 `tag_detector.TAG_ID` 取编号，改尺寸或 ID 后重跑即可。车上不要同时贴两枚同 ID，否则会检出两个位姿。
- 规格：**DICT_4X4_50**，黑色方块 **50.0 mm**，四边留白 12.5 mm（1 模块），600 dpi。50 mm 在 640×480 全场视野下只有 21–27 px（每模块 2.6–3.3 px），合成探针显示模糊 σ≈1 px 时检出率会掉到 42%，因此**不再用 640×480 直接认 tag**。
- 打印：选"实际大小 / 100%"（禁用"适应页面"）；用卡尺核对页脚 100 mm 标尺与黑方块边长，偏差 >0.5 mm 说明被缩放，重打。
- 安装：车顶前部居中、水平、哑光；tag 的上边朝车头（ID 位自带方向，不必再做不对称标记）。先只装一枚。
- 分辨率方案（2026-11-02 定）：车顶只能放 5 cm，改走**高分辨率取图 + 透视校正**。手机给 1920 宽流，按场地四角校正到 960×960（1.25 mm/px），tag 变 40 px、20 mm 红块变 16 px；合成端到端自检位置误差 P50 0.39 mm / P95 0.66 mm，朝向 P50 0.85° / P95 1.66°，15/15 检出。
- 解算：场地四角做单应 → tag 四角映射到场地毫米坐标 → 重心为位置、tag 上边方向为 heading；不需要相机内参。三道自检：四角边长与标称偏差 >15% 丢帧、连续两帧位置差 <10 mm 且 heading 差 <3° 才锁定、丢失即停车（沿用 0.35 s 场景过期规则，禁止轮速外推）。
- 验收门槛：检出率 ≥95%、位置 ≤2 mm、**heading 抖动 ≤2°**。
- 红块毫米筛选：[red_blocks.filter_by_size_mm](red_blocks.py) 把 √面积 折算成等效边长，默认 **12–32 mm** 之外直接剔除（过小=反光点/木屑，过大=粘连成团）。像素面积阈值在 640 与 1280 下含义不同，毫米阈值不随分辨率变；`min_area_px` 只留作粗筛。台架模式控制台显示 `blocks=N(xM of K)`，M 是剔除数。

### tag 链路脚本

| 脚本 | 作用 |
|---|---|
| [capture_hi.py](capture_hi.py) | 按 `--max-size`（默认 1280）取流并存**原始分辨率** PNG；不做检测、不发指令 |
| [field_view.py](field_view.py) | 点场地四角存标定（`calibrate`），再用标定预览校正效果（`preview`） |
| [pose_tag.py](pose_tag.py) | 单张图测 tag 位姿：位置、heading、边长自检 |
| [check_pose.py](check_pose.py) | 整目录批量跑，出检出率、误差与多帧抖动；可对 `truth.csv` 比真值 |
| [selftest_pose.py](selftest_pose.py) | 纯合成端到端自检，不连相机 |
| [bench_check.py](bench_check.py) | 无场地台架自检：用 tag 的 50 mm 边长自换算 mm/px，同时报红块数量与等效边长 |
| [live_bench.py](live_bench.py) | 连拍 N 帧的台架统计：tag 检出率、mm/px、heading 抖动、红块数与等效边长、单帧耗时 |
| [record_diag.py](record_diag.py) | 连续帧诊断：逐帧记录 tag/分界线/候选/拆分与块坐标，汇总整帧作废比例、块数变化和同块抖动 |
| [block_tracks.py](block_tracks.py) | 块级合并/确认/位置平滑：抑制单帧鬼影与短时漏检，不改检测阈值 |
| [collect_red_frames.py](collect_red_frames.py) | 从手机流按间隔截帧做训练集，自动跳过重复帧、记录红色像素数 |
| [import_photos.py](import_photos.py) | 把手机照片收进数据集：按蓝黄方向统一旋转成“黄上蓝下”、改名、去重、写清单 |
| [merge_dataset.py](merge_dataset.py) | 把另一批图片并进主训练集：复制、改名、合并标注、记录来源 |
| [import_points.py](import_points.py) | 把只有“中心点”标注的老批次并入：用人工四角算尺度，按 20 mm 边长扩成框 |
| [evaluate_detector.py](evaluate_detector.py) | 拿人工框当标准答案，比较 HSV 与 ONNX 的 precision/recall/F1 和单帧耗时 |
| [onnx_blocks.py](onnx_blocks.py) | 加载 `.onnx` 红块模型，输出结构与 HSV 检测器一致，供 `run_task --detector onnx` 使用 |
| [models/](models/README.md) | 放 `red_block.onnx`；含比较与实跑命令 |
| [label_boxes.py](label_boxes.py) | 矩形框标注器：逐帧框选红块，存 `labels.json`，不改原图 |
| [export_yolo.py](export_yolo.py) | 把框标注导出成 YOLO 目录与 `data.yaml`，按文件名稳定划分训练/验证 |

流程：`capture_hi` 拍图 → `field_view calibrate` 标定一次 → `check_pose` 批量验收。相机位置一旦改动必须重新标定。没有场地时先用 `bench_check` 验 tag 与红块。

### 无白边 tag 的软件补充检测（2026-11-03）

`tag_detector.py` 统一供 `bench_check.find_tag` 与 `pose_tag.detect` 使用，实时
`capture_hi --bench`、`live_bench`、场地位姿检测均自动生效，无须新增启动参数。
原标准 ArUco 检测优先；找到指定 ID 时直接使用其亚像素角点，不改已有白边码的检测结果。

标准检测找不到指定 ID 时，补充路径从原图的亮色连通区域与该 ID 的内部图案匹配，
提出外框候选，再仅在候选外侧加临时白边解码。不会补画黑框或码内数据。
补充解码禁止纠错，并要求全部 16 个数据格及 20 个黑边格一致、黑白对比足够、
内部图案 ECC 对齐成功；对齐后的角点映射回原始像素坐标。候选数量有上限，
不沿用前帧结果，不改变场景过期停车或边长毫米检查。补充路径需要指定 ID，
仅在内部连通图案足够约束几何时可用；亮格铺满 4×4 数据区的 ID（0、7）六种角度都能补检，
亮格稀疏的码（如 ID 30）只覆盖部分角度。

离线验证：保存原图 `captures_hi/hi_020725_700_720x960.png` 修改前未检出，
修改后检出 ID 0（当时车上的旧码，约 38.6 px），毫米筛保留 1 个红块（等效边长约 19.6 mm）。
同图四种 90° 旋转与三种亮度增益共 12 种变换均检出；这些是同一照片派生样本，
不是 12 次实机采集。合成无白边六种角度（ID 0）的最大角点误差均 <0.75 px；
合成白边六种角度的角点与原检测器逐值相同。测试还覆盖其他 ID、遮挡、随机噪声
与九张既有无 tag 照片；运行 `python windows/test_tag_detector.py -v`。

2026-11-10 换码后补充：亮色块的**最小外接矩形**会随图案变形（ID 7 在 70°
时退化成 37.6×25.9），原候选框因此失效。现在同时用亮色块拟合“4×4 数据区的最小
外接正方形”再放大 1.5 倍作为候选；数据区外接矩形路径保留。
解码、ECC 对齐、36 格校验和候选上限都没改，720×960 原图的补充路径耗时仍约 27 ms。
合成 0°/20°/70°/130°/230°/310° 六种角度均能补检出 ID 7，角点误差 <0.75 px。
补充路径是**图案相关**的：亮格稀疏的 ID 30 只覆盖部分角度（当天试过又改回 7）；
实机连续检出率未验收。

单独离线重复处理该 720×960 图的检测耗时 P50 约 27 ms；放大到 1440×1920
的同图约 49 ms（含标准检测与补充路径，不含 USB 传输、解码和显示）。
只有一张真实 tag 原图，无实测坐标真值，不能据此宣称实机毫米精度或连续检出率达标。
强透视、反光、遮挡、码格太小仍可能漏检；补白边后通常更容易走标准检测路径，
但真实定位精度和稳定性仍须重新实拍验收。

### 没有颜色场地时的实机自检

2026-11-03 七张照片补充：原 `E:\robot_competition\？\1.jpg` 至 `7.jpg` 已按用户要求
移至 [phone_tag_blocks 样本目录](test_data/phone_tag_blocks/README.md)，顺序命名为
`phone_tag_block_01.jpg` 至 `07.jpg`，逐文件 SHA256 与原件一致。
新增多尺度 tag 检测：高分辨率原图标准检测失败后，在长边 960 的图上提出候选，
映回原图做全部码格校验及 ECC 对齐，图 04/05 现均可检出 ID 0（当时车上的旧码）。
原图已有标准检出时仍直接使用原结果。

红块配置新增可选 `shadow_min_value`（当前配置为 40），在原颜色检测之外补充暗光路径。
暗光路径沿用原 hue/saturation 范围，使用更严格的长宽比 ≥0.80、矩形填充率 ≥0.85、
实心度 ≥0.90，并合并同一物块的亮区/暗区重复框；不按文件名、位置或最大面积选块。
最终仍走 12–32 mm 的尺寸筛选。单纯把主亮度阈值降低会让历史车轮图误检，故保留
原主路径阈值。未配置 `shadow_min_value` 的自定义配置仍使用原单路颜色检测。

当前七张原图及其 90° 旋转样本均检出一枚 ID 0（旧码）和一个标注位置的地面红块；
七张原图的红块等效边长约 18.4–21.2 mm，仅是按 50 mm tag 推算的筛选量。
60 项 Windows 离线测试通过；批量入口也已改用支持 Unicode 路径的读图方式。
单张高分辨率图的 tag+红块检测约 50–125 ms（一次离线扫描，不含采集与显示），
未实测实时连续检出率。重新标定仍需固定镜头、取景和照明，实物定位精度尚未验收。

实时看画面并按 S 存图（推荐先跑这个，确认取景和光照）：

```powershell
Set-Location 'E:\robot_competition'
& 'E:\scrcpy\scrcpy-win64-v5.0.1\.venv\Scripts\python.exe' .\windows\capture_hi.py `
  --scrcpy-dir 'E:\scrcpy\scrcpy-win64-v5.0.1' --rotate 90 --max-size 1280 --bench
```

控制台每秒一行 `tag xx.xpx -> 0.xxx mm/px | blocks=N(xM of K) side=xx.xmm [a-b] d~xxxmm`；`blocks` 是毫米筛后的数量，`xM of K` 里的 M 是被剔除的候选。S 会同时存原始图和标注图到 `windows/captures_hi/`。

无人值守跑 60 帧出统计（不算 GUI，退出码只反映相机是否启动成功）：

```powershell
& 'E:\scrcpy\scrcpy-win64-v5.0.1\.venv\Scripts\python.exe' .\windows\live_bench.py `
  --scrcpy-dir 'E:\scrcpy\scrcpy-win64-v5.0.1' --frames 60
```

它输出 tag 检出率（门槛默认 95%）、边长 px、mm/px、图像朝向圆周标准差、中心抖动、红块数与等效边长的中位/分位、每帧耗时，并写 `frames.csv` 和 `best_annotated.png` 到 `windows/captures_hi/live_<时间>/`。画面方向不对时加 `--rotate 0/90/180/270`。这台 Mate60 Pro 在 1920 档实测停帧，菜单及任务使用 1280 档。

判读：tag 应报出 `id=7`；`side` 反推的红块等效边长应接近 **20 mm**（12–32 mm 之外会被当碎块/粘连剔除）；`mm/px` 与手机高度成反比（5 cm tag 占 500 px 时约 0.1 mm/px）。tag 检出 0 但红块正常，多半是打印尺寸/反光/离得太远；两者都 0，先查镜头是否真的对着桌面。

### 块数为什么跳：连续帧诊断（2026-11-10，不改检测参数）

[record_diag.py](record_diag.py) 连拍一段并把**每一帧的判决过程**写下来，用来区分"整帧作废"和"单帧判决抖"这两类不同的跳变。不需要任何标注：

```powershell
& 'E:\scrcpy\scrcpy-win64-v5.0.1\.venv\Scripts\python.exe' .\windows\record_diag.py `
  --scrcpy-dir 'E:\scrcpy\scrcpy-win64-v5.0.1' --secs 15 --save-frames
```

- 没有 `windows/field_calib.json` 时自动走**台架模式**（tag 50 mm 换算 mm/px，与 `capture_hi --bench` 同一调用序列）；有标定时走**场地模式**，与 `run_task.observe` 同一调用序列，不另写一套判定。
- 每帧记录：tag 是否解码、蓝黄分界线是否找到、红候选连通域数、当帧单块面积估计、拆分实例数、每个块坐标，写 `frames.csv`；`--save-frames` 另存 JPEG 供回放。
- `summary.json` 直接给判读数：整帧作废比例、块数 0 的帧占比、相邻帧块数变化（中位/P90）、同块位置抖动（中位/P90 mm）、失效原因分布。
- 只读画面，不发底盘指令。默认不开窗口；`--show` 才显示，且会计入每帧耗时。

判读顺序：先看 `整帧可用帧` 比例，低则修 tag/分界线丢失后的降级；再看块数变化与抖动，大则问题在单帧判决（面积估计、拆分、阈值），与标注数量无关。本轮只做了离线单元检查和命令行检查，未连手机实跑。

### 块级稳定层（2026-11-10，默认开启）

[block_tracks.py](block_tracks.py) 在红块列表交给策略之前做三件事：

1. **最小间距合并**：两个 2 cm 块的圆心不可能小于 20 mm，所以相距 <12 mm 的检测判为同一个块被拆分，直接合并，没有时间延迟。
2. **确认**：新块连续出现 2 帧才报告，单帧鬼影不会进入策略；冷启动首帧直接接受，不白等一帧。
3. **平滑与短时保持**：每个块的位置做指数平均；跟踪丢失后最多保留 3 帧，单帧漏检不会清空列表。

它不改检测阈值，也不改策略。任务入口默认开启，`--no-track` 回到逐帧原始列表；`record_diag.py --track` 会在同一段录像里同时记原始块数和稳定后的块数，便于对照。

离线验证（同一张照片做 ±1 px 平移、±1.5% 亮度，12 帧，跳过 3 帧预热）：5 张照片的相邻帧块数变化由 **0.5–1.5 降到 0**；人为注入的单帧鬼影和单帧漏块都被吸收，块数保持 20。**它不能修正系统性多检**：`field_11` 真值 20、稳定后仍是 24，那是单块面积估计把一堆拆得太细，属于检测器调参，不是稳定层能解决的。

### 红块检测训练集（2026-11-10 起）

现有 HSV 在聚堆、反光和遮挡下计数仍不稳，用户决定同时走学习式路线，先做训练集试验。这条路线此前被“只用 HSV、不训练”的约定挡住，现在按用户明确要求开工；**验证结果出来之前，比赛默认链路仍是 HSV**，两者互不影响。

**0. 收拢手机直接拍的照片**（可选，和取流截帧可混用）：

```powershell
& 'E:\scrcpy\scrcpy-win64-v5.0.1\.venv\Scripts\python.exe' .\windows\import_photos.py `
  --src 111 --out windows\test_data\red_photos_20261110
```

手机相册里同一场地可能是竖拍（蓝左黄右）或横拍（黄上蓝下），模型在实机看到的是**黄上蓝下**（`find_seam` 的前提）。这个脚本按蓝黄重心自动旋转到该方向、重命名成 `photo_001.jpg` 起、跳过重复照片，并写 `manifest.json` 记录原文件名与旋转角度；`111` 里的原图不动。

**0.5 并进主训练集**（想只对一个目录标注/导出时）：

```powershell
& 'E:\scrcpy\scrcpy-win64-v5.0.1\.venv\Scripts\python.exe' .\windows\merge_dataset.py `
  --target windows\test_data\red_train_20261110 --source windows\test_data\red_photos_20261110 --prefix photo
```

图片复制进目标 `frames/`、新文件自动预填框、`labels.json` 合并，并在 `sources.json` 记下来源路径；源目录保留。之后标注和导出都只对 `red_train_20261110` 一个目录。

老批次只有“红块中心点”，没有框。用 `import_points.py` 并进来：它按该图的人工四角求局部尺度，再按 20 mm 边长（留 25% 余量）把点扩成框，并随画面一起旋转到“黄上蓝下”。这类框是几何推算的起点，合并后一律 `reviewed=false`，仍需人工确认。

```powershell
& 'E:\scrcpy\scrcpy-win64-v5.0.1\.venv\Scripts\python.exe' .\windows\import_points.py `
  --batch windows\test_data\field_no_robot_20261103 --target windows\test_data\red_train_20261110 --prefix norobot
```

**1. 采帧**（固定机位；静态场地用**手动模式**：摆一批按一次 `S`）：

```powershell
& 'E:\scrcpy\scrcpy-win64-v5.0.1\.venv\Scripts\python.exe' .\windows\collect_red_frames.py `
  --scrcpy-dir 'E:\scrcpy\scrcpy-win64-v5.0.1' --manual `
  --out windows\test_data\red_train_20261110
```

- `--manual`：预览窗口里 `S` 存一帧、`Q` 退出。摆一批块按一次，换光照/换堆法/挪车后再按，比定时抓帧有效得多（静态场景定时抓会全是重复帧）。
- 定时模式仍可用：去掉 `--manual`，给 `--interval 0.7 --seconds 180`，用于有持续变化的场景；会自动跳过几乎重复的帧。
- 每段单独目录；`manifest.json` 记录每帧红色像素数与参数。
- **保存时自动预填框**：用现有 HSV 检测器给这帧写出初始框到 `frames\labels.json`，标注时只需删误检、补漏检；`--no-prelabel` 关闭。已有旧帧可用 `label_boxes.py --prefill` 补。
- 建议覆盖：正常光照、偏暗、偏亮/反光、整排聚堆、墙边聚堆、车身遮挡、空场与只有车轮/红色零件的负样本。
- `--min-red-px` 可只保留含红色的帧；采负样本时不要设这个门槛。
- **朝向自动判断**：`--rotate auto`（默认）从蓝黄地面的重心关系算出该转多少度，把画面统一成“黄上蓝下”，手机横放竖放都不用手工试。判不出来（画面里没有蓝黄地面）时会报错退出，而不是瞎转。实跑 `run_task` 也用同一个 `auto`，但它会额外核对旋转后的尺寸与 `field_calib.json` 是否一致，不一致就要求按判出的角度重新标定。
- 双击 [红块采帧.cmd](红块采帧.cmd) 等价于 `--manual --rotate auto`，不用记参数；把目标目录拖上去可换输出位置。

**2. 框选标注**（双击 [红块框选标注.cmd](红块框选标注.cmd) 也行，默认打开导入的照片目录；把别的 `frames` 文件夹拖到它上面即可标那一段）：

```powershell
& 'E:\scrcpy\scrcpy-win64-v5.0.1\.venv\Scripts\python.exe' .\windows\label_boxes.py `
  windows\test_data\red_photos_20261110\frames
```

- 左键拖框新增，右键点在框内删除，`Z` 撤销、`D` 清空、`N`/空格/`V`/回车 **确认并下一张**、`U` 取消确认、`P` 上一张、`S` 保存、`Q` 退出；确认时自动保存。
- **窗口按屏幕自动铺满**，`+`/`-` 可再缩放；拖框坐标按缩放比例换算回原图像素，框的位置与鼠标一致。
- 只框**看得见的红块**；聚堆逐块框；被挡住看不清个数的不要猜。没有红块的帧直接确认，它就是负样本。
- 结果写同目录 `labels.json`（原图像素坐标），原图不动；未确认的帧不会被导出。

**3. 导出训练格式**：

```powershell
& 'E:\scrcpy\scrcpy-win64-v5.0.1\.venv\Scripts\python.exe' .\windows\export_yolo.py `
  windows\test_data\red_train_20261110 --val-every 5
```

生成 `yolo/images|labels/{train,val}` 与 `data.yaml`（单类 `red_block`）。按文件名取模划分，相邻帧不会横跨训练/验证，避免用几乎重复的画面虚高验证分数。

**规模建议**：先做 200–300 帧试验集（约 4000–6000 个框），验证学习式在这个尺度（校正后 16 px、原图约 40 px）上能不能真的超过 HSV，再决定是否扩到两千帧。本机有 **RTX 4060 Laptop 8 GB**，可直接本地训练（双击 [训练红块模型.cmd](训练红块模型.cmd)，首次自动装 CUDA 版 PyTorch）；没有 GPU 的机器再用 [training/colab_train.ipynb](../training/colab_train.ipynb)。

**4. 打包与训练**：双击 [打包训练集.cmd](打包训练集.cmd) 生成 `training\red_train_yolo.zip`，按 [training/colab_train.ipynb](../training/colab_train.ipynb) 在 Colab 上训练（`imgsz=1280`，红块只有 9–25 px）。

**5. 回本机验延迟**：`python windows\detect_onnx.py --model windows\models\red_block.onnx --frames <目录> --limit 20`（需 `pip install onnxruntime`）。单帧 < 100 ms 才能 10 Hz；超时就先试 960 或改为 4 Hz 检测 + 现有跟踪层。

## 视觉伺服点动（导航联调第一步）

现用固件只有 `MECANUM <forward> <left> <ccw> <ms>` 定时开环动作（运行中拒绝、停后约 1 s 冷却），所以这一层做成“发一个短脉冲 → 停 → 重新测位姿”的**点动伺服**，1–2 s 一拍；不是 10–20 Hz 连续闭环，后者要等固件增加流式目标接口。

- [chassis_link.py](chassis_link.py)：串口链路。按固件协议握手（`WHO` → `MODE CHASSIS` → `CAPS` → `LAYOUT ADBC 1 1 1 1` → `STATUS`）、维持心跳、STOP 优先、按 1.05 s 冷却门控脉冲；`pulse()` 只入队，不阻塞 UI。
- [nav_servo.py](nav_servo.py)：位姿 → 车体误差 → 百分比与时长。先原地转到目标朝向，再平移；`plan_step()` 先判安全（90 s 截止、位姿过期 0.35 s、无位姿）再判到达。
- 测试：[test_chassis_link.py](test_chassis_link.py) 用假固件验证握手顺序、busy/冷却门控、STOP 抢占；[test_nav_servo.py](test_nav_servo.py) 验证坐标换算、限幅与仿真收敛。共 30 项，`python windows/test_chassis_link.py` / `python windows/test_nav_servo.py`。

离线仿真（不需要相机、底盘或 pyserial）：

```powershell
python windows\nav_servo.py --sim --start 150 200 90 --target 400 300 --heading 0
```

实机（需要 tag 上车、`field_calib.json`、底盘串口；`pyserial` 见 requirements.txt）：

```powershell
python windows\nav_servo.py --port COM4 --config windows\field_calib.json `
  --scrcpy-dir 'E:\scrcpy\scrcpy-win64-v5.0.1' --target 400 300 --heading 0 `
  --log windows\logs\nav.csv
```

`mm_per_percent_second` 与 `deg_per_percent_second` 默认 3.0 / 1.2，是**占位值**，必须先用固定脉冲实测替换；标定前只用小百分比、短脉冲。相机和底盘都未接时不要跑实机模式。

## 当前工作边界

Windows 不运行 ROS、Gazebo 或 colcon。h15 仿真保持暂停。现场视觉标定、ESP32 实物参数与整车闭环尚待验收。本轮仅开发和离线验证，没有启动采集、仿真或电机。
