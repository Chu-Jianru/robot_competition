# 比赛清扫机器人

根目录 `/home/cjr/robot_competition` 是 ROS2 Humble 的 colcon 工作空间。功能包是 `src/sweep_robot`。Gazebo 场地和车在 `competition_robot/`，由 `competition_robot/models/competition_robot/build_robot.py` 生成。改场地或车之后运行这个脚本，不要手改 `competition_robot/worlds/competition.world`。

`build/`、`install/`、`log/` 是编译产物。

## 命令

新终端会通过 `~/.bashrc` 加载 `/opt/ros/humble/setup.bash` 和本仓库的 `install/setup.bash`。不要改 bashrc 里的 `cmu_exploration_ws`。

改代码后：

```bash
cd /home/cjr/robot_competition
colcon build --packages-select sweep_robot
ros2 launch sweep_robot sim.launch.py
```

`ros2 launch` 会同时打开 Gazebo、俯视检测和推块。只要键盘遥控时加 `push:=false`。

## 比赛

场地 1.2m×1.2m，蓝黄各半，围栏高 10cm。己方是蓝区，坐标 y<0。约 2cm 的红块共 20 个，中线两侧各 10 个。车从蓝区底边出发，车头朝黄区。90 秒内己方红块更少的一方获胜。比赛中无人干预，和车之间无线连接。

## 视觉和推块

用俯视相机加 HSV，不用 YOLO，也不要在这台没有 GPU 的虚拟机里训练。己方按红块周围的地面颜色判断：蓝多于黄才算己方。不要把场地四边形的几何中心当成中线。

推块先在己方空区对齐到一条道，再只沿 +y 直推，块带里不要横移。相机保持 640×480、10Hz。「俯视相机」窗口只供观看，不要为了显示再渲染一张大图。

未要求时不要加对手机器人。

## 上场

比赛当天不跑虚拟机，也不跑 Gazebo。Windows 上用 Python 和 OpenCV 调用 `src/sweep_robot/sweep_robot/field_vision.py`，把 `vx vy wz` 经无线发给单片机。
