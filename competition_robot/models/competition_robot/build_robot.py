import math
import random
from pathlib import Path

s = ['''<?xml version="1.0"?>
<sdf version="1.6">
<model name="competition_robot">
  <pose>STARTX -0.54 0 0 0 1.57079632679</pose>
  <static>false</static>
  <allow_auto_disable>false</allow_auto_disable>
''']


def box_inertia(mass, sx, sy, sz):
    return (
        mass / 12.0 * (sy * sy + sz * sz),
        mass / 12.0 * (sx * sx + sz * sz),
        mass / 12.0 * (sx * sx + sy * sy),
    )


def box(name, x, y, z, sx, sy, sz, color, mass=0.1, mu=1.0):
    ixx, iyy, izz = box_inertia(mass, sx, sy, sz)
    s.append(f'''
  <link name="{name}">
    <pose>{x} {y} {z} 0 0 0</pose>

    <inertial>
      <mass>{mass}</mass>
      <inertia>
        <ixx>{ixx:.8f}</ixx>
        <iyy>{iyy:.8f}</iyy>
        <izz>{izz:.8f}</izz>
      </inertia>
    </inertial>

    <collision name="collision">
      <geometry>
        <box>
          <size>{sx} {sy} {sz}</size>
        </box>
      </geometry>
      <surface>
        <friction>
          <ode>
            <mu>{mu}</mu>
            <mu2>{mu}</mu2>
          </ode>
        </friction>
      </surface>
    </collision>

    <visual name="visual">
      <geometry>
        <box>
          <size>{sx} {sy} {sz}</size>
        </box>
      </geometry>

      <material>
        <ambient>{color}</ambient>
        <diffuse>{color}</diffuse>
      </material>
    </visual>
  </link>
''')


def fixed(name, parent, child):
    s.append(f'''
  <joint name="{name}" type="fixed">
    <parent>{parent}</parent>
    <child>{child}</child>
  </joint>
''')


# 机器人底盘：底面高于地面，避免刮地
box(
    "base_link",
    0, 0, 0.045,
    0.09, 0.085, 0.045,
    "0.1 0.55 0.2 1",
    1.0,
    mu=0.0,
)

# 四个车轮：零摩擦，planar_move 才能横移/原地转
wheels = [
    ("front_left",   0.03,  0.047),
    ("front_right",  0.03, -0.047),
    ("rear_left",   -0.03,  0.047),
    ("rear_right",  -0.03, -0.047),
]

wheel_mass = 0.08
wheel_r = 0.018
wheel_h = 0.012
wheel_ixx = wheel_mass / 12.0 * (3.0 * wheel_r * wheel_r + wheel_h * wheel_h)
wheel_izz = 0.5 * wheel_mass * wheel_r * wheel_r

for name, x, y in wheels:
    s.append(f'''
  <link name="{name}_wheel">
    <pose>{x} {y} 0.018 1.57079632679 0 0</pose>

    <inertial>
      <mass>{wheel_mass}</mass>
      <inertia>
        <ixx>{wheel_ixx:.8f}</ixx>
        <iyy>{wheel_ixx:.8f}</iyy>
        <izz>{wheel_izz:.8f}</izz>
      </inertia>
    </inertial>

    <collision name="collision">
      <geometry>
        <cylinder>
          <radius>{wheel_r}</radius>
          <length>{wheel_h}</length>
        </cylinder>
      </geometry>
      <surface>
        <friction>
          <ode>
            <mu>0.0</mu>
            <mu2>0.0</mu2>
            <slip1>1.0</slip1>
            <slip2>1.0</slip2>
          </ode>
        </friction>
      </surface>
    </collision>

    <visual name="visual">
      <geometry>
        <cylinder>
          <radius>{wheel_r}</radius>
          <length>{wheel_h}</length>
        </cylinder>
      </geometry>
      <material>
        <ambient>0.08 0.08 0.08 1</ambient>
        <diffuse>0.08 0.08 0.08 1</diffuse>
      </material>
    </visual>
  </link>

  <joint name="{name}_joint" type="fixed">
    <parent>base_link</parent>
    <child>{name}_wheel</child>
  </joint>
''')

# 前置推铲抬离地面，避免刮地卡住偏航
box(
    "front_shovel",
    0.056, 0, 0.020,
    0.018, 0.115, 0.024,
    "0.2 0.2 0.2 1",
    0.15,
    mu=0.2,
)

fixed("shovel_joint", "base_link", "front_shovel")

# 俯视四角色标。前两角品红，后两角青色。位置取两对中心的中点，朝向从青指向品红。
for name, x, y, color in (
    ("front_left_marker", 0.032, 0.026, "1 0 1 1"),
    ("front_right_marker", 0.032, -0.026, "1 0 1 1"),
    ("rear_left_marker", -0.032, 0.026, "0 1 1 1"),
    ("rear_right_marker", -0.032, -0.026, "0 1 1 1"),
):
    box(name, x, y, 0.102, 0.02, 0.02, 0.004, color, 0.005, mu=0.0)
    fixed(f"{name}_joint", "base_link", name)

# 摄像头安装座
box(
    "camera_mount",
    0, 0, 0.085,
    0.025, 0.025, 0.025,
    "0.1 0.1 0.1 1",
    0.05,
    mu=0.0,
)

fixed("camera_mount_joint", "base_link", "camera_mount")

s.append('''
  <plugin name="omni_drive" filename="libgazebo_ros_planar_move.so">
    <ros>
      <remapping>cmd_vel:=cmd_vel</remapping>
      <remapping>odom:=odom</remapping>
    </ros>
    <update_rate>0</update_rate>
    <publish_rate>50</publish_rate>
    <publish_odom>true</publish_odom>
    <publish_odom_tf>true</publish_odom_tf>
    <odometry_frame>odom</odometry_frame>
    <robot_base_frame>base_link</robot_base_frame>
  </plugin>

</model></sdf>
''')

model_xml = "".join(s)

def block_layout(seed: int = 20260922):
    """中线两侧各 10 个方块，离中线大约 5cm 排成一排，x 方向大致均匀。

    返回 (x, y, yaw)。y<0 在己方（蓝区），y>0 在对方（黄区）。
    """
    rng = random.Random(seed)
    poses = []
    span = 0.96
    n = 10
    for side in (-1, 1):
        for i in range(n):
            x = -0.48 + span * (i + 0.5) / n
            x += rng.uniform(-0.02, 0.02)
            y = side * (0.05 + rng.uniform(-0.005, 0.005))
            x = max(-0.52, min(0.52, x))
            yaw = rng.uniform(-0.4, 0.4)
            poses.append((x, y, yaw))
    return poses


def _check_blocks(poses) -> None:
    if len(poses) != 20:
        raise RuntimeError(f"方块数量应为 20，实际 {len(poses)}")
    own = [p for p in poses if p[1] < 0]
    opp = [p for p in poses if p[1] > 0]
    if len(own) != 10 or len(opp) != 10:
        raise RuntimeError("中线两侧应各有 10 个方块")
    for x, y, _yaw in poses:
        if not 0.04 <= abs(y) <= 0.06 or abs(x) > 0.55:
            raise RuntimeError(f"方块不在中线附近: {x:.3f}, {y:.3f}")
    for i, a in enumerate(poses):
        for b in poses[i + 1 :]:
            dist = math.hypot(a[0] - b[0], a[1] - b[1])
            if dist < 0.03:
                raise RuntimeError(f"方块重叠: {a} 和 {b}，间距 {dist:.3f}")


def block_models(poses) -> str:
    mass = 0.015
    side = 0.02
    inertia = mass / 12.0 * (side * side + side * side)
    parts = []
    for idx, (x, y, yaw) in enumerate(poses):
        parts.append(f'''
    <model name="block_{idx:02d}">
      <pose>{x:.4f} {y:.4f} 0.010 0 0 {yaw:.4f}</pose>
      <link name="link">
        <inertial>
          <mass>{mass}</mass>
          <inertia>
            <ixx>{inertia:.8e}</ixx>
            <iyy>{inertia:.8e}</iyy>
            <izz>{inertia:.8e}</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry><box><size>{side} {side} {side}</size></box></geometry>
          <surface>
            <friction>
              <ode><mu>0.9</mu><mu2>0.9</mu2></ode>
            </friction>
            <contact>
              <ode>
                <kp>1000000</kp>
                <kd>100</kd>
                <max_vel>0.5</max_vel>
                <min_depth>0.001</min_depth>
              </ode>
            </contact>
          </surface>
        </collision>
        <visual name="visual">
          <geometry><box><size>{side} {side} {side}</size></box></geometry>
          <material>
            <ambient>0.95 0.05 0.05 1</ambient>
            <diffuse>0.95 0.05 0.05 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
''')
    return "".join(parts)


# 640x480、10Hz。虚拟机没有显卡，大图会把整个仿真拖卡。
CAMERA_XML = '''
    <model name="overhead_camera">
      <pose>0 0 2.55 0 1.570796 0</pose>
      <static>true</static>
      <link name="link">
        <sensor type="camera" name="camera">
          <update_rate>10</update_rate>
          <visualize>false</visualize>
          <camera>
            <horizontal_fov>1.05</horizontal_fov>
            <image>
              <width>640</width>
              <height>480</height>
              <format>R8G8B8</format>
            </image>
            <clip>
              <near>0.05</near>
              <far>8</far>
            </clip>
            <noise>
              <type>gaussian</type>
              <mean>0.0</mean>
              <stddev>0.004</stddev>
            </noise>
          </camera>
          <plugin name="overhead_camera_controller" filename="libgazebo_ros_camera.so">
            <ros>
              <namespace>overhead</namespace>
            </ros>
            <camera_name>camera</camera_name>
            <frame_name>overhead_camera_link</frame_name>
          </plugin>
        </sensor>
      </link>
    </model>
'''


block_poses = block_layout()
_check_blocks(block_poses)
start_x = min(x for x, y, _yaw in block_poses if y < 0)
model_xml = model_xml.replace("STARTX", f"{start_x:.4f}")
out = Path(__file__).with_name("model.sdf")
out.write_bytes(model_xml.replace("\r\n", "\n").encode("utf-8"))
print(f"Robot model generated successfully: {out}  start x={start_x:.4f}")

start = model_xml.find("<model")
end = model_xml.rfind("</model>") + len("</model>")
model_body = model_xml[start:end]
blocks_xml = block_models(block_poses)

world_xml = f'''<?xml version="1.0"?>
<sdf version="1.6">
  <world name="competition">

    <scene>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.75 0.75 0.75 1</background>
      <shadows>false</shadows>
    </scene>

    <light name="sun" type="directional">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="field">
      <static>true</static>
      <link name="field_link">

        <visual name="blue">
          <pose>0 -0.3 -0.005 0 0 0</pose>
          <geometry>
            <box><size>1.2 0.6 0.01</size></box>
          </geometry>
          <material>
            <ambient>0.05 0.25 0.9 1</ambient>
            <diffuse>0.05 0.25 0.9 1</diffuse>
          </material>
        </visual>

        <visual name="yellow">
          <pose>0 0.3 -0.005 0 0 0</pose>
          <geometry>
            <box><size>1.2 0.6 0.01</size></box>
          </geometry>
          <material>
            <ambient>0.95 0.8 0.05 1</ambient>
            <diffuse>0.95 0.8 0.05 1</diffuse>
          </material>
        </visual>

        <collision name="floor_collision">
          <pose>0 0 -0.005 0 0 0</pose>
          <geometry>
            <box><size>1.2 1.2 0.01</size></box>
          </geometry>
        </collision>

        <collision name="wall_left">
          <pose>-0.61 0 0.05 0 0 0</pose>
          <geometry>
            <box><size>0.02 1.24 0.1</size></box>
          </geometry>
        </collision>
        <visual name="wall_left_visual">
          <pose>-0.61 0 0.05 0 0 0</pose>
          <geometry>
            <box><size>0.02 1.24 0.1</size></box>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.3 0.3 0.3 1</diffuse>
          </material>
        </visual>

        <collision name="wall_right">
          <pose>0.61 0 0.05 0 0 0</pose>
          <geometry>
            <box><size>0.02 1.24 0.1</size></box>
          </geometry>
        </collision>
        <visual name="wall_right_visual">
          <pose>0.61 0 0.05 0 0 0</pose>
          <geometry>
            <box><size>0.02 1.24 0.1</size></box>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.3 0.3 0.3 1</diffuse>
          </material>
        </visual>

        <collision name="wall_front">
          <pose>0 0.61 0.05 0 0 0</pose>
          <geometry>
            <box><size>1.24 0.02 0.1</size></box>
          </geometry>
        </collision>
        <visual name="wall_front_visual">
          <pose>0 0.61 0.05 0 0 0</pose>
          <geometry>
            <box><size>1.24 0.02 0.1</size></box>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.3 0.3 0.3 1</diffuse>
          </material>
        </visual>

        <collision name="wall_back">
          <pose>0 -0.61 0.05 0 0 0</pose>
          <geometry>
            <box><size>1.24 0.02 0.1</size></box>
          </geometry>
        </collision>
        <visual name="wall_back_visual">
          <pose>0 -0.61 0.05 0 0 0</pose>
          <geometry>
            <box><size>1.24 0.02 0.1</size></box>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.3 0.3 0.3 1</diffuse>
          </material>
        </visual>

      </link>
    </model>

{model_body}

{blocks_xml}

{CAMERA_XML}

  </world>
</sdf>
'''

world_path = Path(__file__).resolve().parents[2] / "worlds" / "competition.world"
world_path.write_bytes(world_xml.replace("\r\n", "\n").encode("utf-8"))
print(f"World generated successfully: {world_path}")
