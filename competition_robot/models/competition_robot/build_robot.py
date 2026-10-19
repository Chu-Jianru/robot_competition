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


def _contact(mu, slip=False):
    slip_xml = ""
    if slip:
        slip_xml = "\n              <slip1>1.0</slip1>\n              <slip2>1.0</slip2>"
    return f'''        <surface>
          <friction>
            <ode>
              <mu>{mu}</mu>
              <mu2>{mu}</mu2>{slip_xml}
            </ode>
          </friction>
          <contact>
            <ode>
              <kp>2.0e4</kp>
              <kd>20</kd>
              <max_vel>0.2</max_vel>
              <min_depth>0.001</min_depth>
            </ode>
          </contact>
        </surface>'''


# 整车只有 base_link。外观和碰撞都挂在这一个刚体上，没有关节，不会散开或脱落。
bodies = []
pieces = []


def _remember(mass, x, y, z, ixx, iyy, izz, ixy=0.0):
    bodies.append((mass, x, y, z, ixx, iyy, izz, ixy))


def add_box(name, x, y, z, sx, sy, sz, color, mass, mu, yaw=0.0):
    ixx, iyy, izz = box_inertia(mass, sx, sy, sz)
    c = math.cos(yaw)
    s = math.sin(yaw)
    _remember(
        mass, x, y, z,
        c * c * ixx + s * s * iyy,
        s * s * ixx + c * c * iyy,
        izz,
        c * s * (ixx - iyy),
    )
    pose = f"{x} {y} {z} 0 0 {yaw}"
    pieces.append(f'''
    <collision name="{name}">
      <pose>{pose}</pose>
      <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
{_contact(mu)}
    </collision>
    <visual name="{name}">
      <pose>{pose}</pose>
      <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
      <material>
        <ambient>{color}</ambient>
        <diffuse>{color}</diffuse>
      </material>
    </visual>''')


def add_cylinder(name, x, y, z, radius, length, color, mass, mu, roll=0.0, slip=False, collide=True):
    perp = mass / 12.0 * (3.0 * radius * radius + length * length) if mass else 0.0
    axial = 0.5 * mass * radius * radius
    if abs(roll - math.pi / 2.0) < 1e-3:
        ixx, iyy, izz = perp, axial, perp
    else:
        ixx, iyy, izz = perp, perp, axial
    if mass:
        _remember(mass, x, y, z, ixx, iyy, izz)
    pose = f"{x} {y} {z} {roll} 0 0"
    if collide:
        pieces.append(f'''
    <collision name="{name}">
      <pose>{pose}</pose>
      <geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry>
{_contact(mu, slip)}
    </collision>''')
    pieces.append(f'''
    <visual name="{name}">
      <pose>{pose}</pose>
      <geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry>
      <material>
        <ambient>{color}</ambient>
        <diffuse>{color}</diffuse>
      </material>
    </visual>''')


# 车顶是一圈框，中间镂空。色块贴在前后横梁上。
add_box("roof_rear", -0.038, 0, 0.050, 0.020, 0.080, 0.006, "0.25 0.25 0.28 1", 0.25, 0.0)
add_box("roof_front", 0.038, 0, 0.050, 0.020, 0.080, 0.006, "0.25 0.25 0.28 1", 0.25, 0.0)
add_box("roof_left", 0, 0.032, 0.050, 0.080, 0.016, 0.006, "0.25 0.25 0.28 1", 0.12, 0.0)
add_box("roof_right", 0, -0.032, 0.050, 0.080, 0.016, 0.006, "0.25 0.25 0.28 1", 0.12, 0.0)

# 左右纵梁顶到车顶、前端伸进滚刷。中间不封底。
add_box("rail_left", 0.005, 0.032, 0.029, 0.102, 0.010, 0.044, "0.15 0.15 0.16 1", 1.2, 0.0)
add_box("rail_right", 0.005, -0.032, 0.029, 0.102, 0.010, 0.044, "0.15 0.15 0.16 1", 1.2, 0.0)

# 腔体三面挡板，底面不封。开口朝滚刷，块扫进来后由后挡板带着往前走。
add_box("wall_left", 0.008, 0.029, 0.027, 0.080, 0.010, 0.046, "0.32 0.34 0.38 1", 0.06, 0.4)
add_box("wall_right", 0.008, -0.029, 0.027, 0.080, 0.010, 0.046, "0.32 0.34 0.38 1", 0.06, 0.4)
add_box("wall_rear", -0.032, 0, 0.027, 0.012, 0.068, 0.046, "0.32 0.34 0.38 1", 0.06, 0.4)

# 四个麦克纳姆轮。碰撞无摩擦，仿真仍由 planar_move 平移。轮轴只是外观。
axle_roll = math.pi / 2.0
for name, x, y in (
    ("front_left", 0.028, 0.048),
    ("front_right", 0.028, -0.048),
    ("rear_left", -0.028, 0.048),
    ("rear_right", -0.028, -0.048),
):
    add_cylinder(f"{name}_wheel", x, y, 0.016, 0.016, 0.012, "0.05 0.05 0.05 1", 0.08, 0.0, axle_roll, slip=True)
    add_cylinder(f"{name}_hub", x, y, 0.016, 0.010, 0.013, "0.75 0.08 0.08 1", 0.0, 0.0, axle_roll, collide=False)
    side = 1.0 if y > 0 else -1.0
    add_box(f"{name}_axle", x, side * 0.041, 0.016, 0.006, 0.022, 0.006, "0.55 0.55 0.58 1", 0.02, 0.0)

# 前滚刷和中间的轴是同一刚体。轴只显示，不单独碰撞。
add_cylinder("brush", 0.064, 0, 0.016, 0.012, 0.100, "0.05 0.35 0.12 1", 0.16, 0.4, axle_roll)
add_cylinder("brush_shaft", 0.064, 0, 0.016, 0.004, 0.122, "0.55 0.55 0.58 1", 0.0, 0.0, axle_roll, collide=False)

# 两侧薄板套在刷轴两端。前端更靠近墙，往前走时把墙边的块拨向中间。
add_box("plate_left", 0.064, 0.057, 0.016, 0.016, 0.004, 0.016, "0.12 0.12 0.12 1", 0.04, 0.2, yaw=0.26)
add_box("plate_right", 0.064, -0.057, 0.016, 0.016, 0.004, 0.016, "0.12 0.12 0.12 1", 0.04, 0.2, yaw=-0.26)

# 四角色块压进顶板 0.5mm，贴在车顶上。前品红，后青色。
roof_top = 0.053
tile_h = 0.003
tile_z = roof_top - 0.0005 + tile_h / 2.0
for name, x, y, color in (
    ("front_left_marker", 0.038, 0.028, "1 0 1 1"),
    ("front_right_marker", 0.038, -0.028, "1 0 1 1"),
    ("rear_left_marker", -0.038, 0.028, "0 1 1 1"),
    ("rear_right_marker", -0.038, -0.028, "0 1 1 1"),
):
    add_box(name, x, y, tile_z, 0.016, 0.016, tile_h, color, 0.004, 0.0)

total_m = sum(item[0] for item in bodies)
cx = sum(item[0] * item[1] for item in bodies) / total_m
cy = sum(item[0] * item[2] for item in bodies) / total_m
cz = sum(item[0] * item[3] for item in bodies) / total_m
ixx = iyy = izz = ixy = ixz = iyz = 0.0
for mass, x, y, z, part_ixx, part_iyy, part_izz, part_ixy in bodies:
    dx, dy, dz = x - cx, y - cy, z - cz
    ixx += part_ixx + mass * (dy * dy + dz * dz)
    iyy += part_iyy + mass * (dx * dx + dz * dz)
    izz += part_izz + mass * (dx * dx + dy * dy)
    ixy += part_ixy - mass * dx * dy
    ixz += -mass * dx * dz
    iyz += -mass * dy * dz
minor = ixx * iyy - ixy * ixy
det = (
    ixx * (iyy * izz - iyz * iyz)
    - ixy * (ixy * izz - iyz * ixz)
    + ixz * (ixy * iyz - iyy * ixz)
)
if ixx <= 0 or minor <= 0 or det <= 0:
    raise RuntimeError(f"整车惯量不是正定的: {ixx} {minor} {det}")

s.append(f'''
  <link name="base_link">
    <inertial>
      <pose>{cx:.6f} {cy:.6f} {cz:.6f} 0 0 0</pose>
      <mass>{total_m:.6f}</mass>
      <inertia>
        <ixx>{ixx:.8f}</ixx>
        <iyy>{iyy:.8f}</iyy>
        <izz>{izz:.8f}</izz>
        <ixy>{ixy:.8f}</ixy>
        <ixz>{ixz:.8f}</ixz>
        <iyz>{iyz:.8f}</iyz>
      </inertia>
    </inertial>
{"".join(pieces)}
  </link>
''')

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

  <plugin name="keep_upright" filename="libkeep_upright.so"/>

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
                <kp>2.0e4</kp>
                <kd>20</kd>
                <max_vel>0.2</max_vel>
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
          <surface>
            <contact>
              <ode>
                <kp>2.0e4</kp>
                <kd>20</kd>
                <max_vel>0.2</max_vel>
                <min_depth>0.001</min_depth>
              </ode>
            </contact>
          </surface>
        </collision>

        <collision name="wall_left">
          <pose>-0.61 0 0.05 0 0 0</pose>
          <geometry>
            <box><size>0.02 1.24 0.1</size></box>
          </geometry>
          <surface>
            <contact>
              <ode>
                <kp>2.0e4</kp>
                <kd>20</kd>
                <max_vel>0.2</max_vel>
                <min_depth>0.001</min_depth>
              </ode>
            </contact>
          </surface>
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
          <surface>
            <contact>
              <ode>
                <kp>2.0e4</kp>
                <kd>20</kd>
                <max_vel>0.2</max_vel>
                <min_depth>0.001</min_depth>
              </ode>
            </contact>
          </surface>
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
          <surface>
            <contact>
              <ode>
                <kp>2.0e4</kp>
                <kd>20</kd>
                <max_vel>0.2</max_vel>
                <min_depth>0.001</min_depth>
              </ode>
            </contact>
          </surface>
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
          <surface>
            <contact>
              <ode>
                <kp>2.0e4</kp>
                <kd>20</kd>
                <max_vel>0.2</max_vel>
                <min_depth>0.001</min_depth>
              </ode>
            </contact>
          </surface>
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
