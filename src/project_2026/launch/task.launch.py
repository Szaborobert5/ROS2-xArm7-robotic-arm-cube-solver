#!/usr/bin/env python3
"""
xarm7_demo_scene.launch.py
==========================
Environment: ROS 2 Jazzy + Gazebo Sim 8 (gz-sim, NOT Gazebo Classic).

Launches:
  1. xarm7 + UFACTORY gripper + RealSense D435i in Gazebo Sim
  2. 4 floating cubes spawned on the desk for pick-and-place operations
  3. 3 flat surfaces (15x15cm) for cube placement
  4. DetachableJoint plugins injected into UF_ROBOT for each cube

Key improvements:
  • Ultra-stable cube physics (no rotation, no bouncing)
  • Proper spawn positions relative to robot base
  • Fixed seed for reproducible layouts
  • Runtime DetachableJoint injection via /world/default/entity/system/add
"""

import math
import random
import re
import subprocess

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# ── Constants ─────────────────────────────────────────────────────────────────

# Robot base position
ROBOT_BASE_X = -0.2
ROBOT_BASE_Y = -0.5
ROBOT_BASE_Z = 1.021

TABLE_SEED = 10          # fixed seed → same layout every run; change freely
CUBE_SIDE = 0.04         # metres (4 cm cubes - easier to grip)
TABLE_HEIGHT = ROBOT_BASE_Z
TABLE_THICKNESS = 0.02
SPAWN_DELAY_S = 5.0      # seconds; increase on slow machines

# Extra delay after cube spawn before injecting plugins.
# Cubes must be fully registered in ECM before injection.
INJECT_DELAY_S = SPAWN_DELAY_S + 3.0

PATCH_X_MIN = ROBOT_BASE_X - 0.25
PATCH_X_MAX = ROBOT_BASE_X + 0.25
PATCH_Y_MIN = ROBOT_BASE_Y - 0.2
PATCH_Y_MAX = PATCH_Y_MIN - 0.3

MIN_CUBE_SEPARATION = 0.10
NUM_CUBES = 5

# Surface parameters
SURFACE_SIZE = 0.15
SURFACE_THICKNESS = 0.005
SURFACE_SEPARATION = 0.02

GZ_WORLD_NAME = "default"
ROBOT_MODEL_NAME = "UF_ROBOT"
GRIPPER_PARENT_LINK = "left_finger"

# ──────────────────────────────────────────────────────────────────────────────


# ── SDF helpers ───────────────────────────────────────────────────────────────

def _inertia_val(mass: float, size: float = CUBE_SIDE) -> str:
    """Inertia of a solid cube: I = (1/6) * m * a^2."""
    return f"{(1.0 / 6.0) * mass * size ** 2:.6f}"


def make_stable_cube_sdf(name: str, r: float, g: float, b: float) -> str:
    mass = 0.15
    i_val = _inertia_val(mass)
    s = CUBE_SIDE

    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>false</static>
    <self_collide>false</self_collide>
    <allow_auto_disable>false</allow_auto_disable>
    <plugin
      filename="gz-sim-pose-publisher-system"
      name="gz::sim::systems::PosePublisher">
      <publish_link_pose>true</publish_link_pose>
      <publish_sensor_pose>true</publish_sensor_pose>
      <publish_collision_pose>false</publish_collision_pose>
      <publish_visual_pose>false</publish_visual_pose>
      <publish_nested_model_pose>true</publish_nested_model_pose>
    </plugin>
    <link name="link">
      <gravity>true</gravity>
      <self_collide>false</self_collide>
      <kinematic>false</kinematic>

      <inertial>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{i_val}</ixx>
          <iyy>{i_val}</iyy>
          <izz>{i_val}</izz>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyz>0</iyz>
        </inertia>
        <pose>0 0 0 0 0 0</pose>
      </inertial>

      <collision name="collision">
        <geometry>
          <box><size>{s} {s} {s}</size></box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>2.0</mu>
              <mu2>2.0</mu2>
            </ode>
          </friction>
          <contact>
            <ode>
              <kp>1e4</kp>
              <kd>1e2</kd>
              <max_vel>0.01</max_vel>
              <min_depth>0.0</min_depth>
            </ode>
          </contact>
        </surface>
      </collision>

      <visual name="visual">
        <geometry>
          <box><size>{s} {s} {s}</size></box>
        </geometry>
        <material>
          <ambient>{r:.2f} {g:.2f} {b:.2f} 1</ambient>
          <diffuse>{r:.2f} {g:.2f} {b:.2f} 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


def make_flat_surface_sdf(name: str, r: float, g: float, b: float) -> str:
    s = SURFACE_SIZE
    t = SURFACE_THICKNESS

    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <self_collide>false</self_collide>

    <link name="link">
      <gravity>false</gravity>
      <self_collide>false</self_collide>
      <kinematic>false</kinematic>

      <collision name="collision">
        <geometry>
          <box><size>{s} {s} {t}</size></box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>500.0</mu>
              <mu2>500.0</mu2>
            </ode>
          </friction>
          <contact>
            <ode>
              <kp>1e8</kp>
              <kd>1e6</kd>
              <max_vel>0.001</max_vel>
              <min_depth>0.0001</min_depth>
            </ode>
          </contact>
        </surface>
      </collision>

      <visual name="visual">
        <geometry>
          <box><size>{s} {s} {t}</size></box>
        </geometry>
        <material>
          <!-- Use alpha channel for transparency instead of invalid <transparency> tag -->
          <ambient>{r:.2f} {g:.2f} {b:.2f} 0.7</ambient>
          <diffuse>{r:.2f} {g:.2f} {b:.2f} 0.7</diffuse>
          <specular>0.3 0.3 0.3 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


# ── Position generators ───────────────────────────────────────────────────────

def check_collision(x1, y1, x2, y2, min_dist):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2) < min_dist


def generate_cube_configs(seed: int, n: int = 4) -> list:
    rng = random.Random()

    palette = [
        (1.0, 0.0, 0.0),  # Red
        (0.0, 1.0, 0.0),  # Green
        (0.0, 0.0, 1.0),  # Blue
    ]
    colors = rng.choices(palette, k=n)
    rng.shuffle(colors)

    z_spawn = TABLE_HEIGHT + (CUBE_SIDE / 2.0)
    positions = []
    max_attempts = 100

    for i in range(n):
        best_pos = None
        for _ in range(max_attempts):
            x = rng.uniform(PATCH_X_MIN, PATCH_X_MAX)
            y = rng.uniform(PATCH_Y_MIN, PATCH_Y_MAX)
            if all(not check_collision(x, y, px, py, MIN_CUBE_SEPARATION)
                   for px, py in positions):
                best_pos = (x, y)
                break

        if best_pos is None:
            cols = int(math.ceil(math.sqrt(n)))
            row, col = divmod(i, cols)
            x = PATCH_X_MIN + (PATCH_X_MAX - PATCH_X_MIN) * (col + 0.5) / cols
            y = PATCH_Y_MIN + (PATCH_Y_MAX - PATCH_Y_MIN) * (row + 0.5) / int(math.ceil(n / cols))
            best_pos = (x, y)

        positions.append(best_pos)

    configs = []
    for i, ((r, g, b), (x, y)) in enumerate(zip(colors, positions)):
        name = f"cube_{i}"
        configs.append({
            "name": name,
            "sdf": make_stable_cube_sdf(name, r, g, b),
            "x": x, "y": y, "z": z_spawn, "yaw": 0.0,
        })
    return configs


def generate_surface_configs(seed: int) -> list:
    rng = random.Random()
    colors = [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)]
    rng.shuffle(colors)
    rng.shuffle(colors)
    rng.shuffle(colors)

    y_pos = PATCH_Y_MAX - SURFACE_SIZE / 2 - 0.05
    total_width = 3 * SURFACE_SIZE + 2 * SURFACE_SEPARATION
    start_x = ROBOT_BASE_X - total_width / 2

    configs = []
    for i, (r, g, b) in enumerate(colors):
        x_pos = start_x + i * (SURFACE_SIZE + SURFACE_SEPARATION) + SURFACE_SIZE / 2
        name = f"surface_{i}"
        configs.append({
            "name": name,
            "sdf": make_flat_surface_sdf(name, r, g, b),
            "x": x_pos, "y": y_pos,
            "z": TABLE_HEIGHT + 0.005, "yaw": 0.0,
        })
    return configs


# ── DetachableJoint injection ─────────────────────────────────────────────────

def get_model_entity_id(model_name: str) -> int | None:
    """
    Query gz-sim for the numeric ECM entity ID of a model by name.
    Returns None if not found or gz command fails.
    """
    try:
        result = subprocess.run(
            ["gz", "model", "-m", model_name],
            capture_output=True, text=True, timeout=10,
        )
        match = re.search(r"^Model: \[(\d+)\]", result.stdout, re.MULTILINE)
        if match:
            return int(match.group(1))
    except (subprocess.TimeoutExpired, FileNotFoundError) as e:
        print(f"[DetachableJoint] gz model query failed: {e}")
    return None


def inject_detachable_joint(entity_id: int, cube_name: str) -> bool:
    """
    Inject a DetachableJoint plugin into the robot model at runtime via
    /world/default/entity/system/add using the confirmed-working entity ID approach.
    """
    innerxml = (
        f"<parent_link>{GRIPPER_PARENT_LINK}</parent_link>"
        f"<child_model>{cube_name}</child_model>"
        f"<child_link>link</child_link>"
        f"<attach_topic>/gz/{cube_name}/attach</attach_topic>"
        f"<detach_topic>/gz/{cube_name}/detach</detach_topic>"
        f"<suppress_child_warning>true</suppress_child_warning>"
    )

    pbtxt = (
        f'entity: {{id: {entity_id} type: 2}}\n'
        f'plugins: {{\n'
        f'  name: "gz::sim::systems::DetachableJoint"\n'
        f'  filename: "gz-sim-detachable-joint-system"\n'
        f'  innerxml: "{innerxml}"\n'
        f'}}\n'
    )

    pbtxt_path = f"/tmp/inject_{cube_name}.pbtxt"
    with open(pbtxt_path, "w") as f:
        f.write(pbtxt)

    try:
        result = subprocess.run(
            [
                "gz", "service",
                "-s", f"/world/{GZ_WORLD_NAME}/entity/system/add",
                "--reqtype", "gz.msgs.EntityPlugin_V",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "5000",
                "--req", pbtxt,
            ],
            capture_output=True, text=True, timeout=10,
        )
        success = "data: true" in result.stdout
        if success:
            print(f"[DetachableJoint] ✓ Injected for {cube_name}")

            # Immediately detach — plugin attaches on init by default
            import time
            time.sleep(0.5)  # give plugin time to initialize
            subprocess.run([
                "gz", "topic",
                "-t", f"/gz/{cube_name}/detach",
                "-m", "gz.msgs.Empty",
                "-p", " ",
            ], timeout=5)
            print(f"[DetachableJoint] Detached {cube_name} (start state)")

        return success
    except (subprocess.TimeoutExpired, FileNotFoundError) as e:
        print(f"[DetachableJoint] Exception for {cube_name}: {e}")
        return False


def inject_all_detachable_joints(context, *args, **kwargs):
    """
    OpaqueFunction: resolves UF_ROBOT entity ID once, then injects
    DetachableJoint plugins for every cube. Runs after INJECT_DELAY_S.
    """
    # Resolve robot entity ID once — same for all cubes
    entity_id = get_model_entity_id(ROBOT_MODEL_NAME)
    if entity_id is None:
        print(f"[DetachableJoint] Could not find entity ID for "
              f"'{ROBOT_MODEL_NAME}'. Is the simulation running?")
        return []

    print(f"[DetachableJoint] Found '{ROBOT_MODEL_NAME}' entity ID: {entity_id}")

    # Inject one plugin per cube
    for i in range(NUM_CUBES):
        inject_detachable_joint(entity_id, f"cube_{i}")

    # No ROS nodes needed — injection is done via gz service subprocess
    return []


def spawn_cubes(context, *args, **kwargs):
    """Generate and spawn cube + surface entities in Gazebo."""
    actions = []

    # Spawn cubes
    for cfg in generate_cube_configs(seed=TABLE_SEED, n=NUM_CUBES):
        sdf_path = f"/tmp/{cfg['name']}_cube.sdf"
        with open(sdf_path, "w") as fh:
            fh.write(cfg["sdf"])

        actions.append(Node(
            package="ros_gz_sim",
            executable="create",
            name=f"spawn_{cfg['name']}",
            output="screen",
            arguments=[
                "-world", GZ_WORLD_NAME,
                "-file",  sdf_path,
                "-name",  cfg["name"],
                "-x",     str(round(cfg["x"], 4)),
                "-y",     str(round(cfg["y"], 4)),
                "-z",     str(round(cfg["z"], 4)),
                "-Y",     str(round(cfg["yaw"], 4)),
            ],
        ))

    # Spawn surfaces
    for cfg in generate_surface_configs(seed=TABLE_SEED):
        sdf_path = f"/tmp/{cfg['name']}_surface.sdf"
        with open(sdf_path, "w") as fh:
            fh.write(cfg["sdf"])

        actions.append(Node(
            package="ros_gz_sim",
            executable="create",
            name=f"spawn_{cfg['name']}",
            output="screen",
            arguments=[
                "-world", GZ_WORLD_NAME,
                "-file",  sdf_path,
                "-name",  cfg["name"],
                "-x",     str(round(cfg["x"], 4)),
                "-y",     str(round(cfg["y"], 4)),
                "-z",     str(round(cfg["z"], 4)),
                "-Y",     str(round(cfg["yaw"], 4)),
            ],
        ))

    return actions


def generate_launch_description():

    declared_args = [
        DeclareLaunchArgument(
            "add_d435i_links",
            default_value="true",
            description="Expose TF frames for D435i sub-sensors.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="All nodes use /clock from Gazebo Sim.",
        ),
    ]

    # Robot + Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("xarm_moveit_config"),
                "launch",
                "xarm7_moveit_gazebo.launch.py",
            ])
        ]),
        launch_arguments={
            "dof":                 "7",
            "add_gripper":         "true",
            "add_realsense_d435i": "true",
            "add_d435i_links":     LaunchConfiguration("add_d435i_links"),
            "use_sim_time":        LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    bridge_arguments = []

    for i in range(NUM_CUBES):
        # Pose bridge (Gazebo -> ROS 2)
        r = f"/model/cube_{i}/pose@geometry_msgs/msg/TransformStamped[gz.msgs.Pose"
        bridge_arguments.append(r)
        # Detach bridge (ROS 2 -> Gazebo)
        detach_arg = f"/gz/cube_{i}/detach@std_msgs/msg/Empty]gz.msgs.Empty"
        bridge_arguments.append(detach_arg)

        #  Attach bridge (ROS 2 -> Gazebo)
        attach_arg = f"/gz/cube_{i}/attach@std_msgs/msg/Empty]gz.msgs.Empty"
        bridge_arguments.append(attach_arg)

    # Bridge gz model poses → ROS 2 TF
    pose_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="pose_bridge",
        output="screen",
        arguments=bridge_arguments
    )

    # Delay: spawn cubes after Gazebo is ready
    spawn_action = TimerAction(
        period=SPAWN_DELAY_S,
        actions=[OpaqueFunction(function=spawn_cubes)],
    )

    # Delay: inject DetachableJoint plugins after cubes are registered in ECM
    # Must run AFTER spawn_action completes — INJECT_DELAY_S > SPAWN_DELAY_S
    inject_action = TimerAction(
        period=INJECT_DELAY_S,
        actions=[OpaqueFunction(function=inject_all_detachable_joints)],
    )

    closest_cube_service = Node(
        package="project_2026",
        executable="closest_cube",
        name="closest_cube",
        output="screen"
    )

    return LaunchDescription(declared_args + [
        gazebo_launch,
        spawn_action,
        inject_action,
        pose_bridge,
        closest_cube_service
    ])
