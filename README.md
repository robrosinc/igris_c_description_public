# igris_c_description

ROS 2 package containing the IGRIS-C robot description (URDF / xacro, meshes,
RViz config) plus MuJoCo MJCF models and helper scripts to visualize and
simulate the robot.

## Contents

- `urdf/` — URDF and xacro sources. The xacro entry point is
  `urdf/igris_c_v2.urdf.xacro`; static URDFs for each variant
  (`igris_c_v2.urdf`, `igris_c_v2_parallel.urdf`, `igris_c_v2_hand.urdf`,
  `igris_c_v2_pelvis_base*.urdf`, ...) are pre-generated alongside.
- `mujoco/` — MuJoCo MJCF models matching the URDF variants
  (`igris_c_v2.xml`, `igris_c_v2_parallel.xml`, `igris_c_v2_hand.xml`, ...).
- `meshes/` — Geometry referenced by the URDF / MJCF models.
- `launch/display_urdf_with_gui.launch.py` — Launch RViz with
  joint-state sliders and `robot_state_publisher`.
- `rviz/urdf.rviz` — Preconfigured RViz view.
- `scripts/imu_tf_broadcaster.py` — ROS 2 node that republishes an
  `/igris_c/imu` orientation as a TF on `base_link`.
- `scripts/mujoco_file_test.py` — Minimal MuJoCo passive-viewer runner that
  loads `mujoco/igris_c_v2.xml` and steps the simulation.

## Requirements

- ROS 2 with `ament_cmake`, `robot_state_publisher`, `joint_state_publisher_gui`,
  `rviz2`, and `xacro`.
- Python 3.8+.
- MuJoCo Python package: `pip install mujoco` (only needed for
  `scripts/mujoco_file_test.py`).
- A working X / Wayland display and OpenGL drivers for the MuJoCo viewer.
  If you hit GLX errors under Conda, prefer a venv that uses system libGL.

## Install (ROS 2)

```bash
cd /path/to/your/ros2_ws/src
git clone <this repo> igris_c_description
cd ..
colcon build --packages-select igris_c_description
source install/setup.bash
```

## RViz visualization

Launch the URDF / xacro with sliders and RViz:

```bash
ros2 launch igris_c_description display_urdf_with_gui.launch.py
```

Launch arguments (all optional):

| Argument        | Default                        | Description                                                |
| --------------- | ------------------------------ | ---------------------------------------------------------- |
| `urdf_package`  | `igris_c_description`          | Package the URDF / xacro is loaded from.                   |
| `urdf_path`     | `urdf/igris_c_v2.urdf.xacro`   | URDF or xacro path inside the package's share directory.   |
| `hand_type`     | `dummy`                        | End-effector variant when loading a xacro. See below.      |

`hand_type` accepts:

- `dummy` — no end effector (default).
- `hand` — articulated hand variant.

Examples:

```bash
# Load a different static URDF (no xacro processing)
ros2 launch igris_c_description display_urdf_with_gui.launch.py \
    urdf_path:=urdf/igris_c_v2_parallel.urdf

# Build the xacro with the articulated hand
ros2 launch igris_c_description display_urdf_with_gui.launch.py \
    urdf_path:=urdf/igris_c_v2.urdf.xacro hand_type:=hand
```

The joint-state GUI publishes on `/igris_c/joint_states` (remapped from
`/joint_states`), and `robot_state_publisher` subscribes to the same topic.

## IMU → base_link TF broadcaster

`scripts/imu_tf_broadcaster.py` subscribes to `/igris_c/imu` (`sensor_msgs/Imu`)
and broadcasts the orientation as a TF whose parent and child are both
`base_link` (translation zeroed). It is installed as an executable, so after
sourcing the workspace:

```bash
ros2 run igris_c_description imu_tf_broadcaster.py
```

## MuJoCo simulation (Python)

`scripts/mujoco_file_test.py` opens MuJoCo's passive viewer on
`mujoco/igris_c_v2.xml` and steps the model until the window is closed. It
takes no CLI arguments — edit the script if you want to load a different MJCF.

Recommendation: use a local venv to avoid Conda libGL conflicts.

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install mujoco
python scripts/mujoco_file_test.py
```

To try a different variant, change the path in
[`scripts/mujoco_file_test.py`](scripts/mujoco_file_test.py) — available MJCF
files are listed in [`mujoco/`](mujoco/).

### Viewer tips

- Run from a terminal inside your desktop session (`DISPLAY` set,
  `glxinfo -B` works).
- If the viewer fails with GLX errors under Conda, switch to the venv above
  or ensure system libGL is picked up.
