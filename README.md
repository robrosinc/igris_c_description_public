# igris_c_description

ROS 2 package containing the IGRIS-C robot description xacro sources, meshes,
RViz config, MuJoCo xacro sources, and helper scripts to visualize and simulate
the robot.

## Contents

- `urdf/` — robot xacro sources. The main entry point is
  `urdf/igris_c_v2.urdf.xacro`; the standalone parallel-link models are
  generated from `urdf/igris_c_v2_parallel_links.urdf.xacro`.
- `mujoco/` — MuJoCo xacro source `mujoco/igris_c_v2.xml.xacro` plus the
  specialty static models that are intentionally kept separate:
  `igris_c_v2_parallel_collision.xml` and `igris_c_v2_parallel_toe.xml`.
- `meshes/` — Geometry referenced by the URDF / MJCF models.
- `launch/display_urdf_with_gui.launch.py` — Launch RViz with
  joint-state sliders and `robot_state_publisher`.
- `rviz/urdf.rviz` — Preconfigured RViz view.
- `scripts/imu_tf_broadcaster.py` — ROS 2 node that republishes an
  `/igris_c/imu` orientation as a TF on `base_link`.
- `scripts/export_model.py` — Standalone exporter that expands the canonical
  URDF or MuJoCo xacro and writes generated files into `urdf/` or `mujoco/`.
- `scripts/mujoco_file_test.py` — Minimal MuJoCo passive-viewer runner that
  expands `mujoco/igris_c_v2.xml.xacro`, writes a temporary MJCF, and steps the
  simulation.

## Canonical model structure

The description package now treats the xacro sources as the canonical robot
models. Legacy generated `.urdf` / `.xml` files are no longer the primary
authoring format.

- `urdf/igris_c_v2.urdf.xacro`
  - Args: `base_type`, `parallel`, `end_effector`
  - `base_type`: selects the torso-base or pelvis-base assembly order.
  - `parallel`: adds the waist / ankle / wrist parallel-link substructures.
  - `end_effector`: selects the lower-arm attachment (`dummy`, `none`, `hand`,
    `magnet`, `1dof`).
- `urdf/igris_c_v2_parallel_links.urdf.xacro`
  - Arg: `variant`
  - Variants: `waist`, `left_ankle`, `right_ankle`, `left_wrist`,
    `right_wrist`
  - Purpose: standalone 4-bar linkage models used by `JointManager`.
- `mujoco/igris_c_v2.xml.xacro`
  - Args: `base_type`, `parallel`, `end_effector`, `fixed`
  - `base_type`: torso floating-base chain vs pelvis floating/fixed chain.
  - `parallel`: selects serial vs parallel waist / ankle / wrist topology.
  - `end_effector`: selects the lower-arm payload module.
  - `fixed`: selects floating-base vs fixed-base MuJoCo model setup.

In both URDF and MuJoCo:

- `parallel` should only change the closed-loop mechanism topology and its
  directly dependent inertial / actuator / keyframe data.
- `end_effector` should only change the wrist-end payload module and its
  directly dependent assets.
- Common body chains stay in the main entry xacro so that variant-specific
  logic remains localized.

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
| `end_effector` | `dummy`                         | End-effector variant when loading a xacro. See below.      |

`end_effector` accepts:

- `dummy` — dummy spherical payload.
- `none` — no end effector.
- `hand` — articulated hand variant.
- `magnet` — magnet end-effector variant.
- `1dof` — 1-DOF gripper variant.

Examples:

```bash
# Build the canonical xacro with the default dummy payload
ros2 launch igris_c_description display_urdf_with_gui.launch.py \
    urdf_path:=urdf/igris_c_v2.urdf.xacro

# Build the xacro with the articulated hand
ros2 launch igris_c_description display_urdf_with_gui.launch.py \
    urdf_path:=urdf/igris_c_v2.urdf.xacro end_effector:=hand
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

`scripts/mujoco_file_test.py` expands `mujoco/igris_c_v2.xml.xacro` with
hard-coded defaults near the top of the file, writes a temporary MJCF, and
opens MuJoCo's passive viewer until the window is closed.

Recommendation: use a local venv to avoid Conda libGL conflicts.

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install mujoco
python scripts/mujoco_file_test.py
```

To try a different variant, edit the constants in
[`scripts/mujoco_file_test.py`](scripts/mujoco_file_test.py):

- `BASE_TYPE`: `torso` or `pelvis`
- `PARALLEL`: `true` or `false`
- `END_EFFECTOR`: `dummy`, `none`, `hand`, `magnet`, or `1dof`
- `FIXED`: `true` or `false`

## Export generated files

If you want to inspect a standalone generated `.urdf` or `.xml` file directly,
use `scripts/export_model.py`.

Examples:

```bash
# Generate only URDF into urdf/
python3 igris_c_description/scripts/export_model.py \
  --format urdf \
  --base-type pelvis \
  --parallel false \
  --end-effector dummy

# Generate only MuJoCo XML into mujoco/
python3 igris_c_description/scripts/export_model.py \
  --format xml \
  --base-type pelvis \
  --parallel true \
  --end-effector dummy \
  --fixed true
```

Default output naming:

- URDF: `urdf/igris_c_v2_<base_type>_[parallel_]<end_effector>.urdf`
- XML: `mujoco/igris_c_v2_<base_type>_[parallel_]<end_effector>_<free|fixed>.xml`

The script writes into the package's `urdf/` and `mujoco/` directories so the
generated files can be opened independently without going through launch files
or the temporary-file path used by the MuJoCo viewer helper.

## Runtime config layout

The repository config uses this description structure:

```yaml
igris_c:
  description:
    config_root_dir: "/path/to/igris_c_description"
    urdf:
      robot_model_path: "urdf/igris_c_v2.urdf.xacro"
      parallel_links_path: "urdf/igris_c_v2_parallel_links.urdf.xacro"
    mujoco:
      robot_model_path: "mujoco/igris_c_v2.xml.xacro"
      base_type: "pelvis"
      parallel: "true"
      fixed: "true"
  end_effector:
    type: "none"
```

`igris_c_v2.xml.xacro` covers the normal MuJoCo model variants. Only
`igris_c_v2_parallel_collision.xml` and `igris_c_v2_parallel_toe.xml` remain as
specialty static MJCF files.

Path handling rules:

- `igris_c.description.config_root_dir` points at the description package root.
- `igris_c.description.urdf.robot_model_path` may be relative to
  `config_root_dir` or absolute. Launch accepts `.urdf` and `.urdf.xacro`.
- `igris_c.description.urdf.parallel_links_path` may be relative to
  `config_root_dir` or absolute, but it is consumed as a xacro entry for the
  five parallel-link variants.
- `igris_c.description.mujoco.robot_model_path` may be relative to
  `config_root_dir` or absolute. Launch accepts `.xml` and `.xml.xacro`.
- For `.xacro` paths, launch expects a compatible entry contract:
  - URDF main model xacro must declare `base_type`, `parallel`, `end_effector`
  - Parallel-links xacro must declare `variant`
  - MuJoCo main model xacro must declare `base_type`, `parallel`,
    `end_effector`, `fixed`
## End-to-end examples

Generate the default controller URDF from xacro:

```bash
xacro igris_c_description/urdf/igris_c_v2.urdf.xacro \
  base_type:=pelvis \
  parallel:=false \
  end_effector:=dummy
```

Generate the default simulation MJCF from xacro:

```bash
xacro igris_c_description/mujoco/igris_c_v2.xml.xacro \
  base_type:=pelvis \
  parallel:=true \
  end_effector:=dummy \
  fixed:=true
```

Run the full simulation stack with the development config:

```bash
ros2 launch igris_c_controller simulation.launch.py \
  config_file:=/home/robros3/ros2_ws/src/igris_c/configs/robot/dev_robot.yaml
```

The current development config maps to this MuJoCo variant:

- `base_type:=pelvis`
- `parallel:=true`
- `end_effector:=dummy`
- `fixed:=true`

Examples of other useful combinations:

- `base_type:=torso parallel:=false end_effector:=hand fixed:=false`
- `base_type:=pelvis parallel:=true end_effector:=magnet fixed:=true`
- `base_type:=torso parallel:=true end_effector:=1dof fixed:=true`

Config-to-xacro mapping:

- `igris_c.description.urdf.robot_model_path` → main URDF xacro entry
- `igris_c.description.urdf.parallel_links_path` → standalone parallel-link URDF xacro entry
- `igris_c.description.mujoco.robot_model_path` → main MuJoCo xacro entry
- `igris_c.description.mujoco.base_type` → xacro `base_type`
- `igris_c.description.mujoco.parallel` → xacro `parallel`
- `igris_c.end_effector.type` → xacro `end_effector` directly
- `igris_c.description.mujoco.fixed` → xacro `fixed`

Runtime consumers:

- `igris_c_controller` launch path
  - resolves description paths first
  - expands the main controller URDF once with fixed controller defaults:
    `base_type:=pelvis parallel:=false end_effector:=<igris_c.end_effector.type>`
  - expands the main MuJoCo XML once with `base_type`, `parallel`,
    `end_effector`, `fixed` from robot config
  - publishes:
    - `igris_c.description.urdf.robot_model_xml`
    - resolved absolute `igris_c.description.urdf.parallel_links_path`
    - resolved/generated `igris_c.description.mujoco.robot_model_path`
- `igris_c_controller/RobotManager`
  - reads `igris_c.description.urdf.robot_model_xml`
  - does not expand xacro itself anymore
- `igris_c_lib/JointManager`
  - loads `igris_c.description.urdf.parallel_links_path`
  - expands the xacro five times with `variant:=...`
- `igris_c_mujoco`
  - reads the already generated/resolved
    `igris_c.description.mujoco.robot_model_path`
  - does not expand xacro itself anymore
- `igris_c_controller/MujocoSimController`
  - reads `igris_c.description.mujoco.parallel`
  - when `parallel:=false`, treats the MuJoCo actuator set as serial
    joint-space actuators and remaps controller motor-space outputs back to
    joint space before writing commands into MuJoCo SHM
  - uses the same `igris_c.description.urdf.parallel_links_path` source as
    `JointManager` so sim-only remapping stays aligned with the controller's
    parallel-link kinematics

### Viewer tips

- Run from a terminal inside your desktop session (`DISPLAY` set,
  `glxinfo -B` works).
- If the viewer fails with GLX errors under Conda, switch to the venv above
  or ensure system libGL is picked up.
