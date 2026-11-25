# igris_c_description

ROS 2 package containing the IGRIS-C robot description xacro sources, meshes,
RViz config, MuJoCo xacro sources, and helper scripts to visualize and simulate
the robot.

## Contents

- `urdf/` — robot xacro sources. The main entry point is
  `urdf/<model>/<model>.urdf.xacro`; the standalone parallel-link models are
  generated from `urdf/<model>/parallel_links.urdf.xacro`. Parts that are the
  same on every model live once in `urdf/common/` (see "Include tree").
- `mujoco/` — MuJoCo xacro source `mujoco/<model>/<model>.xml.xacro`, the shared
  parts in `mujoco/common/`, plus the specialty static models that are
  intentionally kept separate: `standalone/parallel_collision.xml` and
  `standalone/parallel_toe.xml`.
- `meshes/` — Geometry referenced by the URDF / MJCF models.
- `launch/display_urdf_with_gui.launch.py` — Launch RViz with
  joint-state sliders and `robot_state_publisher`.
- `rviz/urdf.rviz` — Preconfigured RViz view.
- `scripts/imu_tf_broadcaster.py` — ROS 2 node that republishes an
  `/igris_c/imu` orientation as a TF on `base_link`.
- `scripts/export_model.py` — Standalone exporter that expands the canonical
  URDF or MuJoCo xacro and writes generated files into `urdf/` or `mujoco/`.
- `scripts/mujoco_file_test.py` — Minimal MuJoCo passive-viewer runner that
  expands `mujoco/<model>/<model>.xml.xacro`, writes a temporary MJCF, and steps the
  simulation.

## Canonical model structure

The description package now treats the xacro sources as the canonical robot
models. Legacy generated `.urdf` / `.xml` files are no longer the primary
authoring format.

Every `<xacro:arg>` default is the value production expands, so a no-argument run
yields what a robot actually loads. URDF and MJCF differ here: the URDF is always
expanded `base_type:=pelvis parallel:=false` (hardcoded in
`igris_c_controller/launch/launch_common.py` and `utils/generate_core_description.sh`;
the robot config has no key for either), while the MJCF takes `base_type`, `parallel`
and `fixed` from `igris_c.description.mujoco`.

- `urdf/<model>/<model>.urdf.xacro`
  - Args: `base_type`, `parallel`, `end_effector`, `ankle_version`, `wrist_version`,
    `battery_type` (igris_max only)
  - `base_type`: selects the torso-base or pelvis-base assembly order. Production
    expands `pelvis`, which is also the xacro default.
  - `parallel`: adds the waist / ankle / wrist parallel-link substructures.
  - `end_effector`: selects the lower-arm attachment (`dummy`, `none`, `hand`,
    `magnet`, `1dof`).
  - `ankle_version`: `v1` (original foot) or `v2` (reinforced foot). Selects the
    foot link inertial and mesh.
  - `wrist_version`: `v1` (intersecting roll/pitch axes) or `v2` (offset axes, both
    four-bars reworked). Selects the wrist link geometry, limits and meshes.
  - `battery_type`: `original` (the igris_max's own two hot-swap packs) or `legacy`
    (one carried-over igris_c pack). Swaps the torso mesh and inertial. Declared only by
    igris_max; igris_c has no such variant.
- `urdf/<model>/parallel_links.urdf.xacro`
  - Args: `variant`, `ankle_version`, `wrist_version`
  - `variant`: which single linkage to emit -- `waist`, `left_ankle`,
    `right_ankle`, `left_wrist`, `right_wrist`.
  - `ankle_version` / `wrist_version`: same meaning as above; only the matching
    `variant` reads them.
  - Purpose: standalone 4-bar linkage models used by `JointManager`.
- `mujoco/<model>/<model>.xml.xacro`
  - Args: `base_type`, `parallel`, `end_effector`, `fixed`, `ankle_version`,
    `wrist_version`, `mesh_dir`, `battery_type` (igris_max only)
  - `base_type`: torso floating-base chain vs pelvis floating/fixed chain. Production
    expands `torso` (`igris_c.description.mujoco.base_type`), which is also the xacro
    default.
  - `parallel`: selects serial vs parallel waist / ankle / wrist topology.
  - `end_effector`: selects the lower-arm payload module.
  - `fixed`: selects floating-base vs fixed-base MuJoCo model setup.
  - `ankle_version`: `v1` (original foot) or `v2` (reinforced foot). Selects the
    foot `<inertial>`, visual mesh, and the ankle parallel-link `<connect>` anchors.
  - `wrist_version`: `v1` or `v2`. v2 offsets the pitch axis from the roll axis, replaces the
    universal+hinge rod pair with a ball joint, and moves every wrist `<connect>` anchor. The
    ball joint adds 8 qpos, so the parallel keyframes branch on it too.
  - `battery_type`: same meaning and values as the URDF arg above.
  - `mesh_dir`: the `<compiler meshdir>` MuJoCo resolves mesh files against. MuJoCo
    resolves it relative to the MODEL FILE, so the default `../../meshes/` is correct
    only while this file sits at `mujoco/<model>/`. It does not on a robot: `install.sh`
    installs the selected model's tree FLAT, stripping that directory level, and launch
    writes the expanded XML to a temp directory anyway. Launch therefore passes
    `<description.config_root_dir>/meshes/` absolute, and the layout stops mattering.
    `export_model.py` and `mujoco_file_test.py` work on the repo tree and pass nothing,
    so the default covers them.

In both URDF and MuJoCo:

- `parallel` should only change the closed-loop mechanism topology and its
  directly dependent inertial / actuator / keyframe data.
- `end_effector` should only change the wrist-end payload module and its
  directly dependent assets.
- `ankle_version` is a shared physical fact (single key
  `igris_c.description.common.ankle_version`) and must match between the URDF and
  MuJoCo models. It changes only the foot link / geometry (and, in MuJoCo, the
  ankle parallel-link `<connect>` anchors).
- `wrist_version` is the same kind of shared physical fact (single key
  `igris_c.description.common.wrist_version`), applied to both wrists at once. It also selects
  the `<model>/parallel_links.urdf.xacro` wrist sub-model that `JointManager` loads, so the
  model and the kinematics constants stay in step.
- `battery_type` is a shared physical fact (single key
  `igris_c.description.common.battery_type`) that exists for igris_max only. It changes
  only the torso `<inertial>` and mesh -- no joint, frame or actuator changes -- so the
  gravity-compensation URDF and the simulated dynamics move together.
- Each body part lives in its own `body/` include; the entry file only declares
  the args and picks the base. Branching stays inside the file that owns the part.

## Include tree

Both formats use the same `body/` file set, so a part is found at the same path
under `urdf/<model>/` and `mujoco/<model>/`. The arg in brackets is what that
file branches on. `(shared)` marks a file that is a symlink into `common/`.

```
urdf/<model>/<model>.urdf.xacro                 args, base pick
└── body/{torso,pelvis}_base.urdf.xacro         [base_type] waist chain, parallel waist, IMU
    ├── body/left_leg.urdf.xacro      (shared)  [parallel, ankle_version]
    ├── body/right_leg.urdf.xacro     (shared)
    ├── body/left_arm.urdf.xacro                [parallel, wrist_version, end_effector=dummy]
    │   └── body/left_end_effector.urdf.xacro   (shared) [end_effector=hand|magnet|1dof]
    ├── body/right_arm.urdf.xacro
    │   └── body/right_end_effector.urdf.xacro  (shared)
    └── body/head.urdf.xacro
urdf/<model>/parallel_links.urdf.xacro          standalone 4-bar models [variant, ankle/wrist_version]

mujoco/<model>/<model>.xml.xacro                args, compiler/option/visual, root body, <sensor>
├── defaults.xml.xacro                          joint/motor classes
├── assets.xml.xacro                            materials, meshes
├── worldbody root <body>                       [base_type, fixed]
│   └── body/{torso,pelvis}_base.xml.xacro      [parallel] waist chain, parallel waist, IMU site
│       ├── body/left_leg.xml.xacro   (shared)  [parallel, ankle_version]
│       ├── body/right_leg.xml.xacro  (shared)
│       ├── body/left_arm.xml.xacro             [parallel, wrist_version, end_effector=dummy]
│       │   └── body/left_end_effector.xml.xacro  (shared) [end_effector=hand|magnet|1dof]
│       ├── body/right_arm.xml.xacro
│       │   └── body/right_end_effector.xml.xacro (shared)
│       └── body/head.xml.xacro
├── equality.xml.xacro            (shared)      [parallel, ankle/wrist_version] loop closures
├── actuator.xml.xacro                          [parallel] motor + position/velocity sets
└── keyframe.xml.xacro            (shared)      [all args] home pose, assembled per part
```

### Shared vs model-specific files

`igris_c/` and `igris_max/` list the same files, so one `ls` of a model directory is
the full parts list. A file that holds model-specific numbers (head, arms, bases,
assets, defaults, actuator) is a real file in each model directory. A file that is
the same on both models (legs, end effectors, equality, keyframe) exists once, in
`urdf/common/` or `mujoco/common/`, and each model directory holds a relative
symlink to it:

```
mujoco/igris_c/body/left_leg.xml.xacro -> ../../common/body/left_leg.xml.xacro
mujoco/igris_c/keyframe.xml.xacro      -> ../common/keyframe.xml.xacro
```

Rules:

- Editing a shared file changes both models. Its header comment says so.
- To make a shared part model-specific, replace the symlink in BOTH model
  directories with a real copy and remove the file from `common/`. Do not add a
  model branch inside a `common/` file.
- When two model copies become identical again, move one to `common/` and link both.
- Include paths never mention `common/`: the symlink keeps every include relative
  to the model directory, which is what the flat robot install relies on (below).

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

| Argument       | Default               | Description                                                                      |
| -------------- | --------------------- | -------------------------------------------------------------------------------- |
| `model`        | `igris_c`             | `igris_c` or `igris_max`. Derives `urdf_path` when that is empty.                |
| `urdf_package` | `igris_c_description` | Package the URDF / xacro is loaded from.                                         |
| `urdf_path`    | (empty)               | URDF or xacro path inside the package's share directory; empty derives it from `model`. |
| `end_effector` | `hand`                | End-effector variant when loading a xacro. See below.                            |

`base_type`, `parallel` and the version args are not launch arguments; they come from
the URDF xacro defaults (`pelvis`, `false`, `v1`, `v1`).

`end_effector` accepts:

- `dummy` — dummy spherical payload.
- `none` — no end effector.
- `hand` — articulated hand variant.
- `magnet` — magnet end-effector variant.
- `1dof` — 1-DOF gripper variant.

Examples:

```bash
# igris_c with the product end effector (hand)
ros2 launch igris_c_description display_urdf_with_gui.launch.py

# igris_max with the magnet end effector
ros2 launch igris_c_description display_urdf_with_gui.launch.py \
    model:=igris_max end_effector:=magnet
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

`scripts/mujoco_file_test.py` expands `mujoco/<model>/<model>.xml.xacro` with
defaults near the top of the file (the product MJCF variant), writes a temporary
MJCF, and opens MuJoCo's passive viewer until the window is closed.

Recommendation: use a local venv to avoid Conda libGL conflicts.

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install mujoco
python scripts/mujoco_file_test.py
```

To try a different variant, edit the constants in
[`scripts/mujoco_file_test.py`](scripts/mujoco_file_test.py):

- `MODEL`: `igris_c` or `igris_max`
- `BASE_TYPE`: `torso` or `pelvis`
- `PARALLEL`: `true` or `false`
- `END_EFFECTOR`: `dummy`, `none`, `hand`, `magnet`, or `1dof`
- `FIXED`: `true` or `false`
- `ANKLE_VERSION` / `WRIST_VERSION`: `v1` or `v2`
- `BATTERY_TYPE`: `original`, `legacy`, or `None` for the xacro default (igris_max only)

Each constant also has a matching `--flag`, so a one-off variant needs no edit:

```bash
python scripts/mujoco_file_test.py --model igris_max --battery-type legacy
```

## Export generated files

If you want to inspect a standalone generated `.urdf` or `.xml` file directly,
use `scripts/export_model.py`.

Examples:

Every flag defaults to what production expands, so a bare run exports the product
model. `--base-type` and `--parallel` resolve from `--format` because the two formats
differ: `pelvis` / `false` for `urdf`, `torso` / `true` for `xml`.

```bash
# Product URDF (gravity-compensation model): pelvis, serial, hand
python3 igris_c_description/scripts/export_model.py --format urdf --model igris_max

# Product MJCF: torso, parallel, free base, hand
python3 igris_c_description/scripts/export_model.py --format xml --model igris_max

# igris_max carrying the legacy (igris_c) battery pack
python3 igris_c_description/scripts/export_model.py \
  --format xml --model igris_max --battery-type legacy
```

Default output naming:

- URDF: `urdf/<model>/<model>_<base_type>_[parallel_]<end_effector>_<ankle_version>_<wrist_version>[_battery_<battery_type>].urdf`
- XML: `mujoco/<model>/<model>_<base_type>_[parallel_]<end_effector>_<free|fixed>_<ankle_version>_<wrist_version>[_battery_<battery_type>].xml`

The `_battery_*` suffix appears only when `--battery-type` is passed, so igris_c names
are unchanged.

The script writes into the package's `urdf/` and `mujoco/` directories so the
generated files can be opened independently without going through launch files
or the temporary-file path used by the MuJoCo viewer helper. Those outputs are
git-ignored (`urdf/igris_*/igris_*.urdf`, `mujoco/igris_*/igris_*.xml`); the xacro
sources stay the only tracked models.

### Wrist joint-limit stiffness

`mujoco/<model>/body/{left,right}_arm.xml.xacro` give `solreflimit="0.002 1"` to the
12 wrist joints (`*_wrist_flange_{front,back}`, `*_wrist_{roll,pitch}`).
Their `range` limits act as the load-bearing stops of the wrist four-bar.

MuJoCo's default limit `solref` timeconst is 0.02 s, 42x the timestep, which is
soft enough for a floor impact to push `*_wrist_roll` about 13 deg past its
limit. That drives the two `*_wrist_flange_*` joints to a pair
`Parallel4BarLink::fk()` cannot invert, and `fk()` then spends tens of
milliseconds failing — inside the MuJoCo control callback, on the physics
thread, which stalls stepping and rendering together.

| Limit `solreflimit` timeconst | Roll overshoot | Worst `motor2Joint` |
| --- | --- | --- |
| 0.02 (MuJoCo default) | 13.4 deg | 79,304 us |
| 0.01 | 7.7 deg | 59,530 us |
| 0.005 | 3.1 deg | 2,306 us |
| 0.002 (current) | 0.8 deg | 2,272 us |

`solimplimit` is left at the MuJoCo default: raising the impedance on top of
the timeconst change moved overshoot only from 0.8 to 0.4 deg. `dampratio`
stays 1 (critically damped).

This narrows how the wrist enters the unsolvable range; it does not remove the
underlying `fk()` cost. A hard enough impact still reaches it.

## Runtime config layout

The repository config uses this description structure:

```yaml
igris_c:
  description:
    config_root_dir: "/path/to/igris_c_description"
    urdf:
      robot_model_path: "urdf/igris_c/igris_c.urdf.xacro"
      parallel_links_path: "urdf/igris_c/parallel_links.urdf.xacro"
    mujoco:
      robot_model_path: "mujoco/igris_c/igris_c.xml.xacro"
      base_type: "pelvis"
      parallel: "true"
      fixed: "true"
  end_effector:
    type: "none"
```

### Repo layout vs installed layout

The paths above are the REPO layout, which is what `configs/robot/dev_<model>.yaml` uses.
A robot does not get that tree. `install.sh --model <m>` installs only the selected
model's files, with the `<model>/` level stripped:

```
repo                                    installed (<install_root>/configs/description)
urdf/igris_max/igris_max.urdf.xacro     urdf/igris_max.urdf.xacro
urdf/igris_max/parallel_links.urdf.xacro  urdf/parallel_links.urdf.xacro
urdf/igris_max/body/                    urdf/body/        (symlinks resolved to real files)
urdf/common/                            (not installed as such; reached via the links above)
mujoco/igris_max/igris_max.xml.xacro    mujoco/igris_max.xml.xacro
mujoco/igris_max/standalone/            (not installed)
meshes/{common,igris_max}/              meshes/{common,igris_max}/   (unchanged)
(expanded at package time)              urdf/igris_max_pelvis_<ee>.urdf   (web 3D view)
```

The installer copies the model tree with `tar -h`, so every `common/` symlink lands
as a plain file and the installed tree has no symlinks and no `common/`.

The installed config is `robot.yaml` in the robot's configs directory (one name per robot,
whichever model was installed), and it names `urdf/<model>.urdf.xacro`, not
`urdf/<model>/<model>.urdf.xacro`; only `configs/robot/dev_<model>.yaml` keeps the nested
form, because dev runs read the repo tree directly. Nothing inside a model directory refers to that
directory by name: xacro `<xacro:include>` paths are relative to their own file and move
with it, the URDF reaches meshes through `package://igris_c_description/meshes/...`, and
MJCF `meshdir` is injected absolute (see `mesh_dir` above). `meshes/` keeps its
subdirectories because MJCF asset paths name `common/` and `<model>/` explicitly.

`<model>.xml.xacro` covers the normal MuJoCo model variants. Only
`standalone/parallel_collision.xml` and `standalone/parallel_toe.xml` remain as
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
    `end_effector`, `fixed`; `mesh_dir` is optional (xacro drops a mapping the file does
    not declare, and launch then falls back to rewriting `meshdir` to an absolute path
    relative to the xacro's own directory)
## End-to-end examples

Generate the default controller URDF from xacro:

```bash
xacro igris_c_description/urdf/igris_c/igris_c.urdf.xacro \
  base_type:=pelvis \
  parallel:=false \
  end_effector:=dummy
```

Generate the default simulation MJCF from xacro:

```bash
xacro igris_c_description/mujoco/igris_c/igris_c.xml.xacro \
  base_type:=pelvis \
  parallel:=true \
  end_effector:=dummy \
  fixed:=true
```

The default MuJoCo variant used for simulation is:

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
