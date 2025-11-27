# igris_c_description_public

ROS 2 package containing the IGRIS-C robot description (URDF, meshes, RViz config) plus MuJoCo MJCF models and helper scripts to visualize and simulate the robot.

## Contents
- `urdf/`: URDF for the robot (default: `urdf/igris_c_v2.urdf`).
- `mujoco/`: MuJoCo MJCF models (e.g., `igris_c_v2.xml`, parallel variants, keyframes).
- `meshes/`: Geometry used by the URDF.
- `launch/display_urdf_with_gui.launch.py`: Launch RViz with joint sliders and state publisher.
- `rviz/urdf.rviz`: Preconfigured RViz view.
- Scripts:
  - `scripts/imu_tf_broadcaster.py`: ROS 2 node to broadcast IMU orientation as TF.
  - `scripts/mujoco_file_test.py`: Full-feature MuJoCo runner (keyframe init, realtime pacing, optional viewer/headless).
  - `scripts/mujoco_minimal.py`: Minimal MuJoCo runner for quick checks.

## Requirements
- ROS 2 with `ament_cmake`, `robot_state_publisher`, `joint_state_publisher_gui`, and `rviz2`.
- Python 3.8+.
- MuJoCo Python package (`pip install mujoco`) and GLFW (`pip install glfw`).
- A working X/Wayland display and OpenGL drivers if you want the MuJoCo viewer. If you hit GLX/GLFW errors under Conda, prefer a venv that uses system `libGL`.

## Install (ROS 2)
```bash
cd /path/to/your/ros2_ws/src
git clone <this repo> igris_c_description_public
cd ..
colcon build --packages-select igris_c_description_public
source install/setup.bash
```

## RViz visualization
Launch the URDF with sliders and RViz:
```bash
ros2 launch igris_c_description_public display_urdf_with_gui.launch.py \
  urdf_path:=urdf/igris_c_v2.urdf
```
Override `urdf_path` if you want a different URDF in `share/igris_c_description_public`.

## MuJoCo simulation (Python)
Recommendation: use a local venv to avoid Conda `libGL` conflicts.
```bash
python3 -m venv .venv
source .venv/bin/activate
pip install mujoco glfw
```

### Full runner
```bash
python scripts/mujoco_file_test.py --gl glfw --realtime \
  --xml mujoco/igris_c_v2.xml --keyframe initial
```
- `--gl {glfw,egl,osmesa}` sets `MUJOCO_GL`.
- `--keyframe` initializes state from a keyframe in the XML (default `initial`).
- `--realtime` paces sim to wall clock; omit to run as fast as possible.
- `--headless` skips the viewer; `--force-viewer` tries even with headless backends.

### Minimal runner
```bash
python scripts/mujoco_minimal.py --gl glfw --xml mujoco/igris_c_v2.xml
```
Add `--headless` to skip the viewer or `--steps N` to limit headless steps.

### Common viewer tips
- Run from a terminal inside your desktop session (`DISPLAY` set, `glxinfo -B` works).
- If the viewer fails with GLX/GLFW errors under Conda, use the venv above or ensure system `libGL` is picked up.
