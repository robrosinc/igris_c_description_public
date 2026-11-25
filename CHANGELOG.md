# Changelog

Changes in each IGRIS-C Description release. Every release lists the changes
in English first, then in Korean.

<!--
How to add a release:
  - Add a `## vX.Y.Z` section at the top, matching the release tag exactly.
  - Write for the people who use the release: what changed for them, not how.
  - No ticket keys, internal paths or service names.
  - Jenkins publishes this section as the GitHub Release notes and fails
    the release build if the section is missing.
-->

## v2.1.3

### English

- Added the igris_max model. Each model has its own entry files: `urdf/<model>/<model>.urdf.xacro`,
  `urdf/<model>/parallel_links.urdf.xacro` and `mujoco/<model>/<model>.xml.xacro`, with the
  meshes under `meshes/<model>/` and shared parts under `meshes/common/`, `urdf/common/` and
  `mujoco/common/`. The earlier single-model files (`igris_c_v2.*`) are replaced by the
  `igris_c` entries.
- New `battery_type` argument for igris_max (`original` or `legacy`). It changes the torso
  mesh and inertia only.
- `display_urdf_with_gui.launch.py` takes a `model` argument (`igris_c` or `igris_max`), and
  its default end effector is now `hand`.
- The `igris_c` xacro argument defaults changed from v2.1.1: `base_type` is now `pelvis` (was
  `torso`) and `end_effector` is now `hand` (was `dummy`), so expanding it without arguments
  gives a different model than in v2.1.1. The same explicit arguments (for example
  `base_type:=torso end_effector:=dummy`) give the same joint tree as in v2.1.1.
- The magnet end effector has a revolute rotation joint (`l_magnet_rotate` / `r_magnet_rotate`)
  in both the URDF and the MuJoCo model, adhesion actuators in the MuJoCo model, and updated
  magnet meshes. URDF users of the magnet configuration see one more joint per arm. The URDF
  of the magnet configuration has no `left_hand_camera_link` / `right_hand_camera_link`.
- `torso_imu_link` is fixed to `torso` rotated +90 degrees about Y (`rpy="0 1.570796327 0"`) in
  both models: the frame the IMU data is published in, and the same rotation as the IMU site
  in the MuJoCo model. For igris_c the TF tree is the same as in v2.1.1, and IMU data stamped
  with `torso_imu_link` transforms into `torso` the same way on igris_max.
- The `package.xml` license is written as `BSD-3-Clause`, and a `LICENSE` file is included.
- Removed unused legacy meshes (`meshes/wasted/`).

### 한국어

- igris_max 모델을 추가했습니다. 모델마다 진입 파일이 따로 있습니다:
  `urdf/<model>/<model>.urdf.xacro`, `urdf/<model>/parallel_links.urdf.xacro`,
  `mujoco/<model>/<model>.xml.xacro`. 메시는 `meshes/<model>/`, 공용 부품은
  `meshes/common/`, `urdf/common/`, `mujoco/common/`에 있습니다. 이전의 단일 모델 파일(`igris_c_v2.*`)은
  `igris_c` 진입 파일로 대체되었습니다.
- igris_max용 `battery_type` 인자(`original` 또는 `legacy`)를 추가했습니다. 토르소 메시와 관성만 바뀝니다.
- `display_urdf_with_gui.launch.py`에 `model` 인자(`igris_c` 또는 `igris_max`)가 생겼고, 기본 엔드이펙터는
  `hand`입니다.
- `igris_c` xacro 인자의 기본값이 v2.1.1에서 바뀌었습니다. `base_type`은 `pelvis`(이전 `torso`), `end_effector`는
  `hand`(이전 `dummy`)이므로 인자 없이 전개하면 v2.1.1과 다른 모델이 나옵니다. 같은 인자를 명시하면(예:
  `base_type:=torso end_effector:=dummy`) v2.1.1과 같은 관절 트리가 나옵니다.
- 자석 엔드이펙터에 회전 관절(`l_magnet_rotate` / `r_magnet_rotate`, revolute)을 URDF와 MuJoCo 모델 모두에
  추가하고, MuJoCo 모델에 흡착 액추에이터를 추가했으며, 자석 메시를 갱신했습니다. 자석 구성을 쓰는 URDF
  사용자에게는 팔마다 관절이 하나 늘어납니다. 자석 구성의 URDF에는 `left_hand_camera_link` /
  `right_hand_camera_link`가 없습니다.
- `torso_imu_link`는 두 모델 모두 `torso`에서 Y축으로 +90도 회전한 자세(`rpy="0 1.570796327 0"`)로
  고정됩니다. IMU 데이터가 발행되는 프레임이며 MuJoCo 모델의 IMU site와 같은 회전입니다. igris_c의 TF
  트리는 v2.1.1과 같고, igris_max에서도 `torso_imu_link`로 찍힌 IMU 데이터가 같은 방식으로 `torso`로
  변환됩니다.
- `package.xml`의 라이선스를 `BSD-3-Clause`로 표기하고 `LICENSE` 파일을 포함했습니다.
- 사용하지 않는 예전 메시(`meshes/wasted/`)를 삭제했습니다.
