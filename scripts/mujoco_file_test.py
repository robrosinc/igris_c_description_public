import tempfile
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco.viewer
import xacro

import mujoco

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
XACRO_PATH = PACKAGE_ROOT / "mujoco" / "igris_c_v2.xml.xacro"

# Adjust these defaults to try a different generated MJCF variant.
BASE_TYPE = "pelvis"  # "torso" or "pelvis"
PARALLEL = "true"  # "true" or "false"
END_EFFECTOR = "dummy"  # "dummy", "none", "hand", "magnet", or "1dof"
FIXED = "true"  # "true" or "false"

processed = xacro.process_file(
    str(XACRO_PATH),
    mappings={
        "base_type": BASE_TYPE,
        "parallel": PARALLEL,
        "end_effector": END_EFFECTOR,
        "fixed": FIXED,
    },
)

root = ET.fromstring(processed.toxml())
compiler = root.find("compiler")
if compiler is not None:
    meshdir = compiler.get("meshdir", "").strip()
    if meshdir and not Path(meshdir).is_absolute():
        compiler.set("meshdir", str((XACRO_PATH.parent / meshdir).resolve()))
    texturedir = compiler.get("texturedir", "").strip()
    if texturedir and not Path(texturedir).is_absolute():
        compiler.set("texturedir", str((XACRO_PATH.parent / texturedir).resolve()))

with tempfile.NamedTemporaryFile(suffix=".xml", delete=False) as fp:
    generated_xml_path = Path(fp.name)

ET.ElementTree(root).write(generated_xml_path, encoding="utf-8", xml_declaration=False)

model = mujoco.MjModel.from_xml_path(str(generated_xml_path))


data = mujoco.MjData(model)

# Launch interactive viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    print("Viewer launched. Press ESC to quit.")

    # Run simulation until viewer is closed
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.01)
