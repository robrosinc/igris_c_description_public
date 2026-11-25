import argparse
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco.viewer
import xacro

import mujoco

PACKAGE_ROOT = Path(__file__).resolve().parents[1]

# Defaults are the product MJCF variant (description.mujoco in the robot config).
# Adjust them to try a different variant, or pass the matching --flag. Hint: python3 scripts/mujoco_file_test.py --model igris_max
MODEL = "igris_c"  # "igris_c" or "igris_max"
BASE_TYPE = "torso"  # "torso" or "pelvis"
PARALLEL = "true"  # "true" or "false"
END_EFFECTOR = "hand"  # "dummy", "none", "hand", "magnet", or "1dof"
FIXED = "false"  # "true" or "false"
ANKLE_VERSION = "v1"  # "v1" (original foot) or "v2" (reinforced foot)
WRIST_VERSION = "v1"  # "v1" (intersecting roll/pitch axes) or "v2" (offset axes)
BATTERY_TYPE = None  # igris_max only: "original", "legacy", or None for the xacro default

parser = argparse.ArgumentParser(
    description="Open one generated MJCF variant in the MuJoCo viewer."
)
parser.add_argument("--model", choices=["igris_c", "igris_max"], default=MODEL)
parser.add_argument("--base-type", choices=["torso", "pelvis"], default=BASE_TYPE)
parser.add_argument("--parallel", choices=["true", "false"], default=PARALLEL)
parser.add_argument(
    "--end-effector", choices=["dummy", "none", "hand", "magnet", "1dof"], default=END_EFFECTOR
)
parser.add_argument("--fixed", choices=["true", "false"], default=FIXED)
parser.add_argument("--ankle-version", choices=["v1", "v2"], default=ANKLE_VERSION)
parser.add_argument("--wrist-version", choices=["v1", "v2"], default=WRIST_VERSION)
parser.add_argument("--battery-type", choices=["original", "legacy"], default=BATTERY_TYPE)
args = parser.parse_args()

XACRO_PATH = PACKAGE_ROOT / "mujoco" / args.model / f"{args.model}.xml.xacro"
if not XACRO_PATH.is_file():
    sys.exit(f"No MJCF for model {args.model!r}: {XACRO_PATH} does not exist.")

processed = xacro.process_file(
    str(XACRO_PATH),
    mappings={
        "base_type": args.base_type,
        "parallel": args.parallel,
        "end_effector": args.end_effector,
        "fixed": args.fixed,
        "ankle_version": args.ankle_version,
        "wrist_version": args.wrist_version,
        **({"battery_type": args.battery_type} if args.battery_type else {}),
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
