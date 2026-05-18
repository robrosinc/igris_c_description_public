import time
from pathlib import Path

import mujoco
import mujoco.viewer

# Load your MJCF file
model = mujoco.MjModel.from_xml_path(
    str(Path(__file__).resolve().parents[1] / "mujoco" / "igris_c_v2.xml")
)  # or .mjcf


data = mujoco.MjData(model)

# Launch interactive viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    print("Viewer launched. Press ESC to quit.")

    # Run simulation until viewer is closed
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.01)
