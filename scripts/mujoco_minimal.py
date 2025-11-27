"""Minimal MuJoCo runner for a given XML."""
import argparse
import os
import time
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a MuJoCo model from XML (viewer optional).")
    parser.add_argument(
        "--xml",
        type=Path,
        help="Path to MJCF/XML file (defaults to repo mujoco/igris_c_v2.xml)",
    )
    parser.add_argument("--steps", type=int, default=1000, help="Steps to run in headless mode")
    parser.add_argument("--headless", action="store_true", help="Skip viewer and run headless")
    parser.add_argument("--gl", choices=["egl", "osmesa", "glfw"], help="Set MUJOCO_GL backend")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    if args.gl:
        os.environ["MUJOCO_GL"] = args.gl

    repo_root = Path(__file__).resolve().parents[1]
    xml_path = args.xml if args.xml else repo_root / "mujoco" / "igris_c_v2.xml"

    import mujoco

    model = mujoco.MjModel.from_xml_path(xml_path.as_posix())
    data = mujoco.MjData(model)

    if not args.headless:
        try:
            import mujoco.viewer as mj_viewer
        except BaseException as exc:
            print(f"Viewer unavailable ({exc}); falling back to headless.")
        else:
            try:
                with mj_viewer.launch_passive(model, data) as viewer:
                    print("Viewer running. Press ESC to quit.")
                    while viewer.is_running():
                        mujoco.mj_step(model, data)
                        viewer.sync()
                        time.sleep(0.01)
                    return
            except KeyboardInterrupt:
                raise
            except BaseException as exc:
                print(f"Viewer failed ({exc}); falling back to headless.")

    for _ in range(args.steps):
        mujoco.mj_step(model, data)
    print(f"Headless run complete for {args.steps} steps using {xml_path}")


if __name__ == "__main__":
    main()
