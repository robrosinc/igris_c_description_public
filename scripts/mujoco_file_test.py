import argparse
import os
import time
from pathlib import Path


def load_model(xml_path: Path):
    import mujoco

    model = mujoco.MjModel.from_xml_path(xml_path.as_posix())
    data = mujoco.MjData(model)
    return model, data, mujoco


def run_headless(mujoco, model, data, steps: int, xml_path: Path, realtime: bool) -> None:
    start_time = time.time()
    for _ in range(steps):
        if realtime:
            target = start_time + data.time
            now = time.time()
            if target > now:
                time.sleep(target - now)
        mujoco.mj_step(model, data)
    print(f"Headless run complete for {steps} steps using {xml_path}")


def run_viewer(mujoco, model, data, realtime: bool) -> bool:
    try:
        import mujoco.viewer as mj_viewer
    except Exception as exc:  # pragma: no cover - viewer import is optional
        print(f"Viewer unavailable ({exc}); try --gl egl/osmesa or --headless")
        return False

    try:
        with mj_viewer.launch_passive(model, data) as viewer:
            print("Viewer launched. Press ESC to quit.")
            start_time = time.time()
            while viewer.is_running():
                if realtime:
                    target = start_time + data.time
                    now = time.time()
                    if target > now:
                        time.sleep(target - now)
                mujoco.mj_step(model, data)
                viewer.sync()
        return True
    except KeyboardInterrupt:
        raise
    except BaseException as exc:  # pragma: no cover - viewer errors are environment-specific
        # Some environments raise SystemExit when GLFW/GLX cannot create a window.
        print(f"Viewer failed to launch ({exc}); try --gl egl/osmesa or --headless")
        return False


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true", help="Run without viewer and just step the sim")
    parser.add_argument(
        "--force-viewer",
        action="store_true",
        help="Attempt viewer even if display/GL backend look unsuitable (may crash in headless setups)",
    )
    parser.add_argument("--steps", type=int, default=1000, help="Number of steps to run in headless mode")
    parser.add_argument(
        "--gl",
        choices=["egl", "osmesa", "glfw"],
        help="Set MUJOCO_GL backend before creating the context",
    )
    parser.add_argument(
        "--xml",
        default=None,
        help="Path to MJCF/XML file (defaults to repo mujoco/igris_c_v2.xml)",
    )
    parser.add_argument(
        "--keyframe",
        default="initial",
        help="Keyframe name to initialize state from (must exist in the XML).",
    )
    parser.add_argument(
        "--realtime",
        action="store_true",
        help="Pace simulation to wall clock based on model timestep.",
    )
    args = parser.parse_args()

    if args.gl:
        os.environ["MUJOCO_GL"] = args.gl

    repo_root = Path(__file__).resolve().parents[1]
    xml_path = Path(args.xml) if args.xml else repo_root / "mujoco" / "igris_c_v2.xml"

    model, data, mujoco = load_model(xml_path)

    # Initialize from keyframe if available.
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, args.keyframe)
    if key_id >= 0:
        mujoco.mj_resetDataKeyframe(model, data, key_id)
        print(f"Initialized state from keyframe '{args.keyframe}'.")
    else:
        print(f"Keyframe '{args.keyframe}' not found; using default initial state.")

    has_display = bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    gl_backend = (os.environ.get("MUJOCO_GL") or "").lower()
    backend_headless = gl_backend in {"egl", "osmesa"}

    should_try_viewer = (
        not args.headless and has_display and (not backend_headless or args.force_viewer)
    )

    if not args.headless and not has_display:
        print(
            "No DISPLAY/WAYLAND_DISPLAY detected; skipping viewer. "
            "Use --headless, or run under a virtual display (e.g. xvfb-run) if you need the viewer."
        )
    elif backend_headless and not args.force_viewer and not args.headless:
        print(
            f"MUJOCO_GL={gl_backend} indicates a headless backend; skipping viewer. "
            "Pass --force-viewer to try anyway (may fail/crash in headless setups)."
        )

    if should_try_viewer:
        if run_viewer(mujoco, model, data, args.realtime):
            return
        print("Falling back to headless mode after viewer failure.")

    run_headless(mujoco, model, data, args.steps, xml_path, args.realtime)


if __name__ == "__main__":
    main()
