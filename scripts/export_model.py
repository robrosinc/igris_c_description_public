#!/usr/bin/env python3

import argparse
from pathlib import Path

import xacro

PACKAGE_NAME = "igris_c_description"
MODEL_OPTIONS = ("igris_c", "igris_max")


def _str_to_bool_string(value: str) -> str:
    normalized = value.strip().lower()
    if normalized not in {"true", "false"}:
        raise argparse.ArgumentTypeError("expected 'true' or 'false'")
    return normalized


def _find_package_root() -> Path:
    script_path = Path(__file__).resolve()

    for candidate in [script_path.parent] + list(script_path.parents):
        if (candidate / "package.xml").is_file() and (candidate / "urdf").is_dir():
            return candidate

    for candidate in [script_path.parent] + list(script_path.parents):
        share_dir = candidate / "share" / PACKAGE_NAME
        if (share_dir / "urdf").is_dir() and (share_dir / "mujoco").is_dir():
            return share_dir

    raise RuntimeError("Failed to locate igris_c_description package root")


def _build_variant_suffix(
    model: str,
    base_type: str,
    parallel: str,
    end_effector: str,
    fixed: str | None = None,
    ankle_version: str = "v1",
    wrist_version: str = "v1",
    battery_type: str | None = None,
) -> str:
    parts = [model, base_type]
    if parallel == "true":
        parts.append("parallel")
    parts.append(end_effector)
    if fixed is not None:
        parts.append("fixed" if fixed == "true" else "free")
    parts.append(ankle_version)
    parts.append(wrist_version)
    # igris_max only; omitted from the suffix when absent so igris_c names are unchanged.
    if battery_type is not None:
        parts.append(f"battery_{battery_type}")
    return "_".join(parts)


def _write_generated_file(xacro_path: Path, mappings: dict[str, str], output_path: Path) -> None:
    processed = xacro.process_file(str(xacro_path), mappings=mappings)
    output_path.write_text(processed.toprettyxml(indent="  "), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Generate standalone URDF or MuJoCo XML files from igris_c_description xacro sources."
    )
    parser.add_argument(
        "--format",
        choices=["urdf", "xml"],
        required=True,
        help="Which output format to generate.",
    )
    parser.add_argument(
        "--model",
        choices=list(MODEL_OPTIONS),
        default="igris_c",
        help="Robot model; selects urdf/<model>/<model>.urdf.xacro and mujoco/<model>/<model>.xml.xacro.",
    )
    parser.add_argument(
        "--base-type",
        choices=["torso", "pelvis"],
        default=None,
        help="xacro base_type argument. Defaults to what production expands for the chosen "
        "format: pelvis for urdf, torso for xml.",
    )
    parser.add_argument(
        "--parallel",
        type=_str_to_bool_string,
        default=None,
        help="xacro parallel argument: true or false. Defaults to what production expands "
        "for the chosen format: false for urdf, true for xml.",
    )
    parser.add_argument(
        "--end-effector",
        choices=["dummy", "none", "hand", "magnet", "1dof"],
        default="hand",
        help="xacro end-effector variant. Default matches the product config; a magnet robot needs --end-effector magnet.",
    )
    parser.add_argument(
        "--fixed",
        type=_str_to_bool_string,
        default="false",
        help="xacro fixed argument used for MuJoCo XML: true or false. Default matches the "
        "product config: a free-floating base, not one pinned in the air.",
    )
    parser.add_argument(
        "--ankle-version",
        choices=["v1", "v2"],
        default="v1",
        help="xacro ankle_version argument: v1 (original foot) or v2 (reinforced foot).",
    )
    parser.add_argument(
        "--wrist-version",
        choices=["v1", "v2"],
        default="v1",
        help="xacro wrist_version argument: v1 (intersecting roll/pitch axes) or v2 (offset axes).",
    )
    parser.add_argument(
        "--battery-type",
        choices=["original", "legacy"],
        default=None,
        help="xacro battery_type argument (igris_max only): original (two hot-swap "
        "igris_max packs) or legacy (one carried-over igris_c pack). Defaults to the "
        "xacro default.",
    )
    parser.add_argument(
        "--package-root",
        type=Path,
        default=None,
        help="Optional override for the igris_c_description package root.",
    )
    args = parser.parse_args()

    # Production expands the URDF pelvis-rooted and serial (launch_common.py and
    # generate_core_description.sh both hardcode it; the robot config has no urdf key for
    # either), and the MJCF from description.mujoco in the robot config.
    if args.base_type is None:
        args.base_type = "pelvis" if args.format == "urdf" else "torso"
    if args.parallel is None:
        args.parallel = "false" if args.format == "urdf" else "true"

    package_root = args.package_root.resolve() if args.package_root else _find_package_root()

    generated_paths: list[Path] = []

    if args.format == "urdf":
        urdf_xacro_path = package_root / "urdf" / args.model / f"{args.model}.urdf.xacro"
        urdf_output_path = (
            urdf_xacro_path.parent
            / f"{_build_variant_suffix(args.model, args.base_type, args.parallel, args.end_effector, ankle_version=args.ankle_version, wrist_version=args.wrist_version, battery_type=args.battery_type)}.urdf"
        )
        _write_generated_file(
            urdf_xacro_path,
            {
                "base_type": args.base_type,
                "parallel": args.parallel,
                "end_effector": args.end_effector,
                "ankle_version": args.ankle_version,
                "wrist_version": args.wrist_version,
                **({"battery_type": args.battery_type} if args.battery_type else {}),
            },
            urdf_output_path,
        )
        generated_paths.append(urdf_output_path)

    if args.format == "xml":
        xml_xacro_path = package_root / "mujoco" / args.model / f"{args.model}.xml.xacro"
        xml_output_path = (
            xml_xacro_path.parent
            / f"{_build_variant_suffix(args.model, args.base_type, args.parallel, args.end_effector, args.fixed, args.ankle_version, args.wrist_version, args.battery_type)}.xml"
        )
        _write_generated_file(
            xml_xacro_path,
            {
                "base_type": args.base_type,
                "parallel": args.parallel,
                "end_effector": args.end_effector,
                "fixed": args.fixed,
                "ankle_version": args.ankle_version,
                "wrist_version": args.wrist_version,
                **({"battery_type": args.battery_type} if args.battery_type else {}),
            },
            xml_output_path,
        )
        generated_paths.append(xml_output_path)

    for path in generated_paths:
        print(path)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
