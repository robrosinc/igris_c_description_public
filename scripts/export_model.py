#!/usr/bin/env python3

import argparse
from pathlib import Path

import xacro

PACKAGE_NAME = "igris_c_description"


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
    base_type: str, parallel: str, end_effector: str, fixed: str | None = None
) -> str:
    parts = ["igris_c_v2", base_type]
    if parallel == "true":
        parts.append("parallel")
    parts.append(end_effector)
    if fixed is not None:
        parts.append("fixed" if fixed == "true" else "free")
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
        "--base-type",
        choices=["torso", "pelvis"],
        default="pelvis",
        help="xacro base_type argument.",
    )
    parser.add_argument(
        "--parallel",
        type=_str_to_bool_string,
        default="false",
        help="xacro parallel argument: true or false.",
    )
    parser.add_argument(
        "--end-effector",
        choices=["dummy", "none", "hand", "magnet", "1dof"],
        default="dummy",
        help="xacro end-effector variant.",
    )
    parser.add_argument(
        "--fixed",
        type=_str_to_bool_string,
        default="true",
        help="xacro fixed argument used for MuJoCo XML: true or false.",
    )
    parser.add_argument(
        "--package-root",
        type=Path,
        default=None,
        help="Optional override for the igris_c_description package root.",
    )
    args = parser.parse_args()

    package_root = args.package_root.resolve() if args.package_root else _find_package_root()

    generated_paths: list[Path] = []

    if args.format == "urdf":
        urdf_xacro_path = package_root / "urdf" / "igris_c_v2.urdf.xacro"
        urdf_output_path = (
            package_root
            / "urdf"
            / f"{_build_variant_suffix(args.base_type, args.parallel, args.end_effector)}.urdf"
        )
        _write_generated_file(
            urdf_xacro_path,
            {
                "base_type": args.base_type,
                "parallel": args.parallel,
                "end_effector": args.end_effector,
            },
            urdf_output_path,
        )
        generated_paths.append(urdf_output_path)

    if args.format == "xml":
        xml_xacro_path = package_root / "mujoco" / "igris_c_v2.xml.xacro"
        xml_output_path = (
            package_root
            / "mujoco"
            / f"{_build_variant_suffix(args.base_type, args.parallel, args.end_effector, args.fixed)}.xml"
        )
        _write_generated_file(
            xml_xacro_path,
            {
                "base_type": args.base_type,
                "parallel": args.parallel,
                "end_effector": args.end_effector,
                "fixed": args.fixed,
            },
            xml_output_path,
        )
        generated_paths.append(xml_output_path)

    for path in generated_paths:
        print(path)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
