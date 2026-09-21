from __future__ import annotations

import argparse
from pathlib import Path

from urdf_parser_py.urdf import Joint, URDF


_REPO_ROOT = Path(__file__).resolve().parents[2]
_DEFAULT_URDF = (
    _REPO_ROOT / "models" / "ninezzhou" / "urdf" / "ninezzhou.urdf"
)
_DEFAULT_OUTPUT = (
    _REPO_ROOT / "portable_oscbf" / "work" / "kinematics_data.py"
)
_ACTIVE_JOINT_TYPES = frozenset({"prismatic", "revolute"})
_SUPPORTED_JOINT_TYPES = _ACTIVE_JOINT_TYPES | {"fixed"}
_LINK_ALIASES = {"tool0": "ee_link"}


def _vector(
    values: list[float] | tuple[float, ...] | None,
    default: tuple[float, float, float],
) -> tuple[float, float, float]:
    if values is None:
        return default
    if len(values) != len(default):
        raise ValueError(f"expected {len(default)} vector values, found {len(values)}")
    return tuple(float(value) for value in values)


def _ordered_joints(robot: URDF) -> tuple[str, list[Joint]]:
    root_link = robot.get_root()
    joints_by_name = {joint.name: joint for joint in robot.joints}
    ordered: list[Joint] = []
    current_link = root_link

    while current_link in robot.child_map:
        candidates = robot.child_map[current_link]
        if len(candidates) != 1:
            names = [joint_name for joint_name, _ in candidates]
            raise ValueError(
                f"URDF is not a serial chain at {current_link!r}: {names}"
            )
        joint_name, current_link = candidates[0]
        joint = joints_by_name[joint_name]
        if joint.type not in _SUPPORTED_JOINT_TYPES:
            raise ValueError(
                f"joint {joint.name!r} has unsupported type {joint.type!r}"
            )
        ordered.append(joint)

    if len(ordered) != len(robot.joints):
        raise ValueError("URDF contains joints outside the root-to-tip serial chain")
    return root_link, ordered


def _alias_link(link_name: str) -> str:
    return _LINK_ALIASES.get(link_name, link_name)


def _joint_row(joint: Joint) -> tuple[object, ...]:
    origin = joint.origin
    xyz = _vector(origin.xyz if origin is not None else None, (0.0, 0.0, 0.0))
    rpy = _vector(origin.rpy if origin is not None else None, (0.0, 0.0, 0.0))
    default_axis = (
        (1.0, 0.0, 0.0)
        if joint.type in _ACTIVE_JOINT_TYPES
        else (0.0, 0.0, 1.0)
    )
    axis = _vector(joint.axis, default_axis)
    return (
        _alias_link(joint.parent),
        _alias_link(joint.child),
        joint.type,
        *xyz,
        *rpy,
        axis,
    )


def _position_limits(
    joints: list[Joint],
) -> tuple[tuple[float, ...], tuple[float, ...]]:
    lower: list[float] = []
    upper: list[float] = []
    for joint in joints:
        if joint.type not in _ACTIVE_JOINT_TYPES:
            continue
        if (
            joint.limit is None
            or joint.limit.lower is None
            or joint.limit.upper is None
        ):
            raise ValueError(
                f"active joint {joint.name!r} is missing position limits"
            )
        lower.append(float(joint.limit.lower))
        upper.append(float(joint.limit.upper))
    return tuple(lower), tuple(upper)


def _format_tuple(name: str, values: tuple[float, ...]) -> list[str]:
    lines = [f"{name} = ("]
    lines.extend(f"    {value!r}," for value in values)
    lines.append(")")
    return lines


def render_kinematics_data(urdf_path: Path) -> str:
    robot = URDF.from_xml_string(urdf_path.read_bytes())
    root_link, joints = _ordered_joints(robot)
    rows = [
        (
            "world",
            _alias_link(root_link),
            "fixed",
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            (0.0, 0.0, 1.0),
        )
    ]
    rows.extend(_joint_row(joint) for joint in joints)
    lower, upper = _position_limits(joints)

    lines = [
        "# 由 portable_oscbf/scripts/generate_kinematics_data.py 生成。",
        "# 手工来源：models/ninezzhou/urdf/ninezzhou.urdf。",
        "",
        "from work.robot_geometry import LINK_NAMES",
        "",
        "",
        "JOINT_CHAIN = [",
    ]
    lines.extend(f"    {row!r}," for row in rows)
    lines.extend(["]", ""])
    lines.extend(_format_tuple("JOINT_POSITION_LOWER", lower))
    lines.append("")
    lines.extend(_format_tuple("JOINT_POSITION_UPPER", upper))
    lines.extend(["", f"N_JOINTS = {len(lower)}", ""])
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", type=Path, default=_DEFAULT_URDF)
    parser.add_argument("--output", type=Path, default=_DEFAULT_OUTPUT)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--check", action="store_true")
    mode.add_argument("--stdout", action="store_true")
    args = parser.parse_args()

    rendered = render_kinematics_data(args.urdf)
    if args.check:
        current = args.output.read_text(encoding="utf-8")
        if current != rendered:
            generator = Path(__file__).relative_to(_REPO_ROOT)
            raise SystemExit(
                f"generated kinematics data differs: run {generator}"
            )
        return 0
    if args.stdout:
        print(rendered, end="")
        return 0

    args.output.write_text(rendered, encoding="utf-8", newline="\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
