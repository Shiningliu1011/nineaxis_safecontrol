import argparse
import io
import json
import platform
import shlex
import sys
from contextlib import redirect_stdout
from hashlib import sha256
from importlib.metadata import version
from itertools import pairwise
from pathlib import Path

import numpy as np
import yaml
from scipy.integrate import quad
from scipy.interpolate import PPoly
from scipy.io import loadmat
from work import kinematics_data
from work.continuous_path_reference import (
    ContinuousPathReference,
    curve_jet,
    load_reference_bundle,
    reference_identity,
)
from work.ik_data_loader import load_repository_trajectory

from .. import robot_spec
from .pipe_nurbs_geometry import _integrate, _spline, pipe_nurbs_geometry

SOURCE_FILES = ("ik_input.mat", "control_points.txt", "knot_vector.txt", "nurbs_blocks.mat")


def _digest(path):
    return sha256(Path(path).read_bytes()).hexdigest()


def generate_reference(source_dir, kinematics_config, controller_profile, output_dir, *,
                       scenario_config=None, command=None):
    source_dir, output_dir = Path(source_dir).resolve(), Path(output_dir).resolve()
    if output_dir.exists():
        raise FileExistsError(output_dir)
    sources = {name: source_dir / name for name in SOURCE_FILES}
    for path in sources.values():
        if not path.is_file():
            raise FileNotFoundError(path)
    kinematics_config, controller_profile = Path(kinematics_config).resolve(), Path(controller_profile).resolve()
    profile = yaml.safe_load(controller_profile.read_text())["/**"]["ros__parameters"]
    scenario = None if scenario_config is None else yaml.safe_load(Path(scenario_config).read_text())
    if profile["orientation_mode"] != "surface_normal" or profile["task_mode"] != "tool_axis_5d":
        raise ValueError("蝴蝶参考需要 surface_normal/tool_axis_5d profile")
    feed_multiplier = float(profile["reference_feedrate_scale"])
    radius = float(profile["path_radius_m"]) or None
    if scenario is not None:
        radius = float(scenario["path_radius_m"])
        feed_multiplier *= float(scenario["control"]["nominal_feed_multiplier"])
    captured = io.StringIO()
    with redirect_stdout(captured):
        trajectory = load_repository_trajectory(str(sources["ik_input.mat"]), str(kinematics_config), feed_multiplier)
        trajectory.set_surface_normal_orientation(
            profile["cylinder_axis_direction"], axis_point=profile.get("cylinder_center") or None,
            radius_m=radius)
    geometry = pipe_nurbs_geometry("load", source_dir)
    raw = loadmat(sources["ik_input.mat"], simplify_cells=True)["ik_input"]
    parameters = np.asarray(raw["u_series"])
    values = pipe_nurbs_geometry("eval", geometry, parameters)[0]
    raw_error = float(np.max(np.abs(values - raw["position_series"])))
    if raw_error > 1e-8:
        raise ValueError(f"source NURBS 与 MAT 位置不一致：{raw_error} mm")
    knots, control = geometry["knots"], geometry["ctrl_pts"]
    breaks = np.unique(knots)
    indices = np.searchsorted(knots, breaks[:-1], side="right") - 1
    coefficients = np.stack([PPoly.from_spline((knots, control[:, axis], 3)).c[:, indices].T
                             for axis in range(3)], axis=-1)
    transform = trajectory.T_traj_to_base
    coefficients = coefficients @ transform[:3, :3].T / 1000
    coefficients[:, -1] += transform[:3, 3]
    stages = trajectory.projection_stages
    reference = ContinuousPathReference.build(
        breaks, coefficients, np.array([stage["axis"] for stage in stages]),
        np.array([stage["centre_m"] for stage in stages]), np.array([stage["radius_m"] for stage in stages]),
        parameters, raw["time_series"], trajectory._feedrate,
    )
    samples = reference.query_u(parameters)
    position_error = float(np.max(np.abs(samples.position_m - trajectory._pos_world)))
    axis_error = float(np.max(np.abs(samples.tool_axis - trajectory._R_des_series[:, :, 0])))
    if position_error > 1e-12 or axis_error > 2e-14:
        raise ValueError("完整投影表达与实际跟踪目标不一致")
    config_inputs = {"kinematics": {"path": str(kinematics_config), "sha256": _digest(kinematics_config)},
                     "controller": {"path": str(controller_profile), "sha256": _digest(controller_profile)}}
    work_distance = {"status": "not_provided", "applied": True, "application": "existing_path_radius"}
    if scenario is not None:
        workpiece_radius = float(scenario["workpiece_radius_m"])
        if workpiece_radius <= 0 or workpiece_radius >= trajectory.surface_radius:
            raise ValueError("scenario workpiece_radius_m 需要位于路径圆柱内部")
        work_distance.update(status="nominal_simulation", distance_m=trajectory.surface_radius - workpiece_radius,
                             workpiece_radius_m=workpiece_radius, provenance="scenario_config")
        config_inputs["scenario"] = {"path": str(Path(scenario_config).resolve()), "sha256": _digest(scenario_config)}
    description = {
        "source_sha256": {name: _digest(path) for name, path in sources.items()},
        "frame": "base_link", "length_unit": "m", "angle_unit": "rad", "time_unit": "s",
        "source_length_unit": "mm", "parameter": "normalized_final_arc_length",
        "derivatives_wrt": "s", "position_derivative_unit": "m", "axis_derivative_unit": "dimensionless",
        "source_parameter": "u", "source_arc_unit": "mm", "final_arc_unit": "m", "dtype": "float64",
        "transform_kind": "nominal_simulation_similarity", "transform": transform.tolist(),
        "projection_axes": reference.arrays.projection_axes.tolist(),
        "projection_centres_m": reference.arrays.projection_centres_m.tolist(),
        "projection_radii_m": reference.arrays.projection_radii_m.tolist(),
        "tool_frame": "tool0", "tool_axis": "+X", "axis_definition": "inward_cylinder_radial",
        "normal_definition": "generated_final_cylinder_outward_radial",
        "original_normal_status": "not_available",
        "model_identity": {"name": "nineaxis", "controlled_joints": list(robot_spec.DEFAULT_JOINT_NAMES),
                           "joint_types": ["prismatic", *(["revolute"] * 8)],
                           "kinematics_data_sha256": _digest(kinematics_data.__file__),
                           "geometry_config_sha256": _digest(kinematics_config)},
        "C_source_tcp": [[0., 0., -1.], [0., 1., 0.], [1., 0., 0.]],
        "roll": "free_about_tool_axis", "nominal_rotation": "X_inward_Y_cylinder_axis",
        "side": "right_at_internal_nodes_left_at_endpoint", "source_degree": 3,
        "source_weights": "unit", "continuity": "C2_with_piecewise_third_derivatives",
        "calibration_status": "nominal_simulation_not_measured", "original_surface_association": "not_available",
        "nominal_feed_multiplier": feed_multiplier, "work_distance": work_distance,
        "nominal_time": "source_samples_with_linear_interpolation_and_duplicate_u_intervals",
        "nominal_plan_units": {"F": "mm/s", "At": "mm/s^2", "An": "mm/s^2", "Jt": "mm/s^3", "Jn": "mm/s^3"},
    }
    source_samples = {key: np.asarray(value) for key, value in raw.items()
                      if isinstance(value, (np.ndarray, int, float, np.number))}
    for key, value in list(source_samples.items()):
        source_samples[f"valid__{key}"] = np.isfinite(value)
    blocks = loadmat(sources["nurbs_blocks.mat"], simplify_cells=True)
    nominal_plan = {key: np.asarray(value) for key, value in blocks.items() if not key.startswith("__")}
    plan_cells = {}
    for key, value in nominal_plan.items():
        if value.dtype.hasobject:
            plan_cells[key] = {"shape": list(value.shape), "arrays": []}
            for index, cell in enumerate(value.ravel()):
                name = f"plan__{key}__cell_{index}"
                source_samples[name] = np.asarray(cell, dtype=float)
                plan_cells[key]["arrays"].append(name)
        else:
            source_samples[f"plan__{key}"] = value
    source_arc = pipe_nurbs_geometry("arc", geometry, [0., 1.])
    validation = {
        "source_points": len(parameters), "source_blocks": int(raw["num_blocks"]),
        "raw_position_max_error_mm": raw_error, "source_position_max_error_m": position_error,
        "source_axis_max_error": axis_error, "final_length_m": reference.total_length_m,
        "source_length_mm": float(source_arc["total_length"]),
        "source_duration_s": float(raw["time_series"][-1]),
        "certified_minimum_parameter_speed_m_per_u": reference.minimum_speed_m_per_u,
        "speed_certificate": "Bernstein_interval_bounds_axial_and_angular_polynomials",
        "source_nonfinite": {key: int(np.count_nonzero(~np.isfinite(value))) for key, value in source_samples.items()
                             if not key.startswith(("valid__", "plan__")) and not np.isfinite(value).all()},
    }
    # 最终弧长与源弧长分别记录；误差估计用独立复合积分比较取得。
    independent = [quad(lambda u: float(np.linalg.norm(curve_jet(reference.arrays, np.asarray(u))[0][1])),
                        low, high, epsabs=1e-11, epsrel=1e-11)
                   for low, high in pairwise(breaks)]
    validation["arc_error_estimate_m"] = float(sum(error for _, error in independent))
    validation["independent_arc_difference_m"] = float(abs(sum(value for value, _ in independent) - reference.total_length_m))
    inverse_grid = np.linspace(0, 1, 4097)
    roundtrip = reference.query(reference.query_u(inverse_grid).s)
    validation["inverse_max_u_error"] = float(np.max(np.abs(roundtrip.u - inverse_grid)))
    validation["inverse_max_residual_m"] = float(np.max(np.abs(roundtrip.arc_residual_m)))
    events = [{"kind": "source_cubic_node", "u": float(u), "s": float(reference.query_u(u).s),
               "continuity": "C2", "third_derivative": "left_and_right"} for u in breaks[1:-1]]
    events.extend({"kind": "nominal_block_boundary", "source_index": int(index - 1),
                   "u": float(parameters[index - 1]), "time_s": float(raw["time_series"][index - 1])}
                  for index in np.asarray(raw["boundary_point_index"], dtype=int).ravel())
    events.append({"kind": "endpoint_hold", "u": 1., "s": 1.,
                   "source_time_interval_s": samples.source_time_interval_s[-1].tolist()})
    output_dir.mkdir(parents=True)
    (output_dir / "source").mkdir()
    for name, path in sources.items():
        (output_dir / "source" / name).write_bytes(path.read_bytes())
    np.savez_compressed(output_dir / "reference.npz", **reference.arrays._asdict())
    np.savez_compressed(output_dir / "source_samples.npz", **source_samples)
    source_interval = np.clip(np.searchsorted(source_arc["u"], parameters, side="right") - 1,
                              0, len(source_arc["u"]) - 2)
    source_distance = source_arc["s"][source_interval] + _integrate(
        _spline(geometry), source_arc["u"][source_interval], parameters)
    np.savez_compressed(output_dir / "samples.npz", **samples._asdict(), source_u=parameters,
                        source_time_s=raw["time_series"], source_point_index=raw["point_index"],
                        source_arc_mm=source_distance)
    code_paths = [Path(__file__), Path(sys.modules[ContinuousPathReference.__module__].__file__),
                  Path(sys.modules[load_repository_trajectory.__module__].__file__),
                  Path(sys.modules[pipe_nurbs_geometry.__module__].__file__), Path(robot_spec.__file__)]
    manifest = {
        "schema_version": 1, "format": "nineaxis-butterfly-continuous", "identity": reference_identity(description),
        "description": description, "config_inputs": config_inputs, "events": events, "validation": validation,
        "nominal_plan_cells": plan_cells,
        "generator": {"command": command or "Python generate_reference API", "python": platform.python_version(),
                      "packages": {name: version(name) for name in ("numpy", "scipy", "PyYAML")},
                      "source_code_sha256": {str(path): _digest(path) for path in code_paths}},
        "loader_output": captured.getvalue(),
        "outputs": {str(path.relative_to(output_dir)): _digest(path) for path in sorted(output_dir.rglob("*")) if path.is_file()},
    }
    (output_dir / "manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False, allow_nan=False) + "\n")
    load_reference_bundle(output_dir)
    return manifest


def main():
    parser = argparse.ArgumentParser(description="现有九轴蝴蝶连续参考文件")
    commands = parser.add_subparsers(dest="action", required=True)
    generate = commands.add_parser("generate")
    generate.add_argument("--source-dir", required=True, type=Path)
    generate.add_argument("--kinematics-config", required=True, type=Path)
    generate.add_argument("--controller-profile", required=True, type=Path)
    generate.add_argument("--scenario-config", type=Path)
    generate.add_argument("--output", required=True, type=Path)
    check = commands.add_parser("check")
    check.add_argument("bundle", type=Path)
    query = commands.add_parser("query")
    query.add_argument("bundle", type=Path)
    query.add_argument("--s", required=True, type=float)
    args = parser.parse_args()
    if args.action == "generate":
        result = generate_reference(args.source_dir, args.kinematics_config, args.controller_profile, args.output,
                                    scenario_config=args.scenario_config, command=shlex.join([sys.executable, "-m", __spec__.name, *sys.argv[1:]]))
        print(json.dumps({"identity": result["identity"], "validation": result["validation"]}, ensure_ascii=False))
    else:
        bundle = load_reference_bundle(args.bundle)
        result = {"identity": bundle.manifest["identity"], "total_length_m": bundle.reference.total_length_m}
        if args.action == "query":
            sample = bundle.reference.query(args.s)
            result.update({"u": float(sample.u), "position_m": sample.position_m.tolist(),
                           "tool_axis": sample.tool_axis.tolist(), "valid": bool(sample.valid)})
        print(json.dumps(result, ensure_ascii=False))


if __name__ == "__main__":
    main()
