#!/usr/bin/env python3
"""M3: per-pair DCOL alpha calibration from FCL near-contact data.

For every self-collision pair the script samples random joint configurations,
keeps near-contact samples (FCL OBB-Box distance in [0.005, 0.05] m), and:

1. records the DCOL-vs-FCL distance residual RMS for the pair;
2. estimates the pair's worst-case approach speed from the DCOL distance
   gradient and the actuator velocity limits;
3. calibrates the CBF gain ``alpha = max(alpha_min,
   safety_factor * v_95 / d_safe)`` so the discrete CBF can stop the pair
   before the safety margin is consumed.

Outputs ``config/dcol_alpha.yaml`` and
``output/dcol_alpha_calibration_report.md``.  The calibration is synthetic
(FCL near-contact data generated in joint space), not hardware measured.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import sys


_PORTABLE_ROOT = Path(__file__).resolve().parents[1]
if str(_PORTABLE_ROOT) not in sys.path:
    sys.path.insert(0, str(_PORTABLE_ROOT))

import jax

# Enable x64 before any JAX model is built (the control pipeline uses float64).
jax.config.update("jax_enable_x64", True)

import jax.numpy as jnp
import numpy as np

from work.dpax_collision import (
    self_collision_grads_jit,
    self_collision_distances_jit,
)
from work.nineaxis_manipulator_jax import NineaxisManipulatorJAX
from work.obb_collision_model import (
    OBB_COLLISION_PAIRS,
    OBB_HALF_EXTENTS_M,
    OBB_LOCAL_CENTERS_M,
    OBB_LOCAL_ROTATIONS,
)


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUTPUT_YAML = REPO_ROOT / "portable_oscbf" / "config" / "dcol_alpha.yaml"
DEFAULT_OUTPUT_REPORT = REPO_ROOT / "output" / "dcol_alpha_calibration_report.md"

D_SAFE = 0.03
ALPHA_MIN = 5.0
ALPHA_MAX = 30.0
SAFETY_FACTOR = 2.0
NEAR_CONTACT_MIN = 0.005
NEAR_CONTACT_MAX = 0.05
MIN_SAMPLES_PER_PAIR = 15
LOCAL_PERTURB_SIGMA = 0.03


@dataclass
class PairCalibration:
    i: int
    j: int
    samples: int
    rms_m: float
    v_max_m_s: float
    alpha: float


def _fcl_box_distance(robot, transform, i, j) -> float:
    import fcl

    distance = 0.0
    objects = []
    for index in (i, j):
        rotation = transform[index][:3, :3] @ OBB_LOCAL_ROTATIONS[index]
        center = (
            transform[index][:3, :3] @ OBB_LOCAL_CENTERS_M[index]
            + transform[index][:3, 3])
        half = OBB_HALF_EXTENTS_M[index]
        tf = fcl.Transform()
        tf.setRotation(rotation)
        tf.setTranslation(center)
        box = fcl.Box(2.0 * half[0], 2.0 * half[1], 2.0 * half[2])
        objects.append(fcl.CollisionObject(box, tf))
    request = fcl.DistanceRequest()
    request.enable_nearest_points = False
    result = fcl.DistanceResult()
    fcl.distance(objects[0], objects[1], request, result)
    return float(result.min_distance)


def calibrate(*, mesh_dir: Path | None = None,
              max_global_samples: int = 20000,
              max_local_samples_per_pair: int = 300,
              seed: int = 20260806,
              output_yaml: Path | None = None,
              output_report: Path | None = None) -> list[PairCalibration]:
    """Run the synthetic near-contact calibration (global + local sampling)."""

    del mesh_dir  # FCL Box uses the OBB model directly (no STL needed)
    robot = NineaxisManipulatorJAX()
    rng = np.random.default_rng(seed)
    lower = np.asarray(robot.joint_lower_limits)
    upper = np.asarray(robot.joint_upper_limits)
    margin = 0.05 * np.ones(9)
    margin[0] = 0.05
    dq_max = np.asarray(robot.joint_max_velocities)
    pairs = np.asarray(OBB_COLLISION_PAIRS)
    pair_count = len(pairs)

    # Phase 1: global random sampling; keep near-contact seeds per pair.
    seeds = [[] for _ in range(pair_count)]
    for _ in range(max_global_samples):
        q = rng.uniform(lower + margin, upper - margin)
        transforms = np.asarray(robot._compute_all_link_transforms(
            jnp.asarray(q)))
        for pair_index, (i, j) in enumerate(pairs):
            distance = _fcl_box_distance(
                robot, transforms, int(i), int(j))
            if NEAR_CONTACT_MIN <= distance <= NEAR_CONTACT_MAX:
                seeds[pair_index].append(q.copy())

    # Phase 2: local perturbation around seeds until enough samples per pair.
    per_pair_fcl = [[] for _ in range(pair_count)]
    per_pair_dcol = [[] for _ in range(pair_count)]
    per_pair_grad_norm = [[] for _ in range(pair_count)]
    for pair_index in range(pair_count):
        attempts = 0
        while len(per_pair_fcl[pair_index]) < MIN_SAMPLES_PER_PAIR:
            if attempts >= max_local_samples_per_pair or not seeds[pair_index]:
                break
            attempts += 1
            base = seeds[pair_index][rng.integers(len(seeds[pair_index]))]
            sigma = LOCAL_PERTURB_SIGMA * np.ones(9)
            sigma[0] = 0.01
            candidate = np.clip(
                rng.normal(base, sigma), lower + margin, upper - margin)
            transforms = np.asarray(robot._compute_all_link_transforms(
                jnp.asarray(candidate)))
            i, j = pairs[pair_index]
            distance = _fcl_box_distance(robot, transforms, int(i), int(j))
            if not (NEAR_CONTACT_MIN <= distance <= NEAR_CONTACT_MAX):
                continue
            q_jax = jnp.asarray(candidate)
            per_pair_fcl[pair_index].append(distance)
            per_pair_dcol[pair_index].append(float(np.asarray(
                self_collision_distances_jit(q_jax))[pair_index]))
            grads = np.asarray(self_collision_grads_jit(
                q_jax, jnp.arange(len(OBB_COLLISION_PAIRS))))  # (14,9)
            per_pair_grad_norm[pair_index].append(float(np.linalg.norm(
                grads[pair_index] @ dq_max)))

    calibrations: list[PairCalibration] = []
    for pair_index, (i, j) in enumerate(pairs):
        fcl_values = np.asarray(per_pair_fcl[pair_index])
        dcol_values = np.asarray(per_pair_dcol[pair_index])
        grad_speeds = np.asarray(per_pair_grad_norm[pair_index])
        samples = len(fcl_values)
        if samples == 0:
            calibrations.append(PairCalibration(
                i=int(i), j=int(j), samples=0, rms_m=0.0,
                v_max_m_s=0.0, alpha=ALPHA_MIN))
            continue
        residual = dcol_values - fcl_values
        rms = float(np.sqrt(np.mean(residual ** 2)))
        v_max = float(np.percentile(grad_speeds, 95)) if samples >= 5 else 0.0
        v_max = max(v_max, 1e-6)
        alpha = float(np.clip(
            SAFETY_FACTOR * v_max / D_SAFE, ALPHA_MIN, ALPHA_MAX))
        calibrations.append(PairCalibration(
            i=int(i), j=int(j), samples=samples, rms_m=rms,
            v_max_m_s=v_max, alpha=alpha))

    _write_outputs(calibrations, output_yaml, output_report,
                   max_global_samples, max_local_samples_per_pair)
    return calibrations


def _write_outputs(calibrations, output_yaml, output_report,
                   max_global_samples, max_local_samples_per_pair) -> None:
    import yaml

    yaml_path = Path(output_yaml) if output_yaml else DEFAULT_OUTPUT_YAML
    report_path = Path(output_report) if output_report else DEFAULT_OUTPUT_REPORT
    document = {
        "schema_version": 1,
        "generated_by": "portable_oscbf/scripts/calibrate_dcol_alpha.py",
        "basis": "synthetic FCL near-contact data (0.005-0.05 m), not hardware",
        "d_safe_collision_m": D_SAFE,
        "alpha_min": ALPHA_MIN,
        "alpha_max": ALPHA_MAX,
        "safety_factor": SAFETY_FACTOR,
        "formula": (
            "alpha = clip(safety_factor * v_95 / d_safe, "
            f"{ALPHA_MIN}, {ALPHA_MAX})"),
        "sampling": {
            "global": max_global_samples,
            "local_per_pair": max_local_samples_per_pair,
            "min_samples_per_pair": MIN_SAMPLES_PER_PAIR,
            "perturb_sigma": LOCAL_PERTURB_SIGMA,
        },
        "pairs": [
            {
                "i": cal.i,
                "j": cal.j,
                "alpha": round(cal.alpha, 4),
                "samples": cal.samples,
                "rms_m": round(cal.rms_m, 6),
                "v_max_m_s": round(cal.v_max_m_s, 6),
            }
            for cal in calibrations
        ],
    }
    yaml_path.parent.mkdir(parents=True, exist_ok=True)
    yaml_path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")

    lines = [
        "# DCOL Alpha Calibration Report (synthetic near-contact data)",
        "",
        f"- Date: 2026-08-06",
        f"- Basis: FCL OBB-Box near-contact samples in [0.005, 0.05] m",
        f"- Formula: alpha = clip({SAFETY_FACTOR} * v_95 / {D_SAFE}, "
        f"{ALPHA_MIN}, {ALPHA_MAX})",
        f"- d_safe = {D_SAFE} m, v_95 = 95th percentile of |d(distance)/dq @ dq_max|",
        "",
        "| pair | samples | RMS (m) | v_max (m/s) | alpha |",
        "|------|--------:|--------:|------------:|------:|",
    ]
    for cal in calibrations:
        lines.append(
            f"| ({cal.i},{cal.j}) | {cal.samples} | {cal.rms_m:.6f} "
            f"| {cal.v_max_m_s:.6f} | {cal.alpha:.4f} |")
    lines.append("")
    lines.append(
        "> Note: calibration is synthetic (joint-space FCL sampling); "
        "hardware near-contact data must replace it before field use.")
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--max-global-samples", type=int, default=20000)
    parser.add_argument("--max-local-samples-per-pair", type=int, default=300)
    parser.add_argument("--seed", type=int, default=20260806)
    parser.add_argument("--output-yaml", type=Path, default=DEFAULT_OUTPUT_YAML)
    parser.add_argument("--output-report", type=Path, default=DEFAULT_OUTPUT_REPORT)
    args = parser.parse_args()
    calibrations = calibrate(
        max_global_samples=args.max_global_samples,
        max_local_samples_per_pair=args.max_local_samples_per_pair,
        seed=args.seed,
        output_yaml=args.output_yaml,
        output_report=args.output_report,
    )
    for cal in calibrations:
        print(f"pair ({cal.i:2d},{cal.j:2d}): samples={cal.samples:4d} "
              f"rms={cal.rms_m*1000:.3f}mm v_max={cal.v_max_m_s:.4f} "
              f"alpha={cal.alpha:.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
