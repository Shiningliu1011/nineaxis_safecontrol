from __future__ import annotations

import json
from dataclasses import dataclass
from hashlib import sha256
from itertools import pairwise
from math import comb
from pathlib import Path
from typing import TYPE_CHECKING, NamedTuple

import numpy as np
from numpy.polynomial.legendre import leggauss

if TYPE_CHECKING:
    from jax import Array as JaxArray


class ContinuousReferenceArrays(NamedTuple):
    breaks_u: np.ndarray | JaxArray
    coefficients_m: np.ndarray | JaxArray
    projection_axes: np.ndarray | JaxArray
    projection_centres_m: np.ndarray | JaxArray
    projection_radii_m: np.ndarray | JaxArray
    arc_u: np.ndarray | JaxArray
    arc_m: np.ndarray | JaxArray
    gauss_nodes: np.ndarray | JaxArray
    gauss_weights: np.ndarray | JaxArray
    source_u: np.ndarray | JaxArray
    source_time_s: np.ndarray | JaxArray
    source_feedrate_m_s: np.ndarray | JaxArray


class ContinuousReferenceSample(NamedTuple):
    s: np.ndarray
    u: np.ndarray
    position_m: np.ndarray
    tool_axis: np.ndarray
    normal: np.ndarray
    rotation: np.ndarray
    position_d1: np.ndarray
    position_d2: np.ndarray
    position_d3: np.ndarray
    axis_d1: np.ndarray
    axis_d2: np.ndarray
    axis_d3: np.ndarray
    source_time_interval_s: np.ndarray
    source_index_interval: np.ndarray
    nominal_feedrate_m_s: np.ndarray
    event_interval: np.ndarray
    arc_residual_m: np.ndarray
    valid: np.ndarray


def _normalize_jet(jet, xp):
    z = jet[0]
    radius = xp.linalg.norm(z, axis=-1, keepdims=True)
    radius = xp.where(radius > 1e-12, radius, xp.nan)
    a = z / radius
    if len(jet) == 1:
        return (a,)
    z1 = jet[1]
    r1 = xp.sum(a * z1, axis=-1, keepdims=True)
    a1 = (z1 - a * r1) / radius
    if len(jet) == 2:
        return a, a1
    z2 = jet[2]
    r2 = xp.sum(a1 * z1 + a * z2, axis=-1, keepdims=True)
    a2 = (z2 - a * r2 - 2 * a1 * r1) / radius
    if len(jet) == 3:
        return a, a1, a2
    z3 = jet[3]
    r3 = xp.sum(a2 * z1 + 2 * a1 * z2 + a * z3, axis=-1, keepdims=True)
    a3 = (z3 - a * r3 - 3 * a1 * r2 - 3 * a2 * r1) / radius
    return a, a1, a2, a3


def curve_jet(data: ContinuousReferenceArrays, u, *, xp=np, side="right", order=3, include_axis=True):
    index = xp.clip(xp.searchsorted(data.breaks_u, u, side=side) - 1,
                    0, data.coefficients_m.shape[0] - 1)
    c = data.coefficients_m[index]
    h = (u - data.breaks_u[index])[..., None]
    jet: tuple[np.ndarray | JaxArray, ...] = (
        ((c[..., 0, :] * h + c[..., 1, :]) * h + c[..., 2, :]) * h + c[..., 3, :],
    )
    if order >= 1:
        jet += ((3 * c[..., 0, :] * h + 2 * c[..., 1, :]) * h + c[..., 2, :],)
    if order >= 2:
        jet += (6 * c[..., 0, :] * h + 2 * c[..., 1, :],)
    if order >= 3:
        jet += (6 * c[..., 0, :],)
    for stage in range(data.projection_radii_m.shape[0]):
        axis, centre, radius = (data.projection_axes[stage], data.projection_centres_m[stage],
                                data.projection_radii_m[stage])
        relative = (jet[0] - centre, *jet[1:])
        axial = tuple(xp.sum(value * axis, axis=-1, keepdims=True) * axis for value in relative)
        radial = tuple(value - part for value, part in zip(relative, axial))
        normal = _normalize_jet(radial, xp)
        jet = tuple(part + radius * direction for part, direction in zip(axial, normal))
        jet = (jet[0] + centre, *jet[1:])
    if not include_axis:
        return jet, ()
    axis = data.projection_axes[-1]
    relative = (jet[0] - data.projection_centres_m[-1], *jet[1:])
    radial = tuple(value - xp.sum(value * axis, axis=-1, keepdims=True) * axis for value in relative)
    tool_axis = tuple(-value for value in _normalize_jet(radial, xp))
    return jet, tool_axis


def _integrate(data, low, high, xp=np):
    half = (high - low) / 2
    parameters = (low + high)[..., None] / 2 + half[..., None] * data.gauss_nodes
    velocity = curve_jet(data, parameters, xp=xp, order=1, include_axis=False)[0][1]
    return half * xp.sum(xp.linalg.norm(velocity, axis=-1) * data.gauss_weights, axis=-1)


def arc_at_u(data, u, *, xp=np):
    index = xp.clip(xp.searchsorted(data.arc_u, u, side="right") - 1, 0, data.arc_u.shape[0] - 2)
    distance = data.arc_m[index] + _integrate(data, data.arc_u[index], u, xp)
    return xp.where(u == 1, data.arc_m[-1], distance)


def _u_at_s(data, s, xp):
    target = s * data.arc_m[-1]
    index = xp.clip(xp.searchsorted(data.arc_m, target, side="right") - 1, 0, data.arc_m.shape[0] - 2)
    low, high = data.arc_u[index], data.arc_u[index + 1]
    base = data.arc_m[index]
    u = low + (high - low) * (target - base) / (data.arc_m[index + 1] - base)
    # 所有消费者使用同一计算次数；反解残差随查询结果返回。
    for _ in range(6):
        residual = base + _integrate(data, data.arc_u[index], u, xp) - target
        if xp is np and np.all(np.abs(residual) <= 2e-14):
            break
        speed = xp.linalg.norm(curve_jet(data, u, xp=xp, order=1, include_axis=False)[0][1], axis=-1)
        low = xp.where(residual < 0, u, low)
        high = xp.where(residual > 0, u, high)
        candidate = u - residual / speed
        candidate = xp.where((candidate >= low) & (candidate <= high), candidate, (low + high) / 2)
        u = xp.where(xp.abs(residual) <= 2e-14, u, candidate)
    tolerance = 8 * xp.finfo(data.arc_m.dtype).eps * data.arc_m[-1]
    u = xp.where(xp.abs(target - base) <= tolerance, data.arc_u[index], u)
    u = xp.where(xp.abs(target - data.arc_m[index + 1]) <= tolerance, data.arc_u[index + 1], u)
    residual = base + _integrate(data, data.arc_u[index], u, xp) - target
    return u, residual


def refine_projection(data, point, seed_m, lower_m, upper_m, *, xp=np):
    parameters, _ = _u_at_s(data, xp.stack((seed_m, lower_m, upper_m)) / data.arc_m[-1], xp)
    u, low, high = parameters[0], parameters[1], parameters[2]
    original_low, original_high = low, high
    for _ in range(12):
        p, _ = curve_jet(data, u, xp=xp, order=2, include_axis=False)
        delta = p[0] - point
        gradient = xp.sum(delta * p[1], axis=-1)
        curvature = xp.sum(p[1]**2 + delta * p[2], axis=-1)
        low = xp.where(gradient < 0, u, low)
        high = xp.where(gradient > 0, u, high)
        candidate = u - gradient / xp.where(curvature > 0, curvature, xp.inf)
        candidate = xp.where((curvature > 0) & (candidate >= low) & (candidate <= high),
                             candidate, (low + high) / 2)
        u = xp.where(xp.abs(gradient) <= 1e-17, u, candidate)
    candidates = xp.stack((original_low, u, original_high))
    positions = curve_jet(data, candidates, xp=xp, order=0, include_axis=False)[0][0]
    distance = xp.linalg.norm(positions - point, axis=-1)
    selected = xp.argmin(distance)
    parameter = candidates[selected]
    return arc_at_u(data, parameter, xp=xp), distance[selected], parameter


def _sample_at_u(data, u, s, residual, xp, side):
    position, axis = curve_jet(data, u, xp=xp, side=side)
    speed = xp.linalg.norm(position[1], axis=-1)
    speed1 = xp.sum(position[1] * position[2], axis=-1) / speed
    speed2 = (xp.sum(position[2]**2 + position[1] * position[3], axis=-1) - speed1**2) / speed
    length = data.arc_m[-1]
    u1 = length / speed
    u2 = -length**2 * speed1 / speed**3
    u3 = length**3 * (3 * speed1**2 / speed**5 - speed2 / speed**4)

    def change_parameter(jet):
        return (jet[1] * u1[..., None],
                jet[2] * u1[..., None]**2 + jet[1] * u2[..., None],
                jet[3] * u1[..., None]**3 + 3 * jet[2] * (u1 * u2)[..., None]
                + jet[1] * u3[..., None])

    p1, p2, p3 = change_parameter(position)
    a1, a2, a3 = change_parameter(axis)
    cylinder_axis = xp.broadcast_to(data.projection_axes[-1], axis[0].shape)
    rotation = xp.stack((axis[0], cylinder_axis, xp.cross(axis[0], cylinder_axis)), axis=-1)
    nominal_time = xp.interp(u, data.source_u, data.source_time_s)
    first = xp.clip(xp.searchsorted(data.source_u, u, side="left"), 0, data.source_u.shape[0] - 1)
    last = xp.clip(xp.searchsorted(data.source_u, u, side="right") - 1, 0, data.source_u.shape[0] - 1)
    exact = data.source_u[first] == u
    time_low = xp.where(exact, data.source_time_s[first], nominal_time)
    time_high = xp.where(exact, data.source_time_s[last], nominal_time)
    first = xp.where(exact, first, xp.maximum(first - 1, 0))
    last = xp.where(exact, last, xp.minimum(first + 1, data.source_u.shape[0] - 1))
    event = xp.clip(xp.searchsorted(data.breaks_u, u, side=side) - 1, 0, data.breaks_u.shape[0] - 2)
    valid = (xp.isfinite(s) & (s >= 0) & (s <= 1) & xp.isfinite(u) & (u >= 0) & (u <= 1)
             & (speed > 1e-12) & (xp.abs(residual) <= 2e-11)
             & xp.all(xp.isfinite(xp.stack((*position, *axis), axis=-2)), axis=(-2, -1)))
    return ContinuousReferenceSample(
        s, u, position[0], axis[0], -axis[0], rotation, p1, p2, p3, a1, a2, a3,
        xp.stack((time_low, time_high), axis=-1), xp.stack((first, last), axis=-1),
        xp.interp(u, data.source_u, data.source_feedrate_m_s), event, residual, valid,
    )


def sample_reference(data: ContinuousReferenceArrays, s, *, xp=np, side="right", inverse=None):
    if inverse is None:
        u, residual = _u_at_s(data, s, xp)
    else:
        u = inverse(data, s)
        residual = arc_at_u(data, u, xp=xp) - s * data.arc_m[-1]
    return _sample_at_u(data, u, s, residual, xp, side)


def time_derivatives(sample: ContinuousReferenceSample, nu, nu_dot=0., nu_ddot=0., *, xp=np):
    rates = tuple(xp.asarray(value)[..., None] for value in (nu, nu_dot, nu_ddot))
    if xp is np and (not all(np.isfinite(value).all() for value in rates) or np.any(rates[0] < 0)):
        raise ValueError("时间导数需要非负 nu 和有限数值")
    nu, nu_dot, nu_ddot = rates
    return {
        name: (d1 * nu, d2 * nu**2 + d1 * nu_dot,
               d3 * nu**3 + 3 * d2 * nu * nu_dot + d1 * nu_ddot)
        for name, d1, d2, d3 in (
            ("position", sample.position_d1, sample.position_d2, sample.position_d3),
            ("tool_axis", sample.axis_d1, sample.axis_d2, sample.axis_d3),
        )
    }


def _parameters(value, name):
    array = np.asarray(value)
    if np.iscomplexobj(array):
        raise ValueError(f"{name} 需要实数")
    array = array.astype(float)
    if not np.isfinite(array).all() or np.any(array < 0) or np.any(array > 1):
        raise ValueError(f"{name} 必须为 [0,1] 中的有限数值")
    return array


def _bisect_bernstein(control):
    levels = [control]
    for _ in range(len(control) - 1):
        levels.append((levels[-1][:-1] + levels[-1][1:]) / 2)
    return (np.stack([level[0] for level in levels]),
            np.stack([level[-1] for level in levels[::-1]]))


def _certify_parameter_speed(bezier, spans, axes, centres, radii):
    delta = centres[1] - centres[0]
    transverse = delta - np.dot(delta, axes[0]) * axes[0]
    shift = np.linalg.norm(transverse)
    final_factor = min(1., radii[1] * (radii[0] - shift) / (radii[0] + shift)**2)
    bound = np.inf
    for source, span in zip(bezier, spans):
        derivative = 3 * np.diff(source, axis=0)
        axial = derivative @ axes[0]
        radial = source - centres[0]
        radial -= (radial @ axes[0])[:, None] * axes[0]
        angular = np.zeros(6)
        for i in range(4):
            for j in range(3):
                angular[i + j] += (comb(3, i) * comb(2, j) / comb(5, i + j)
                                  * np.dot(axes[0], np.cross(radial[i], derivative[j])))
        # Bernstein 凸包界贯通完整区间；浮点运算余量随区间层数增加。
        source_scale = max(float(np.max(np.abs(source))), float(np.max(np.abs(centres))), 1e-30)
        margin_axial = 256 * np.finfo(float).eps * source_scale
        margin_angular = margin_axial * max(float(np.max(np.linalg.norm(radial, axis=1))), 1e-30)
        pending = [(axial, angular, radial, 0)]
        visited = 0
        while pending:
            axial_part, angular_part, radial_part, depth = pending.pop()
            visited += 1
            axial_low = max(0., float(axial_part.min()), -float(axial_part.max()))
            angular_low = max(0., float(angular_part.min()), -float(angular_part.max()))
            axial_low = max(0., axial_low - margin_axial * (depth + 1))
            angular_low = max(0., angular_low - margin_angular * (depth + 1))
            radius_high = float(np.max(np.linalg.norm(radial_part, axis=1))) + margin_axial * (depth + 1)
            speed_low = final_factor * np.hypot(axial_low, radii[0] * angular_low / radius_high**2) / span
            if speed_low > 1e-12:
                bound = min(bound, speed_low)
                continue
            if depth >= 24 or visited >= 65536:
                raise ValueError("整个区间无法证明最终参数速度非退化")
            axial_parts = _bisect_bernstein(axial_part)
            angular_parts = _bisect_bernstein(angular_part)
            radial_parts = _bisect_bernstein(radial_part)
            pending.extend((axial_parts[side], angular_parts[side], radial_parts[side], depth + 1)
                           for side in (0, 1))
    return float(bound)


@dataclass(frozen=True)
class ContinuousPathReference:
    arrays: ContinuousReferenceArrays
    minimum_speed_m_per_u: float = 0.

    @classmethod
    def build(cls, breaks_u, coefficients_m, projection_axes, projection_centres_m,
              projection_radii_m, source_u, source_time_s, source_feedrate_m_s):
        supplied = (breaks_u, coefficients_m, projection_axes, projection_centres_m,
                    projection_radii_m, source_u, source_time_s, source_feedrate_m_s)
        if any(np.iscomplexobj(value) for value in supplied):
            raise ValueError("连续参考的所有数组需要实数")
        breaks, coeff, axes, centres, radii, u, time, feed = (np.array(value, dtype=float, copy=True) for value in supplied)
        if (breaks.ndim != 1 or len(breaks) < 2 or breaks[0] != 0 or breaks[-1] != 1
                or np.any(np.diff(breaks) <= 0) or coeff.shape != (len(breaks) - 1, 4, 3)):
            raise ValueError("breaks_u/coefficients_m 需要 [0,1] 递增区间及三次三维系数")
        if (axes.shape != (2, 3) or centres.shape != (2, 3) or radii.shape != (2,)
                or not np.allclose(np.linalg.norm(axes, axis=1), 1., atol=2e-14, rtol=0)
                or np.any(radii <= 0)):
            raise ValueError("projection 需要两次有效圆柱投影及单位轴向")
        if (u.ndim != 1 or len(u) < 2 or u[0] != 0 or u[-1] != 1 or np.any(np.diff(u) < 0)
                or time.shape != u.shape or np.any(np.diff(time) <= 0) or feed.shape != u.shape
                or np.any(feed < 0)):
            raise ValueError("source 需要有序源参数、递增时间及非负名义进给")
        if not all(np.isfinite(value).all() for value in (breaks, coeff, axes, centres, radii, u, time, feed)):
            raise ValueError("连续参考需要有限数值")
        spans = np.diff(breaks)[:-1, None]
        a, b, c, d = (coeff[:-1, index] for index in range(4))
        endings = (((a * spans + b) * spans + c) * spans + d,
                   (3 * a * spans + 2 * b) * spans + c, 6 * a * spans + 2 * b)
        beginnings = (coeff[1:, 3], coeff[1:, 2], 2 * coeff[1:, 1])
        if any(not np.allclose(left, right, atol=2e-8, rtol=2e-11)
               for left, right in zip(endings, beginnings)):
            raise ValueError("source cubic 需要 C2 连续区间")
        span = np.diff(breaks)[:, None]
        a, b, c, d = (coeff[:, index] for index in range(4))
        bezier = np.stack((d, d + c * span / 3,
                           d + 2 * c * span / 3 + b * span**2 / 3,
                           d + c * span + b * span**2 + a * span**3), axis=1)
        radial = bezier - centres[0]
        radial -= np.sum(radial * axes[0], axis=-1, keepdims=True) * axes[0]
        lower, upper = radial.min(axis=1), radial.max(axis=1)
        bound = np.where(lower > 0, lower, np.where(upper < 0, -upper, 0.))
        if np.any(np.linalg.norm(bound, axis=1) <= 1e-12):
            raise ValueError("projection 的源区间无法证明径向非退化")
        delta = centres[1] - centres[0]
        transverse = delta - np.dot(delta, axes[0]) * axes[0]
        if (not np.array_equal(axes[0], axes[1])
                or radii[0] - np.linalg.norm(transverse) <= 1e-12):
            raise ValueError("projection 的第二圆柱需要共同轴向及非退化径向范围")
        speed_bound = _certify_parameter_speed(bezier, np.diff(breaks), axes, centres, radii)
        nodes, weights = leggauss(16)
        grid = np.unique(np.concatenate([np.linspace(a, b, 9) for a, b in pairwise(breaks)]))
        data = ContinuousReferenceArrays(breaks, coeff, axes, centres, radii, grid,
                                         np.zeros_like(grid), nodes, weights, u, time, feed)
        for depth in range(12):
            integral = _integrate(data, grid[:-1], grid[1:])
            half = (grid[:-1] + grid[1:]) / 2
            divided = _integrate(data, grid[:-1], half) + _integrate(data, half, grid[1:])
            if not np.isfinite(integral).all() or np.any(integral <= 0):
                raise ValueError("projection/arc 包含退化区间")
            refine = np.abs(integral - divided) > 2e-13 * (grid[1:] - grid[:-1])
            if not np.any(refine):
                break
            if depth == 11:
                raise RuntimeError("arc 积分未达到规定精度")
            grid = np.unique(np.r_[grid, half[refine]])
        data = data._replace(arc_u=grid, arc_m=np.r_[0., np.cumsum(divided)])
        check_position, check_axis = curve_jet(data, grid)
        if (not np.isfinite(np.stack((*check_position, *check_axis))).all()
                or np.any(np.linalg.norm(check_position[1], axis=-1) <= 1e-12)):
            raise ValueError("projection/arc 存在无效导数或驻点")
        for value in data:
            value.flags.writeable = False
        return cls(data, speed_bound)

    @property
    def total_length_m(self):
        return float(self.arrays.arc_m[-1])

    def query(self, s, *, side="right"):
        if side not in ("left", "right"):
            raise ValueError("side 需要 left 或 right")
        result = sample_reference(self.arrays, _parameters(s, "s"), side=side)
        if not np.all(result.valid):
            raise ValueError("连续参考查询无效或弧长反解未收敛")
        return result

    def query_u(self, u, *, side="right"):
        if side not in ("left", "right"):
            raise ValueError("side 需要 left 或 right")
        parameter = _parameters(u, "u")
        s = arc_at_u(self.arrays, parameter) / self.total_length_m
        result = _sample_at_u(self.arrays, parameter, s, np.zeros_like(parameter), np, side)
        if not np.all(result.valid):
            raise ValueError("连续参考源参数查询无效")
        return result

    def project(self, position_m, *, s_interval):
        point = np.asarray(position_m, dtype=float)
        bounds = _parameters(s_interval, "s_interval")
        if point.shape != (3,) or not np.isfinite(point).all() or bounds.shape != (2,) or bounds[0] >= bounds[1]:
            raise ValueError("project 需要有限三维位置及递增路径区间")
        low, high = bounds * self.total_length_m
        grid = np.r_[low, self.arrays.arc_m[(self.arrays.arc_m > low) & (self.arrays.arc_m < high)], high]
        sample = self.query(grid / self.total_length_m)
        starts, deltas = sample.position_m[:-1], np.diff(sample.position_m, axis=0)
        fraction = np.clip(np.sum((point - starts) * deltas, axis=1) / np.sum(deltas**2, axis=1), 0, 1)
        closest = starts + fraction[:, None] * deltas
        segment = int(np.argmin(np.sum((closest - point)**2, axis=1)))
        seed = grid[segment] + fraction[segment] * (grid[segment + 1] - grid[segment])
        progress, distance, u = refine_projection(self.arrays, point, seed,
                                                 grid[max(0, segment - 1)], grid[min(len(grid) - 1, segment + 2)])
        if not np.isfinite([progress, distance, u]).all() or not low - 2e-13 <= progress <= high + 2e-13:
            raise ValueError("连续路径局部投影无效")
        return {"s": float(progress / self.total_length_m), "u": float(u), "distance_m": float(distance),
                "s_interval": bounds.copy(), "valid": True}


@dataclass(frozen=True)
class ReferenceBundle:
    reference: ContinuousPathReference
    manifest: dict
    source_samples: dict


def reference_identity(description):
    return sha256(json.dumps(description, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def load_reference_bundle(path, *, expected_source_sha256=None):
    from scipy.interpolate import PPoly

    directory = Path(path).resolve()
    manifest = json.loads((directory / "manifest.json").read_text())
    if manifest["schema_version"] != 1 or manifest["format"] != "nineaxis-butterfly-continuous":
        raise ValueError("不支持的 continuous reference 文件版本")
    if manifest["identity"] != reference_identity(manifest["description"]):
        raise ValueError("reference identity 不匹配")
    if (expected_source_sha256 is not None
            and manifest["description"]["source_sha256"]["ik_input.mat"] != expected_source_sha256):
        raise ValueError("source identity 不匹配")
    required = {"reference.npz", "source_samples.npz", "samples.npz",
                "source/control_points.txt", "source/knot_vector.txt", "source/ik_input.mat",
                "source/nurbs_blocks.mat"}
    if not required.issubset(manifest["outputs"]):
        raise ValueError("manifest 缺少必要产物")
    for name, expected in manifest["outputs"].items():
        item = (directory / name).resolve()
        if not item.is_relative_to(directory) or not item.is_file():
            raise ValueError(f"manifest 产物路径无效：{name}")
        if sha256(item.read_bytes()).hexdigest() != expected:
            raise ValueError(f"SHA256 不匹配：{name}")
    for name, expected in manifest["description"]["source_sha256"].items():
        if manifest["outputs"].get(f"source/{name}") != expected:
            raise ValueError(f"source SHA256 不匹配：{name}")
    with np.load(directory / "reference.npz", allow_pickle=False) as saved:
        if set(saved.files) != set(ContinuousReferenceArrays._fields):
            raise ValueError("reference.npz 数组字段不匹配")
        data = ContinuousReferenceArrays(*(saved[name] for name in ContinuousReferenceArrays._fields))
    rebuilt = ContinuousPathReference.build(
        data.breaks_u, data.coefficients_m, data.projection_axes, data.projection_centres_m,
        data.projection_radii_m, data.source_u, data.source_time_s, data.source_feedrate_m_s)
    for name, actual, expected in zip(data._fields, data, rebuilt.arrays):
        if actual.shape != expected.shape or not np.allclose(actual, expected, rtol=2e-13, atol=2e-14):
            raise ValueError(f"reference.npz 的 {name} 与连续表达不一致")
    description = manifest["description"]
    for name in ("projection_axes", "projection_centres_m", "projection_radii_m"):
        if not np.array_equal(np.asarray(description[name]), getattr(data, name)):
            raise ValueError(f"manifest 的 {name} 与连续表达不一致")
    transform = np.asarray(description["transform"], dtype=float)
    if (transform.shape != (4, 4) or not np.isfinite(transform).all()
            or not np.array_equal(transform[-1], [0., 0., 0., 1.])
            or description["length_unit"] != "m" or description["source_length_unit"] != "mm"
            or description["tool_axis"] != "+X" or description["source_degree"] != 3
            or description["source_weights"] != "unit"):
        raise ValueError("manifest 的变换、工具轴、曲线或单位无效")
    control = np.loadtxt(directory / "source/control_points.txt", ndmin=2)
    knots = np.loadtxt(directory / "source/knot_vector.txt").ravel()
    indices = np.searchsorted(knots, data.breaks_u[:-1], side="right") - 1
    coefficients = np.stack([PPoly.from_spline((knots, control[:, axis], 3)).c[:, indices].T
                             for axis in range(3)], axis=-1)
    coefficients = coefficients @ transform[:3, :3].T / 1000
    coefficients[:, -1] += transform[:3, 3]
    if not np.allclose(coefficients, data.coefficients_m, rtol=2e-14, atol=2e-14):
        raise ValueError("source 控制点与连续区间系数不一致")
    with np.load(directory / "source_samples.npz", allow_pickle=False) as saved:
        source = {name: saved[name] for name in saved.files}
    return ReferenceBundle(rebuilt, manifest, source)
