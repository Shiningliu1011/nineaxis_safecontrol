from functools import partial

import numpy as np
from scipy.spatial.transform import Rotation

from .combined_pipe_model import (
    _evaluate_segment,
    _finite_scalar,
    _finite_vector,
    _paired_parameters,
)


def _axial_center(u, origin, tangent):
    return origin + _finite_vector(u, "u", allow_empty=True)[:, None] * tangent


def _elbow_center(u, origin, tangent, toward, rb):
    u = _finite_vector(u, "u", allow_empty=True)
    return origin + rb * (np.sin(u)[:, None] * tangent + (1 - np.cos(u))[:, None] * toward)


def _elbow_surface(u, v, origin, frame, radius, rb, toward, axis_vector, beta):
    u, v = _paired_parameters(u, v)
    if np.any(u < 0) or np.any(u > beta):
        raise ValueError("u 超出本管段的 uRange")
    rotations = Rotation.from_rotvec(u[:, None] * axis_vector)
    e1, e2 = rotations.apply(frame[:, 0]), rotations.apply(frame[:, 1])
    n = np.cos(v)[:, None] * e1 + np.sin(v)[:, None] * e2
    center = _elbow_center(u, origin, frame[:, 2], toward, rb)
    p = center + radius * n
    center_derivative = rb * (np.cos(u)[:, None] * frame[:, 2] + np.sin(u)[:, None] * toward)
    pu = center_derivative + radius * np.cross(axis_vector, n)
    pv = radius * (-np.sin(v)[:, None] * e1 + np.cos(v)[:, None] * e2)
    return p, n, pu, pv


def pipe_complex_demo_model(lengthScale=1):
    lengthScale = _finite_scalar(lengthScale, "lengthScale", positive=True)
    specs = [
        ("straight", 450, 30, 0, 0), ("reducer", 180, 42, 0, 0),
        ("straight", 300, 42, 0, 0), ("elbow", 180, 42, 90, 0),
        ("straight", 500, 42, 0, 0), ("elbow", 220, 42, 60, 90),
        ("straight", 350, 42, 0, 0), ("reducer", 200, 28, 0, 0),
        ("straight", 420, 28, 0, 0), ("elbow", 160, 28, 90, 180),
        ("straight", 450, 28, 0, 0), ("elbow", 200, 28, 45, -90),
        ("straight", 320, 28, 0, 0), ("reducer", 160, 36, 0, 0),
        ("straight", 380, 36, 0, 0), ("elbow", 190, 36, 60, 45),
        ("straight", 400, 36, 0, 0),
    ]
    origin, frame, radius = np.zeros(3), np.eye(3), 30.0
    segments = []
    for index, (kind, dimension, out_radius, beta_degrees, psi_degrees) in enumerate(specs, 1):
        dimension *= lengthScale
        beta, psi = np.deg2rad([beta_degrees, psi_degrees])
        segment = {
            "name": "", "canvasLength": 0.0, "uRange": np.array([0.0, 1.0]),
            "thetaOffsetRad": 0.0, "thetaSign": 1, "kind": kind,
            "radiusInMM": radius, "radiusOutMM": out_radius,
            "bendRadiusMM": 0.0, "bendAngleDeg": 0.0, "bendPlaneDeg": 0.0,
            "startCenter": origin.copy(),
        }
        if kind == "elbow":
            if dimension <= radius:
                raise ValueError("lengthScale 对应的弯头半径必须大于管截面半径")
            toward = frame[:, 0] * np.cos(psi) + frame[:, 1] * np.sin(psi)
            axis_vector = np.cross(frame[:, 2], toward)
            axis_vector /= np.linalg.norm(axis_vector)
            segment.update({
                "name": f"{index:02d} 弯头 {beta_degrees:g}°", "canvasLength": dimension * beta,
                "uRange": np.array([0.0, beta]), "bendRadiusMM": dimension,
                "bendAngleDeg": beta_degrees, "bendPlaneDeg": psi_degrees,
                "evaluate": partial(_elbow_surface, origin=origin.copy(), frame=frame.copy(),
                                    radius=radius, rb=dimension, toward=toward.copy(),
                                    axis_vector=axis_vector.copy(), beta=beta),
                "centerline": partial(_elbow_center, origin=origin.copy(), tangent=frame[:, 2].copy(),
                                      toward=toward.copy(), rb=dimension),
            })
            next_origin = _elbow_center([beta], origin, frame[:, 2], toward, dimension)[0]
            next_frame = Rotation.from_rotvec(beta * axis_vector).as_matrix() @ frame
        else:
            if kind == "straight" and abs(out_radius - radius) > 1e-12:
                raise ValueError("直管必须保持管截面半径")
            label = "直管" if kind == "straight" else "变径"
            spec = {"type": kind, "limit": dimension, "r0": radius, "r1": out_radius,
                    "R": frame.copy(), "t": origin.copy()}
            segment.update({
                "name": f"{index:02d} {label}", "canvasLength": dimension,
                "uRange": np.array([0.0, dimension]),
                "evaluate": lambda u, v, spec=spec: _evaluate_segment(u, v, spec),
                "centerline": partial(_axial_center, origin=origin.copy(), tangent=frame[:, 2].copy()),
            })
            next_origin = origin + dimension * frame[:, 2]
            next_frame = frame
        segment["endCenter"] = next_origin.copy()
        segments.append(segment)
        origin, frame, radius = next_origin, next_frame, float(out_radius)
    return {
        "name": "长管测试：17 段 / 5 弯头 / 3 变径", "lengthUnit": "mm",
        "coordinateFrame": "工件坐标系 W", "segments": segments,
        "totalCenterlineLengthMM": sum(s["canvasLength"] for s in segments),
        "lengthScale": lengthScale,
    }
