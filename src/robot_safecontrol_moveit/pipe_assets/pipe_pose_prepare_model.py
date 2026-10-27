import numpy as np

from .pipe_prepare_model import pipe_prepare_model


def pipe_pose_prepare_model(model: dict, verifiedJumps=()) -> dict:
    jumps = np.asarray(verifiedJumps, dtype=float).reshape(-1)
    if not len(jumps):
        return pipe_prepare_model(model)
    if np.any(~np.isfinite(jumps)) or np.any(jumps != np.round(jumps)) or np.any(jumps < 1) or np.any(jumps >= len(model["segments"])):
        raise ValueError("PipePose:VerifiedSeam 接缝编号超出模型范围")
    model = dict(model)
    model["segments"] = [pipe_prepare_model({**model, "segments": [segment]})["segments"][0] for segment in model["segments"]]
    model["breaks"] = np.r_[0, np.cumsum([item["canvasLength"] for item in model["segments"]])]
    model["totalLength"] = model["breaks"][-1]
    model["joinReport"] = np.zeros((len(model["segments"]) - 1, 2))
    angle = np.linspace(0, 2 * np.pi, 97)
    for index, (first, second) in enumerate(zip(model["segments"][:-1], model["segments"][1:])):
        p, n = first["evaluate"](np.full_like(angle, first["uRange"][1]), first["thetaSign"] * angle + first["thetaOffsetRad"])[:2]
        q, m = second["evaluate"](np.full_like(angle, second["uRange"][0]), second["thetaSign"] * angle + second["thetaOffsetRad"])[:2]
        gap, jump = np.max(np.linalg.norm(p - q, axis=1)), np.max(np.linalg.norm(n - m, axis=1))
        model["joinReport"][index] = gap, jump
        if gap > 1e-3:
            raise ValueError(f"PipePose:ModelGap 第 {index + 1} 处接缝存在位置间隙")
        if jump > 1e-4 and index + 1 not in jumps:
            raise ValueError(f"PipePose:UnverifiedNormalJump 第 {index + 1} 处法向跳变未经确认")
    return model
