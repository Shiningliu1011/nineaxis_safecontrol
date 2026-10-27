import numpy as np
from scipy.signal import find_peaks


def pipe_curvature_keypoints(u, kappa, kappa_cr, arc_mm, options=None):
    defaults = {"MinPeakProminence": 1e-6, "MaxRelativeSaddleDepth": .005,
                "MaxPeakDistanceMM": 2., "MaxCurvatureSpan": .15}
    if options is not None and (not isinstance(options, dict) or set(options) - set(defaults)):
        raise ValueError("PipeCurvatureKeypoints:Options：未知或无效参数")
    settings = {**defaults, **(options or {})}
    if any(not np.isscalar(value) or not np.isfinite(value) or value < 0 for value in settings.values()):
        raise ValueError("PipeCurvatureKeypoints:Options：参数需要有限非负标量")
    if settings["MaxRelativeSaddleDepth"] > 1:
        raise ValueError("PipeCurvatureKeypoints:Options：相对鞍点深度不能超过 1")
    u, kappa, arc_mm = [np.asarray(value, dtype=float).ravel() for value in (u, kappa, arc_mm)]
    if (len(u) < 2 or len(kappa) != len(u) or len(arc_mm) != len(u)
            or not np.isfinite(np.r_[u, kappa, arc_mm, kappa_cr]).all() or kappa_cr < 0
            or np.any(kappa < 0) or np.any(np.diff(u) <= 0) or np.any(np.diff(arc_mm) <= 0)):
        raise ValueError("PipeCurvatureKeypoints:Samples：需要对应、有限且严格递增的参数和弧长")
    peaks, properties = find_peaks(kappa, height=kappa_cr, prominence=settings["MinPeakProminence"],
                                  plateau_size=(1, None))
    peaks = properties["left_edges"].astype(int)
    if kappa[0] > kappa_cr and kappa[0] >= kappa[1]:
        peaks = np.r_[0, peaks]
    if kappa[-1] > kappa_cr and kappa[-1] >= kappa[-2]:
        peaks = np.r_[peaks, len(u) - 1]
    peaks = np.unique(peaks)

    def describe(members):
        members = np.asarray(members, dtype=int)
        representative = members[np.argmax(kappa[members])]
        saddle_index = members[0] + np.argmin(kappa[members[0]:members[-1] + 1])
        peak, saddle = kappa[representative], kappa[saddle_index]
        span = arc_mm[members[-1]] - arc_mm[members[0]]
        return {"member_indices": members + 1, "representative_index": representative + 1,
                "representative_u": u[representative], "peak_kappa": peak, "saddle_kappa": saddle,
                "saddle_index": saddle_index + 1, "relative_saddle_depth": max(0., peak - saddle) / peak if peak else 0.,
                "span_mm": span, "curvature_span": peak * span, "start_u": u[members[0]],
                "end_u": u[members[-1]], "merged": len(members) > 1,
                "includes_endpoint": bool(np.any((members == 0) | (members == len(u) - 1)))}

    groups, decisions = [], []
    if len(peaks):
        members = [peaks[0]]
        for peak in peaks[1:]:
            proposed = [*members, peak]
            metrics = describe(proposed)
            conditions = (
                (metrics["includes_endpoint"], "endpoint_boundary"),
                (metrics["saddle_kappa"] < kappa_cr, "below_curvature_threshold"),
                (metrics["relative_saddle_depth"] > settings["MaxRelativeSaddleDepth"], "deep_saddle"),
                (metrics["span_mm"] > settings["MaxPeakDistanceMM"], "distance_exceeds_limit"),
                (metrics["curvature_span"] > settings["MaxCurvatureSpan"], "curvature_span_exceeds_limit"),
            )
            reasons = [name for exceeded, name in conditions if exceeded]
            decisions.append({key: metrics[key] for key in (
                "member_indices", "span_mm", "curvature_span", "relative_saddle_depth", "saddle_kappa",
            )} | {"accepted": not reasons, "reasons": reasons})
            if reasons:
                groups.append(describe(members))
                members = [peak]
            else:
                members = proposed
        groups.append(describe(members))
    selected = np.array([group["representative_index"] for group in groups], dtype=int)
    return selected, {
        "schema_version": 1, "thresholds": settings | {"CurvatureThreshold": kappa_cr},
        "raw_indices": peaks + 1, "selected_indices": selected,
        "removed_indices": np.setdiff1d(peaks + 1, selected), "selected_kappa": kappa[selected - 1],
        "groups": groups, "decisions": decisions,
        "rule_description": "原样本峰值按鞍点深度、物理距离和曲率距离共同筛选，端点独立保留。",
    }
