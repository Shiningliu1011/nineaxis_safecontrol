"""Reproduce OFF-15 analytic evidence; writes only beside this script."""

from pathlib import Path
import hashlib
import json
import platform

import numpy as np

from robot_safecontrol_moveit.tracking_contract import EvaluationScope, EvidenceContext
from robot_safecontrol_moveit.tracking_evaluator import TrackingEvaluator


def main():
    root = Path(__file__).resolve().parent
    script_hash = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    summaries = {}
    for name in ("full-path", "subinterval", "held", "invalid-sample", "admission-rejected", "reversed-axis"):
        start = 0.6 if name == "subinterval" else 0.
        evidence = EvidenceContext(
            run_id=name, kind="analytic", boundary="kernel_candidate",
            model_id="analytic point on x-axis; tool X direction",
            config_id="OFF-15 inherited default thresholds",
            trajectory_id="line-x:0..1m", data_id="sha256:" + script_hash + "#" + name,
            scenario=name, measurement="analytic positions and same-time reference",
            time_basis="explicit test seconds",
        )
        ev = TrackingEvaluator(scope=EvaluationScope(1., start, 1.), evidence=evidence, deadline_ms=20.)
        for i, progress in enumerate(np.linspace(start, 0.5 if name == "held" else 1., 11)):
            position = np.array([progress, 0., 0.])
            if name == "invalid-sample" and i == 5:
                position[1] = np.nan
            rotation = np.diag([-1., -1., 1.]) if name == "reversed-axis" else np.eye(3)
            ev.update(dict(
                ee_pos=position, reference_position_m=np.array([progress, 0., 0.]),
                reference_tangent=np.array([1., 0., 0.]), ee_rot=rotation,
                reference_rotation=np.eye(3), reference_at_endpoint=progress == 1.,
                projected_progress_m=float(progress), reference_progress_m=float(progress),
                reference_source_time_s=1000. + i,  # deliberately unrelated to arc fraction
                qp_ok=True, admission_ok=not (name == "admission-rejected" and i == 5),
                overlap=False, feedrate_m_s=0.01, step_latency_ms=1.,
            ), wall_time_s=float(i))
        ev.finish("held" if name == "held" else "completed")
        report = ev.report()
        (root / (name + ".md")).write_text(report.markdown())
        (root / (name + ".json")).write_text(report.json())
        trace = ev.trace_json()
        (root / (name + ".samples.json")).write_text(trace)
        assert hashlib.sha256(trace.encode()).hexdigest() == report.sample_data_sha256
        summaries[name] = {
            "task": report.task_verdict, "online": report.online_verdict,
            "all_criteria": report.verdict, "completed": report.completed,
            "arc_fraction": report.completion_fraction,
            "scope_fraction": report.scope_completion_fraction,
            "full_path_covered": report.full_path_covered,
            "full_path_verified": report.full_path_verified,
            "sample_data_sha256": report.sample_data_sha256,
        }
    assert summaries["full-path"]["task"] == summaries["subinterval"]["task"] == "pass"
    assert summaries["subinterval"]["full_path_covered"] is False
    assert all(summaries[n]["task"] == "fail" for n in ("held", "invalid-sample", "admission-rejected", "reversed-axis"))
    document = {"python": platform.python_version(), "numpy": np.__version__, "script_sha256": script_hash, "cases": summaries}
    (root / "examples-summary.json").write_text(json.dumps(document, indent=2))
    print(json.dumps(document, indent=2))


if __name__ == "__main__":
    main()
