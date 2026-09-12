"""Isolated ROS timer + real 3000-sample background report workload.

Writes only to a temporary directory and publishes on a test-only topic in an
isolated DDS domain. It does not construct a controller, bridge or hardware I/O.
"""

from dataclasses import replace
import hashlib
import json
import os
from pathlib import Path
import tempfile
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState

from robot_safecontrol_moveit.tracking_contract import EvaluationScope, EvidenceContext
from robot_safecontrol_moveit.tracking_evaluator import TrackingEvaluator, TrackingStepData
from robot_safecontrol_moveit.tracking_report_writer import TrackingReportWriter


def main():
    archived = Path(__file__).resolve().parent.parent / "2026-09-12-off15/controller-tracking.samples.json"
    archive = json.loads(archived.read_text())
    sample = dict(archive["samples"][0])
    sample.pop("wall_time_s")
    evidence = replace(EvidenceContext(**archive["evidence"]),
                       run_id="off15-report-concurrency", kind="analytic",
                       data_id="repeated archived sample sha256:" + hashlib.sha256(archived.read_bytes()).hexdigest(),
                       scenario="3000 repeated records; writer load only, not a tracking-quality experiment")
    evaluator = TrackingEvaluator(scope=EvaluationScope(**archive["scope"]), evidence=evidence)
    for index in range(3000):
        evaluator.update(TrackingStepData(**sample), wall_time_s=index * .01)
    evaluator.finish("held")

    context = Context()
    rclpy.init(context=context, domain_id=210 + os.getpid() % 20)
    node = rclpy.create_node("off15_report_load_probe", context=context)
    publisher = node.create_publisher(JointState, "/off15_report_load_test/hold", 10)
    received, stamps = [], []
    node.create_subscription(JointState, "/off15_report_load_test/hold", received.append, 10)
    message = JointState()
    message.position = [0.] * 9  # test-only payload, never sent to a command topic

    def tick():
        stamps.append(time.perf_counter())
        publisher.publish(message)

    node.create_timer(.01, tick)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    writer = TrackingReportWriter()
    try:
        # Complete DDS discovery before measuring the concurrent workload.
        until = time.perf_counter() + .2
        while time.perf_counter() < until:
            executor.spin_once(timeout_sec=.01)
        stamps.clear()
        received.clear()
        with tempfile.TemporaryDirectory(prefix="off15-report-load-") as temporary:
            path = Path(temporary) / "tracking.md"
            started = time.perf_counter()
            writer.submit(evaluator, str(path))
            submitted = time.perf_counter()
            while writer.status()["state"] == "writing":
                executor.spin_once(timeout_sec=.01)
                if time.perf_counter() - started > 30.:
                    raise RuntimeError("report workload exceeded 30 seconds")
            finished = time.perf_counter()
            assert writer.status()["state"] == "saved", writer.status()
            summary = json.loads(path.with_suffix(".json").read_text())
            samples = path.with_suffix(".samples.json").read_bytes()
            assert summary["sample_data_sha256"] == hashlib.sha256(samples).hexdigest()
            assert len(stamps) >= 3 and len(received) >= 3
            gaps = [b - a for a, b in zip([started] + stamps, stamps + [finished])]
            result = dict(samples=3000, submit_ms=(submitted-started)*1000,
                          report_s=finished-started, published=len(stamps), received=len(received),
                          nominal_timer_ms=10., max_observed_gap_ms=max(gaps)*1000,
                          sample_bytes=len(samples), hash_matches=True,
                          scope="isolated ROS timer and writer workload; no physical control or realtime guarantee")
            print(json.dumps(result, ensure_ascii=False, indent=2))
    finally:
        writer.close()
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)


if __name__ == "__main__":
    main()
