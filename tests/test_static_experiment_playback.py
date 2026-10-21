import os
import json
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np
import pytest

from robot_safecontrol_moveit.static_experiment_config import content_hash
from robot_safecontrol_moveit.static_experiment_recording import read_stream


ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def recorded_run():
    path = os.environ.get("STATIC_PLAYBACK_RUN")
    if path is None:
        pytest.skip("STATIC_PLAYBACK_RUN 指定已有完整仿真原件")
    run = Path(path).resolve(strict=True)
    before = {name: content_hash(run / name) for name in (
        "manifest.json", "feedback_report.json", "plant.sqlite3", "controller.sqlite3",
    )}
    yield run
    assert {name: content_hash(run / name) for name in before} == before


def test_saved_run_loads_original_feedback_and_relocated_resources(recorded_run):
    from robot_safecontrol_moveit.static_experiment_playback import load_playback_recording

    recording = load_playback_recording(recorded_run, ROOT)
    original = read_stream(recorded_run / "plant.sqlite3", "feedback")
    assert len(recording.feedback) == len(original) > 100
    for actual, expected in zip(recording.feedback, original):
        assert actual["simulation_s"] == expected["simulation_s"]
        assert actual["ros_time_ns"] == expected["ros_time_ns"]
        np.testing.assert_array_equal(actual["values"]["q"], expected["values"]["q"])
        np.testing.assert_array_equal(actual["values"]["velocity"], expected["values"]["velocity"])
    assert recording.manifest["config"]["mode"] == "sim"
    assert recording.manifest["resource_root"] == str(ROOT)
    assert recording.identity["records_sha256"]["plant"] == content_hash(recorded_run / "plant.sqlite3")


def test_playback_publishes_all_original_states_in_order_and_recorded_time(recorded_run):
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from robot_safecontrol_moveit.robot_spec import DEFAULT_JOINT_NAMES
    from robot_safecontrol_moveit.ros_conventions import JOINT_STATE_TOPIC, OSCBF_COMMAND_TOPIC
    from robot_safecontrol_moveit.static_experiment_playback import (
        PlaybackNode, load_playback_recording,
    )

    recording = load_playback_recording(recorded_run, ROOT)
    expected = recording.feedback
    duration = expected[-1]["simulation_s"] - expected[0]["simulation_s"]
    context = Context()
    rclpy.init(context=context, args=[], domain_id=232)
    executor = SingleThreadedExecutor(context=context)
    observer = Node("static_playback_observer", context=context)
    playback = PlaybackNode(recording, context=context)
    received = []
    observer.create_subscription(
        JointState, playback.topic, lambda message: received.append((time.monotonic(), message)),
        QoSProfile(depth=10000, reliability=ReliabilityPolicy.RELIABLE),
    )
    executor.add_node(observer)
    executor.add_node(playback)
    try:
        deadline = time.monotonic() + 5
        while not playback.ready and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
        assert playback.ready
        assert playback.topic.startswith("/static_experiment_playback/")
        assert observer.count_publishers(JOINT_STATE_TOPIC) == 0
        assert observer.count_publishers(OSCBF_COMMAND_TOPIC) == 0
        started = time.monotonic()
        playback.start()
        deadline = started + duration + 5
        while (playback.status != "completed" or len(received) < len(expected)) and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.005)
        assert playback.status == "completed"
        assert len(received) == len(expected)
        assert duration - .05 <= received[-1][0] - received[0][0] <= duration + .5
        for (_, message), row in zip(received, expected):
            assert message.name == list(DEFAULT_JOINT_NAMES)
            assert message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec == row["ros_time_ns"]
            np.testing.assert_array_equal(message.position, row["values"]["q"])
            np.testing.assert_array_equal(message.velocity, row["values"]["velocity"])
        count = len(received)
        for _ in range(10):
            executor.spin_once(timeout_sec=.01)
        assert len(received) == count
    finally:
        executor.shutdown()
        playback.destroy_node()
        observer.destroy_node()
        context.shutdown()


def test_public_playback_cli_finishes_at_original_model_state(recorded_run, tmp_path):
    output = tmp_path / "playback"
    result = subprocess.run(
        [sys.executable, str(ROOT / "scripts/static_obstacle_experiment.py"), "playback",
         "--output", str(recorded_run), "--playback-output", str(output),
         "--resource-root", str(ROOT), "--domain", "232", "--time-scale", ".1", "--headless"],
        capture_output=True, text=True, timeout=60,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    identity = json.loads((output / "playback.json").read_text())
    report = json.loads((output / "playback_result.json").read_text())
    original = read_stream(recorded_run / "plant.sqlite3", "feedback")
    assert identity["source"]["records_sha256"]["plant"] == content_hash(recorded_run / "plant.sqlite3")
    assert identity["mode"] == "sim"
    assert identity["domain"] == 232
    assert report["status"] == "completed"
    assert report["published_samples"] == report["total_samples"] == len(original)
    assert report["first_simulation_s"] == original[0]["simulation_s"]
    assert report["last_simulation_s"] == original[-1]["simulation_s"]
    assert report["applied_ros_time_ns"] == original[-1]["ros_time_ns"]
    np.testing.assert_allclose(report["model_position"], original[-1]["values"]["q"], atol=1e-12, rtol=0.)
    assert abs(report["elapsed_s"] - report["scheduled_duration_s"]) < .5
    assert report["nodes_destroyed"]


@pytest.mark.parametrize("signum", (signal.SIGINT, signal.SIGTERM))
def test_public_playback_termination_cleans_display_nodes(recorded_run, tmp_path, signum):
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from robot_safecontrol_moveit.ros_conventions import JOINT_STATE_TOPIC, OSCBF_COMMAND_TOPIC

    output = tmp_path / "interrupted"
    context = Context()
    rclpy.init(context=context, args=[], domain_id=232)
    observer = Node("playback_cleanup_observer", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(observer)
    process = None
    received = []
    try:
        with (tmp_path / "process.log").open("w") as log:
            process = subprocess.Popen(
                [sys.executable, str(ROOT / "scripts/static_obstacle_experiment.py"), "playback",
                 "--output", str(recorded_run), "--playback-output", str(output),
                 "--resource-root", str(ROOT), "--domain", "232", "--headless"],
                stdout=log, stderr=subprocess.STDOUT,
            )
            deadline = time.monotonic() + 40.
            subscription = None
            while not received and time.monotonic() < deadline and process.poll() is None:
                executor.spin_once(timeout_sec=.01)
                topics = [name for name, types in observer.get_topic_names_and_types()
                          if name.startswith("/static_experiment_playback/") and "sensor_msgs/msg/JointState" in types]
                if topics and subscription is None:
                    subscription = observer.create_subscription(
                        JointState, topics[0], received.append,
                        QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE),
                    )
            assert received, (tmp_path / "process.log").read_text()
            assert observer.count_publishers(JOINT_STATE_TOPIC) == 0
            assert observer.count_publishers(OSCBF_COMMAND_TOPIC) == 0
            assert set(observer.get_node_names()) == {
                "playback_cleanup_observer", "mujoco_joint_state_viewer",
                next(name for name in observer.get_node_names() if name.startswith("static_playback_")),
            }
            process.send_signal(signum)
            assert process.wait(timeout=10) == 130, (tmp_path / "process.log").read_text()
        report = json.loads((output / "playback_result.json").read_text())
        assert report["status"] == "interrupted"
        assert 0 < report["published_samples"] < report["total_samples"]
        assert report["nodes_destroyed"]
        deadline = time.monotonic() + 5.
        while set(observer.get_node_names()) != {"playback_cleanup_observer"} and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
        assert set(observer.get_node_names()) == {"playback_cleanup_observer"}
        assert observer.count_publishers(report["topic"]) == 0
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            process.wait(timeout=10)
        executor.shutdown()
        observer.destroy_node()
        context.shutdown()
