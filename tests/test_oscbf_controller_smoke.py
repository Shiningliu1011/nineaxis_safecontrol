"""M10 acceptance smoke tests for the ``oscbf_controller`` ROS 2 node.

These tests deliberately avoid MoveIt: they construct the controller with
parameter injection, exercise its pure ``step_once`` method, and verify the
published ``JointState`` over a short-lived ROS graph on an isolated DDS
domain.  The node's JAX warm-up runs once in a module-scoped fixture.
"""

from __future__ import annotations

import os
import json
import re
import sys
import time
from pathlib import Path

# Keep the single-threaded XLA contract of the portable suite before the node
# imports JAX inside its constructor.
# 单线程 XLA 限制已移除：M6 等价门使用 1e-4 容差，多线程 XLA 在此容差内
# 行为一致；生产环境允许多线程以加速 JIT 编译。

import numpy as np
import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
_SRC = REPO_ROOT / "src"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


_DOMAIN_ID = 130 + (os.getpid() % 20)
_STATE_TOPIC = "/mujoco_joint_states"
_COMMAND_TOPIC = "/oscbf_command"
_START_SERVICE = "/oscbf_controller/start_tracking"
_START_Q = np.array([
    0.2303562, 0.1112539, 1.0167209, -0.6810303, -1.8294025,
    -0.4664294, 0.4743473, -1.0429228, 0.0289233,
])


@pytest.fixture(scope="module")
def controller_fixture(tmp_path_factory):
    from robot_safecontrol_moveit.oscbf_controller import OscbfController

    context = Context()
    rclpy.init(context=context, domain_id=_DOMAIN_ID)
    perf_path = tmp_path_factory.mktemp("oscbf_m10") / "perf.md"
    node = OscbfController(
        node_name="oscbf_controller_smoke",
        context=context,
        parameter_overrides=[
            rclpy.parameter.Parameter(
                "production_config_yaml",
                value=str(REPO_ROOT / "config" / "oscbf_controller.yaml"),
            ),
            rclpy.parameter.Parameter(
                "portable_oscbf_root",
                value=str(REPO_ROOT / "portable_oscbf"),
            ),
            rclpy.parameter.Parameter(
                "trajectory_mat",
                value=str(REPO_ROOT / "data" / "nurbs" / "ik_input.mat"),
            ),
            rclpy.parameter.Parameter(
                "portable_config_yaml",
                value=str(REPO_ROOT / "portable_oscbf" / "config" / "nineaxis.yaml"),
            ),
            rclpy.parameter.Parameter("perf_report_path", value=str(perf_path)),
            rclpy.parameter.Parameter("dt", value=0.012),
            rclpy.parameter.Parameter("dt_path", value=0.013),
            rclpy.parameter.Parameter("kp_pos", value=161.0),
            rclpy.parameter.Parameter("w_pos", value=41.0),
            rclpy.parameter.Parameter("publish_frequency_hz", value=80.0),
            rclpy.parameter.Parameter("telemetry_period_s", value=0.5),
            rclpy.parameter.Parameter("wait_for_start", value=True),
        ],
    )
    try:
        yield {"node": node, "context": context, "perf_path": perf_path}
    finally:
        node.write_perf_report()
        node.destroy_node()
        rclpy.shutdown(context=context)


def _in_bounds(node, positions: np.ndarray) -> bool:
    lower, upper = node._limits
    return bool(
        np.all(np.isfinite(positions))
        and np.all(positions >= lower - 1e-9)
        and np.all(positions <= upper + 1e-9)
    )


def test_state_subscription_uses_deep_best_effort_queue():
    # 状态流 QoS 契约已集中到 ros_conventions.state_stream_qos()（构造与
    # 消费共用同一函数），此处断言其属性防回归。
    from robot_safecontrol_moveit.ros_conventions import state_stream_qos

    qos = state_stream_qos()
    assert qos.depth == 20
    assert qos.reliability == rclpy.qos.ReliabilityPolicy.BEST_EFFORT
    assert qos.durability == rclpy.qos.DurabilityPolicy.VOLATILE


def test_node_starts_without_move_group(controller_fixture):
    node = controller_fixture["node"]
    assert node.get_name() == "oscbf_controller_smoke"
    assert node._loop.path_is_configured


def test_production_values_reach_actual_consumers_and_provenance(
    controller_fixture,
):
    node = controller_fixture["node"]

    effective = node.effective_configuration()
    diagnostics = node.runtime_configuration_diagnostics()

    assert effective["values"]["dt"] == 0.012
    assert effective["values"]["dt_path"] == 0.013
    assert effective["values"]["kp_pos"] == 161.0
    assert effective["sources"]["kp_pos"] == "explicit_override"
    assert effective["override_chains"]["kp_pos"] == [
        {"source": "production_yaml", "value": 160.0},
        {"source": "explicit_override", "value": 161.0},
    ]
    assert diagnostics["facade"] == {
        "dt": 0.012,
        "dt_path": 0.013,
        "w_pos": 41.0,
        "w_orient": 10.0,
        "w_joint": 0.1,
        "temporal_lambda": 0.2,
        "enable_x64": True,
        "solver_tol": 0.001,
        "task_mode": "tool_axis_5d",
    }
    assert diagnostics["path_tracking_step"]["kp_pos"] == 161.0
    assert diagnostics["timer_period_s"] == pytest.approx(1.0 / 80.0)
    assert diagnostics["telemetry_period_s"] == pytest.approx(0.5)


def test_tracking_sentinels_reach_the_actual_facade_call(controller_fixture):
    node = controller_fixture["node"]
    captured = {}
    original = node._loop.path_tracking_step

    def _capture_path_tracking_step(**kwargs):
        captured.update(kwargs)
        return original(**kwargs)

    node._loop.path_tracking_step = _capture_path_tracking_step
    try:
        node.step_once(_START_Q)
    finally:
        node._loop.path_tracking_step = original

    assert captured["kp_pos"] == 161.0
    assert captured["kp_orient"] == 10.0
    assert captured["kp_joint"] == 0.45
    assert captured["nullspace_speed_limit"] == 0.18
    assert captured["damping"] == 0.05


def test_runtime_single_managed_parameter_change_is_rejected(controller_fixture):
    node = controller_fixture["node"]
    before = node.runtime_configuration_diagnostics()

    result = node.set_parameters(
        [rclpy.parameter.Parameter("kp_pos", value=170.0)]
    )[0]

    assert not result.successful
    assert "restart" in result.reason
    assert node.get_parameter("kp_pos").value == 161.0
    assert node.runtime_configuration_diagnostics() == before


def test_runtime_atomic_batch_change_is_rejected_without_partial_update(
    controller_fixture,
):
    node = controller_fixture["node"]
    before = node.runtime_configuration_diagnostics()

    result = node.set_parameters_atomically(
        [
            rclpy.parameter.Parameter("dt", value=0.02),
            rclpy.parameter.Parameter("dt_path", value=0.03),
            rclpy.parameter.Parameter("publish_frequency_hz", value=50.0),
        ]
    )

    assert not result.successful
    assert "restart" in result.reason
    assert node.get_parameter("dt").value == 0.012
    assert node.get_parameter("dt_path").value == 0.013
    assert node.get_parameter("publish_frequency_hz").value == 80.0
    assert node.runtime_configuration_diagnostics() == before


def test_runtime_snapshot_matches_consumers_and_records_version_identity(
    controller_fixture,
):
    node = controller_fixture["node"]
    path = node.runtime_snapshot_path
    snapshot = json.loads(path.read_text(encoding="utf-8"))
    diagnostics = node.runtime_configuration_diagnostics()

    assert path.is_absolute()
    assert snapshot["snapshot_path"] == str(path)
    assert snapshot["final_values"]["dt"] == diagnostics["facade"]["dt"]
    assert snapshot["final_values"]["dt_path"] == diagnostics["facade"]["dt_path"]
    assert snapshot["final_values"]["publish_frequency_hz"] == 80.0
    assert snapshot["parameter_sources"]["kp_pos"] == "explicit_override"
    assert snapshot["override_chains"]["kp_pos"][0]["value"] == 160.0
    assert snapshot["resources"]["trajectory_mat"]["resolved_path"] == str(
        REPO_ROOT / "data" / "nurbs" / "ik_input.mat"
    )
    assert snapshot["topic_connections"]["joint_state_topic"] == {
        "raw_value": "/mujoco_joint_states",
        "resolved_topic": "/mujoco_joint_states",
    }
    assert snapshot["topic_connections"]["publish_joint_state_topic"] == {
        "raw_value": "/oscbf_command",
        "resolved_topic": "/oscbf_command",
    }
    assert snapshot["production_config"]["sha256"]
    assert snapshot["software"]["source_sha256"]
    assert isinstance(snapshot["software"]["git_dirty"], bool)
    assert snapshot["geometry"]["cylinder_center_source"] == "trajectory_fit"
    assert len(snapshot["geometry"]["resolved_center"]) == 3
    assert snapshot["obstacle_alpha_contract"] == {
        "baseline_source": "NineaxisOSCBFVelocityConfig",
        "baseline_value": 10.0,
        "runtime_value_source": "perception track slot 10",
    }


def test_step_once_returns_valid_safe_state(controller_fixture):
    node = controller_fixture["node"]
    step = node.step_once(_START_Q)
    q_next = step["q_next"]
    assert q_next.shape == (9,)
    assert _in_bounds(node, q_next)
    assert step["u_safe"].shape == (9,)
    assert step["err_6d"].shape == (6,)
    assert np.all(np.isfinite(step["u_safe"]))
    assert np.all(np.isfinite(step["err_6d"]))
    assert step["qp_ok"]
    assert step["min_obs_dist"] > 0.0


def test_tracking_evaluator_integration(controller_fixture):
    """评价器在控制器内正确累积跟踪指标。"""
    node = controller_fixture["node"]
    # 初始状态：评价器为 None（尚未开始跟踪）
    assert node.tracking_report() is None
    # 模拟几步跟踪
    from robot_safecontrol_moveit.tracking_evaluator import TrackingEvaluator
    node._evaluator = TrackingEvaluator(trajectory_duration_s=30.0)
    for i in range(5):
        step = node.step_once(_START_Q)
        node._evaluator.update(step, wall_time_s=float(i) * 0.01)
    report = node.tracking_report()
    assert report is not None
    assert report.total_steps == 5
    assert report.qp_success_rate == 1.0
    assert report.tracking_score > 0.0
    assert "score=" in report.summary()


def test_no_command_before_start_signal(controller_fixture):
    node = controller_fixture["node"]
    context = controller_fixture["context"]

    probe = rclpy.create_node(
        "oscbf_gate_probe", context=context
    )
    received = []

    def _on_state(message: JointState) -> None:
        received.append(message)

    probe.create_subscription(
        JointState, _COMMAND_TOPIC, _on_state, qos_profile_sensor_data
    )
    plant_pub = probe.create_publisher(
        JointState, _STATE_TOPIC, qos_profile_sensor_data
    )
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(probe)
    executor.add_node(node)

    plant = JointState()
    plant.name = ["J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9"]
    plant.position = [float(value) for value in _START_Q]

    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        plant_pub.publish(plant)
        executor.spin_once(timeout_sec=0.2)

    executor.remove_node(node)
    probe.destroy_node()
    assert not received, "controller published before the start signal"


def test_start_signal_unlocks_safe_state(controller_fixture):
    from std_srvs.srv import Trigger

    node = controller_fixture["node"]
    context = controller_fixture["context"]

    client_node = rclpy.create_node("oscbf_start_client", context=context)
    client = client_node.create_client(Trigger, _START_SERVICE)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(client_node)
    executor.add_node(node)
    deadline = time.monotonic() + 5.0
    while not client.service_is_ready() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.1)
    assert client.service_is_ready(), "start service never became ready"
    future = client.call_async(Trigger.Request())
    deadline = time.monotonic() + 5.0
    while not future.done() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.1)
    assert future.done(), "start service call timed out"
    assert future.result().success
    executor.remove_node(client_node)
    executor.remove_node(node)
    client_node.destroy_node()

    probe = rclpy.create_node("oscbf_controller_smoke_probe", context=context)
    received = []
    probe.create_subscription(
        JointState, _COMMAND_TOPIC,
        lambda message: received.append(message),
        qos_profile_sensor_data,
    )
    plant_pub = probe.create_publisher(
        JointState, _STATE_TOPIC, qos_profile_sensor_data
    )
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(probe)
    executor.add_node(node)

    plant = JointState()
    plant.name = ["J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9"]
    plant_positions = _START_Q.copy()
    plant_positions[0] += 0.01
    plant.position = [float(value) for value in plant_positions]

    deadline = time.monotonic() + 15.0
    output = None
    while time.monotonic() < deadline:
        plant_pub.publish(plant)
        executor.spin_once(timeout_sec=0.2)
        for message in received:
            positions = np.asarray(message.position, dtype=float)
            if positions.shape == (9,) and np.max(
                np.abs(positions - plant_positions)
            ) > 1e-6:
                output = message
                break
        if output is not None:
            break

    executor.remove_node(node)
    probe.destroy_node()
    assert output is not None, "controller published no safe state after start"
    assert output.name == ["J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8", "J9"]
    assert len(output.position) == 9
    assert _in_bounds(node, np.asarray(output.position, dtype=float))


def test_progress_snapshot_reports_tracking_state(controller_fixture):
    node = controller_fixture["node"]
    snapshot = node.progress_snapshot()
    assert snapshot["tracking_started"]
    assert snapshot["ready"]
    assert snapshot["steps"] >= 1
    assert 0.0 <= snapshot["arc_fraction"] <= 1.0
    assert np.isfinite(snapshot["cross_track_error_m"])
    assert np.isfinite(snapshot["latency_p95_ms"])


def _call_start_tracking(node, context) -> None:
    """Explicitly start tracking via the service (order-independent tests).

    The controller only steps once ``/oscbf_controller/start_tracking`` has
    been served, so any test that needs real step-latency samples must start
    it itself instead of relying on a sibling test's side effect.
    """
    from std_srvs.srv import Trigger

    client_node = rclpy.create_node("oscbf_start_client", context=context)
    client = client_node.create_client(Trigger, _START_SERVICE)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(client_node)
    executor.add_node(node)
    deadline = time.monotonic() + 5.0
    while not client.service_is_ready() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.1)
    assert client.service_is_ready(), "start service never became ready"
    future = client.call_async(Trigger.Request())
    deadline = time.monotonic() + 5.0
    while not future.done() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.1)
    assert future.done(), "start service call timed out"
    assert future.result().success
    executor.remove_node(client_node)
    client_node.destroy_node()


def test_perf_report_p95_within_budget(controller_fixture):
    node = controller_fixture["node"]
    context = controller_fixture["context"]
    budget_ms = float(node.get_parameter("latency_budget_ms").value)

    # 该测试必须自给自足：显式启动跟踪，不依赖同模块前序测试的泄漏状态
    # （见 ticket 02/perf 孤立性）。模块内顺序运行时节点可能已处于跟踪状态,
    # 服务幂等返回 ALREADY_TRACKING, 重复调用安全。
    if not node._tracking_started:
        _call_start_tracking(node, context)

    # 采集真实的 path_tracking_step 延迟样本。不用 ROS 管线泵送: 测试里
    # spin_once(0.02)+密集 publish 会使订阅回调饿死控制定时器(单线程
    # executor 每轮只处理一个 waitable), 实测 5s 只得到 2 个样本; 而
    # _control_tick 计时段就是 start→step_once, 与 step_once 直接循环
    # 测量的是同一段代码。用 q_next 闭环推进(等价于 plant 跟随命令),
    # 让跟踪持续移动、大样本确定可复现。
    if len(node._step_durations) < 20:
        node._hold_q = None
        q_follow = _START_Q.copy()
        for _ in range(200):
            t0 = time.perf_counter()
            step = node.step_once(q_follow)
            node._step_durations.append((time.perf_counter() - t0) * 1000.0)
            q_follow = np.asarray(step["q_next"], dtype=float)

    node.write_perf_report()
    text = controller_fixture["perf_path"].read_text(encoding="utf-8")
    # Preserve failed measurements too; a stale passing report must not hide
    # the latest regression. Keep isolated evidence separate from demo output.
    evidence_path = REPO_ROOT / "output" / "oscbf_m10_perf_isolated.md"
    evidence_path.parent.mkdir(parents=True, exist_ok=True)
    evidence_path.write_text(text, encoding="utf-8")
    match = re.search(r"p95: ([0-9.]+) ms", text)
    assert match is not None, f"missing p95 in report:\n{text}"
    p95 = float(match.group(1))
    assert 0.0 < p95 < budget_ms, (
        f"p95 step latency {p95:.3f} ms out of budget ({budget_ms:.0f} ms)")
    # 01B 口径: 50Hz 预算 + miss rate<=1%（超预算步数占比）。
    miss_match = re.search(r"miss rate = ([0-9.]+)%", text)
    assert miss_match is not None, f"missing miss rate in report:\n{text}"
    miss_rate = float(miss_match.group(1)) / 100.0
    assert miss_rate <= 0.01, (
        f"miss rate {miss_rate * 100:.2f}% exceeds the 1% budget")
    assert len(node._step_durations) >= 20, (
        "perf report needs >= 20 step samples to be meaningful, got "
        f"{len(node._step_durations)}")
