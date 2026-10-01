import concurrent.futures
import hashlib
import json
import threading
import time
from pathlib import Path

import numpy as np
import pytest
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from robot_safecontrol_moveit.oscbf_controller import OscbfController
from robot_safecontrol_moveit.ros_conventions import command_stream_qos, state_stream_qos
from robot_safecontrol_moveit.transition_planning_server import notify_oscbf_start
from transition_runtime import ROOT, TransitionRuntime, wait_for


@pytest.fixture(scope="module")
def runtime(tmp_path_factory):
    runtime = TransitionRuntime(tmp_path_factory.mktemp("transition-runtime"))
    yield runtime
    runtime.close()


def test_real_planning_and_ros_busy_share_execution(runtime):
    client = runtime.probe.create_client(Trigger, "/plan_transition_once")
    assert client.wait_for_service(timeout_sec=5.)
    with concurrent.futures.ThreadPoolExecutor(max_workers=1) as pool:
        execution = pool.submit(runtime.server.execute_plan)
        wait_for(lambda: runtime.server.is_planning)
        future = client.call_async(Trigger.Request())
        wait_for(future.done)
        assert not future.result().success
        assert future.result().message == "error_code=PLANNING_ALREADY_RUNNING|trajectory_points=0|planning_time=0.000"
        result = execution.result(timeout=45.)
    assert result.code == "TRANSITION_PLANNED", result
    assert result.success and result.trajectory_points > 0
    assert result.planning_time_s > 0
    assert not result.handoff_requested and result.handoff_code is None
    assert not runtime.server.is_planning


def test_real_state_failure_and_unexpected_failure_clear_busy(runtime, tmp_path):
    runtime.executor.remove_node(runtime.plant)
    try:
        time.sleep(1.1)
        result = runtime.server.execute_plan()
        assert result.code == "START_STATE_UNAVAILABLE"
        assert not result.success and not runtime.server.is_planning
    finally:
        runtime.executor.add_node(runtime.plant)
    wait_for(lambda: time.time() - (runtime.states[-1].header.stamp.sec + runtime.states[-1].header.stamp.nanosec * 1e-9) < .1)
    runtime.set_parameters(trajectory_mat=str(tmp_path / "missing.mat"))
    try:
        result = runtime.server.execute_plan()
        assert result.code == "UNEXPECTED_ERROR"
        assert "File not found" in dict(result.context)["detail"]
        assert not runtime.server.is_planning
    finally:
        runtime.set_parameters(trajectory_mat=str(ROOT / "data/nurbs/ik_input.mat"))


def test_real_ros_handoff_unavailable_and_timeout(runtime, tmp_path):
    assert notify_oscbf_start(runtime.probe, "/unstarted_controller/start_tracking", timeout_s=.1) == "START_SERVICE_UNAVAILABLE"
    controller = OscbfController(context=runtime.context, parameter_overrides=[
        Parameter("production_config_yaml", value=str(ROOT / "config/oscbf_controller.yaml")),
        Parameter("perf_report_path", value=str(tmp_path / "timeout" / "perf.md")),
        Parameter("wait_for_start", value=True),
    ])
    try:
        # 节点保持存在，暂停其 executor 调度，产生真实服务响应超时。
        assert notify_oscbf_start(runtime.probe, "/oscbf_controller/start_tracking", timeout_s=1.) == "START_SERVICE_TIMEOUT"
        assert controller.execution_state == "waiting"
    finally:
        controller.destroy_node()


def test_real_replay_without_handoff(runtime):
    runtime.set_parameters(transition_result_mode="joint_state_replay", notify_oscbf_start=False,
                           replay_min_duration_s=.5, replay_time_scale=1.)
    try:
        result = runtime.server.execute_plan()
        assert result.code == "TRANSITION_REPLAYED", result
        assert result.success and result.trajectory_points > 0
        assert not result.handoff_requested and result.handoff_code is None
    finally:
        runtime.set_parameters(transition_result_mode="plan_only")


def test_real_replay_keeps_success_when_handoff_is_unavailable(runtime):
    runtime.set_parameters(transition_result_mode="joint_state_replay", notify_oscbf_start=True,
                           oscbf_start_service="/absent_controller/start_tracking",
                           replay_min_duration_s=.5, replay_time_scale=1.)
    try:
        result = runtime.server.execute_plan()
        assert result.code == "TRANSITION_REPLAYED" and result.success
        assert result.handoff_requested and result.handoff_code == "START_SERVICE_UNAVAILABLE"
    finally:
        runtime.set_parameters(transition_result_mode="plan_only", notify_oscbf_start=False,
                               oscbf_start_service="/oscbf_controller/start_tracking")


def test_ros_request_through_moveit_replay_tracking_and_terminal_report(runtime, tmp_path):
    controller = OscbfController(context=runtime.context, parameter_overrides=[
        Parameter("production_config_yaml", value=str(ROOT / "config/oscbf_controller.yaml")),
        Parameter("perf_report_path", value=str(tmp_path / "perf.md")),
        Parameter("wait_for_start", value=True),
    ])
    commands = []
    runtime.probe.create_subscription(
        JointState, "/oscbf_command", lambda message: commands.append((time.monotonic(), message)), command_stream_qos(),
    )
    runtime.add_node(controller)
    runtime.set_parameters(transition_result_mode="joint_state_replay", notify_oscbf_start=True,
                           replay_min_duration_s=1., replay_time_scale=1.)
    client = runtime.probe.create_client(Trigger, "/plan_transition_once")
    assert client.wait_for_service(timeout_sec=5.)
    observations = []
    export_rejected = False
    try:
        future = client.call_async(Trigger.Request())
        wait_for(future.done, 60.)
        assert future.result().success, future.result().message
        assert "error_code=TRANSITION_REPLAYED|" in future.result().message
        assert controller.progress_snapshot()["tracking_started"]
        wait_for(lambda: controller.progress_snapshot()["steps"] >= 200, 15.)
        deadline = time.monotonic() + 180.
        while time.monotonic() < deadline:
            snapshot = controller.progress_snapshot()
            observations.append((time.monotonic(), snapshot["execution_state"], snapshot["report_status"]["state"], len(commands)))
            if snapshot["report_status"]["state"] == "writing" and not export_rejected:
                with pytest.raises(RuntimeError, match="still writing"):
                    controller.write_tracking_report(str(tmp_path / "concurrent.md"))
                export_rejected = True
            if snapshot["execution_state"] == "holding" and snapshot["report_status"]["state"] in ("saved", "failed"):
                break
            time.sleep(.01)
        assert snapshot["execution_state"] == "holding", snapshot
        assert snapshot["termination"] in ("held", "completed")
        assert snapshot["report_status"]["state"] == "saved", snapshot["report_status"]
        step_count, command_count = snapshot["steps"], len(commands)
        wait_for(lambda: len(commands) >= command_count + 10)
        assert controller.progress_snapshot()["steps"] == step_count
        assert notify_oscbf_start(runtime.probe, "/oscbf_controller/start_tracking") == "ALREADY_TRACKING"
        assert controller.tracking_report().total_steps == step_count
        report = json.loads((tmp_path / "tracking_report.json").read_text())
        assert report["termination"] == snapshot["termination"]
        assert report["sample_data_sha256"] == hashlib.sha256((tmp_path / "tracking_report.samples.json").read_bytes()).hexdigest()
        assert export_rejected
        writing = [row for row in observations if row[2] == "writing"]
        assert writing and writing[-1][3] > writing[0][3]
        (tmp_path / "report-observations.json").write_text(json.dumps(observations))
        (tmp_path / "command-times.json").write_text(json.dumps([
            {"received_monotonic_s": received,
             "ros_stamp_ns": message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec}
            for received, message in commands
        ]))
    finally:
        for timer in controller.timers:
            timer.cancel()
        runtime.executor.remove_node(controller)
        runtime.set_parameters(transition_result_mode="plan_only", notify_oscbf_start=False)


def test_real_missing_moveit_does_not_consume_auto_attempts(tmp_path):
    runtime = TransitionRuntime(tmp_path, moveit=False, auto_plan_once=True, auto_plan_attempts=2)
    try:
        time.sleep(2.2)
        assert runtime.server.auto_plan_snapshot() == dict(enabled=True, done=False, attempts_made=0)
        result = runtime.server.execute_plan()
        assert result.code == "IK_SERVICE_UNAVAILABLE"
        assert not runtime.server.is_planning
    finally:
        runtime.close()


def test_real_auto_retries_resample_and_stop_at_limit(tmp_path):
    runtime = TransitionRuntime(tmp_path, auto_plan_once=True, auto_plan_attempts=2,
                                trajectory_mat=str(tmp_path / "missing.mat"))
    try:
        wait_for(lambda: runtime.server.auto_plan_snapshot()["done"], 20.)
        assert runtime.server.auto_plan_snapshot()["attempts_made"] == 2
        from transition_runtime import START
        assert not np.allclose(runtime.plant.state, START)
        assert not runtime.server.is_planning
        time.sleep(1.1)
        assert runtime.server.auto_plan_snapshot()["attempts_made"] == 2
    finally:
        runtime.close()


def test_auto_waits_for_active_ros_execution_without_consuming_attempt(tmp_path):
    runtime = TransitionRuntime(
        tmp_path, auto_plan_once=True, notify_oscbf_start=True,
        oscbf_start_service="/not_started/start_tracking", transition_result_mode="joint_state_replay",
        replay_min_duration_s=4., replay_time_scale=1.,
    )
    client = runtime.probe.create_client(Trigger, "/plan_transition_once")
    try:
        assert client.wait_for_service(timeout_sec=5.)
        first = client.call_async(Trigger.Request())
        wait_for(lambda: runtime.server.is_planning)
        runtime.set_parameters(notify_oscbf_start=False)
        competing = client.call_async(Trigger.Request())
        wait_for(competing.done)
        assert "error_code=PLANNING_ALREADY_RUNNING|" in competing.result().message
        time.sleep(1.2)
        assert runtime.server.is_planning
        assert runtime.server.auto_plan_snapshot()["attempts_made"] == 0
        wait_for(first.done, 45.)
        assert first.result().success
        wait_for(lambda: runtime.server.is_planning and runtime.server.auto_plan_snapshot()["attempts_made"] == 1, 15.)
        wait_for(lambda: runtime.server.auto_plan_snapshot()["done"], 45.)
        assert runtime.server.auto_plan_snapshot()["attempts_made"] == 1
        assert not runtime.server.is_planning
    finally:
        runtime.close()


def test_active_controller_close_preserves_report_with_state_stream_running(tmp_path):
    runtime = TransitionRuntime(tmp_path, moveit=False)
    controller = OscbfController(context=runtime.context, parameter_overrides=[
        Parameter("production_config_yaml", value=str(ROOT / "config/oscbf_controller.yaml")),
        Parameter("perf_report_path", value=str(tmp_path / "perf.md")),
        Parameter("wait_for_start", value=False),
    ])
    controller_executor = MultiThreadedExecutor(num_threads=2, context=runtime.context)
    controller_executor.add_node(controller)
    controller_thread = threading.Thread(target=controller_executor.spin, daemon=True)
    controller_thread.start()
    try:
        try:
            wait_for(lambda: controller.progress_snapshot()["steps"] >= 5, 15.)
            assert controller.execution_state == "tracking"
        finally:
            assert controller_executor.shutdown(timeout_sec=15.)
            controller_thread.join(timeout=5.)
            controller.destroy_node()
        assert controller.execution_state == "closed"
        assert controller.progress_snapshot()["report_status"]["state"] == "saved"
        assert controller.tracking_report().termination == "interrupted"
        count = len(runtime.states)
        wait_for(lambda: len(runtime.states) >= count + 20)
        assert runtime.thread.is_alive()
    finally:
        runtime.close()


def test_real_state_validity_rejection_stops_before_planning(tmp_path):
    from moveit_msgs.msg import CollisionObject
    from moveit_msgs.srv import ApplyPlanningScene, GetPositionFK, GetStateValidity
    from shape_msgs.msg import SolidPrimitive
    from robot_safecontrol_moveit.robot_spec import DEFAULT_JOINT_NAMES

    runtime = TransitionRuntime(tmp_path, plant=False)
    publisher = runtime.probe.create_publisher(JointState, "/mujoco_joint_states", state_stream_qos())
    # 通过真实状态输入和 MoveIt 有效性服务验证拒绝原因。
    state = JointState(name=list(DEFAULT_JOINT_NAMES), position=[.5] + [0.] * 8)

    def publish_state():
        state.header.stamp = runtime.probe.get_clock().now().to_msg()
        publisher.publish(state)

    timer = runtime.probe.create_timer(.01, publish_state)
    client = runtime.probe.create_client(GetStateValidity, "/check_state_validity")
    try:
        fk = runtime.probe.create_client(GetPositionFK, "/compute_fk")
        assert fk.wait_for_service(timeout_sec=5.)
        fk_request = GetPositionFK.Request()
        fk_request.header.frame_id = "base_link"
        fk_request.fk_link_names = ["tool0"]
        fk_request.robot_state.joint_state = state
        future = fk.call_async(fk_request)
        wait_for(future.done)
        assert future.result().error_code.val == 1
        obstacle = CollisionObject()
        obstacle.id = "architecture_test_start_obstacle"
        obstacle.header.frame_id = "base_link"
        obstacle.operation = CollisionObject.ADD
        obstacle.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[.03])]
        obstacle.primitive_poses = [future.result().pose_stamped[0].pose]
        scene = runtime.probe.create_client(ApplyPlanningScene, "/apply_planning_scene")
        assert scene.wait_for_service(timeout_sec=5.)
        scene_request = ApplyPlanningScene.Request()
        scene_request.scene.is_diff = True
        scene_request.scene.world.collision_objects = [obstacle]
        future = scene.call_async(scene_request)
        wait_for(future.done)
        assert future.result().success
        assert client.wait_for_service(timeout_sec=5.)
        request = GetStateValidity.Request()
        request.robot_state.joint_state = state
        request.group_name = "arm"
        future = client.call_async(request)
        wait_for(future.done)
        assert not future.result().valid
        wait_for(lambda: bool(runtime.states))
        result = runtime.server.execute_plan()
        assert result.code == "START_STATE_COLLISION", result
        assert "START_STATE is invalid" in dict(result.context)["detail"]
        assert not result.success and result.trajectory_points == 0
        assert not result.handoff_requested and not runtime.server.is_planning
    finally:
        timer.cancel()
        runtime.close()


def test_real_auto_waits_for_handoff_readiness_then_succeeds(tmp_path):
    runtime = TransitionRuntime(tmp_path, auto_plan_once=True, notify_oscbf_start=True,
                                oscbf_start_service="/not_started/start_tracking")
    try:
        time.sleep(1.1)
        assert runtime.server.auto_plan_snapshot()["attempts_made"] == 0
        runtime.set_parameters(notify_oscbf_start=False)
        wait_for(lambda: runtime.server.auto_plan_snapshot()["done"], 30.)
        assert runtime.server.auto_plan_snapshot()["attempts_made"] == 1
    finally:
        runtime.close()


def test_real_planner_and_execution_failures_keep_diagnostics(tmp_path):
    runtime = TransitionRuntime(tmp_path / "planner", planning_pipeline="unconfigured_pipeline")
    try:
        result = runtime.server.execute_plan()
        assert result.code == "PLANNER_FAILED", result
        assert dict(result.context)["detail"]
        assert not runtime.server.is_planning
    finally:
        runtime.close()
    runtime = TransitionRuntime(tmp_path / "execute", transition_result_mode="moveit_execute")
    try:
        result = runtime.server.execute_plan()
        assert result.code == "UNEXPECTED_ERROR", result
        assert "execution failed" in dict(result.context)["detail"]
        assert not runtime.server.is_planning
    finally:
        runtime.close()


def test_real_scene_collision_rejects_goal_ik(runtime):
    from geometry_msgs.msg import Pose
    from moveit_msgs.msg import CollisionObject
    from moveit_msgs.srv import ApplyPlanningScene
    from shape_msgs.msg import SolidPrimitive

    client = runtime.probe.create_client(ApplyPlanningScene, "/apply_planning_scene")
    assert client.wait_for_service(timeout_sec=5.)
    obstacle = CollisionObject()
    obstacle.id = "architecture_test_workspace_obstacle"
    obstacle.header.frame_id = "base_link"
    obstacle.operation = CollisionObject.ADD
    box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[20., 20., 20.])
    pose = Pose()
    pose.orientation.w = 1.
    obstacle.primitives = [box]
    obstacle.primitive_poses = [pose]

    def apply_scene():
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.world.collision_objects = [obstacle]
        future = client.call_async(request)
        wait_for(future.done)
        assert future.result().success

    apply_scene()
    runtime.set_parameters(ik_service_timeout_s=.2)
    try:
        result = runtime.server.execute_plan()
        assert result.code == "GOAL_IK_FAILED", result
        context = dict(result.context)
        assert context["moveit_error_code"] == "-31"
        assert context["planning_group"] == "arm"
        assert context["seed_names"] == "J1,J2,J3,J4,J5,J6,J7,J8,J9"
        assert context["avoid_collisions"] == "true"
        assert result.trajectory_points == 0 and not result.handoff_requested
    finally:
        obstacle.operation = CollisionObject.REMOVE
        apply_scene()
        runtime.set_parameters(ik_service_timeout_s=5.)
