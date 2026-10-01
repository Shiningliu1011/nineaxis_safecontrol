import os
import signal
import subprocess
import threading
import time
from itertools import count
from pathlib import Path

import rclpy
import yaml
from moveit_msgs.srv import GetMotionPlan, GetPositionIK, GetStateValidity
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

from robot_safecontrol_moveit.moveit_runtime_config import build_moveit_params
from robot_safecontrol_moveit.oscbf_plant import OscbfPlant
from robot_safecontrol_moveit.ros_conventions import state_stream_qos
from robot_safecontrol_moveit.transition_planning_server import TransitionPlanningServer


ROOT = Path(__file__).resolve().parents[1]
START = [
    .027276550053036794, -.03528867367370082, .5059981669275009,
    -.7240440926105509, 1.4877532069125114, -.2066365806668871,
    .3853811558742877, .22248106729961423, -.12343186955927415,
]
DOMAINS = count(200 + os.getpid() % 5)


def wait_for(predicate, timeout=15.):
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() >= deadline:
            raise TimeoutError("condition not reached")
        time.sleep(.01)


class TransitionRuntime:
    """隔离的真实 MoveIt 服务、生产节点与本地被控对象。"""

    def __init__(self, directory, *, moveit=True, plant=True, start_position=None, **parameters):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.domain = next(DOMAINS)
        self.context = Context()
        rclpy.init(context=self.context, domain_id=self.domain)
        self.executor = MultiThreadedExecutor(num_threads=6, context=self.context)
        self.processes = []
        self.logs = []
        self.nodes = []
        self.thread = None
        try:
            if moveit:
                params = build_moveit_params(ROOT)
                self.start_process("moveit_ros_move_group", "move_group", params)
                self.start_process("robot_state_publisher", "robot_state_publisher",
                                   {"robot_description": params["robot_description"]})
            runtime = yaml.safe_load((ROOT / "config/mujoco_transition_runtime.yaml").read_text())
            values = runtime["transition_planning_server"]["ros__parameters"]
            values.update({"auto_plan_once": False, "trajectory_mat": str(ROOT / "data/nurbs/ik_input.mat"),
                           "transition_result_mode": "plan_only", "notify_oscbf_start": False,
                           "joint_state_timeout_s": .5, "replay_joint_state_topic": "/transition_preview"})
            values.update(parameters)
            self.server = TransitionPlanningServer(
                node_name=f"transition_planning_server_{self.domain}",
                context=self.context, parameter_overrides=[Parameter(k, value=v) for k, v in values.items()],
            )
            self.add_node(self.server)
            self.probe = rclpy.create_node(f"transition_runtime_probe_{self.domain}", context=self.context)
            self.add_node(self.probe)
            self.states = []
            self.probe.create_subscription(JointState, "/mujoco_joint_states", self.states.append, state_stream_qos())
            self.plant = None
            if plant:
                self.plant = OscbfPlant(node_name=f"oscbf_plant_{self.domain}", context=self.context, parameter_overrides=[
                    Parameter("production_config_yaml", value=str(ROOT / "config/oscbf_controller.yaml")),
                    Parameter("portable_oscbf_root", value=str(ROOT / "portable_oscbf")),
                    Parameter("start_position", value=START if start_position is None else start_position),
                ])
                self.add_node(self.plant)
            self.thread = threading.Thread(target=self.executor.spin, daemon=True)
            self.thread.start()
            if moveit:
                for service_type, name in (
                    (GetPositionIK, "/compute_ik"), (GetStateValidity, "/check_state_validity"),
                    (GetMotionPlan, "/plan_kinematic_path"),
                ):
                    client = self.probe.create_client(service_type, name)
                    assert client.wait_for_service(timeout_sec=30.), name
            if plant:
                wait_for(lambda: bool(self.states))
        except BaseException:
            self.close()
            raise

    def add_node(self, node):
        self.nodes.append(node)
        self.executor.add_node(node)

    def set_parameters(self, **values):
        results = self.server.set_parameters([Parameter(k, value=v) for k, v in values.items()])
        assert all(result.successful for result in results)

    def start_process(self, package, executable, parameters):
        config = self.directory / f"{executable}.yaml"
        config.write_text(yaml.safe_dump({"/**": {"ros__parameters": parameters}}))
        environment = dict(os.environ, ROS_DOMAIN_ID=str(self.domain), ROS_LOCALHOST_ONLY="1")
        log = (self.directory / f"{executable}.log").open("w")
        self.logs.append(log)
        process = subprocess.Popen([
            "ros2", "run", package, executable, "--ros-args", "--params-file", str(config),
            "-r", "joint_states:=/mujoco_joint_states",
        ], stdout=log, stderr=subprocess.STDOUT, env=environment, start_new_session=True)
        self.processes.append(process)
        return process

    def close(self):
        if self.thread is not None:
            assert self.executor.shutdown(timeout_sec=30.)
            self.thread.join(timeout=5.)
        for node in reversed(self.nodes):
            node.destroy_node()
        self.nodes.clear()
        if self.context.ok():
            rclpy.shutdown(context=self.context)
        for process in self.processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10.)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=5.)
        self.processes.clear()
        for log in self.logs:
            log.close()
