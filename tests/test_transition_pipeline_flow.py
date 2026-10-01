import os
import threading

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from robot_safecontrol_moveit.ros_conventions import state_stream_qos
from robot_safecontrol_moveit.trajectory_execution import TrajectoryExecutor
from transition_runtime import wait_for


def test_real_replay_preserves_joint_order_interpolation_and_timing():
    context = Context()
    rclpy.init(context=context, domain_id=140 + os.getpid() % 10)
    node = rclpy.create_node("ordered_replay", context=context)
    messages = []
    node.create_subscription(JointState, "/ordered_replay", messages.append, state_stream_qos())
    executor = MultiThreadedExecutor(num_threads=2, context=context)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    trajectory = JointTrajectory()
    trajectory.joint_names = ["J2", "J1"]
    for seconds, values in ((.1, [.2, .1]), (.2, [.21, .11])):
        point = JointTrajectoryPoint()
        point.positions = values
        point.time_from_start = Duration(sec=0, nanosec=int(seconds * 1e9))
        trajectory.points.append(point)
    try:
        result = TrajectoryExecutor(node, None, ("J2", "J1")).replay(
            trajectory, topic="/ordered_replay", rate_hz=100.,
        )
        wait_for(lambda: len(messages) == 21)
        assert result.succeeded
        assert all(message.name == ["J2", "J1"] for message in messages)
        np.testing.assert_allclose(messages[0].position, [.2, .1])
        np.testing.assert_allclose(messages[14].position, [.205, .105])
        np.testing.assert_allclose(messages[-1].position, [.21, .11])
        stamps = [msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 for msg in messages]
        assert np.all(np.diff(stamps) > 0)
        assert not [publisher for publisher in node.publishers if publisher.topic == "/ordered_replay"]
    finally:
        executor.shutdown()
        thread.join()
        node.destroy_node()
        rclpy.shutdown(context=context)
