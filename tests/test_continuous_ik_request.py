"""Production-method tests for the direct MoveIt IK request contract."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace


_SRC = Path(__file__).resolve().parents[1] / "src"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from builtin_interfaces.msg import Time  # noqa: E402
from moveit_msgs.msg import MoveItErrorCodes  # noqa: E402
from moveit_msgs.srv import GetPositionIK  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402

from robot_safecontrol_moveit.continuous_ik import (  # noqa: E402
    ContinuousIK,
    IKFailure,
    IKOptions,
)


class _ImmediateFuture:
    def __init__(self, response) -> None:
        self._response = response

    def done(self) -> bool:
        return True

    def result(self):
        return self._response


class _IKClient:
    def __init__(self, response) -> None:
        self.response = response
        self.requests = []

    def wait_for_service(self, *, timeout_sec: float) -> bool:
        return True

    def call_async(self, request):
        self.requests.append(request)
        return _ImmediateFuture(self.response)


class _Clock:
    def now(self):
        return self

    def to_msg(self) -> Time:
        return Time(sec=7, nanosec=9)


class _Node:
    def __init__(self, client: _IKClient) -> None:
        self.client = client
        self.destroyed_clients = []

    def create_client(self, *args):
        return self.client

    def destroy_client(self, client) -> None:
        self.destroyed_clients.append(client)

    def get_clock(self) -> _Clock:
        return _Clock()


def _response(error_code: int) -> GetPositionIK.Response:
    response = GetPositionIK.Response()
    response.error_code.val = error_code
    response.solution.joint_state.name = ["J1", "J2"]
    response.solution.joint_state.position = [0.2, -0.1]
    return response


class TestDirectIKRequest(unittest.TestCase):
    def _solver(self, response: GetPositionIK.Response):
        client = _IKClient(response)
        node = _Node(client)
        moveit = SimpleNamespace(_node=node, base_link_name="base_link")
        solver = ContinuousIK(
            moveit,
            ("J1", "J2"),
            IKOptions(
                tool_link="tool0",
                orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
                planning_group="arm",
                base_frame="base_link",
                service_timeout_s=1.25,
            ),
        )
        return solver, client, node

    def test_request_carries_group_frame_seed_collision_and_timeout(self) -> None:
        solver, client, node = self._solver(_response(MoveItErrorCodes.SUCCESS))
        seed = JointState(name=["J1", "J2"], position=[0.0, 0.0])

        path = solver.solve([(0.1, 0.2, 0.3)], seed)

        request = client.requests[0].ik_request
        self.assertEqual(list(path.first_state.position), [0.2, -0.1])
        self.assertEqual(request.group_name, "arm")
        self.assertEqual(request.pose_stamped.header.frame_id, "base_link")
        self.assertEqual(request.ik_link_name, "tool0")
        self.assertTrue(request.avoid_collisions)
        self.assertEqual(list(request.robot_state.joint_state.name), ["J1", "J2"])
        self.assertEqual(list(request.robot_state.joint_state.position), [0.0, 0.0])
        self.assertEqual(request.timeout.sec, 1)
        self.assertEqual(request.timeout.nanosec, 250_000_000)
        self.assertEqual(node.destroyed_clients, [client])

    def test_non_success_response_is_ik_failure_with_moveit_code(self) -> None:
        solver, client, node = self._solver(_response(MoveItErrorCodes.NO_IK_SOLUTION))
        seed = JointState(name=["J1", "J2"], position=[0.0, 0.0])

        with self.assertRaises(IKFailure) as raised:
            solver.solve([(0.1, 0.2, 0.3)], seed)

        self.assertEqual(
            raised.exception.moveit_error_code,
            MoveItErrorCodes.NO_IK_SOLUTION,
        )
        self.assertEqual(len(client.requests), 1)
        self.assertEqual(node.destroyed_clients, [client])

    def test_requires_explicit_group_and_base_frame(self) -> None:
        """IK request metadata must not depend on pymoveit2 private fields."""
        client = _IKClient(_response(MoveItErrorCodes.SUCCESS))
        moveit = SimpleNamespace(_node=_Node(client))

        with self.assertRaisesRegex(ValueError, "planning_group"):
            ContinuousIK(
                moveit,
                ("J1", "J2"),
                IKOptions(
                    tool_link="tool0",
                    orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
                    base_frame="base_link",
                ),
            )


if __name__ == "__main__":
    unittest.main()
