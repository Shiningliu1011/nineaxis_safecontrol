"""Behaviour tests for the Viewer transition-planning client."""

from __future__ import annotations

import importlib
import sys
import unittest
from pathlib import Path


_SRC = Path(__file__).resolve().parents[1] / "src"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from std_srvs.srv import Trigger  # noqa: E402

_viewer_module = importlib.import_module(
    "robot_safecontrol_moveit.mujoco_viewer_with_cylinder"
)
DYNAMIC_COLLISION_OBJECT_TOPIC = _viewer_module.DYNAMIC_COLLISION_OBJECT_TOPIC
STATIC_COLLISION_OBJECT_TOPIC = _viewer_module.STATIC_COLLISION_OBJECT_TOPIC
MuJoCoJointStateViewer = _viewer_module.MuJoCoJointStateViewer


class _Logger:
    def __init__(self) -> None:
        self.records: list[tuple[str, str]] = []

    def info(self, message: str) -> None:
        self.records.append(("info", message))

    def warning(self, message: str) -> None:
        self.records.append(("warning", message))

    def error(self, message: str) -> None:
        self.records.append(("error", message))


class _Future:
    def __init__(self, done: bool = False) -> None:
        self._done = done

    def done(self) -> bool:
        return self._done


class _TransitionClient:
    def __init__(
        self,
        *,
        ready: bool,
        future: _Future,
        events: list[str],
    ) -> None:
        self._ready = ready
        self._future = future
        self._events = events
        self.requests: list[Trigger.Request] = []

    def service_is_ready(self) -> bool:
        self._events.append("service_is_ready")
        return self._ready

    def call_async(self, request: Trigger.Request) -> _Future:
        self._events.append("call_async")
        self.requests.append(request)
        return self._future


class _ViewerHarness:
    """Minimum collaborator surface consumed by the production method."""

    def __init__(
        self,
        *,
        manual_mode: bool,
        transition_client: _TransitionClient,
        transition_future: _Future | None = None,
    ) -> None:
        self._manual_mode = manual_mode
        self._transition_client = transition_client
        self._transition_future = transition_future
        self._transition_status = ""
        self.events = transition_client._events
        self.logger = _Logger()
        self.publish_count = 0

    def publish_current_qpos(self) -> None:
        self.events.append("publish_current_qpos")
        self.publish_count += 1

    def get_logger(self) -> _Logger:
        return self.logger


class _ParameterRecorder:
    def __init__(self) -> None:
        self.parameters: dict[str, object] = {}

    def declare_parameter(self, name: str, default_value: object) -> None:
        self.parameters[name] = default_value


class TestViewerTransitionClient(unittest.TestCase):
    """Exercise the production request method without constructing a GUI."""

    def test_manual_request_publishes_and_starts_async_call(self) -> None:
        events: list[str] = []
        future = _Future()
        client = _TransitionClient(ready=True, future=future, events=events)
        viewer = _ViewerHarness(manual_mode=True, transition_client=client)

        MuJoCoJointStateViewer.request_transition_plan(viewer)

        self.assertEqual(
            events,
            ["publish_current_qpos", "service_is_ready", "call_async"],
        )
        self.assertEqual(viewer.publish_count, 1)
        self.assertEqual(len(client.requests), 1)
        self.assertIsInstance(client.requests[0], Trigger.Request)
        self.assertIs(viewer._transition_future, future)
        self.assertEqual(viewer._transition_status, "TRANSITION_REQUEST_SENT")
        self.assertEqual(
            viewer.logger.records[-1],
            ("info", "TRANSITION_REQUEST_SENT"),
        )

    def test_tracking_mode_does_not_contact_planning_service(self) -> None:
        events: list[str] = []
        client = _TransitionClient(ready=True, future=_Future(), events=events)
        viewer = _ViewerHarness(manual_mode=False, transition_client=client)

        MuJoCoJointStateViewer.request_transition_plan(viewer)

        self.assertEqual(viewer.publish_count, 0)
        self.assertEqual(client.requests, [])
        self.assertIsNone(viewer._transition_future)
        self.assertEqual(events, [])
        self.assertEqual(viewer.logger.records[0][0], "warning")

    def test_pending_request_is_not_replaced_by_second_key_press(self) -> None:
        events: list[str] = []
        pending = _Future(done=False)
        client = _TransitionClient(ready=True, future=_Future(), events=events)
        viewer = _ViewerHarness(
            manual_mode=True,
            transition_client=client,
            transition_future=pending,
        )

        MuJoCoJointStateViewer.request_transition_plan(viewer)

        self.assertIs(viewer._transition_future, pending)
        self.assertEqual(viewer.publish_count, 0)
        self.assertEqual(client.requests, [])
        self.assertEqual(events, [])
        self.assertEqual(viewer.logger.records[0][0], "warning")

    def test_unavailable_service_never_receives_an_async_request(self) -> None:
        events: list[str] = []
        client = _TransitionClient(
            ready=False,
            future=_Future(),
            events=events,
        )
        viewer = _ViewerHarness(manual_mode=True, transition_client=client)

        MuJoCoJointStateViewer.request_transition_plan(viewer)

        self.assertEqual(viewer.publish_count, 1)
        self.assertEqual(
            events,
            ["publish_current_qpos", "service_is_ready"],
        )
        self.assertEqual(client.requests, [])
        self.assertIsNone(viewer._transition_future)
        self.assertEqual(viewer.logger.records[0][0], "error")


class TestViewerObstacleTopicParameters(unittest.TestCase):
    """The production parameter declaration exposes both QoS endpoints."""

    def test_collision_topics_have_distinct_defaults(self) -> None:
        recorder = _ParameterRecorder()

        MuJoCoJointStateViewer._declare_parameters(recorder)

        self.assertEqual(
            recorder.parameters["static_collision_object_topic"],
            STATIC_COLLISION_OBJECT_TOPIC,
        )
        self.assertEqual(
            recorder.parameters["collision_object_topic"],
            DYNAMIC_COLLISION_OBJECT_TOPIC,
        )
        self.assertNotEqual(
            recorder.parameters["static_collision_object_topic"],
            recorder.parameters["collision_object_topic"],
        )


if __name__ == "__main__":
    unittest.main()
