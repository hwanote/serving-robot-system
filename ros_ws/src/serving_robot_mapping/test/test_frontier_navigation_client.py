"""Offline tests for the Mapping Nav2 action client."""

import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time

from serving_robot_mapping.exploration_goal_manager import (
    VerifiedFrontierGoal,
)
import serving_robot_mapping.frontier_navigation_client as nav


class FakeFuture:
    def __init__(self):
        self.callbacks = []
        self.value = None

    def add_done_callback(self, callback):
        self.callbacks.append(callback)

    def resolve(self, value):
        self.value = value
        for callback in list(self.callbacks):
            callback(self)

    def result(self):
        return self.value


class FakeHandle:
    accepted = True

    def __init__(self):
        self.result_future = FakeFuture()
        self.cancel_count = 0
        self.cancel_future = FakeFuture()

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancel_count += 1
        return self.cancel_future


class FakeActionClient:
    def __init__(self, node, action_type, action_name):
        self.ready = True
        self.sent = []
        self.accept_future = FakeFuture()
        self.handle = FakeHandle()

    def server_is_ready(self):
        return self.ready

    def send_goal_async(self, request):
        self.sent.append(request)
        return self.accept_future


class FakeNode:
    def get_clock(self):
        return SimpleNamespace(
            now=lambda: SimpleNamespace(
                to_msg=lambda: Time(sec=10)
            )
        )

    def get_logger(self):
        return SimpleNamespace(
            info=lambda message: None,
            error=lambda message: None,
        )


class NavigationClientTests(unittest.TestCase):
    def setUp(self):
        patcher = patch.object(
            nav, "ActionClient", FakeActionClient
        )
        patcher.start()
        self.addCleanup(patcher.stop)

        self.node = FakeNode()
        self.goal = VerifiedFrontierGoal(
            x=0.42,
            y=0.03,
            path_length_m=0.44,
        )

    def make_client(self, **kwargs):
        return nav.FrontierNavigationClient(
            self.node, **kwargs
        )

    def test_disabled_by_default(self):
        client = self.make_client(
            safety_check=lambda: True
        )
        self.assertFalse(client.start(self.goal))
        self.assertEqual(client.client.sent, [])
        self.assertFalse(client.busy)

    def test_missing_safety_check(self):
        client = self.make_client(enabled=True)
        self.assertFalse(client.start(self.goal))
        self.assertEqual(client.client.sent, [])

    def test_safety_check_failure(self):
        client = self.make_client(
            enabled=True,
            safety_check=lambda: False,
        )
        self.assertFalse(client.start(self.goal))
        self.assertEqual(client.client.sent, [])

    def test_successful_action_result(self):
        events = []
        client = self.make_client(
            enabled=True,
            safety_check=lambda: True,
            on_finished=lambda *args: events.append(args),
        )

        self.assertTrue(client.start(self.goal))
        self.assertEqual(client.phase, "WAIT_ACCEPT")
        self.assertEqual(len(client.client.sent), 1)

        pose = client.client.sent[0].pose
        self.assertEqual(pose.header.frame_id, "map")
        self.assertAlmostEqual(pose.pose.position.x, 0.42)
        self.assertAlmostEqual(pose.pose.position.y, 0.03)

        handle = client.client.handle
        client.client.accept_future.resolve(handle)

        self.assertEqual(client.phase, "MOVING")

        handle.result_future.resolve(
            SimpleNamespace(
                status=GoalStatus.STATUS_SUCCEEDED
            )
        )

        self.assertFalse(client.busy)
        self.assertEqual(events[0][1], "SUCCEEDED")

    def test_safety_loss_requests_cancel(self):
        state = {"safe": True}

        client = self.make_client(
            enabled=True,
            safety_check=lambda: state["safe"],
        )

        self.assertTrue(client.start(self.goal))

        handle = client.client.handle
        client.client.accept_future.resolve(handle)

        state["safe"] = False
        client.tick()

        self.assertEqual(client.phase, "CANCELLING")
        self.assertEqual(handle.cancel_count, 1)

    def test_cancel_before_acceptance(self):
        client = self.make_client(
            enabled=True,
            safety_check=lambda: True,
        )

        self.assertTrue(client.start(self.goal))
        self.assertTrue(client.cancel("Test stop"))

        handle = client.client.handle
        client.client.accept_future.resolve(handle)

        self.assertEqual(client.phase, "CANCELLING")
        self.assertEqual(handle.cancel_count, 1)

    def test_navigation_timeout(self):
        client = self.make_client(
            enabled=True,
            safety_check=lambda: True,
        )

        self.assertTrue(client.start(self.goal))

        handle = client.client.handle
        client.client.accept_future.resolve(handle)

        client.deadline = time.monotonic() - 1.0
        client.tick()

        self.assertEqual(client.phase, "CANCELLING")
        self.assertEqual(handle.cancel_count, 1)


    def test_cancel_accepted_waits_for_terminal_result(self):
        events = []
        client = self.make_client(
            enabled=True,
            safety_check=lambda: True,
            on_finished=lambda *args: events.append(args),
        )

        self.assertTrue(client.start(self.goal))
        handle = client.client.handle
        client.client.accept_future.resolve(handle)

        self.assertTrue(client.cancel("Safety stop"))
        self.assertEqual(handle.cancel_count, 1)

        handle.cancel_future.resolve(
            SimpleNamespace(goals_canceling=[object()])
        )

        self.assertEqual(client.cancel_ack, "ACCEPTED")
        self.assertTrue(client.busy)
        self.assertEqual(events, [])

        handle.result_future.resolve(
            SimpleNamespace(
                status=GoalStatus.STATUS_CANCELED
            )
        )

        self.assertFalse(client.busy)
        self.assertEqual(events[0][1], "CANCELED")

    def test_cancel_rejected_blocks_new_goal_until_result(self):
        events = []
        client = self.make_client(
            enabled=True,
            safety_check=lambda: True,
            on_finished=lambda *args: events.append(args),
        )

        self.assertTrue(client.start(self.goal))
        handle = client.client.handle
        client.client.accept_future.resolve(handle)

        self.assertTrue(client.cancel("Safety stop"))

        handle.cancel_future.resolve(
            SimpleNamespace(goals_canceling=[])
        )

        self.assertEqual(client.cancel_ack, "REJECTED")
        self.assertTrue(client.busy)
        self.assertFalse(client.start(self.goal))

        handle.result_future.resolve(
            SimpleNamespace(
                status=GoalStatus.STATUS_SUCCEEDED
            )
        )

        self.assertFalse(client.busy)
        self.assertEqual(
            events[0][1],
            "UNSAFE_COMPLETION",
        )

    def test_cancel_pending_blocks_new_goal(self):
        client = self.make_client(
            enabled=True,
            safety_check=lambda: True,
        )

        self.assertTrue(client.start(self.goal))
        handle = client.client.handle
        client.client.accept_future.resolve(handle)

        self.assertTrue(client.cancel("Safety stop"))
        self.assertEqual(client.cancel_ack, "PENDING")
        self.assertTrue(client.busy)

        self.assertFalse(client.start(self.goal))
        self.assertEqual(len(client.client.sent), 1)


    def test_safety_cancel_records_diagnostic_reason(self):
        state = {"safe": True}

        client = self.make_client(
            enabled=True,
            safety_check=lambda: state["safe"],
            safety_reason=lambda: "PERSON_LIMIT_STALE",
        )

        self.assertTrue(client.start(self.goal))

        handle = client.client.handle
        client.client.accept_future.resolve(handle)

        state["safe"] = False
        client.tick()

        self.assertEqual(client.phase, "CANCELLING")
        self.assertIn(
            "PERSON_LIMIT_STALE",
            client.cancel_reason,
        )

if __name__ == "__main__":
    unittest.main(verbosity=2)
