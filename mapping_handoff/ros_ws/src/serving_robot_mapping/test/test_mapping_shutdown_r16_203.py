"""R16-203 shutdown behavior without starting a ROS graph."""

from serving_robot_mapping.exploration_manager import (
    _r16_203_settle_navigation,
)


class FakeClient:
    def __init__(self, busy=True):
        self.busy = busy
        self.cancel_calls = 0

    def cancel(self, _reason):
        self.cancel_calls += 1
        return True


class FakeNode:
    def __init__(self, busy=True):
        self.navigation_client = FakeClient(busy)
        self.stop_calls = 0

    def stop_robot(self):
        self.stop_calls += 1


def test_r16_203_already_idle():
    node = FakeNode(busy=False)

    result = _r16_203_settle_navigation(
        node,
        lambda *_args, **_kwargs: None,
        lambda: True,
        lambda: 0.0,
    )

    assert result == "NO_ACTIVE_MAPPING_GOAL"
    assert node.navigation_client.cancel_calls == 0


def test_r16_203_context_invalid_fails_closed():
    node = FakeNode()

    result = _r16_203_settle_navigation(
        node,
        lambda *_args, **_kwargs: None,
        lambda: False,
        lambda: 0.0,
    )

    assert result == "CONTEXT_INVALID_CANCEL_UNCONFIRMED"
    assert node.navigation_client.cancel_calls == 0


def test_r16_203_goal_becomes_idle():
    node = FakeNode()

    def spin_once(_node, timeout_sec):
        assert timeout_sec == 0.1
        node.navigation_client.busy = False

    result = _r16_203_settle_navigation(
        node,
        spin_once,
        lambda: True,
        lambda: 0.0,
    )

    assert result == (
        "CLIENT_IDLE_TERMINAL_STATUS_NOT_INDEPENDENTLY_VERIFIED"
    )
    assert node.navigation_client.cancel_calls == 1
    assert node.stop_calls >= 1


def test_r16_203_unconfirmed_cancel_fails_closed():
    node = FakeNode()
    times = iter([0.0, 0.0, 7.0])

    result = _r16_203_settle_navigation(
        node,
        lambda *_args, **_kwargs: None,
        lambda: True,
        lambda: next(times),
    )

    assert result == "CANCEL_UNCONFIRMED"
    assert node.navigation_client.busy is True
    assert node.navigation_client.cancel_calls == 1
    assert node.stop_calls >= 1
