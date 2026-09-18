"""Unit tests for the Navigation Supervisor."""

import rclpy
import pytest

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import TaskResult

from navigation_supervisor.navigation_supervisor import NavigationSupervisor
from navigation_supervisor.navigation_types import (
    NavigationOutcome,
    NavigationState,
)


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    """Initialise ROS for Navigation Supervisor tests."""
    if not rclpy.ok():
        rclpy.init()

    yield

    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def supervisor():
    """Create a fresh Navigation Supervisor for each test."""
    node = NavigationSupervisor()

    yield node

    node.destroy_node()


def make_goal(
    x: float = 1.0,
    y: float = 2.0,
    frame_id: str = "map",
) -> PoseStamped:
    """Create a valid map-frame navigation goal."""
    goal = PoseStamped()

    goal.header.frame_id = frame_id
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.orientation.w = 1.0

    return goal


def test_valid_goal_is_accepted(supervisor):
    """Accept a structurally valid map-frame goal."""
    accepted = supervisor.submit_goal(make_goal())

    assert accepted is True
    assert supervisor.state == NavigationState.IDLE
    assert supervisor.last_outcome is None


def test_wrong_frame_is_rejected(supervisor):
    """Reject a goal expressed in the wrong coordinate frame."""
    accepted = supervisor.submit_goal(make_goal(frame_id="odom"))

    assert accepted is False
    assert supervisor.state == NavigationState.IDLE
    assert supervisor.last_outcome == NavigationOutcome.INVALID_GOAL


def test_zero_quaternion_is_rejected(supervisor):
    """Reject a goal containing a zero quaternion."""
    goal = make_goal()
    goal.pose.orientation.w = 0.0

    accepted = supervisor.submit_goal(goal)

    assert accepted is False
    assert supervisor.last_outcome == NavigationOutcome.INVALID_GOAL


def test_pending_goal_starts_navigation(
    supervisor,
    monkeypatch,
):
    """Send an accepted pending goal to Nav2."""
    monkeypatch.setattr(
        supervisor,
        "goToPose",
        lambda goal: True,
    )

    assert supervisor.submit_goal(make_goal()) is True

    outcome = supervisor.update()

    assert outcome is None
    assert supervisor.state == NavigationState.NAVIGATING


def test_successful_navigation_returns_success(
    supervisor,
    monkeypatch,
):
    """Return SUCCEEDED when Nav2 completes successfully."""
    monkeypatch.setattr(
        supervisor,
        "goToPose",
        lambda goal: True,
    )

    supervisor.submit_goal(make_goal())
    supervisor.update()

    monkeypatch.setattr(
        supervisor,
        "isTaskComplete",
        lambda: True,
    )
    monkeypatch.setattr(
        supervisor,
        "getResult",
        lambda: TaskResult.SUCCEEDED,
    )

    outcome = supervisor.update()

    assert outcome == NavigationOutcome.SUCCEEDED
    assert supervisor.state == NavigationState.IDLE
    assert supervisor.last_outcome == NavigationOutcome.SUCCEEDED


def test_failed_navigation_returns_failure(
    supervisor,
    monkeypatch,
):
    """Return NAVIGATION_FAILED when Nav2 reports failure."""
    monkeypatch.setattr(
        supervisor,
        "goToPose",
        lambda goal: True,
    )

    supervisor.submit_goal(make_goal())
    supervisor.update()

    monkeypatch.setattr(
        supervisor,
        "isTaskComplete",
        lambda: True,
    )
    monkeypatch.setattr(
        supervisor,
        "getResult",
        lambda: TaskResult.FAILED,
    )

    outcome = supervisor.update()

    assert outcome == NavigationOutcome.NAVIGATION_FAILED
    assert supervisor.state == NavigationState.IDLE


def test_navigation_can_be_canceled(
    supervisor,
    monkeypatch,
):
    """Cancel an active Nav2 navigation task."""
    monkeypatch.setattr(
        supervisor,
        "goToPose",
        lambda goal: True,
    )
    monkeypatch.setattr(
        supervisor,
        "cancelTask",
        lambda: None,
    )

    supervisor.submit_goal(make_goal())
    supervisor.update()

    assert supervisor.cancel_navigation() is True
    assert supervisor.state == NavigationState.CANCELING

    monkeypatch.setattr(
        supervisor,
        "isTaskComplete",
        lambda: True,
    )
    monkeypatch.setattr(
        supervisor,
        "getResult",
        lambda: TaskResult.CANCELED,
    )

    outcome = supervisor.update()

    assert outcome == NavigationOutcome.CANCELED
    assert supervisor.state == NavigationState.IDLE
