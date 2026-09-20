"""Unit tests for the Navigation Supervisor."""

import asyncio

import pytest
import rclpy

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import TaskResult
from navigation_interfaces.action import ManagedNavigate
from rclpy.action import CancelResponse, GoalResponse

from navigation_supervisor.navigation_supervisor import NavigationSupervisor
from navigation_supervisor.navigation_types import (
    FailureReason,
    NavigationOutcome,
    NavigationState,
)


class FakeManagedGoalHandle:
    """Provide the minimal goal-handle behaviour needed by action tests."""

    def __init__(
        self,
        target_pose: PoseStamped,
        cancel_requested: bool = False,
    ) -> None:
        """Create a fake ManagedNavigate goal handle."""
        self.request = ManagedNavigate.Goal()
        self.request.target_pose = target_pose

        self.is_cancel_requested = cancel_requested
        self.terminal_state = None

    def succeed(self) -> None:
        """Record successful action completion."""
        self.terminal_state = "succeeded"

    def abort(self) -> None:
        """Record aborted action completion."""
        self.terminal_state = "aborted"

    def canceled(self) -> None:
        """Record canceled action completion."""
        self.terminal_state = "canceled"


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


def test_failed_navigation_records_failed_region(
    supervisor,
    monkeypatch,
):
    """Record the failed goal location after Nav2 reports failure."""
    monkeypatch.setattr(
        supervisor,
        "goToPose",
        lambda goal: True,
    )

    supervisor.submit_goal(make_goal(x=1.5, y=-0.5))
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

    regions = supervisor._failed_region_manager.get_regions()

    assert len(regions) == 1
    assert regions[0].center_x == pytest.approx(1.5)
    assert regions[0].center_y == pytest.approx(-0.5)
    assert regions[0].failure_count == 1


def test_successful_navigation_does_not_record_failed_region(
    supervisor,
    monkeypatch,
):
    """Do not create a failed region after successful navigation."""
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
    assert len(supervisor._failed_region_manager.get_regions()) == 0


def test_goal_inside_failed_region_is_still_accepted(
    supervisor,
):
    """Allow navigation to a location with previous failure history."""
    supervisor._failed_region_manager.record_failure(
        x=1.0,
        y=2.0,
        failure_reason=FailureReason.NAV_ABORTED,
        timestamp=1.0,
    )

    accepted = supervisor.submit_goal(make_goal(x=1.1, y=2.0))

    assert accepted is True
    assert supervisor.last_outcome is None
    assert supervisor.state == NavigationState.IDLE


def test_managed_goal_is_accepted_when_available(
    supervisor,
):
    """Accept a ManagedNavigate request while the supervisor is available."""
    request = ManagedNavigate.Goal()
    request.target_pose = make_goal()

    response = supervisor._managed_goal_callback(request)

    assert response == GoalResponse.ACCEPT
    assert supervisor._action_goal_reserved is True


def test_second_managed_goal_is_rejected_while_reserved(
    supervisor,
):
    """Reject another ManagedNavigate request while one is reserved."""
    request = ManagedNavigate.Goal()
    request.target_pose = make_goal()

    first_response = supervisor._managed_goal_callback(request)
    second_response = supervisor._managed_goal_callback(request)

    assert first_response == GoalResponse.ACCEPT
    assert second_response == GoalResponse.REJECT


def test_invalid_managed_goal_returns_invalid_goal(
    supervisor,
):
    """Return INVALID_GOAL for an accepted action containing an invalid pose."""
    goal_handle = FakeManagedGoalHandle(make_goal(frame_id="odom"))

    response = supervisor._managed_goal_callback(goal_handle.request)

    assert response == GoalResponse.ACCEPT

    result = asyncio.run(supervisor._execute_managed_navigation(goal_handle))

    assert result.result_code == ManagedNavigate.Result.INVALID_GOAL
    assert result.failed_region_recorded is False
    assert goal_handle.terminal_state == "aborted"
    assert supervisor._action_goal_reserved is False


def test_immediate_managed_cancellation_returns_canceled(
    supervisor,
):
    """Return CANCELED when an accepted action is immediately canceled."""
    goal_handle = FakeManagedGoalHandle(
        make_goal(),
        cancel_requested=True,
    )

    goal_response = supervisor._managed_goal_callback(goal_handle.request)
    cancel_response = supervisor._managed_cancel_callback(goal_handle)

    assert goal_response == GoalResponse.ACCEPT
    assert cancel_response == CancelResponse.ACCEPT

    result = asyncio.run(supervisor._execute_managed_navigation(goal_handle))

    assert result.result_code == ManagedNavigate.Result.CANCELED
    assert result.failed_region_recorded is False
    assert goal_handle.terminal_state == "canceled"
    assert supervisor._action_goal_reserved is False


def test_managed_nav2_failure_records_failed_region(
    supervisor,
    monkeypatch,
):
    """Return NAVIGATION_FAILED and report a stored failed region."""
    goal_handle = FakeManagedGoalHandle(make_goal(x=1.5, y=-0.5))

    monkeypatch.setattr(
        supervisor,
        "goToPose",
        lambda goal: True,
    )

    assert supervisor.submit_goal(goal_handle.request.target_pose) is True

    assert supervisor.update() is None
    assert supervisor.state == NavigationState.NAVIGATING

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
    assert supervisor._failed_region_recorded is True

    result = supervisor._finish_managed_action(
        goal_handle,
        outcome,
    )

    assert result.result_code == ManagedNavigate.Result.NAVIGATION_FAILED
    assert result.failed_region_recorded is True
    assert goal_handle.terminal_state == "aborted"

    regions = supervisor._failed_region_manager.get_regions()

    assert len(regions) == 1
    assert regions[0].center_x == pytest.approx(1.5)
    assert regions[0].center_y == pytest.approx(-0.5)


@pytest.mark.parametrize(
    ("outcome", "expected_code", "expected_terminal_state"),
    (
        (
            NavigationOutcome.SUCCEEDED,
            ManagedNavigate.Result.SUCCEEDED,
            "succeeded",
        ),
        (
            NavigationOutcome.BLACKLISTED,
            ManagedNavigate.Result.BLACKLISTED,
            "aborted",
        ),
        (
            NavigationOutcome.PLANNING_FAILED,
            ManagedNavigate.Result.PLANNING_FAILED,
            "aborted",
        ),
        (
            NavigationOutcome.NAVIGATION_FAILED,
            ManagedNavigate.Result.NAVIGATION_FAILED,
            "aborted",
        ),
        (
            NavigationOutcome.STUCK,
            ManagedNavigate.Result.STUCK,
            "aborted",
        ),
        (
            NavigationOutcome.CANCELED,
            ManagedNavigate.Result.CANCELED,
            "canceled",
        ),
        (
            NavigationOutcome.INVALID_GOAL,
            ManagedNavigate.Result.INVALID_GOAL,
            "aborted",
        ),
        (
            NavigationOutcome.INTERNAL_ERROR,
            ManagedNavigate.Result.INTERNAL_ERROR,
            "aborted",
        ),
    ),
)
def test_managed_result_code_mapping(
    supervisor,
    outcome,
    expected_code,
    expected_terminal_state,
):
    """Preserve every result code defined by the ManagedNavigate interface."""
    goal_handle = FakeManagedGoalHandle(make_goal())

    result = supervisor._finish_managed_action(
        goal_handle,
        outcome,
    )

    assert result.result_code == expected_code
    assert goal_handle.terminal_state == expected_terminal_state
