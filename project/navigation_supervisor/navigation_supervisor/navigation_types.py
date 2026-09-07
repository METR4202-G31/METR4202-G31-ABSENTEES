"""Shared internal types for the navigation supervisor."""

from dataclasses import dataclass
from enum import Enum, auto
from typing import Any, Optional


class NavigationState(Enum):
    """Represent the current activity of the navigation manager."""

    IDLE = auto()
    VALIDATING = auto()
    PLANNING = auto()
    NAVIGATING = auto()
    CANCELING = auto()


class NavigationOutcome(Enum):
    """Represent the final outcome of one navigation attempt."""

    SUCCEEDED = auto()
    BLACKLISTED = auto()
    PLANNING_FAILED = auto()
    NAVIGATION_FAILED = auto()
    STUCK = auto()
    CANCELED = auto()
    INVALID_GOAL = auto()
    INTERNAL_ERROR = auto()


class FailureReason(Enum):
    """Represent the detailed internal reason for a navigation failure."""

    NONE = auto()

    INVALID_POSE = auto()
    WRONG_FRAME = auto()
    BLACKLISTED_REGION = auto()

    PLANNER_SERVER_UNAVAILABLE = auto()
    PLANNER_GOAL_REJECTED = auto()
    PLANNER_ABORTED = auto()
    EMPTY_PATH = auto()
    PLANNER_TIMEOUT = auto()

    NAV_SERVER_UNAVAILABLE = auto()
    NAV_GOAL_REJECTED = auto()
    NAV_ABORTED = auto()
    NAV_TIMEOUT = auto()

    NO_PROGRESS = auto()

    EXTERNAL_CANCEL = auto()
    INTERNAL_EXCEPTION = auto()


class ProgressStatus(Enum):
    """Represent the result of evaluating navigation progress."""

    PROGRESSING = auto()
    STUCK = auto()
    NAVIGATION_TIMEOUT = auto()


class ActiveRequest(Enum):
    """Represent the type of Nav2 action currently owned by Nav2Client."""

    NONE = auto()
    PLANNING = auto()
    NAVIGATION = auto()


class Nav2EventType(Enum):
    """Represent events emitted by Nav2Client."""

    PLANNER_GOAL_ACCEPTED = auto()
    PLANNER_GOAL_REJECTED = auto()
    PLANNER_SUCCEEDED = auto()
    PLANNER_ABORTED = auto()
    PLANNER_CANCELED = auto()

    NAV_GOAL_ACCEPTED = auto()
    NAV_GOAL_REJECTED = auto()
    NAV_FEEDBACK = auto()
    NAV_SUCCEEDED = auto()
    NAV_ABORTED = auto()
    NAV_CANCELED = auto()

    SERVER_UNAVAILABLE = auto()

    GOAL_RESPONSE_TIMEOUT = auto()
    PLANNER_RESULT_TIMEOUT = auto()

    CANCEL_ACCEPTED = auto()
    CANCEL_REJECTED = auto()
    CANCEL_RESPONSE_TIMEOUT = auto()

    CLIENT_ERROR = auto()


@dataclass
class FailedRegionRecord:
    """Store the internal representation of a failed navigation region."""

    region_id: int

    center_x: float
    center_y: float
    radius: float

    failure_count: int

    first_failure_time: float
    last_failure_time: float

    last_failure_reason: FailureReason


@dataclass
class Nav2Event:
    """Represent an event passed from Nav2Client to NavigationManager."""

    event_type: Nav2EventType
    attempt_id: int

    request_type: ActiveRequest = ActiveRequest.NONE

    path: Optional[Any] = None
    feedback: Optional[Any] = None

    message: str = ""


@dataclass
class NavigationAttempt:
    """Store state belonging to one managed navigation request."""

    attempt_id: int
    target_pose: Any
    managed_goal_handle: Any
    start_time: float

    termination_outcome: Optional[NavigationOutcome] = None
    failure_reason: FailureReason = FailureReason.NONE

    failed_region_recorded: bool = False

    latest_distance_remaining: Optional[float] = None
    latest_navigation_time: Optional[float] = None
    latest_recovery_count: int = 0
