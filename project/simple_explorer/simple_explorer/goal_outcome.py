"""
Policy for what the explorer does with each ManagedNavigate result code.

Kept separate from the node so it can be unit tested without a ROS graph.
"""

from dataclasses import dataclass

from navigation_interfaces.action import ManagedNavigate

Result = ManagedNavigate.Result


@dataclass(frozen=True)
class GoalDecision:
    """What to do after a goal finished."""

    mark_visited: bool
    blacklist: bool
    retry: bool = False


_FAILURES = (
    Result.BLACKLISTED,
    Result.PLANNING_FAILED,
    Result.NAVIGATION_FAILED,
    Result.STUCK,
    Result.INVALID_GOAL,
)


def decide(result_code: int, cancelled_by_explorer: bool = False) -> GoalDecision:
    """Map a ManagedNavigate result code to an action for the explorer."""
    if result_code == Result.SUCCEEDED:
        return GoalDecision(mark_visited=True, blacklist=False)
    if result_code in _FAILURES:
        return GoalDecision(mark_visited=True, blacklist=True)
    if result_code == Result.CANCELED:
        # Our own cancel means the frontier went away or we were stopped:
        # nothing to remember. Anyone else cancelling it is not a failure.
        return GoalDecision(
            mark_visited=not cancelled_by_explorer, blacklist=False
        )
    # INTERNAL_ERROR or an unknown code: the goal itself may be fine.
    return GoalDecision(mark_visited=False, blacklist=False, retry=True)
