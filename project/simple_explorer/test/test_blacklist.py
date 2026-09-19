"""Unit tests for blacklist.GoalBlacklist and goal_outcome.decide."""

from types import SimpleNamespace

from navigation_interfaces.action import ManagedNavigate
import pytest

from simple_explorer.blacklist import GoalBlacklist
from simple_explorer.goal_outcome import decide


def region(region_id, x, y, radius=0.5, failure_count=1, reason=0):
    return SimpleNamespace(
        region_id=region_id,
        center=SimpleNamespace(x=x, y=y),
        radius=radius,
        failure_count=failure_count,
        last_failure_reason=reason,
    )


def snapshot(*regions):
    return SimpleNamespace(regions=list(regions))


def test_add_blocks_inside_radius_only():
    blacklist = GoalBlacklist(default_radius=0.5)
    blacklist.add(1.0, 1.0, "no_path")

    assert blacklist.is_blocked(1.0, 1.0)
    assert blacklist.is_blocked(1.4, 1.0)
    assert not blacklist.is_blocked(1.6, 1.0)


def test_repeated_add_in_same_circle_does_not_grow_the_list():
    blacklist = GoalBlacklist(default_radius=0.5)
    blacklist.add(1.0, 1.0, "no_path")
    blacklist.add(1.2, 1.0, "result_3")

    assert len(blacklist) == 1
    assert next(iter(blacklist.entries())).count == 2


def test_add_rejects_non_finite_coordinates():
    with pytest.raises(ValueError):
        GoalBlacklist().add(float("nan"), 0.0)


def test_rejects_bad_radius():
    with pytest.raises(ValueError):
        GoalBlacklist(default_radius=0.0)


def test_supervisor_regions_block_and_are_replaced_by_each_snapshot():
    blacklist = GoalBlacklist()
    blacklist.update_from_failed_regions(snapshot(region(1, 5.0, 5.0)))
    assert blacklist.is_blocked(5.1, 5.0)

    blacklist.update_from_failed_regions(snapshot())
    assert not blacklist.is_blocked(5.1, 5.0)


def test_min_failure_count_filters_supervisor_regions():
    blacklist = GoalBlacklist(min_failure_count=2)
    blacklist.update_from_failed_regions(
        snapshot(region(1, 5.0, 5.0, failure_count=1),
                 region(2, 8.0, 8.0, failure_count=2))
    )

    assert not blacklist.is_blocked(5.0, 5.0)
    assert blacklist.is_blocked(8.0, 8.0)


def test_forget_external_ignores_seen_regions_until_they_fail_again():
    blacklist = GoalBlacklist()
    blacklist.update_from_failed_regions(snapshot(region(1, 5.0, 5.0)))
    blacklist.forget_external()
    assert not blacklist.is_blocked(5.0, 5.0)

    # Same snapshot again: still forgotten.
    blacklist.update_from_failed_regions(snapshot(region(1, 5.0, 5.0)))
    assert not blacklist.is_blocked(5.0, 5.0)

    # Failed once more: blocked again.
    blacklist.update_from_failed_regions(
        snapshot(region(1, 5.0, 5.0, failure_count=2))
    )
    assert blacklist.is_blocked(5.0, 5.0)


def test_clear_drops_own_entries_only():
    blacklist = GoalBlacklist()
    blacklist.add(1.0, 1.0)
    blacklist.update_from_failed_regions(snapshot(region(1, 5.0, 5.0)))

    blacklist.clear()

    assert not blacklist.is_blocked(1.0, 1.0)
    assert blacklist.is_blocked(5.0, 5.0)


Result = ManagedNavigate.Result


def test_success_marks_visited_only():
    decision = decide(Result.SUCCEEDED)
    assert decision.mark_visited and not decision.blacklist and not decision.retry


@pytest.mark.parametrize(
    "code",
    [
        Result.BLACKLISTED,
        Result.PLANNING_FAILED,
        Result.NAVIGATION_FAILED,
        Result.STUCK,
        Result.INVALID_GOAL,
    ],
)
def test_failures_blacklist_and_mark_visited(code):
    decision = decide(code)
    assert decision.mark_visited and decision.blacklist and not decision.retry


def test_own_cancel_is_forgotten_but_external_cancel_marks_visited():
    own = decide(Result.CANCELED, cancelled_by_explorer=True)
    other = decide(Result.CANCELED, cancelled_by_explorer=False)

    assert not own.mark_visited and not own.blacklist
    assert other.mark_visited and not other.blacklist


@pytest.mark.parametrize("code", [Result.INTERNAL_ERROR, 200])
def test_internal_errors_ask_for_a_retry(code):
    decision = decide(code)
    assert decision.retry and not decision.blacklist
