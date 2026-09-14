"""Unit tests for the failed-region manager."""

import math

import pytest

from navigation_supervisor.failed_region_manager import FailedRegionManager
from navigation_supervisor.navigation_types import FailureReason


def test_invalid_radius_is_rejected():
    """Reject non-positive and non-finite region radii."""
    with pytest.raises(ValueError):
        FailedRegionManager(0.0)

    with pytest.raises(ValueError):
        FailedRegionManager(-0.1)

    with pytest.raises(ValueError):
        FailedRegionManager(math.inf)

    with pytest.raises(ValueError):
        FailedRegionManager(math.nan)


def test_empty_manager_does_not_block_goal():
    """Allow goals when no failed regions exist."""
    manager = FailedRegionManager(radius=0.30)

    assert not manager.is_goal_blocked(1.0, 2.0)


def test_first_failure_creates_region():
    """Create a region for the first recorded failure."""
    manager = FailedRegionManager(radius=0.30)

    region = manager.record_failure(
        x=1.0,
        y=2.0,
        failure_reason=FailureReason.NO_PROGRESS,
        timestamp=10.0,
    )

    assert region.region_id == 1
    assert region.center_x == pytest.approx(1.0)
    assert region.center_y == pytest.approx(2.0)
    assert region.radius == pytest.approx(0.30)
    assert region.failure_count == 1
    assert region.first_failure_time == pytest.approx(10.0)
    assert region.last_failure_time == pytest.approx(10.0)
    assert region.last_failure_reason == FailureReason.NO_PROGRESS


def test_goal_inside_region_is_blocked():
    """Block a candidate goal inside an existing failed region."""
    manager = FailedRegionManager(radius=0.30)

    manager.record_failure(
        x=1.0,
        y=2.0,
        failure_reason=FailureReason.NO_PROGRESS,
        timestamp=10.0,
    )

    assert manager.is_goal_blocked(1.10, 2.05)


def test_goal_outside_region_is_not_blocked():
    """Allow a candidate goal outside all failed regions."""
    manager = FailedRegionManager(radius=0.30)

    manager.record_failure(
        x=1.0,
        y=2.0,
        failure_reason=FailureReason.NO_PROGRESS,
        timestamp=10.0,
    )

    assert not manager.is_goal_blocked(1.50, 2.0)


def test_failure_inside_region_updates_existing_region():
    """Update rather than duplicate a nearby failed region."""
    manager = FailedRegionManager(radius=0.30)

    first = manager.record_failure(
        x=1.0,
        y=2.0,
        failure_reason=FailureReason.PLANNER_ABORTED,
        timestamp=10.0,
    )

    second = manager.record_failure(
        x=1.10,
        y=2.0,
        failure_reason=FailureReason.NO_PROGRESS,
        timestamp=20.0,
    )

    assert first is second
    assert second.failure_count == 2
    assert second.first_failure_time == pytest.approx(10.0)
    assert second.last_failure_time == pytest.approx(20.0)
    assert second.last_failure_reason == FailureReason.NO_PROGRESS
    assert len(manager.get_regions()) == 1


def test_failure_outside_regions_creates_new_region():
    """Create a second region for a sufficiently distant failure."""
    manager = FailedRegionManager(radius=0.30)

    first = manager.record_failure(
        x=1.0,
        y=2.0,
        failure_reason=FailureReason.NO_PROGRESS,
        timestamp=10.0,
    )

    second = manager.record_failure(
        x=2.0,
        y=2.0,
        failure_reason=FailureReason.NAV_ABORTED,
        timestamp=20.0,
    )

    assert first.region_id == 1
    assert second.region_id == 2
    assert len(manager.get_regions()) == 2


def test_clear_removes_regions_and_resets_ids():
    """Clear stored regions and restart region numbering."""
    manager = FailedRegionManager(radius=0.30)

    manager.record_failure(
        x=1.0,
        y=2.0,
        failure_reason=FailureReason.NO_PROGRESS,
        timestamp=10.0,
    )

    manager.clear()

    assert manager.get_regions() == ()
    assert not manager.is_goal_blocked(1.0, 2.0)

    new_region = manager.record_failure(
        x=5.0,
        y=5.0,
        failure_reason=FailureReason.NO_PROGRESS,
        timestamp=20.0,
    )

    assert new_region.region_id == 1


def test_non_finite_candidate_position_is_rejected():
    """Reject invalid candidate coordinates."""
    manager = FailedRegionManager(radius=0.30)

    with pytest.raises(ValueError):
        manager.is_goal_blocked(math.nan, 0.0)

    with pytest.raises(ValueError):
        manager.is_goal_blocked(0.0, math.inf)


def test_non_finite_failure_timestamp_is_rejected():
    """Reject invalid failure timestamps."""
    manager = FailedRegionManager(radius=0.30)

    with pytest.raises(ValueError):
        manager.record_failure(
            x=1.0,
            y=2.0,
            failure_reason=FailureReason.NO_PROGRESS,
            timestamp=math.nan,
        )
