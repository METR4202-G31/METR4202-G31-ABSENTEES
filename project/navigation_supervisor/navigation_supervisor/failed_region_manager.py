"""Management of failed navigation regions."""

import math
from typing import Optional, Tuple

from .navigation_types import FailedRegionRecord, FailureReason


class FailedRegionManager:
    """Stores and queries circular failed navigation regions."""

    def __init__(self, radius: float) -> None:

        # Initialise the failed-region manager
        if not math.isfinite(radius) or radius <= 0.0:
            raise ValueError(
                "Failed-region radius must be finite and positive."
            )

        self._radius = radius
        self._regions: list[FailedRegionRecord] = []
        self._next_region_id = 1

    @property
    def radius(self) -> float:
        """Return the configured radius for newly created regions."""
        return self._radius

    def find_blocking_region(
        self,
        x: float,
        y: float,
    ) -> Optional[FailedRegionRecord]:
        """Return the failed region containing a candidate position."""
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError("Candidate coordinates must be fineite.")

        for region in self._regions:

            # Calculate distance to the region center
            dx = x - region.center_x
            dy = y - region.center_y
            distance_squared = dx * dx + dy * dy
            radius_squared = region.radius * region.radius

            # If the distance is within the circle than position is within a
            # failed region
            if distance_squared <= radius_squared:
                return region

        return None

    def is_goal_blocked(self, x: float, y: float) -> bool:
        """Return whether a candidate position lies in a failed region."""
        return self.find_blocking_region(x, y) is not None

    def record_failure(
        self,
        x: float,
        y: float,
        failure_reason: FailureReason,
        timestamp: float,
    ) -> FailedRegionRecord:
        """Create or update a failed region at a failed goal position."""
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError("Failure coordinates must be finite.")

        if not math.isfinite(timestamp):
            raise ValueError("Failure timestamp must be finite.")

        existing_region = self.find_blocking_region(x, y)

        if existing_region is not None:
            existing_region.failure_count += 1
            existing_region.last_failure_reason = failure_reason
            existing_region.last_failure_time = timestamp
            return existing_region

        new_region = FailedRegionRecord(
            region_id=self._next_region_id,
            center_x=x,
            center_y=y,
            radius=self._radius,
            failure_count=1,
            first_failure_time=timestamp,
            last_failure_time=timestamp,
            last_failure_reason=failure_reason,
        )

        self._regions.append(new_region)
        self._next_region_id += 1

        return new_region

    def clear(self) -> None:
        """Clear all failed regions for a new exploration session."""
        self._regions.clear()
        self._next_region_id = 1

    def get_regions(self) -> Tuple[FailedRegionRecord, ...]:
        """Return an immutable snapshot of the current failed regions."""
        return tuple(self._regions)
