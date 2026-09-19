"""
Goal blacklist used by the frontier explorer.

The blacklist is a set of circles in the map frame. It merges two sources:
  - own entries: goals the explorer rejected itself (no path, no safe cell)
    or that Nav2 failed to reach;
  - external entries: the navigation supervisor's /failed_regions record,
    replaced wholesale each time a new snapshot arrives.

The supervisor only records what failed; this class decides what is
blocked, so the policy (for example min_failure_count) lives with the explorer.
"""

from dataclasses import dataclass
import math
from typing import Dict, Iterable, List, Optional


@dataclass
class BlacklistEntry:
    """A blocked circle in the map frame."""

    x: float
    y: float
    radius: float
    reason: str = ""
    count: int = 1


class GoalBlacklist:
    """Circles the explorer must not send goals into."""

    def __init__(self, default_radius: float = 0.5, min_failure_count: int = 1):
        if not math.isfinite(default_radius) or default_radius <= 0.0:
            raise ValueError("default_radius must be finite and positive.")
        self.default_radius = default_radius
        self.min_failure_count = min_failure_count
        self._own: List[BlacklistEntry] = []
        self._external: List[BlacklistEntry] = []
        # region_id -> failure_count already retried, see forget_external().
        self._forgotten: Dict[int, int] = {}
        self._latest_regions: list = []

    @staticmethod
    def _contains(entry: BlacklistEntry, x: float, y: float) -> bool:
        return math.hypot(x - entry.x, y - entry.y) <= entry.radius

    def add(self, x: float, y: float, reason: str = "", radius: Optional[float] = None) -> None:
        """Block a goal; repeated failures inside one circle only bump its count."""
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError("Blacklist coordinates must be finite.")
        for entry in self._own:
            if self._contains(entry, x, y):
                entry.count += 1
                entry.reason = reason or entry.reason
                return
        self._own.append(
            BlacklistEntry(x, y, radius or self.default_radius, reason)
        )

    def update_from_failed_regions(self, msg) -> None:
        """Replace the external entries from a FailedRegionArray-like message."""
        entries = []
        for region in msg.regions:
            if region.failure_count < self.min_failure_count:
                continue
            if region.failure_count <= self._forgotten.get(region.region_id, 0):
                continue
            entries.append(
                BlacklistEntry(
                    region.center.x,
                    region.center.y,
                    region.radius,
                    "supervisor:%d" % region.last_failure_reason,
                    region.failure_count,
                )
            )
        self._external = entries
        self._latest_regions = list(msg.regions)

    def forget_external(self) -> None:
        """
        Ignore the supervisor regions seen so far until they fail again.

        Used for a retry pass: the supervisor keeps its record, but goals in
        those regions may become reachable as the map grows.
        """
        self._forgotten = {
            r.region_id: r.failure_count for r in self._latest_regions
        }
        self._external = []

    def is_blocked(self, x: float, y: float) -> bool:
        """Return True if (x, y) lies inside any blocked circle."""
        return any(
            self._contains(e, x, y) for e in self._own + self._external
        )

    def clear(self) -> None:
        """Drop the explorer's own entries."""
        self._own.clear()

    def entries(self) -> Iterable[BlacklistEntry]:
        """Return all current entries, own first."""
        return tuple(self._own + self._external)

    def __len__(self) -> int:
        return len(self._own) + len(self._external)
