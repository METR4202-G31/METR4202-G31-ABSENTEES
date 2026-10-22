"""
Pure-Python helper for the scan filler (no ROS imports, so it can be unit tested).

Background
----------
A LiDAR beam that hits nothing within the sensor's range comes back as +inf
(in Gazebo) or as a value at/above range_max. slam_toolbox (Karto) throws
those readings away completely, so the space the beam travelled through is
never marked as free. In corridors this does not matter because every beam
hits a wall. In a big open room most beams hit nothing, so the middle of the
room stays "unknown" and the frontier explorer keeps trying to visit it.

The fix
-------
Replace each "no hit" reading with a value just under range_max. Combined with
slam_toolbox's `max_laser_range` set LOWER than that value, Karto then:
  * clears free space along the beam up to max_laser_range, and
  * does NOT mark an obstacle at the end of the beam (because the reading is
    beyond max_laser_range it is treated as "nothing hit here").
So the fake value never creates a fake wall.
"""

import math
from typing import List, Sequence, Tuple


def fill_no_return(
    ranges: Sequence[float],
    range_max: float,
    margin: float = 0.01,
) -> Tuple[List[float], int]:
    """
    Return (new_ranges, number_of_beams_filled).

    ranges    : the LaserScan.ranges list, in metres
    range_max : LaserScan.range_max (3.5 m for the TurtleBot3 LiDAR)
    margin    : how far below range_max the fill value sits

    Rules for each beam:
      +inf or >= range_max -> "no hit": replaced with range_max - margin
      NaN                  -> invalid reading: left alone (SLAM ignores it)
      -inf or anything else-> a real reading (or "too close"): left alone
    """
    fill_value = range_max - margin
    filled = []
    count = 0
    for r in ranges:
        if math.isnan(r):
            filled.append(r)
        elif r >= range_max:          # also true for +inf
            filled.append(fill_value)
            count += 1
        else:                         # real hit, or -inf ("too close")
            filled.append(r)
    return filled, count