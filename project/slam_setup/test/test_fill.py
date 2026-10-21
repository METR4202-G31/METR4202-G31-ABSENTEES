"""Unit tests for fill_no_return. Run: python3 -m pytest test/test_fill.py -v"""

import math

from slam_setup.fill import fill_no_return

RANGE_MAX = 3.5


def test_inf_is_filled_just_under_range_max():
    out, count = fill_no_return([math.inf], RANGE_MAX)
    assert count == 1
    assert out[0] == RANGE_MAX - 0.01


def test_readings_at_or_beyond_range_max_are_filled():
    out, count = fill_no_return([3.5, 7.0], RANGE_MAX)
    assert count == 2
    assert out == [3.49, 3.49]


def test_real_hits_are_unchanged():
    ranges = [0.2, 1.0, 3.2, 3.499]
    out, count = fill_no_return(ranges, RANGE_MAX)
    assert count == 0
    assert out == ranges


def test_nan_and_negative_inf_are_left_alone():
    out, count = fill_no_return([math.nan, -math.inf], RANGE_MAX)
    assert count == 0
    assert math.isnan(out[0])
    assert out[1] == -math.inf


def test_mixed_scan_counts_correctly():
    out, count = fill_no_return([1.0, math.inf, 2.0, math.inf], RANGE_MAX, margin=0.1)
    assert count == 2
    assert out == [1.0, 3.4, 2.0, 3.4]