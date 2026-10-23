"""Legacy kernels and continuous vector feature templates."""

from dataclasses import dataclass

from .grid import DOORWAY, FREE, WALL
from .geometry import Segment


@dataclass(frozen=True)
class VectorFeature:
    """A named obstacle described by local wall segments in metres."""

    segments: tuple
    width: float
    height: float


def _enclosed_kernel(width, height, free_cells, doorway_cells=()):
    doorway_cells = set(doorway_cells)
    free_cells = set(free_cells)
    return [
        (
            row,
            col,
            DOORWAY if (row, col) in doorway_cells else
            FREE if (row, col) in free_cells else WALL,
        )
        for row in range(height)
        for col in range(width)
    ]


def doorway_kernel():
    """Return a wall with a two-cell-wide semantic opening in the middle."""
    return [
        (0, 0, WALL), (0, 1, WALL),
        (1, 0, DOORWAY), (1, 1, DOORWAY),
        (2, 0, WALL), (2, 1, WALL),
    ]


def l_corridor_kernel():
    """Return an enclosed L-shaped path with two-cell-wide openings."""
    free_cells = (
        [(row, col) for row in range(1, 7) for col in (7, 8)] +
        [(row, col) for row in (5, 6) for col in range(1, 9)]
    )
    return _enclosed_kernel(
        12, 8, free_cells, doorway_cells=((0, 7), (0, 8), (5, 0), (6, 0)))


def dead_end_kernel():
    """Return a two-cell-wide dead-end corridor with one-cell-thick walls."""
    free_cells = [(row, col) for row in range(1, 7) for col in (1, 2)]
    return _enclosed_kernel(4, 8, free_cells, doorway_cells=((0, 1), (0, 2)))


def narrow_passage_kernel():
    """Return a two-cell-wide passage with one-cell-thick side walls."""
    free_cells = [(row, col) for row in range(1, 7) for col in (1, 2)]
    return _enclosed_kernel(
        4, 8, free_cells,
        doorway_cells=((0, 1), (0, 2), (7, 1), (7, 2)),
    )


def wall_section_kernel():
    """Return a wall section with a single-cell opening."""
    return [
        (row, col, DOORWAY if (row, col) == (1, 4) else WALL)
        for row in range(3)
        for col in range(9)
    ]


def large_empty_space_kernel():
    """Return a 15x15 empty room with a three-cell doorway on each edge."""
    return [
        (
            row,
            col,
            DOORWAY if (
                (row == 0 and 6 <= col <= 8) or
                (row == 14 and 6 <= col <= 8) or
                (col == 0 and 6 <= row <= 8) or
                (col == 14 and 6 <= row <= 8)
            ) else
            WALL if row in (0, 14) or col in (0, 14) else FREE,
        )
        for row in range(15)
        for col in range(15)
    ]


def room_kernel(width=6, height=6):
    """Return a rectangular room with a two-cell-wide top doorway."""
    if width < 4 or height < 3:
        raise ValueError("room width must be at least 4 and height at least 3")
    doorway_start = (width - 2) // 2
    kernel = []
    for row in range(height):
        for col in range(width):
            is_wall = row in (0, height - 1) or col in (0, width - 1)
            value = WALL if is_wall else FREE
            if row == 0 and doorway_start <= col < doorway_start + 2:
                value = DOORWAY
            kernel.append((row, col, value))
    return kernel


def plus_kernel():
    """Return a 7x7 free area with a one-cell-thick wall cross."""
    wall_cells = {
        (row, col)
        for row in range(7)
        for col in range(7)
        if row == 3 or col == 3
    }
    return [
        (row, col, WALL if (row, col) in wall_cells else FREE)
        for row in range(7)
        for col in range(7)
    ]


def _rect(width, height, openings=()):
    """Build a rectangle, omitting wall intervals named by side."""
    half_width = width / 2.0
    half_height = height / 2.0
    segments = []
    for side, start, end in (
        ("bottom", -half_width, half_width),
        ("top", -half_width, half_width),
        ("left", -half_height, half_height),
        ("right", -half_height, half_height),
    ):
        intervals = sorted(
            (low, high) for opening_side, low, high in openings
            if opening_side == side
        )
        points = [start]
        for low, high in intervals:
            points.extend((low, high))
        points.append(end)
        for low, high in zip(points[::2], points[1::2]):
            if side in ("bottom", "top"):
                y = -half_height if side == "bottom" else half_height
                segments.append(Segment(low, y, high, y))
            else:
                x = -half_width if side == "left" else half_width
                segments.append(Segment(x, low, x, high))
    return VectorFeature(tuple(segments), width, height)


def doorway_feature():
    """Return a short wall with a central doorway."""
    return _rect(1.2, 0.12, (("bottom", -0.3, 0.3),))


def room_feature(width=3.0, height=3.0):
    """Return a room with a central doorway on its bottom wall."""
    return _rect(width, height, (("bottom", -0.4, 0.4),))


def dead_end_feature():
    """Return a narrow dead-end corridor open at one end."""
    return _rect(1.0, 2.4, (("bottom", -0.4, 0.4),))


def narrow_passage_feature():
    """Return a narrow passage open at both ends."""
    return _rect(
        1.0, 2.4,
        (("bottom", -0.4, 0.4), ("top", -0.4, 0.4)),
    )


def l_corridor_feature():
    """Return an L-shaped corridor with openings at both ends."""
    width = 0.9
    outer = 3.0
    segments = (
        Segment(-width / 2, -outer / 2, width / 2, -outer / 2),
        Segment(width / 2, -outer / 2, width / 2, -width / 2),
        Segment(width / 2, width / 2, outer / 2, width / 2),
        Segment(outer / 2, width / 2, outer / 2, outer / 2),
        Segment(-width / 2, -outer / 2, -width / 2, width / 2),
        Segment(-width / 2, width / 2, outer / 2, width / 2),
    )
    return VectorFeature(segments, outer, outer)


def wall_section_feature():
    """Return a long wall with a central opening."""
    return VectorFeature(
        (Segment(-1.6, 0.0, -0.35, 0.0),
         Segment(0.35, 0.0, 1.6, 0.0)),
        3.2,
        0.1,
    )


def large_empty_space_feature():
    """Return a room with openings on all four sides."""
    half = 2.5
    gap = 0.5
    return VectorFeature(
        (Segment(-half, -half, -gap, -half),
         Segment(gap, -half, half, -half),
         Segment(-half, half, -gap, half),
         Segment(gap, half, half, half),
         Segment(-half, -half, -half, -gap),
         Segment(-half, gap, -half, half),
         Segment(half, -half, half, -gap),
         Segment(half, gap, half, half)),
        5.0,
        5.0,
    )


def plus_feature():
    """Return a thin cross obstacle with a small central gap."""
    return VectorFeature(
        (Segment(-1.8, 0.0, -0.15, 0.0),
         Segment(0.15, 0.0, 1.8, 0.0),
         Segment(0.0, -1.8, 0.0, -0.15),
         Segment(0.0, 0.15, 0.0, 1.8)),
        3.6,
        3.6,
    )


def angled_wall_feature():
    """Return two angled wall sections with a central opening."""
    return VectorFeature(
        (Segment(-2.0, -0.7, -0.25, 0.0),
         Segment(0.25, 0.0, 2.0, 0.7)),
        4.0,
        1.4,
    )


def zigzag_feature():
    """Return a shallow zig-zag obstacle made from angled segments."""
    return VectorFeature(
        (Segment(-2.0, -0.5, -1.0, 0.5),
         Segment(-1.0, 0.5, 0.0, -0.5),
         Segment(0.0, -0.5, 1.0, 0.5),
         Segment(1.0, 0.5, 2.0, -0.5)),
        4.0,
        1.0,
    )


VECTOR_FEATURES = {
    "doorway": doorway_feature,
    "l_corridor": l_corridor_feature,
    "dead_end": dead_end_feature,
    "narrow_passage": narrow_passage_feature,
    "wall_section": wall_section_feature,
    "large_empty_space": large_empty_space_feature,
    "room": room_feature,
    "plus": plus_feature,
    "angled_wall": angled_wall_feature,
    "zigzag": zigzag_feature,
}


FEATURES = {
    "doorway": doorway_kernel,
    "l_corridor": l_corridor_kernel,
    "dead_end": dead_end_kernel,
    "narrow_passage": narrow_passage_kernel,
    "wall_section": wall_section_kernel,
    "large_empty_space": large_empty_space_kernel,
    "room": room_kernel,
    "plus": plus_kernel,
}
