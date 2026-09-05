"""Feature kernels used by the random world placer."""

from .grid import DOORWAY, FREE, WALL


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
