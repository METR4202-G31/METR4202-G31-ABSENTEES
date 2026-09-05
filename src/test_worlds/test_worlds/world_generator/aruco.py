"""Deterministic ARUCO marker cell selection."""

import random


MARKER_MODEL_IDS = (0, 2, 42)


def place_markers(grid, seed=None, reachable_cells=None):
    """Return cells paired with the three required ARUCO model IDs."""
    free_cells = list(reachable_cells or grid.get_free_cells())
    if len(free_cells) < len(MARKER_MODEL_IDS):
        raise ValueError("world does not have three free cells for markers")
    generator = random.Random(seed)
    selected = generator.sample(free_cells, len(MARKER_MODEL_IDS))
    return [
        (row, col, marker_id) for marker_id, (row, col) in zip(
            MARKER_MODEL_IDS, selected)
    ]
