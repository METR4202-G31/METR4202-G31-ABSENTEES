"""Seeded feature placement and deterministic spawn selection."""

import random

from .grid import FREE, Grid


class Placer:
    """Place feature kernels using a private deterministic random generator."""

    def __init__(self, grid, seed=None, max_attempts=100, min_feature_gap=0):
        self.grid = grid
        self.random = random.Random(seed)
        self.max_attempts = max_attempts
        self.min_feature_gap = min_feature_gap
        self.placed = []
        self.failed = []
        self._placed_bounds = []

    @staticmethod
    def _bounds(kernel, row, col, rotation):
        rotated = Grid.rotate_kernel(kernel, rotation)
        rows = [row + offset_row for offset_row, _, _ in rotated]
        cols = [col + offset_col for _, offset_col, _ in rotated]
        return min(rows), max(rows), min(cols), max(cols)

    def _overlaps_placed_feature(self, bounds):
        min_row, max_row, min_col, max_col = bounds
        gap = self.min_feature_gap
        for (other_min_row, other_max_row,
             other_min_col, other_max_col) in self._placed_bounds:
            separated = (
                max_row + 1 + gap <= other_min_row or
                other_max_row + 1 + gap <= min_row or
                max_col + 1 + gap <= other_min_col or
                other_max_col + 1 + gap <= min_col
            )
            if not separated:
                return True
        return False

    def place_feature(self, name, kernel_factory, *args, **kwargs):
        """Try random positions and rotations for one feature."""
        kernel = kernel_factory(*args, **kwargs)
        for _ in range(self.max_attempts):
            row = self.random.randrange(self.grid.rows)
            col = self.random.randrange(self.grid.cols)
            rotation = self.random.choice((0, 90, 180, 270))
            bounds = self._bounds(kernel, row, col, rotation)
            if self._overlaps_placed_feature(bounds):
                continue
            if self.grid.place_kernel(kernel, row, col, rotation):
                self.placed.append((name, row, col, rotation))
                self._placed_bounds.append(bounds)
                return True
        self.failed.append(name)
        return False

    def place_features(self, features):
        """Place feature specifications in randomized order."""
        specifications = list(features)
        self.random.shuffle(specifications)
        for name, factory, args, kwargs in specifications:
            self.place_feature(name, factory, *args, **kwargs)
        return self.placed

    def connect_free_space(self):
        """Carve deterministic Manhattan paths between free-space components."""
        components = self.grid.free_components()
        if len(components) < 2:
            return 0
        connected = set(components[0])
        repairs = 0
        for component in components[1:]:
            source = min(
                ((left, right) for left in connected for right in component),
                key=lambda pair: (
                    abs(pair[0][0] - pair[1][0]) +
                    abs(pair[0][1] - pair[1][1]),
                    pair,
                ),
            )
            row, col = source[0]
            target_row, target_col = source[1]
            while row != target_row:
                row += 1 if target_row > row else -1
                if self.grid.is_boundary(row, col):
                    raise ValueError("connectivity repair reached grid boundary")
                self.grid.set_cell(row, col, FREE)
                connected.add((row, col))
            while col != target_col:
                col += 1 if target_col > col else -1
                if self.grid.is_boundary(row, col):
                    raise ValueError("connectivity repair reached grid boundary")
                self.grid.set_cell(row, col, FREE)
                connected.add((row, col))
            connected.update(component)
            repairs += 1
        return repairs


def choose_spawn_cell(grid):
    """Choose the free cell nearest the geometric grid center."""
    free_cells = grid.get_free_cells()
    if not free_cells:
        raise ValueError("cannot choose a spawn cell from a grid without free cells")

    center_row = (grid.rows - 1) / 2
    center_col = (grid.cols - 1) / 2
    return min(
        free_cells,
        key=lambda cell: (
            (cell[0] - center_row) ** 2 + (cell[1] - center_col) ** 2,
            cell[0],
            cell[1],
        )
    )
