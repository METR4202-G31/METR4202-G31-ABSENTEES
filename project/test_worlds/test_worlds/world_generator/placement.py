"""Seeded feature placement and deterministic spawn selection."""

import random
import math

from .grid import FREE, Grid
from .geometry import point_segment_distance, segment_bounds, transform_segments


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

    def place_vector_features(self, features, map_width, map_height):
        """Place vector features with seeded continuous transforms."""
        half_width = map_width / 2.0
        half_height = map_height / 2.0
        for name, factory, args, kwargs in features:
            feature = factory(*args, **kwargs)
            placed = False
            for _ in range(self.max_attempts):
                scale = self.random.uniform(0.85, 1.15)
                angle = self.random.uniform(-math.pi, math.pi)
                x = self.random.uniform(
                    -half_width + feature.width * scale / 2.0,
                    half_width - feature.width * scale / 2.0,
                )
                y = self.random.uniform(
                    -half_height + feature.height * scale / 2.0,
                    half_height - feature.height * scale / 2.0,
                )
                segments = transform_segments(
                    feature.segments, scale, angle, (x, y))
                min_x, max_x, min_y, max_y = segment_bounds(segments)
                gap = self.min_feature_gap
                bounds = (min_x - gap, max_x + gap, min_y - gap, max_y + gap)
                if any(
                    not (
                        bounds[1] < other[0] or other[1] < bounds[0] or
                        bounds[3] < other[2] or other[3] < bounds[2]
                    )
                    for other in self._placed_bounds
                ):
                    continue
                self._placed_bounds.append(bounds)
                self.placed.append((name, segments))
                placed = True
                break
            if not placed:
                self.failed.append(name)
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


def choose_spawn_cell(grid, wall_segments=(), wall_thickness=0.0,
                      robot_radius=0.4):
    """Choose the nearest free cell with a clear robot-sized spawn area."""
    if wall_thickness < 0 or robot_radius < 0:
        raise ValueError("wall thickness and robot radius cannot be negative")
    free_cells = grid.get_free_cells()
    if not free_cells:
        raise ValueError("cannot choose a spawn cell from a grid without free cells")

    clearance = wall_thickness / 2.0 + robot_radius
    if wall_segments:
        free_cells = [
            cell for cell in free_cells
            if all(
                point_segment_distance(
                    *grid.cell_to_world(*cell)[:2], segment) >= clearance
                for segment in wall_segments
            )
        ]
    if not free_cells:
        raise ValueError(
            "cannot choose a spawn cell with the requested wall clearance")

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
