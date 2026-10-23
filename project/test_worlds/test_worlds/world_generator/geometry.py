"""Continuous wall geometry and conservative rasterization helpers."""

from dataclasses import dataclass
import math

from .grid import WALL


@dataclass(frozen=True)
class Segment:
    """A finite wall segment in world metres."""

    x1: float
    y1: float
    x2: float
    y2: float

    @property
    def length(self):
        """Return the segment length in metres."""
        return math.hypot(self.x2 - self.x1, self.y2 - self.y1)

    @property
    def yaw(self):
        """Return the segment heading in radians."""
        return math.atan2(self.y2 - self.y1, self.x2 - self.x1)

    @property
    def center(self):
        """Return the segment midpoint."""
        return ((self.x1 + self.x2) / 2.0, (self.y1 + self.y2) / 2.0)


def transform_segments(segments, scale, angle, center):
    """Scale, rotate, and translate local segments into world coordinates."""
    cos_angle = math.cos(angle)
    sin_angle = math.sin(angle)
    cx, cy = center

    def transform(x, y):
        x *= scale
        y *= scale
        return (
            cx + cos_angle * x - sin_angle * y,
            cy + sin_angle * x + cos_angle * y,
        )

    return [
        Segment(*transform(segment.x1, segment.y1),
                *transform(segment.x2, segment.y2))
        for segment in segments
    ]


def segment_bounds(segments):
    """Return min/max x/y for a non-empty segment collection."""
    if not segments:
        raise ValueError("at least one segment is required")
    xs = [value for segment in segments for value in (segment.x1, segment.x2)]
    ys = [value for segment in segments for value in (segment.y1, segment.y2)]
    return min(xs), max(xs), min(ys), max(ys)


def _point_segment_distance(x, y, segment):
    """Return the distance from a point to a segment."""
    dx = segment.x2 - segment.x1
    dy = segment.y2 - segment.y1
    length_squared = dx * dx + dy * dy
    if length_squared == 0:
        return math.hypot(x - segment.x1, y - segment.y1)
    projection = ((x - segment.x1) * dx + (y - segment.y1) * dy)
    projection = max(0.0, min(1.0, projection / length_squared))
    near_x = segment.x1 + projection * dx
    near_y = segment.y1 + projection * dy
    return math.hypot(x - near_x, y - near_y)


def rasterize_segments(grid, segments, wall_thickness):
    """Rasterize wall footprints into a grid conservatively."""
    half_cell = grid.cell_size * math.sqrt(2.0) / 2.0
    radius = wall_thickness / 2.0 + half_cell
    for row in range(grid.rows):
        for col in range(grid.cols):
            x, y, _ = grid.cell_to_world(row, col)
            if any(_point_segment_distance(x, y, segment) <= radius
                   for segment in segments):
                grid.set_cell(row, col, WALL)


def boundary_segments(width, height):
    """Return four perimeter segments for a centred rectangular map."""
    half_width = width / 2.0
    half_height = height / 2.0
    return [
        Segment(-half_width, -half_height, half_width, -half_height),
        Segment(half_width, -half_height, half_width, half_height),
        Segment(half_width, half_height, -half_width, half_height),
        Segment(-half_width, half_height, -half_width, -half_height),
    ]


def grid_wall_segments(grid):
    """Convert grid wall runs to centred horizontal or vertical segments."""
    segments = []
    visited = set()
    for row in range(grid.rows):
        for col in range(grid.cols):
            if (row, col) in visited or grid.get_cell(row, col) != WALL:
                continue
            horizontal = []
            next_col = col
            while (next_col < grid.cols
                   and grid.get_cell(row, next_col) == WALL):
                horizontal.append((row, next_col))
                next_col += 1
            vertical = []
            next_row = row
            while (next_row < grid.rows
                   and grid.get_cell(next_row, col) == WALL):
                vertical.append((next_row, col))
                next_row += 1
            cells = horizontal if len(horizontal) >= len(vertical) else vertical
            visited.update(cells)
            first_row, first_col = cells[0]
            last_row, last_col = cells[-1]
            x1, y1, _ = grid.cell_to_world(first_row, first_col)
            x2, y2, _ = grid.cell_to_world(last_row, last_col)
            if first_row == last_row:
                segments.append(Segment(
                    x1 - grid.cell_size / 2.0, y1,
                    x2 + grid.cell_size / 2.0, y2,
                ))
            else:
                segments.append(Segment(
                    x1, y1 + grid.cell_size / 2.0,
                    x2, y2 - grid.cell_size / 2.0,
                ))
    return segments
