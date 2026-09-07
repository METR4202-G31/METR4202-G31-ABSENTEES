"""Grid representation and collision-aware kernel stamping."""

from enum import Enum


class CellType(str, Enum):
    """Semantic values stored in a grid cell."""

    WALL = "wall"
    FREE = "free"
    DOORWAY = "doorway"


WALL = CellType.WALL
FREE = CellType.FREE
DOORWAY = CellType.DOORWAY


class Grid:
    """A rectangular grid whose coordinates are zero-based row and column."""

    def __init__(self, rows, cols, cell_size=0.5, default_cell=FREE):
        if rows <= 0 or cols <= 0:
            raise ValueError("rows and cols must be positive")
        if cell_size <= 0:
            raise ValueError("cell_size must be positive")
        self.rows = rows
        self.cols = cols
        self.cell_size = float(cell_size)
        self._cells = [
            [default_cell for _ in range(cols)] for _ in range(rows)
        ]
        self._closed = False

    def add_boundary_walls(self):
        """Make the outside edge solid while leaving the interior open."""
        if self.rows < 3 or self.cols < 3:
            raise ValueError("a closed grid needs at least a 1-cell interior")
        for row in range(self.rows):
            for col in range(self.cols):
                if row in (0, self.rows - 1) or col in (0, self.cols - 1):
                    self._cells[row][col] = WALL
        self._closed = True

    def is_boundary(self, row, col):
        """Return whether a coordinate lies on the outer grid ring."""
        return row in (0, self.rows - 1) or col in (0, self.cols - 1)

    def assert_closed(self):
        """Raise an error if a closed grid has an open perimeter."""
        if not self._closed:
            return
        if any(self.get_cell(row, col) != WALL
               for row in range(self.rows)
               for col in range(self.cols)
               if self.is_boundary(row, col)):
            raise ValueError("closed grid perimeter contains a non-wall cell")

    def in_bounds(self, row, col):
        """Return whether a coordinate is inside the grid."""
        return 0 <= row < self.rows and 0 <= col < self.cols

    def get_cell(self, row, col):
        """Return the cell value at a coordinate."""
        if not self.in_bounds(row, col):
            raise IndexError("grid coordinate is out of bounds")
        return self._cells[row][col]

    def set_cell(self, row, col, value):
        """Set a cell value."""
        if not self.in_bounds(row, col):
            raise IndexError("grid coordinate is out of bounds")
        self._cells[row][col] = value

    @staticmethod
    def rotate_kernel(kernel, rotation):
        """Rotate a kernel clockwise and normalize its top-left coordinate."""
        if rotation % 90 != 0:
            raise ValueError("rotation must be a multiple of 90 degrees")
        rotated = list(kernel)
        for _ in range((rotation % 360) // 90):
            rotated = [(-col, row, value) for row, col, value in rotated]
        min_row = min(row for row, _, _ in rotated)
        min_col = min(col for _, col, _ in rotated)
        return [(row - min_row, col - min_col, value)
                for row, col, value in rotated]

    def is_valid_position(self, kernel, row, col, rotation=0):
        """Check bounds and whether a kernel can be stamped atomically."""
        for offset_row, offset_col, value in self.rotate_kernel(kernel, rotation):
            target_row = row + offset_row
            target_col = col + offset_col
            if not self.in_bounds(target_row, target_col):
                return False
            existing = self.get_cell(target_row, target_col)
            if self._closed and self.is_boundary(target_row, target_col):
                if value != WALL:
                    return False
            if value in (FREE, DOORWAY):
                continue
            if existing == DOORWAY:
                return False
        return True

    def place_kernel(self, kernel, row, col, rotation=0):
        """Stamp a kernel, returning false without mutation on collision."""
        rotated = self.rotate_kernel(kernel, rotation)
        if not self.is_valid_position(rotated, row, col):
            return False
        for offset_row, offset_col, value in rotated:
            self.set_cell(row + offset_row, col + offset_col, value)
        return True

    def get_free_cells(self):
        """Return all cells that are navigable, including doorways."""
        return [(row, col) for row in range(self.rows)
                for col in range(self.cols)
                if self.get_cell(row, col) in (FREE, DOORWAY)]

    def reachable_cells(self, start):
        """Return the free cells reachable from a starting coordinate."""
        if start not in self.get_free_cells():
            return set()
        visited = {start}
        pending = [start]
        while pending:
            row, col = pending.pop()
            for next_row, next_col in (
                    (row - 1, col), (row + 1, col),
                    (row, col - 1), (row, col + 1)):
                if (next_row, next_col) in visited or not self.in_bounds(
                        next_row, next_col):
                    continue
                if self.get_cell(next_row, next_col) in (FREE, DOORWAY):
                    visited.add((next_row, next_col))
                    pending.append((next_row, next_col))
        return visited

    def free_components(self):
        """Return connected components of navigable cells."""
        remaining = set(self.get_free_cells())
        components = []
        while remaining:
            component = self.reachable_cells(min(remaining))
            components.append(component)
            remaining.difference_update(component)
        return components
