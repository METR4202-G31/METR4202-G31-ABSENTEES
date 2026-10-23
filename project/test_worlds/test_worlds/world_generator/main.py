"""Command-line entry point for deterministic exploration worlds."""

import argparse
import json

from .aruco import place_markers
from .features import VECTOR_FEATURES
from .geometry import boundary_segments, rasterize_segments, Segment
from .grid import Grid
from .placement import Placer, choose_spawn_cell
from .sdf_builder import SDFBuilder


DEFAULT_FEATURES = "room:3,l_corridor:2,dead_end:1,narrow_passage:1"


def parse_feature_spec(specification):
    """Parse ``name:count`` comma-separated feature specifications."""
    if not specification:
        return []
    result = []
    for item in specification.split(","):
        name, separator, count_text = item.partition(":")
        if not separator or name not in VECTOR_FEATURES:
            raise ValueError(f"invalid feature specification: {item}")
        count = int(count_text)
        if count < 0:
            raise ValueError("feature counts cannot be negative")
        for _ in range(count):
            result.append((name, VECTOR_FEATURES[name], (), {}))
    return result


def generate_world(rows=20, cols=20, cell_size=0.5, seed=42,
                   features=DEFAULT_FEATURES, min_feature_gap=0,
                   output="/tmp/exploration_world.sdf", width=None,
                   height=None, resolution=None, wall_thickness=None,
                   angled_wall=None):
    """Generate a world and return its grid, markers, and spawn cell."""
    if width is None:
        width = cols * cell_size
    if height is None:
        height = rows * cell_size
    width = float(width)
    height = float(height)
    cell_size = float(resolution or 0.05)
    cols = max(3, round(width / cell_size))
    rows = max(3, round(height / cell_size))
    wall_thickness = float(wall_thickness or min(cell_size, 0.1))
    if width <= 0 or height <= 0 or wall_thickness <= 0:
        raise ValueError("map dimensions and wall thickness must be positive")
    grid = Grid(rows, cols, cell_size=cell_size)
    placer = Placer(grid, seed=seed, min_feature_gap=min_feature_gap)
    feature_segments = boundary_segments(width, height)
    placer.place_vector_features(
        parse_feature_spec(features), width, height)
    for _, segments in placer.placed:
        feature_segments.extend(segments)
    rasterize_segments(grid, feature_segments, wall_thickness)
    grid._closed = True
    repairs = placer.connect_free_space()
    if angled_wall:
        values = [float(value) for value in angled_wall.split(",")]
        if len(values) != 4:
            raise ValueError("angled wall must be x1,y1,x2,y2 in metres")
        extra = Segment(*values)
        feature_segments.append(extra)
        rasterize_segments(grid, [extra], wall_thickness)
    spawn_cell = choose_spawn_cell(grid)
    grid.assert_closed()
    reachable_cells = grid.reachable_cells(spawn_cell)
    marker_positions = place_markers(grid, seed=seed, reachable_cells=reachable_cells)
    SDFBuilder(
        grid,
        marker_positions,
        wall_thickness=wall_thickness,
        wall_segments=feature_segments,
    ).write(output)
    spawn_x, spawn_y, _ = SDFBuilder(grid).cell_to_world(*spawn_cell, z=0.0)
    with open(output + ".spawn.json", "w", encoding="utf-8") as spawn_file:
        json.dump({"row": spawn_cell[0], "col": spawn_cell[1], "x": spawn_x, "y": spawn_y}, spawn_file)
    placer.repairs = repairs
    return grid, marker_positions, spawn_cell, placer


def build_parser():
    """Build the world generator argument parser."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rows", type=int, default=20)
    parser.add_argument("--cols", type=int, default=20)
    parser.add_argument("--cell-size", type=float, default=0.5)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--features", default=DEFAULT_FEATURES)
    parser.add_argument("--min-feature-gap", type=float, default=0)
    parser.add_argument("--output", default="/tmp/exploration_world.sdf")
    parser.add_argument("--width", type=float)
    parser.add_argument("--height", type=float)
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--wall-thickness", type=float)
    parser.add_argument(
        "--angled-wall",
        help="extra wall segment as x1,y1,x2,y2 in metres",
    )
    return parser


def main(argv=None):
    """Run the generator CLI."""
    args = build_parser().parse_args(argv)
    grid, marker_positions, spawn_cell, placer = generate_world(
        rows=args.rows,
        cols=args.cols,
        cell_size=args.cell_size,
        seed=args.seed,
        features=args.features,
        min_feature_gap=args.min_feature_gap,
        output=args.output,
        width=args.width,
        height=args.height,
        resolution=args.resolution,
        wall_thickness=args.wall_thickness,
        angled_wall=args.angled_wall,
    )
    print(
        f"seed={args.seed} placed={len(placer.placed)} "
        f"failed={len(placer.failed)}")
    print(f"markers={len(marker_positions)} spawn_cell={spawn_cell}")
    print(
        f"free={len(grid.get_free_cells())} "
        f"reachable={len(grid.reachable_cells(spawn_cell))} "
        f"repairs={placer.repairs}")
    print(f"grid={grid.rows}x{grid.cols} cell_size={grid.cell_size}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
