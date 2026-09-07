"""Command-line entry point for deterministic exploration worlds."""

import argparse
import json

from .aruco import place_markers
from .features import FEATURES
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
        if not separator or name not in FEATURES:
            raise ValueError(f"invalid feature specification: {item}")
        count = int(count_text)
        if count < 0:
            raise ValueError("feature counts cannot be negative")
        for _ in range(count):
            result.append((name, FEATURES[name], (), {}))
    return result


def generate_world(rows=20, cols=20, cell_size=0.5, seed=42,
                   features=DEFAULT_FEATURES, min_feature_gap=0,
                   output="/tmp/exploration_world.sdf"):
    """Generate a world and return its grid, markers, and spawn cell."""
    grid = Grid(rows, cols, cell_size=cell_size)
    grid.add_boundary_walls()
    placer = Placer(grid, seed=seed, min_feature_gap=min_feature_gap)
    placer.place_features(parse_feature_spec(features))
    repairs = placer.connect_free_space()
    spawn_cell = choose_spawn_cell(grid)
    grid.assert_closed()
    reachable_cells = grid.reachable_cells(spawn_cell)
    marker_positions = place_markers(grid, seed=seed, reachable_cells=reachable_cells)
    SDFBuilder(grid, marker_positions).write(output)
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
