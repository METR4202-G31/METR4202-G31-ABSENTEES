"""Tests for the deterministic exploration world generator."""

from xml.etree import ElementTree

import pytest

from test_worlds.world_generator.aruco import place_markers
from test_worlds.world_generator.features import (
    doorway_kernel,
    dead_end_kernel,
    large_empty_space_kernel,
    l_corridor_kernel,
    narrow_passage_kernel,
    plus_kernel,
    room_kernel,
    wall_section_kernel,
)
from test_worlds.world_generator.grid import DOORWAY, FREE, WALL, Grid
from test_worlds.world_generator.main import build_parser, generate_world
from test_worlds.world_generator.placement import Placer, choose_spawn_cell


def test_grid_rejects_collisions_without_partial_mutation():
    grid = Grid(3, 3, default_cell=WALL)
    assert grid.place_kernel([(0, 0, FREE)], 1, 1)
    assert not grid.place_kernel([(0, 0, FREE), (0, 3, FREE)], 1, 1)
    assert grid.get_free_cells() == [(1, 1)]


def test_placer_rejects_overlapping_feature_bounds():
    placer = Placer(Grid(10, 10), min_feature_gap=0)
    placer._placed_bounds.append((2, 4, 2, 4))
    assert placer._overlaps_placed_feature((4, 6, 4, 6))
    assert not placer._overlaps_placed_feature((5, 7, 5, 7))


def test_placer_allows_configured_negative_overlap():
    placer = Placer(Grid(10, 10), min_feature_gap=-1)
    placer._placed_bounds.append((2, 4, 2, 4))
    assert not placer._overlaps_placed_feature((4, 6, 4, 6))
    assert placer._overlaps_placed_feature((3, 5, 3, 5))


def test_min_feature_gap_is_configurable_from_cli():
    args = build_parser().parse_args(["--min-feature-gap", "-1.5"])
    assert args.min_feature_gap == -1.5


def test_doorway_can_connect_to_existing_free_cell():
    grid = Grid(3, 3, default_cell=WALL)
    grid.set_cell(1, 1, FREE)
    assert grid.place_kernel(doorway_kernel(), 0, 1)
    assert grid.get_cell(1, 1) == DOORWAY


def test_doorway_kernel_has_two_cell_opening():
    values = {(row, col): value for row, col, value in doorway_kernel()}
    assert values[(1, 0)] == DOORWAY
    assert values[(1, 1)] == DOORWAY


def test_wall_section_kernel_has_one_cell_opening():
    values = {(row, col): value for row, col, value in wall_section_kernel()}
    assert len(values) == 27
    assert values[(1, 4)] == DOORWAY
    assert sum(value == DOORWAY for value in values.values()) == 1
    assert sum(value == WALL for value in values.values()) == 26


def test_large_empty_space_kernel_is_15_by_15_with_doorways_on_each_edge():
    cells = large_empty_space_kernel()
    assert len(cells) == 225
    values = {(row, col): value for row, col, value in cells}
    assert all(
        values[(row, col)] == WALL
        for row in range(15)
        for col in range(15)
        if (row in (0, 14) or col in (0, 14)) and not (
            (row in (0, 14) and 6 <= col <= 8) or
            (col in (0, 14) and 6 <= row <= 8)
        )
    )
    assert all(
        values[(row, col)] == FREE
        for row in range(1, 14)
        for col in range(1, 14)
    )
    assert [values[(0, col)] for col in range(6, 9)] == [DOORWAY] * 3
    assert [values[(14, col)] for col in range(6, 9)] == [DOORWAY] * 3
    assert [values[(row, 0)] for row in range(6, 9)] == [DOORWAY] * 3
    assert [values[(row, 14)] for row in range(6, 9)] == [DOORWAY] * 3


@pytest.mark.parametrize(
    "kernel_factory",
    [l_corridor_kernel, dead_end_kernel, narrow_passage_kernel],
)
def test_traversable_features_have_walls_and_two_by_two_free_space(
        kernel_factory):
    values = {(row, col): value for row, col, value in kernel_factory()}
    free_cells = {cell for cell, value in values.items() if value == FREE}
    assert any(
        {(row, col), (row + 1, col), (row, col + 1), (row + 1, col + 1)}
        <= free_cells
        for row, col in free_cells
    )
    max_row = max(row for row, _ in values)
    max_col = max(col for _, col in values)
    assert all(
        values[(row, col)] in (WALL, DOORWAY)
        for row, col in values
        if row in (0, max_row) or col in (0, max_col)
    )


def test_room_kernel_has_free_interior_and_doorway():
    cells = room_kernel()
    values = [value for _, _, value in cells]
    assert values.count(FREE) == 16
    assert values.count(DOORWAY) == 2


def test_narrow_passage_has_one_cell_thick_side_walls():
    values = {(row, col): value for row, col, value in narrow_passage_kernel()}
    assert max(row for row, _ in values) == 7
    assert max(col for _, col in values) == 3
    assert all(values[(row, col)] == WALL for row in range(1, 7)
               for col in (0, 3))
    assert all(values[(row, col)] == FREE for row in range(1, 7)
               for col in (1, 2))


def test_dead_end_has_one_cell_thick_side_walls():
    values = {(row, col): value for row, col, value in dead_end_kernel()}
    assert max(row for row, _ in values) == 7
    assert max(col for _, col in values) == 3
    assert all(values[(row, col)] == WALL for row in range(1, 7)
               for col in (0, 3))
    assert all(values[(row, col)] == FREE for row in range(1, 7)
               for col in (1, 2))


def test_plus_kernel_is_7_by_7_with_walls_in_a_cross():
    values = {(row, col): value for row, col, value in plus_kernel()}
    assert len(values) == 49
    assert sum(value == FREE for value in values.values()) == 36
    assert sum(value == WALL for value in values.values()) == 13
    assert all(
        values[(row, col)] == WALL
        for row in range(7)
        for col in range(7)
        if row == 3 or col == 3
    )


def test_markers_are_seeded_and_unique():
    grid = Grid(4, 4, default_cell=FREE)
    assert place_markers(grid, seed=42) == place_markers(grid, seed=42)
    assert len({(row, col) for row, col, _ in place_markers(
        grid, seed=42)}) == 3
    with pytest.raises(ValueError):
        place_markers(Grid(1, 2, default_cell=FREE), seed=42)


def test_spawn_cell_is_nearest_to_center():
    grid = Grid(5, 5, default_cell=FREE)
    assert choose_spawn_cell(grid) == (2, 2)


def test_closed_grid_never_allows_features_to_open_boundary():
    grid = Grid(5, 5)
    grid.add_boundary_walls()
    assert not grid.place_kernel([(0, 0, FREE)], 0, 2)
    grid.assert_closed()
    assert all(grid.get_cell(row, col) == WALL
               for row in range(grid.rows)
               for col in range(grid.cols)
               if grid.is_boundary(row, col))


def test_closed_grid_rejects_degenerate_dimensions():
    with pytest.raises(ValueError):
        Grid(2, 5).add_boundary_walls()


def test_generation_is_reproducible(tmp_path):
    first = tmp_path / "first.sdf"
    second = tmp_path / "second.sdf"
    generate_world(rows=12, cols=12, seed=42, output=str(first))
    generate_world(rows=12, cols=12, seed=42, output=str(second))
    first_root = ElementTree.parse(first).getroot()
    second_root = ElementTree.parse(second).getroot()
    assert ElementTree.tostring(first_root) == ElementTree.tostring(second_root)
    assert len(first_root.findall(
        './/include[uri="model://aruco_column_6x6_100mm_0"]')) == 1


def test_default_generation_is_connected_and_models_are_valid(tmp_path):
    output = tmp_path / "connected.sdf"
    grid, _, spawn_cell, _ = generate_world(output=str(output))
    assert grid.reachable_cells(spawn_cell) == set(grid.get_free_cells())
    root = ElementTree.parse(output).getroot()
    models = root.findall(".//model")
    assert models
    assert all("static" not in model.attrib for model in models)
    assert all(model.findtext("static") == "true" for model in models)


def test_generated_world_matches_reference_structure(tmp_path):
    output = tmp_path / "reference-structure.sdf"
    grid, _, _, _ = generate_world(output=str(output))
    world = ElementTree.parse(output).getroot().find("world")
    assert world.find("light[@name='sun']") is not None
    assert world.find("model[@name='ground_plane']") is not None
    assert world.find("scene/ambient") is not None
    assert world.find("scene/background") is not None
    assert world.find("scene/shadows") is not None
    assert world.find("physics/real_time_update_rate") is not None
    wall_models = [
        model for model in world.findall("model")
        if model.get("name", "").startswith("wall_")
    ]
    assert len(wall_models) < len(grid.get_free_cells())
    assert all(model.find("link/visual/material/script/name") is not None
               for model in wall_models)


def test_world_includes_all_three_aruco_models(tmp_path):
    output = tmp_path / "aruco-models.sdf"
    grid, markers, _, _ = generate_world(output=str(output))
    world = ElementTree.parse(output).getroot().find("world")
    includes = world.findall("include")
    uris = [include.findtext("uri") for include in includes]
    assert uris.count("model://aruco_column_6x6_100mm_0") == 1
    assert uris.count("model://aruco_column_6x6_100mm_2") == 1
    assert uris.count("model://aruco_column_6x6_100mm_42") == 1
    assert len({(row, col) for row, col, _ in markers}) == 3
    assert all(not grid.is_boundary(row, col)
               for row, col, _ in markers)
