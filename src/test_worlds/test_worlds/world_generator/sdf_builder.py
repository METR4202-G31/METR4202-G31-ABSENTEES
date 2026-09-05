"""Build a small Gazebo Classic SDF world from a grid."""

from xml.etree import ElementTree

from .grid import WALL


class SDFBuilder:
    """Serialize grid walls, markers, and world settings into SDF XML."""

    def __init__(self, grid, marker_positions=(), wall_height=1.0,
                 wall_thickness=None):
        self.grid = grid
        self.marker_positions = list(marker_positions)
        self.wall_height = float(wall_height)
        self.wall_thickness = wall_thickness or grid.cell_size

    def cell_to_world(self, row, col, z=0.0):
        """Convert a grid cell to the center of its Gazebo world position."""
        x = (col - self.grid.cols / 2 + 0.5) * self.grid.cell_size
        y = (self.grid.rows / 2 - row - 0.5) * self.grid.cell_size
        return x, y, z

    @staticmethod
    def _pose(parent, x, y, z, yaw=0.0):
        ElementTree.SubElement(parent, "pose").text = (
            f"{x:.6f} {y:.6f} {z:.6f} 0 0 {yaw:.6f}")

    @staticmethod
    def _box_geometry(parent, size_x, size_y, size_z):
        geometry = ElementTree.SubElement(parent, "geometry")
        box = ElementTree.SubElement(geometry, "box")
        ElementTree.SubElement(box, "size").text = (
            f"{size_x:.6f} {size_y:.6f} {size_z:.6f}")

    def _add_wall_run(self, world, row, start_col, end_col):
        """Add one static box for a contiguous horizontal wall run."""
        run_length = end_col - start_col + 1
        model = ElementTree.SubElement(world, "model", {
            "name": f"wall_{row}_{start_col}_{end_col}",
        })
        ElementTree.SubElement(model, "static").text = "true"
        x, y, z = self.cell_to_world(
            row, (start_col + end_col) / 2, self.wall_height / 2)
        self._pose(model, x, y, z)
        link = ElementTree.SubElement(model, "link", {"name": "link"})
        collision = ElementTree.SubElement(
            link, "collision", {"name": "collision"})
        self._box_geometry(collision, run_length * self.grid.cell_size,
                           self.wall_thickness, self.wall_height)
        visual = ElementTree.SubElement(link, "visual", {"name": "visual"})
        self._box_geometry(visual, run_length * self.grid.cell_size,
                           self.wall_thickness, self.wall_height)
        material = ElementTree.SubElement(visual, "material")
        script = ElementTree.SubElement(material, "script")
        ElementTree.SubElement(
            script, "uri").text = "file://media/materials/scripts/gazebo.material"
        ElementTree.SubElement(script, "name").text = "Gazebo/Grey"

    def _add_ground(self, world):
        """Add an explicit ground model instead of relying on an include."""
        model = ElementTree.SubElement(world, "model", {"name": "ground_plane"})
        ElementTree.SubElement(model, "static").text = "true"
        link = ElementTree.SubElement(model, "link", {"name": "link"})
        collision = ElementTree.SubElement(link, "collision", {"name": "collision"})
        geometry = ElementTree.SubElement(collision, "geometry")
        plane = ElementTree.SubElement(geometry, "plane")
        ElementTree.SubElement(plane, "normal").text = "0 0 1"
        ElementTree.SubElement(plane, "size").text = "100 100"
        visual = ElementTree.SubElement(link, "visual", {"name": "visual"})
        geometry = ElementTree.SubElement(visual, "geometry")
        plane = ElementTree.SubElement(geometry, "plane")
        ElementTree.SubElement(plane, "normal").text = "0 0 1"
        ElementTree.SubElement(plane, "size").text = "100 100"
        material = ElementTree.SubElement(visual, "material")
        script = ElementTree.SubElement(material, "script")
        ElementTree.SubElement(
            script, "uri").text = "file://media/materials/scripts/gazebo.material"
        ElementTree.SubElement(script, "name").text = "Gazebo/Grey"

    @staticmethod
    def _add_sun(world):
        """Add the explicit directional light used by the reference world."""
        light = ElementTree.SubElement(world, "light", {
            "name": "sun", "type": "directional"})
        ElementTree.SubElement(light, "cast_shadows").text = "1"
        ElementTree.SubElement(light, "pose").text = "0 0 50 0 0 0"
        ElementTree.SubElement(light, "diffuse").text = "0.8 0.8 0.8 1"
        ElementTree.SubElement(light, "specular").text = "0.2 0.2 0.2 1"
        attenuation = ElementTree.SubElement(light, "attenuation")
        ElementTree.SubElement(attenuation, "range").text = "1000"
        ElementTree.SubElement(attenuation, "constant").text = "0.9"
        ElementTree.SubElement(attenuation, "linear").text = "0.01"
        ElementTree.SubElement(attenuation, "quadratic").text = "0.001"
        ElementTree.SubElement(light, "direction").text = "-0.5 0.1 -0.9"

    def _add_marker(self, world, row, col, marker_id):
        include = ElementTree.SubElement(world, "include")
        ElementTree.SubElement(include, "uri").text = (
            f"model://aruco_column_6x6_100mm_{marker_id}")
        ElementTree.SubElement(include, "name").text = f"aruco_{marker_id}"
        x, y, _ = self.cell_to_world(row, col)
        self._pose(include, x, y, 0.0)

    def build(self):
        """Return a complete SDF XML document as a string."""
        sdf = ElementTree.Element("sdf", {"version": "1.6"})
        world = ElementTree.SubElement(
            sdf, "world", {"name": "exploration_world"})
        ElementTree.SubElement(world, "gravity").text = "0 0 -9.8"
        physics = ElementTree.SubElement(
            world, "physics", {"name": "default_physics", "type": "ode"})
        ElementTree.SubElement(physics, "max_step_size").text = "0.001"
        ElementTree.SubElement(physics, "real_time_factor").text = "1"
        self._add_sun(world)
        self._add_ground(world)
        scene = ElementTree.SubElement(world, "scene")
        ElementTree.SubElement(scene, "ambient").text = "0.4 0.4 0.4 1"
        ElementTree.SubElement(scene, "background").text = "0.7 0.7 0.7 1"
        ElementTree.SubElement(scene, "shadows").text = "1"
        ElementTree.SubElement(physics, "real_time_update_rate").text = "1000"
        for row in range(self.grid.rows):
            col = 0
            while col < self.grid.cols:
                if self.grid.get_cell(row, col) != WALL:
                    col += 1
                    continue
                start_col = col
                while (col + 1 < self.grid.cols and
                       self.grid.get_cell(row, col + 1) == WALL):
                    col += 1
                self._add_wall_run(world, row, start_col, col)
                col += 1
        for row, col, marker_id in self.marker_positions:
            self._add_marker(world, row, col, marker_id)
        ElementTree.indent(sdf, space="  ")
        return ElementTree.tostring(sdf, encoding="unicode", xml_declaration=True)

    def write(self, output_path):
        """Write the generated SDF document to a path."""
        with open(output_path, "w", encoding="utf-8") as output_file:
            output_file.write(self.build())
