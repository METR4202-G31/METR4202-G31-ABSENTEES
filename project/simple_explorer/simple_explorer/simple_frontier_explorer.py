#!/usr/bin/env python3
"""
Simple Frontier Exploration Demonstration
==========================================

A minimal ROS 2 frontier-exploration demonstration for a TurtleBot3 using
SLAM Toolbox and Nav2.

The implementation is based on the following ROS 2 interfaces and general
frontier-exploration concepts:

References
----------
1. Navigation2 (Nav2), "Simple Commander API".
   Used as the reference for BasicNavigator, goToPose(), isTaskComplete(),
   getResult(), and interaction with the Nav2 navigation stack.

2. ROS 2 nav_msgs, "OccupancyGrid".
   Used as the reference for the /map representation, map metadata,
   row-major occupancy-grid storage, and interpretation of unknown/free
   map cells.

3. robo-friends, "m-explore-ros2: Explore_lite port to ROS2".
   Used as a reference for the general ROS 2 frontier-exploration
   architecture and integration of frontier exploration with Nav2,
   SLAM, and TurtleBot3.

Known limitations:
- Frontier goals are selected using a simple heuristic rather than a robust
  exploration cost function.
- Goal selection does not account for obstacle clearance or Nav2 costmap cost.
- Candidate goals are not checked with Nav2 path planning before being sent.
- Frontier selection may favour awkward points near walls, corners, or narrow
  passages.
- Previous-goal filtering is simplistic and can incorrectly reject useful
  future frontiers.
- Exploration completion is approximate and may stop while useful unexplored
  regions still remain.
- The algorithm does not optimise total travel distance or exploration time.
- Dynamic map changes and navigation failures are handled only very simply.
"""

from collections import deque
import math

import rclpy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid

from nav2_simple_commander.robot_navigator import (
    BasicNavigator,
    TaskResult,
)

from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from lifecycle_msgs.srv import GetState


class SimpleFrontierExplorer(BasicNavigator):
    """basic frontier exploration demonstration."""

    def __init__(self):
        super().__init__(node_name="simple_frontier_explorer")

        self.map_msg = None
        self.previous_goals = []

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(
            OccupancyGrid,
            "/map",
            self.map_callback,
            map_qos,
        )

    def wait_for_nav2(self):
        """Wait until the Nav2 BT navigator is active."""

        client = self.create_client(
            GetState,
            "/bt_navigator/get_state",
        )

        self.get_logger().info("Waiting for Nav2 to become active...")

        while rclpy.ok():

            if not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(
                    "Waiting for bt_navigator lifecycle service..."
                )
                continue

            request = GetState.Request()
            future = client.call_async(request)

            rclpy.spin_until_future_complete(
                self,
                future,
                timeout_sec=1.0,
            )

            if future.result() is None:
                continue

            state = future.result().current_state.label

            self.get_logger().info(f"bt_navigator state: {state}")

            if state == "active":
                self.get_logger().info("Nav2 is active.")
                return

    def map_callback(self, msg):
        self.map_msg = msg

    def is_free(self, value):
        # SLAM occupancy grids normally use:
        # -1 = unknown
        #  0 = free
        # 100 = occupied
        return 0 <= value <= 10

    def is_frontier(self, x, y, data, width, height):
        """Return True if cell is free and touches unknown space."""

        index = y * width + x

        if not self.is_free(data[index]):
            return False

        # Check the eight surrounding cells.
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):

                if dx == 0 and dy == 0:
                    continue

                nx = x + dx
                ny = y + dy

                if nx < 0 or nx >= width:
                    continue

                if ny < 0 or ny >= height:
                    continue

                neighbour_index = ny * width + nx

                if data[neighbour_index] == -1:
                    return True

        return False

    def find_frontiers(self):
        """Find and group frontier cells using Breadth-first search (BFS)."""

        msg = self.map_msg

        width = msg.info.width
        height = msg.info.height
        data = msg.data

        frontier_cells = set()

        # Step 1:
        # Find every free cell touching an unknown cell.
        for y in range(1, height - 1):
            for x in range(1, width - 1):

                if self.is_frontier(
                    x,
                    y,
                    data,
                    width,
                    height,
                ):
                    frontier_cells.add((x, y))

        # Step 2:
        # Group connected frontier cells into clusters.
        clusters = []
        visited = set()

        for start in frontier_cells:

            # Ignore already visited cells
            if start in visited:
                continue

            # Start a queue with the first frontier cell
            queue = deque([start])
            visited.add(start)

            cluster = []

            while queue:

                # Grab the first cell and find neighbouring frontier cells
                cell = queue.popleft()
                cluster.append(cell)

                x, y = cell

                # Loop through all offsets around the cell, left/right/up/down
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):

                        if dx == 0 and dy == 0:
                            continue

                        neighbour = (x + dx, y + dy)

                        if neighbour not in frontier_cells:
                            continue

                        if neighbour in visited:
                            continue

                        visited.add(neighbour)
                        queue.append(neighbour)

            # Ignore tiny frontier fragments.
            if len(cluster) >= 8:
                clusters.append(cluster)

        return clusters

    def grid_to_world(self, x, y):
        """Convert occupancy-grid cell to map-frame coordinates."""

        # Get map metadata with OccupancyGrid nav_msgs
        info = self.map_msg.info

        resolution = info.resolution
        origin = info.origin

        # Convert grid indices into metres at the centre of the grid cell
        local_x = (x + 0.5) * resolution
        local_y = (y + 0.5) * resolution

        # Account for map origin rotation.
        q = origin.orientation

        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        # Rotate local grid coordinate
        world_x = (
            origin.position.x
            + math.cos(yaw) * local_x
            - math.sin(yaw) * local_y
        )

        world_y = (
            origin.position.y
            + math.sin(yaw) * local_x
            + math.cos(yaw) * local_y
        )

        return world_x, world_y

    def cluster_goal(self, cluster):
        """Choose a frontier point away from previously used goals."""

        # If this is the first goal, choose a point near the outer
        # extreme of the frontier rather than its centroid.

        centre_x = sum(cell[0] for cell in cluster) / len(cluster)
        centre_y = sum(cell[1] for cell in cluster) / len(cluster)

        if not self.previous_goals:

            best_cell = max(
                cluster,
                key=lambda cell: (cell[0] - centre_x) ** 2
                + (cell[1] - centre_y) ** 2,
            )

            return self.grid_to_world(
                best_cell[0],
                best_cell[1],
            )

        # For later goals, choose the frontier cell furthest from
        # all previous navigation goals.

        best_cell = None
        best_distance = -1.0

        for cell in cluster:

            world_x, world_y = self.grid_to_world(
                cell[0],
                cell[1],
            )

            nearest_old_goal = min(
                math.hypot(
                    world_x - old_x,
                    world_y - old_y,
                )
                for old_x, old_y in self.previous_goals
            )

            if nearest_old_goal > best_distance:
                best_distance = nearest_old_goal
                best_cell = cell

        return self.grid_to_world(
            best_cell[0],
            best_cell[1],
        )

    def goal_already_used(self, x, y):
        """Avoid repeatedly sending almost identical goals."""

        for old_x, old_y in self.previous_goals:

            distance = math.hypot(
                x - old_x,
                y - old_y,
            )

            if distance < 0.1:
                return True

        return False

    def choose_goal(self):
        """Choose the largest frontier not previously attempted."""

        clusters = self.find_frontiers()

        if not clusters:
            return None

        # Largest frontier first.
        clusters.sort(
            key=len,
            reverse=True,
        )

        for cluster in clusters:

            x, y = self.cluster_goal(cluster)

            if not self.goal_already_used(x, y):
                return x, y, len(cluster)

        return None

    def create_goal_pose(self, x, y):
        goal = PoseStamped()

        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()

        goal.pose.position.x = x
        goal.pose.position.y = y

        # Don't care about final heading for this demo.
        goal.pose.orientation.w = 1.0

        return goal


def main(args=None):
    rclpy.init(args=args)

    explorer = SimpleFrontierExplorer()

    # First wait for Nav2 navigation to be ready.
    explorer.wait_for_nav2()

    explorer.get_logger().info("Waiting for SLAM map...")

    # Then wait until SLAM has produced a map.
    while rclpy.ok() and explorer.map_msg is None:
        rclpy.spin_once(
            explorer,
            timeout_sec=0.5,
        )

    explorer.get_logger().info("Map received. Starting exploration.")

    while rclpy.ok():
        rclpy.spin_once(
            explorer,
            timeout_sec=0.2,
        )

        result = explorer.choose_goal()

        if result is None:
            explorer.get_logger().info(
                "No unused frontiers remain. " "Exploration finished."
            )
            break

        x, y, size = result

        explorer.get_logger().info(
            f"Frontier selected: " f"x={x:.2f}, y={y:.2f}, " f"size={size}"
        )

        explorer.previous_goals.append((x, y))

        goal = explorer.create_goal_pose(x, y)

        explorer.goToPose(goal)

        while not explorer.isTaskComplete():
            pass

        navigation_result = explorer.getResult()

        if navigation_result == TaskResult.SUCCEEDED:
            explorer.get_logger().info("Reached frontier.")

        elif navigation_result == TaskResult.CANCELED:
            explorer.get_logger().warn("Navigation canceled.")

        else:
            explorer.get_logger().warn(
                "Navigation failed. " "Trying another frontier."
            )

    explorer.destroyNode()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
