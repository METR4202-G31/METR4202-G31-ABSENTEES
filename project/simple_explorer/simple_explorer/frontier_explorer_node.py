"""
Frontier explorer node.

Chooses where the robot should explore next and hands each goal to the
navigation supervisor's ManagedNavigate action.

Subscribes:
  /map, /global_costmap/costmap, /local_costmap/costmap  (OccupancyGrid)
  /failed_regions   (navigation_interfaces/FailedRegionArray, from the supervisor)
  /explore/resume   (std_msgs/Bool, from mission control: false stops, true resumes)
Publishes:
  /explore            (PoseStamped) every goal sent, just a pose
  /explore/blacklist  (PoseStamped) every goal the explorer rejected itself
  /explore/status     (String) WAITING, EXPLORING, STOPPED,
                      DONE_NO_FRONTIERS or DONE_BLACKLISTED_REMAIN
  /explore/frontiers  (MarkerArray) frontier cells, current goal, blacklist
Action clients:
  managed_navigate (navigation_interfaces/ManagedNavigate), compute_path_to_pose (Nav2)
"""

import math
from typing import List, Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid
from navigation_interfaces.action import ManagedNavigate
from navigation_interfaces.msg import FailedRegionArray
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, ColorRGBA, String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from .blacklist import GoalBlacklist
from .CoordinatePublisher import CoordinatePublisher
from .frontier_selector import (
    CostmapLayer,
    frontier_near,
    FrontierCandidate,
    FrontierSelector,
    cell_to_world,
)
from .goal_outcome import decide

WAITING = "WAITING"
EXPLORING = "EXPLORING"
STOPPED = "STOPPED"
DONE_NO_FRONTIERS = "DONE_NO_FRONTIERS"
DONE_BLACKLISTED_REMAIN = "DONE_BLACKLISTED_REMAIN"

# Internal states.
SELECTING = "SELECTING"
PRECHECKING = "PRECHECKING"
NAVIGATING = "NAVIGATING"
IDLE = "IDLE"   # stopped or done: no goals are sent


class FrontierExplorer(Node):
    """Select frontier goals and drive them through ManagedNavigate."""

    def __init__(self):
        super().__init__("frontier_explorer")
        self._declare_parameters()
        p = self.get_parameter

        self._selector = FrontierSelector(
            min_cluster_size=p("min_cluster_size").value,
            min_goal_distance=p("min_goal_distance").value,
            revisit_radius=p("revisit_radius").value,
            max_history=p("max_history").value,
            distance_weight=p("distance_weight").value,
            size_weight=p("size_weight").value,
            cost_limit=p("cost_limit").value,
            use_travel_distance=p("use_travel_distance").value,
            travel_clearance=p("travel_clearance").value,
            gain_weight=p("gain_weight").value,
            min_gain=p("min_gain").value,
            gain_radius=p("gain_radius").value,
            max_gain_area=p("max_gain_area").value,
            turn_weight=p("turn_weight").value,
        )
        self._blacklist = GoalBlacklist(
            default_radius=p("blacklist_radius").value,
            min_failure_count=p("min_failure_count").value,
        )

        self._map: Optional[OccupancyGrid] = None
        self._global_costmap: Optional[OccupancyGrid] = None
        self._local_costmap: Optional[OccupancyGrid] = None
        self._frontiers: dict[int, Pose] = {}
        self._sent_goals: dict[int, Pose] = {}
        self._blacklisted_goals: dict[int, Pose] = {}
        self._latest_clusters = []
        self._latest_candidates = []

        self._state = SELECTING
        self._status = WAITING
        self._done_status: Optional[str] = None
        self._empty_cycles = 0
        self._next_select_time = 0.0
        self._queue: List[FrontierCandidate] = []
        self._precheck_token = 0
        self._precheck_deadline = 0.0
        self._active: Optional[dict] = None
        self._last_validity_check = 0.0
        self._warned_planner = False
        self._precheck_candidate: Optional[FrontierCandidate] = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, "/map", self._on_map, latched)
        self.create_subscription(
            OccupancyGrid, "/global_costmap/costmap", self._on_global, latched
        )
        self.create_subscription(
            OccupancyGrid, "/local_costmap/costmap", self._on_local, latched
        )
        self.create_subscription(
            FailedRegionArray, "/failed_regions", self._on_failed_regions, 10
        )
        self.create_subscription(Bool, "/explore/resume", self._on_resume, 10)

        self._goal_pub = self.create_publisher(PoseStamped, "/explore", 10)
        self._reject_pub = self.create_publisher(PoseStamped, "/explore/blacklist", 10)
        self._status_pub = self.create_publisher(String, "/explore/status", latched)
        self._marker_pub = self.create_publisher(MarkerArray, "/explore/frontiers", 1)
        self._frontier_coords_pub = CoordinatePublisher(
            node=self,
            coords_dict=self._frontiers,
            topic="frontier",
            namespace="detected frontiers",
            frame_id=p("map_frame").value,
            marker_type=Marker.SPHERE_LIST,
            scale_x=0.1,
            scale_y=0.1,
            color=ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0),
        )
        self._goal_coords_pub = CoordinatePublisher(
            node=self,
            coords_dict=self._sent_goals,
            topic="goal",
            namespace="sent goals",
            frame_id=p("map_frame").value,
            marker_type=Marker.SPHERE_LIST,
            scale_x=0.2,
            scale_y=0.2,
            color=ColorRGBA(r=0.5, g=0.5, b=0.5, a=1.0),
        )
        self._current_goal_pub = self.create_publisher(Marker, "goal", 10)
        self._blacklist_coords_pub = CoordinatePublisher(
            node=self,
            coords_dict=self._blacklisted_goals,
            topic="blacklist",
            namespace="blacklisted goals",
            frame_id=p("map_frame").value,
            marker_type=Marker.SPHERE_LIST,
            scale_x=0.2,
            scale_y=0.2,
            color=ColorRGBA(a=1.0),
        )

        self._nav_client = ActionClient(
            self, ManagedNavigate, p("managed_navigate_action").value
        )
        self._planner_client = ActionClient(
            self, ComputePathToPose, p("compute_path_action").value
        )

        self._publish_status(WAITING)
        self.create_timer(1.0 / p("loop_rate_hz").value, self._tick)
        self.create_timer(5.0, self._republish_markers)
        self.get_logger().info("Frontier explorer started.")

    def _declare_parameters(self):
        defaults = {
            "min_cluster_size": 8,
            "min_goal_distance": 0.6,
            "revisit_radius": 0.5,
            "max_history": 20,
            "distance_weight": 1.0,
            "size_weight": 0.02,
            "cost_limit": 75,
            "use_travel_distance": True,
            "travel_clearance": 0.15,
            "gain_weight": 0.0,
            "min_gain": 0.0,
            "gain_radius": 3.0,
            "max_gain_area": 12.0,
            "turn_weight": 0.0,
            "blacklist_radius": 0.5,
            "min_failure_count": 1,
            "robot_frame": "base_footprint",
            "map_frame": "map",
            "managed_navigate_action": "managed_navigate",
            "compute_path_action": "compute_path_to_pose",
            "precheck_path": True,
            "max_prechecks": 5,
            "precheck_timeout": 3.0,
            "loop_rate_hz": 2.0,
            "settle_time": 1.0,
            "empty_cycles_to_finish": 3,
            "validity_check_period": 2.0,
            "frontier_gone_radius": 1.0,
            "frontier_display_radius": 0.5,
            "max_internal_retries": 2,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    # ---- subscriptions -------------------------------------------------

    def _on_map(self, msg):
        self._map = msg

    def _on_global(self, msg):
        self._global_costmap = msg

    def _on_local(self, msg):
        self._local_costmap = msg

    def _on_failed_regions(self, msg):
        self._blacklist.update_from_failed_regions(msg)
        self._publish_blacklist_markers()

    def _on_resume(self, msg):
        if not msg.data:
            self._stop()
        else:
            self._resume()

    # ---- stop / resume ---------------------------------------------------

    def _stop(self):
        if self._state == IDLE and self._status == STOPPED:
            return
        self.get_logger().info("Stop requested by mission control.")
        self._precheck_token += 1    # ignore any pending planner reply
        self._queue = []
        if self._state == NAVIGATING:
            self._cancel_active()
        self._state = IDLE
        self._publish_status(STOPPED)

    def _resume(self):
        if self._state != IDLE:
            return
        if self._done_status is not None:
            # Retry pass: goals rejected earlier may be reachable now.
            self.get_logger().info("Resuming after completion: retrying blacklisted goals.")
            self._blacklist.clear()
            self._blacklist.forget_external()
            self._publish_blacklist_markers()
            self._selector.clear_history()
            self._done_status = None
        else:
            self.get_logger().info("Resuming exploration.")
        self._empty_cycles = 0
        self._state = SELECTING
        self._publish_status(EXPLORING)

    # ---- helpers ---------------------------------------------------------

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _publish_status(self, status: str):
        self._status = status
        self._status_pub.publish(String(data=status))

    def _robot_pose(self) -> Optional[Pose]:
        try:
            t = self._tf_buffer.lookup_transform(
                self.get_parameter("map_frame").value,
                self.get_parameter("robot_frame").value,
                rclpy.time.Time(),
            )
        except TransformException:
            return None
        pose = Pose()
        pose.position.x = t.transform.translation.x
        pose.position.y = t.transform.translation.y
        pose.orientation = t.transform.rotation
        return pose

    def _costmap_layers(self) -> Optional[List[CostmapLayer]]:
        """Return the costmap layers to check, or None if one is not ready yet."""
        if self._global_costmap is None:
            return None
        layers = [CostmapLayer(self._global_costmap, strict_bounds=True)]
        local = self._local_costmap
        if local is None:
            return layers
        map_frame = self.get_parameter("map_frame").value
        frame = local.header.frame_id
        if frame in ("", map_frame):
            layers.append(CostmapLayer(local, strict_bounds=False))
            return layers
        try:
            t = self._tf_buffer.lookup_transform(frame, map_frame, rclpy.time.Time())
        except TransformException:
            self.get_logger().warn(
                "No transform %s -> %s yet; skipping local costmap." % (map_frame, frame),
                throttle_duration_sec=10.0,
            )
            return layers
        q = t.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        tx, ty = t.transform.translation.x, t.transform.translation.y
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)

        def to_local(x, y):
            return (cos_yaw * x - sin_yaw * y + tx, sin_yaw * x + cos_yaw * y + ty)

        layers.append(CostmapLayer(local, to_local, strict_bounds=False))
        return layers

    # ---- main loop ---------------------------------------------------------

    def _tick(self):
        if self._state == IDLE:
            return
        if self._state == PRECHECKING:
            if self._now() > self._precheck_deadline:
                self._precheck_timed_out()
            return
        if self._state == NAVIGATING:
            self._revalidate_active_goal()
            return
        if self._now() >= self._next_select_time:
            self._select()

    def _select(self):
        robot = self._robot_pose()
        layers = self._costmap_layers()
        if self._map is None or robot is None or layers is None:
            self._publish_status_if_changed(WAITING)
            return
        if not self._nav_client.server_is_ready():
            self.get_logger().info(
                "Waiting for the ManagedNavigate action server...",
                throttle_duration_sec=5.0,
            )
            self._publish_status_if_changed(WAITING)
            return

        result = self._selector.rank_candidates(
            self._map, robot, layers, self._blacklist.is_blocked,
            self.get_parameter("map_frame").value,
        )
        self._publish_markers(result.clusters, result.candidates[:1])

        if not result.candidates:
            self._empty_cycles += 1
            if self._empty_cycles >= self.get_parameter("empty_cycles_to_finish").value:
                if result.low_gain_clusters:
                    self.get_logger().info(
                        "%d frontier(s) ignored: too little unknown space behind them."
                        % result.low_gain_clusters
                    )
                if result.unreachable_clusters:
                    self.get_logger().info(
                        "%d frontier(s) cannot be reached through known free space."
                        % result.unreachable_clusters
                    )
                self._finish(result.blocked_clusters > 0)
            return
        self._empty_cycles = 0
        self._publish_status_if_changed(EXPLORING)

        limit = self.get_parameter("max_prechecks").value
        self._queue = list(result.candidates[:limit])
        self._try_next_candidate()

    def _publish_status_if_changed(self, status: str):
        if self._status != status:
            self._publish_status(status)

    def _finish(self, blocked_remain: bool):
        self._done_status = DONE_BLACKLISTED_REMAIN if blocked_remain else DONE_NO_FRONTIERS
        self._state = IDLE
        self._publish_status(self._done_status)
        self.get_logger().info("Exploration finished: %s" % self._done_status)

    # ---- candidate checking -------------------------------------------------

    def _try_next_candidate(self):
        if not self._queue:
            # Everything queued was rejected; reselect after a pause.
            self._state = SELECTING
            self._next_select_time = self._now() + self.get_parameter("settle_time").value
            return
        candidate = self._queue.pop(0)
        if self.get_parameter("precheck_path").value and self._planner_client.server_is_ready():
            self._precheck(candidate)
        else:
            if self.get_parameter("precheck_path").value and not self._warned_planner:
                self.get_logger().warn(
                    "compute_path_to_pose is not available; sending goals unchecked."
                )
                self._warned_planner = True
            self._send_goal(candidate)

    def _precheck(self, candidate: FrontierCandidate):
        self._state = PRECHECKING
        self._precheck_token += 1
        token = self._precheck_token
        self._precheck_deadline = self._now() + self.get_parameter("precheck_timeout").value

        goal = ComputePathToPose.Goal()
        goal.goal = candidate.pose
        goal.goal.header.stamp = self.get_clock().now().to_msg()
        goal.use_start = False
        self._precheck_candidate = candidate
        future = self._planner_client.send_goal_async(goal)
        future.add_done_callback(
            lambda f: self._on_precheck_accepted(f, candidate, token)
        )

    def _on_precheck_accepted(self, future, candidate, token):
        if token != self._precheck_token:
            return
        handle = future.result()
        if not handle.accepted:
            self._send_goal(candidate)    # planner unavailable: let Nav2 decide
            return
        handle.get_result_async().add_done_callback(
            lambda f: self._on_precheck_result(f, candidate, token)
        )

    def _on_precheck_result(self, future, candidate, token):
        if token != self._precheck_token:
            return
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED and result.result.path.poses:
            self._send_goal(candidate)
        else:
            self._reject(candidate, "no_path")
            self._try_next_candidate()

    def _precheck_timed_out(self):
        self._precheck_token += 1    # ignore the late reply
        self.get_logger().warn("Path pre-check timed out; sending the goal unchecked.")
        self._send_goal(self._precheck_candidate)

    def _reject(self, candidate: FrontierCandidate, reason: str):
        pose = candidate.pose
        x, y = pose.pose.position.x, pose.pose.position.y
        self.get_logger().info("Rejected goal (%.2f, %.2f): %s" % (x, y, reason))
        self._blacklist.add(x, y, reason)
        self._publish_blacklist_markers()
        self._selector.mark_visited(x, y)
        pose.header.stamp = self.get_clock().now().to_msg()
        self._reject_pub.publish(pose)

    # ---- sending and tracking goals ------------------------------------------

    def _send_goal(self, candidate: FrontierCandidate):
        pose = candidate.pose
        pose.header.stamp = self.get_clock().now().to_msg()
        self._goal_pub.publish(pose)
        goal_pose = Pose()
        goal_pose.position = pose.pose.position
        self._sent_goals[len(self._sent_goals)] = goal_pose
        self._goal_coords_pub.publish_coordinates()

        goal = ManagedNavigate.Goal()
        goal.target_pose = pose
        self._active = {
            "candidate": candidate,
            "handle": None,
            "cancel": False,
            "retries": 0,
        }
        self._publish_current_goal_marker()
        self._state = NAVIGATING
        self._last_validity_check = self._now()
        self.get_logger().info(
            "Goal (%.2f, %.2f) size=%d gain=%.1fm2 score=%.2f" % (
                pose.pose.position.x, pose.pose.position.y,
                candidate.cluster_size, candidate.gain, candidate.score,
            )
        )
        self._nav_client.send_goal_async(goal).add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        active = self._active
        if active is None:
            return
        handle = future.result()
        if not handle.accepted:
            # Rejection may just mean the supervisor is busy, so do not
            # blacklist: treat it like an external cancel (visited only).
            self.get_logger().warn("Supervisor rejected the goal.")
            self._finish_goal(ManagedNavigate.Result.CANCELED)
            return
        active["handle"] = handle
        if active["cancel"]:
            handle.cancel_goal_async()
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        active = self._active
        if active is None:
            return
        result = future.result().result
        self.get_logger().info(
            "Goal result %d: %s" % (result.result_code, result.message)
        )
        self._finish_goal(result.result_code)

    def _finish_goal(self, code: int):
        active = self._active
        candidate = active["candidate"]
        x = candidate.pose.pose.position.x
        y = candidate.pose.pose.position.y
        decision = decide(code, cancelled_by_explorer=active["cancel"])
        self._active = None
        self._publish_current_goal_marker()

        if decision.retry and active["retries"] < self.get_parameter("max_internal_retries").value:
            active["retries"] += 1
            self.get_logger().warn("Internal error; resending the goal.")
            self._send_goal(candidate)
            self._active["retries"] = active["retries"]
            return
        if decision.mark_visited or decision.retry:
            self._selector.mark_visited(x, y)
        if decision.blacklist:
            self._blacklist.add(x, y, "result_%d" % code)
            self._publish_blacklist_markers()

        if self._state == IDLE:     # stopped while the goal was running
            return
        self._state = SELECTING
        self._next_select_time = self._now() + self.get_parameter("settle_time").value

    def _cancel_active(self):
        active = self._active
        if active is None:
            return
        active["cancel"] = True
        if active["handle"] is not None:
            active["handle"].cancel_goal_async()

    def _revalidate_active_goal(self):
        """Cancel the goal if its frontier disappeared after a map update."""
        active = self._active
        now = self._now()
        period = self.get_parameter("validity_check_period").value
        if active is None or active["cancel"] or self._map is None:
            return
        if now - self._last_validity_check < period:
            return
        self._last_validity_check = now
        pose = active["candidate"].pose.pose
        radius = self.get_parameter("frontier_gone_radius").value
        if not frontier_near(self._map, pose.position.x, pose.position.y, radius):
            self.get_logger().info("Frontier at the goal is gone; cancelling.")
            self._cancel_active()

    # ---- RViz ------------------------------------------------------------------

    def _publish_markers(self, clusters, candidates):
        if self._map is None:
            return
        self._latest_clusters = clusters
        self._latest_candidates = candidates
        stamp = self.get_clock().now().to_msg()
        frame = self.get_parameter("map_frame").value
        array = MarkerArray()

        self._frontiers.clear()
        frontier = Marker()
        frontier.header.frame_id = frame
        frontier.header.stamp = stamp
        frontier.ns = "frontier_cells"
        frontier.type = Marker.SPHERE_LIST
        frontier.scale.x = frontier.scale.y = frontier.scale.z = 0.1
        frontier.color.r = 1.0
        frontier.color.g = 0.5
        frontier.color.a = 1.0
        display_radius = self.get_parameter("frontier_display_radius").value
        display_radius_squared = display_radius * display_radius
        display_points = []
        display_buckets = {}
        for cluster in clusters:
            for cx, cy in cluster:
                x, y = cell_to_world(self._map, cx, cy)
                bucket_x = math.floor(x / display_radius)
                bucket_y = math.floor(y / display_radius)
                nearby = False
                for neighbour_x in range(bucket_x - 1, bucket_x + 2):
                    for neighbour_y in range(bucket_y - 1, bucket_y + 2):
                        for selected_x, selected_y in display_buckets.get(
                            (neighbour_x, neighbour_y), []
                        ):
                            dx = x - selected_x
                            dy = y - selected_y
                            if dx * dx + dy * dy < display_radius_squared:
                                nearby = True
                                break
                        if nearby:
                            break
                    if nearby:
                        break
                if nearby:
                    continue
                display_buckets.setdefault((bucket_x, bucket_y), []).append((x, y))
                display_points.append((x, y))

        for x, y in display_points:
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = 0.05
            self._frontiers[len(self._frontiers)] = pose
            frontier.points.append(pose.position)
        self._frontier_coords_pub.publish_coordinates()
        array.markers.append(frontier)

        self._publish_blacklist_markers()
        blocked = Marker()
        blocked.header.frame_id = frame
        blocked.header.stamp = stamp
        blocked.ns = "blacklist"
        blocked.type = Marker.SPHERE_LIST
        blocked.scale.x = blocked.scale.y = blocked.scale.z = 0.3
        blocked.color.a = 1.0
        for entry in self._blacklist.entries():
            blocked.points.append(Point(x=entry.x, y=entry.y, z=0.1))
        array.markers.append(blocked)

        for candidate in candidates:
            goal = Marker()
            goal.header.frame_id = frame
            goal.header.stamp = stamp
            goal.ns = "next_goal"
            goal.type = Marker.SPHERE_LIST
            goal.scale.x = goal.scale.y = goal.scale.z = 0.2
            goal.color.g, goal.color.a = 1.0, 1.0
            goal.points.append(candidate.pose.pose.position)
            array.markers.append(goal)
        self._marker_pub.publish(array)

    def _republish_markers(self):
        if self._map is not None:
            self._publish_markers(self._latest_clusters, self._latest_candidates)
        self._goal_coords_pub.publish_coordinates()
        self._publish_current_goal_marker()

    def _publish_current_goal_marker(self):
        marker = Marker()
        marker.header.frame_id = self.get_parameter("map_frame").value
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "current goal"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD if self._active is not None else Marker.DELETE
        marker.scale.x = marker.scale.y = marker.scale.z = 0.3
        marker.color.g = 1.0
        marker.color.a = 1.0
        if self._active is not None:
            marker.points.append(self._active["candidate"].pose.pose.position)
        self._current_goal_pub.publish(marker)

    def _publish_blacklist_markers(self):
        self._blacklisted_goals.clear()
        for index, entry in enumerate(self._blacklist.entries()):
            pose = Pose()
            pose.position.x = entry.x
            pose.position.y = entry.y
            pose.position.z = 0.1
            self._blacklisted_goals[index] = pose
        self._blacklist_coords_pub.publish_coordinates()


def main(args=None):
    """Run the frontier explorer node."""
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
