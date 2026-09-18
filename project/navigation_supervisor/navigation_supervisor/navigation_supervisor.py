"""Core navigation supervision using Nav2 Simple Commander."""

import copy
import math
from typing import Optional

import rclpy

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from navigation_interfaces.msg import FailedRegion, FailedRegionArray

from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from .failed_region_manager import FailedRegionManager
from .navigation_types import (
    FailureReason,
    NavigationOutcome,
    NavigationState,
)


class NavigationSupervisor(BasicNavigator):
    """Supervise exectuion of externally supplied navigation goals."""

    def __init__(self) -> None:
        """Initialise the Navigation Supervisor."""
        super().__init__(node_name="navigation_supervisor")

        self._state = NavigationState.IDLE

        self._pending_goal: Optional[PoseStamped] = None
        self._current_goal: Optional[PoseStamped] = None

        self._last_outcome: Optional[NavigationOutcome] = None
        self._cancel_requested = False

        self.declare_parameter("global_frame", "map")
        self._global_frame = (
            self.get_parameter("global_frame")
            .get_parameter_value()
            .string_value
        )

        self.declare_parameter("goal_topic", "/navigation_goal")
        self._goal_topic = (
            self.get_parameter("goal_topic").get_parameter_value().string_value
        )

        self._goal_subscription = self.create_subscription(
            PoseStamped,
            self._goal_topic,
            self._goal_callback,
            10,
        )

        self.declare_parameter("failed_region_radius", 0.30)
        self.declare_parameter("failed_regions_topic", "/failed_regions")

        failed_region_radius = (
            self.get_parameter("failed_region_radius")
            .get_parameter_value()
            .double_value
        )

        self._failed_regions_topic = (
            self.get_parameter("failed_regions_topic")
            .get_parameter_value()
            .string_value
        )

        self._failed_region_manager = FailedRegionManager(
            radius=failed_region_radius,
        )

        failed_regions_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._failed_regions_publisher = self.create_publisher(
            FailedRegionArray,
            self._failed_regions_topic,
            failed_regions_qos,
        )

        self.get_logger().info(
            f"Navigation Supervisor initialised. "
            f"Listening for goals on '{self._goal_topic}'."
        )

        self._publish_failed_regions()

    @property
    def state(self) -> NavigationState:
        """Return the current navigation state."""
        return self._state

    @property
    def last_outcome(self) -> Optional[NavigationOutcome]:
        """Return the most recently completed navigation outcome."""
        return self._last_outcome

    def submit_goal(self, goal: PoseStamped) -> bool:
        """
        Validate and queue a goal for navigation.

        The supervisor does not depend on where the goal comes from

        Returns True if the goal was accepted by the supervisor.
        """
        if (
            self._state != NavigationState.IDLE
            or self._pending_goal is not None
        ):
            self.get_logger().warn(
                "Ignoring goal because navigation is already busy."
            )
            return False

        self._transition_to(NavigationState.VALIDATING)

        validated_goal = self._validate_goal(goal)

        if validated_goal is None:
            self._last_outcome = NavigationOutcome.INVALID_GOAL
            self._transition_to(NavigationState.IDLE)
            return False

        blocked_region = self._failed_region_manager.find_blocking_region(
            validated_goal.pose.position.x,
            validated_goal.pose.position.y,
        )

        if blocked_region is not None:
            self.get_logger().warn(
                "Rejected goal because it lies inside failed region "
                f"{blocked_region.region_id}."
            )
            self._last_outcome = NavigationOutcome.BLACKLISTED
            self._transition_to(NavigationState.IDLE)
            return False

        self._pending_goal = validated_goal
        self._last_outcome = None

        self.get_logger().info(
            "Accepted navigation goal at "
            f"({validated_goal.pose.position.x:.2f}, "
            f"{validated_goal.pose.position.y:.2f})."
        )

        self._transition_to(NavigationState.IDLE)
        return True

    def update(self) -> Optional[NavigationOutcome]:
        """
        Advance navigation supervision.

        Returns a NavigationOutcome only when an attempt finishes.
        """
        if (
            self._state == NavigationState.IDLE
            and self._pending_goal is not None
        ):
            return self._start_pending_goal()

        if self._state not in (
            NavigationState.NAVIGATING,
            NavigationState.CANCELING,
        ):
            return None

        try:
            if not self.isTaskComplete():
                return None

            result = self.getResult()

        except Exception as exc:
            self.get_logger().error(
                f"Error checking Nav2 navigation result: {exc}"
            )
            return self._finish_navigation(NavigationOutcome.INTERNAL_ERROR)

        if self._cancel_requested:
            outcome = NavigationOutcome.CANCELED

        elif result == TaskResult.SUCCEEDED:
            outcome = NavigationOutcome.SUCCEEDED

        elif result == TaskResult.CANCELED:
            outcome = NavigationOutcome.CANCELED

        elif result == TaskResult.FAILED:
            self._record_current_goal_failure()
            outcome = NavigationOutcome.NAVIGATION_FAILED

        else:
            outcome = NavigationOutcome.INTERNAL_ERROR

        return self._finish_navigation(outcome)

    def cancel_navigation(self) -> bool:
        """Cancel the current or pending navigation request."""
        if (
            self._state == NavigationState.IDLE
            and self._pending_goal is not None
        ):
            self._pending_goal = None
            self._last_outcome = NavigationOutcome.CANCELED
            self.get_logger().info("Pending navigation goal canceled.")
            return True

        if self._state != NavigationState.NAVIGATING:
            return False

        self._cancel_requested = True
        self._transition_to(NavigationState.CANCELING)

        try:
            self.cancelTask()
        except Exception as exc:
            self.get_logger().error(
                f"Failed to request Nav2 cancellation: {exc}"
            )
            self._finish_navigation(NavigationOutcome.INTERNAL_ERROR)
            return False

        return True

    def _start_pending_goal(self) -> Optional[NavigationOutcome]:
        """Send the pending goal to Nav2."""
        goal = self._pending_goal

        if goal is None:
            return None

        self._pending_goal = None
        self._current_goal = goal
        self._cancel_requested = False

        # Refresh the timestamp immediately before sending the pose.
        goal.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info(
            "Sending goal to Nav2: "
            f"({goal.pose.position.x:.2f}, "
            f"{goal.pose.position.y:.2f})."
        )

        try:
            accepted = self.goToPose(goal)

        except Exception as exc:
            self.get_logger().error(f"Failed to send goal to Nav2: {exc}")
            return self._finish_navigation(NavigationOutcome.INTERNAL_ERROR)

        if not accepted:
            self.get_logger().warn("Nav2 rejected the navigation goal.")
            return self._finish_navigation(NavigationOutcome.NAVIGATION_FAILED)

        self._transition_to(NavigationState.NAVIGATING)
        return None

    def _validate_goal(
        self,
        goal: PoseStamped,
    ) -> Optional[PoseStamped]:
        """Validate a goal and return a normalised copy."""
        if not isinstance(goal, PoseStamped):
            self.get_logger().warn("Navigation goal is not a PoseStamped.")
            return None

        if goal.header.frame_id != self._global_frame:
            self.get_logger().warn(
                f"Goal frame '{goal.header.frame_id}' does not match "
                f"configured global frame '{self._global_frame}',"
            )
            return None

        position = goal.pose.position
        orientation = goal.pose.orientation

        values = (
            position.x,
            position.y,
            position.z,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )

        if not all(math.isfinite(value) for value in values):
            self.get_logger().warn(
                "Navigation goal contains non-finite values."
            )
            return None

        quaternion_norm = math.sqrt(
            orientation.x * orientation.x
            + orientation.y * orientation.y
            + orientation.z * orientation.z
            + orientation.w * orientation.w
        )

        if quaternion_norm < 1.0e-6:
            self.get_logger().warn(
                "Navigation goal has an invalid zero quaternion."
            )
            return None

        validated_goal = copy.deepcopy(goal)
        validated_orientation = validated_goal.pose.orientation

        validated_orientation.x /= quaternion_norm
        validated_orientation.y /= quaternion_norm
        validated_orientation.z /= quaternion_norm
        validated_orientation.w /= quaternion_norm

        return validated_goal

    def _finish_navigation(
        self,
        outcome: NavigationOutcome,
    ) -> NavigationOutcome:
        """Finish the current navigation attempt and return to idle."""
        self._last_outcome = outcome

        if outcome == NavigationOutcome.SUCCEEDED:
            self.get_logger().info("Navigation succeeded.")

        elif outcome == NavigationOutcome.CANCELED:
            self.get_logger().info("Navigation canceled.")

        else:
            self.get_logger().warn(
                f"Navigation finished with outcome: {outcome.name}."
            )

        self._current_goal = None
        self._cancel_requested = False

        self._transition_to(NavigationState.IDLE)

        return outcome

    def _transition_to(
        self,
        new_state: NavigationState,
    ) -> None:
        """Transition to a new state and log the change."""
        if new_state == self._state:
            return

        self.get_logger().debug(
            f"Navigation state: " f"{self._state.name} -> {new_state.name}"
        )

        self._state = new_state

    def _goal_callback(self, msg: PoseStamped) -> None:
        """Submit an incoming goal pose for navigation."""
        self.submit_goal(msg)

    def _record_current_goal_failure(self) -> None:
        """Record the current goal as a failed navigation region."""
        if self._current_goal is None:
            self.get_logger().warn(
                "Navigation failed but no current goal was available."
            )
            return

        position = self._current_goal.pose.position

        now_seconds = self.get_clock().now().nanoseconds / 1_000_000_000.0

        region = self._failed_region_manager.record_failure(
            x=position.x,
            y=position.y,
            failure_reason=FailureReason.NAV_ABORTED,
            timestamp=now_seconds,
        )

        self.get_logger().warn(
            "Recorded failed navigation region "
            f"{region.region_id} at "
            f"({region.center_x:.2f}, {region.center_y:.2f})."
        )

        self._publish_failed_regions()

    def _publish_failed_regions(self) -> None:
        """Publish the complete current failed-region blacklist."""
        msg = FailedRegionArray()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._global_frame

        for record in self._failed_region_manager.get_regions():
            region_msg = FailedRegion()

            region_msg.region_id = record.region_id

            region_msg.center.x = record.center_x
            region_msg.center.y = record.center_y
            region_msg.center.z = 0.0

            region_msg.radius = record.radius
            region_msg.failure_count = record.failure_count

            if record.last_failure_reason == FailureReason.NAV_ABORTED:
                region_msg.last_failure_reason = FailedRegion.NAVIGATION_FAILED
            elif record.last_failure_reason == FailureReason.NO_PROGRESS:
                region_msg.last_failure_reason = FailedRegion.STUCK
            elif record.last_failure_reason == FailureReason.NAV_TIMEOUT:
                region_msg.last_failure_reason = FailedRegion.NAVIGATION_TIMEOUT
            else:
                region_msg.last_failure_reason = FailedRegion.PLANNING_FAILED

            first_seconds = int(record.first_failure_time)
            first_nanoseconds = int(
                (record.first_failure_time - first_seconds) * 1_000_000_000
            )

            region_msg.first_failure_time.sec = first_seconds
            region_msg.first_failure_time.nanosec = first_nanoseconds

            last_seconds = int(record.last_failure_time)
            last_nanoseconds = int(
                (record.last_failure_time - last_seconds) * 1_000_000_000
            )

            region_msg.last_failure_time.sec = last_seconds
            region_msg.last_failure_time.nanosec = last_nanoseconds

            msg.regions.append(region_msg)

        self._failed_regions_publisher.publish(msg)


def main(args=None) -> None:
    """Run the Navigation Supervisor node."""
    rclpy.init(args=args)

    supervisor = NavigationSupervisor()

    try:
        while rclpy.ok():
            # Process incoming ROS events first.
            rclpy.spin_once(
                supervisor,
                timeout_sec=0.1,
            )

            # Then allow the supervisor to interact with Simple Commander
            # outside a ROS subscription callback.
            supervisor.update()

    except KeyboardInterrupt:
        pass

    finally:
        supervisor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
