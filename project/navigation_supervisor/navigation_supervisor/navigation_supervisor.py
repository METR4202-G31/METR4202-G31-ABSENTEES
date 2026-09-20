"""Core navigation supervision using Nav2 Simple Commander."""

import copy
import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from navigation_interfaces.action import ManagedNavigate
from navigation_interfaces.msg import FailedRegion, FailedRegionArray
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.task import Future

from .failed_region_manager import FailedRegionManager
from .navigation_types import (
    FailureReason,
    NavigationOutcome,
    NavigationState,
)


class NavigationSupervisor(BasicNavigator):
    """Supervise execution of externally supplied navigation goals."""

    def __init__(self) -> None:
        """Initialise the Navigation Supervisor."""
        # Initialise BasicNavigator so supervisor can use Nav2 Simple Commander.
        super().__init__(node_name="navigation_supervisor")

        # Start with no navigation task active.
        self._state = NavigationState.IDLE

        # Track goals waiting to start and current goals.
        self._pending_goal: Optional[PoseStamped] = None
        self._current_goal: Optional[PoseStamped] = None

        # Keep the recent result and whether Nav2 cancellation was requested.
        self._last_outcome: Optional[NavigationOutcome] = None
        self._cancel_requested = False

        # Configure the global frame
        self.declare_parameter("global_frame", "map")
        self._global_frame = (
            self.get_parameter("global_frame")
            .get_parameter_value()
            .string_value
        )

        # Set up ManagedNavigate action defaults and erver
        self._action_goal_reserved = False
        self._active_action_goal_handle = None
        self._action_result_future: Optional[Future] = None
        self._action_cancel_pending = False
        self._failed_region_recorded = False

        self.declare_parameter(
            "managed_navigate_action",
            "managed_navigate",
        )
        self._managed_navigate_action = (
            self.get_parameter("managed_navigate_action")
            .get_parameter_value()
            .string_value
        )

        # Allow action callbacks to coexist while execution waits for
        # supervisor result.
        self._action_callback_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            ManagedNavigate,
            self._managed_navigate_action,
            execute_callback=self._execute_managed_navigation,
            goal_callback=self._managed_goal_callback,
            cancel_callback=self._managed_cancel_callback,
            callback_group=self._action_callback_group,
        )

        # Configure Nav2 failures
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

        # Store and merge failed navigation locations
        self._failed_region_manager = FailedRegionManager(
            radius=failed_region_radius,
        )

        # Keep latest complete failure history for subscribers that join later.
        failed_regions_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Publish full FailedRegionArray whenthe stored failure history changes.
        self._failed_regions_publisher = self.create_publisher(
            FailedRegionArray,
            self._failed_regions_topic,
            failed_regions_qos,
        )

        self.get_logger().info(
            "Navigation Supervisor initialised. "
            f"ManagedNavigate action: '{self._managed_navigate_action}'."
        )

        # Publish an initial empty history
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

        Returns True if the goal was accepted by the supervisor.
        """
        # Reject submissions while another goal is queued or being processed.
        if (
            self._state != NavigationState.IDLE
            or self._pending_goal is not None
        ):
            self.get_logger().warn(
                "Ignoring goal because navigation is already busy."
            )
            return False

        # Validate the message contents,normalise orientation on a copied pose.
        self._transition_to(NavigationState.VALIDATING)
        validated_goal = self._validate_goal(goal)

        # Return INVALID_GOAL without sending to Nav2 when validation fails.
        if validated_goal is None:
            self._last_outcome = NavigationOutcome.INVALID_GOAL
            self._transition_to(NavigationState.IDLE)
            return False

        # Queue the validated goal, update() starts outside action callback.
        self._pending_goal = validated_goal
        self._last_outcome = None

        self.get_logger().info(
            "Accepted navigation goal at "
            f"({validated_goal.pose.position.x:.2f}, "
            f"{validated_goal.pose.position.y:.2f})."
        )

        # Return to IDLE while the validated goal waits in the pending queue.
        self._transition_to(NavigationState.IDLE)
        return True

    def update(self) -> Optional[NavigationOutcome]:
        """
        Advance navigation supervision.

        Returns a NavigationOutcome only when an attempt finishes.
        """
        # Process action cancellation here instead of calling Simple Commander
        # in the callback.
        if self._action_cancel_pending:
            self._action_cancel_pending = False

            # Pending goal cancels, while active goal enters CANCELING.
            if self.cancel_navigation():
                if self._state == NavigationState.IDLE:
                    return NavigationOutcome.CANCELED

        # Start a validated pending goal only while supervisor is idle.
        if (
            self._state == NavigationState.IDLE
            and self._pending_goal is not None
        ):
            return self._start_pending_goal()

        # Don't poll Nav2 when no active or canceling navigation task exists.
        if self._state not in (
            NavigationState.NAVIGATING,
            NavigationState.CANCELING,
        ):
            return None

        try:
            # Leave state unchanged while the asynchronous Nav2 task is active.
            if not self.isTaskComplete():
                return None

            # Get final Simple Commander result once Nav2 reports completion.
            result = self.getResult()

        except Exception as exc:
            # Convert unexpected Simple Commander errors into an internal error.
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

        # Record only final Nav2 failures as failed-region history.
        elif result == TaskResult.FAILED:
            self._failed_region_recorded = self._record_current_goal_failure()
            outcome = NavigationOutcome.NAVIGATION_FAILED

        # Treat any unknown Simple Commander result as a supervisor error.
        else:
            outcome = NavigationOutcome.INTERNAL_ERROR

        # Complete cleanup and notify the ManagedNavigate execution callback.
        return self._finish_navigation(outcome)

    def cancel_navigation(self) -> bool:
        """Cancel the current or pending navigation request."""
        # Remove a queued goal directly if it has not yet been sent to Nav2.
        if (
            self._state == NavigationState.IDLE
            and self._pending_goal is not None
        ):
            self._pending_goal = None
            self._last_outcome = NavigationOutcome.CANCELED

            self.get_logger().info("Pending navigation goal canceled.")

            # Wake ManagedNavigate executions waiting on the queued goal.
            self._complete_action_outcome(NavigationOutcome.CANCELED)

            return True

        # Reject cancellation when there is no active Nav2 navigation task.
        if self._state != NavigationState.NAVIGATING:
            return False

        self._cancel_requested = True
        self._transition_to(NavigationState.CANCELING)

        try:
            # Forward the cancellation request to Simple Commander.
            self.cancelTask()

        except Exception as exc:
            # Finish as INTERNAL_ERROR if Nav2 cancellation can't be requested.
            self.get_logger().error(
                f"Failed to request Nav2 cancellation: {exc}"
            )
            self._finish_navigation(NavigationOutcome.INTERNAL_ERROR)
            return False

        return True

    def _managed_goal_callback(
        self,
        goal_request: ManagedNavigate.Goal,
    ) -> GoalResponse:
        """Accept one ManagedNavigate goal at a time."""
        if (
            self._action_goal_reserved
            or self._state != NavigationState.IDLE
            or self._pending_goal is not None
        ):
            self.get_logger().warn(
                "Rejecting ManagedNavigate goal because navigation is busy."
            )
            return GoalResponse.REJECT

        # Reserve supervisor before the asynchronous execute callback begins.
        self._action_goal_reserved = True
        self._action_cancel_pending = False

        self.get_logger().info("Accepted ManagedNavigate action goal.")

        return GoalResponse.ACCEPT

    def _managed_cancel_callback(
        self,
        goal_handle,
    ) -> CancelResponse:
        """Accept cancellation of the active ManagedNavigate request."""
        # Reject cancellation when there is no reserved goal.
        if not self._action_goal_reserved:
            return CancelResponse.REJECT

        self.get_logger().info("ManagedNavigate cancellation requested.")

        self._action_cancel_pending = True

        return CancelResponse.ACCEPT

    async def _execute_managed_navigation(
        self,
        goal_handle,
    ) -> ManagedNavigate.Result:
        """Execute a ManagedNavigate request through the supervisor."""
        # Associate ROS action handle and result future with current request.
        self._active_action_goal_handle = goal_handle
        self._action_result_future = Future()

        self._failed_region_recorded = False

        # Finish if cancellation arrived between goal acceptance and execution.
        if goal_handle.is_cancel_requested:
            return self._finish_managed_action(
                goal_handle,
                NavigationOutcome.CANCELED,
            )

        # Validate and queue requested pose through the supervisor interface.
        accepted = self.submit_goal(goal_handle.request.target_pose)

        # Convert validation/submission failure into a ManagedNavigate result.
        if not accepted:
            outcome = self._last_outcome

            # Rejected submissions without a specific outcome treated as
            # internal errors.
            if outcome is None:
                outcome = NavigationOutcome.INTERNAL_ERROR

            return self._finish_managed_action(
                goal_handle,
                outcome,
            )

        # Wait until update() completes Nav2 attempt and resolves the future.
        outcome = await self._action_result_future

        # Translate outcome into the public action result and status.
        return self._finish_managed_action(
            goal_handle,
            outcome,
        )

    def _start_pending_goal(
        self,
    ) -> Optional[NavigationOutcome]:
        """Send the pending goal to Nav2."""
        # Copy the queued goal reference before clearing the pending slot.
        goal = self._pending_goal

        if goal is None:
            return None

        # Promote the queued goal to the current Nav2 goal before sending it.
        self._pending_goal = None
        self._current_goal = goal
        self._cancel_requested = False

        # Refresh the pose timestamp.
        goal.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info(
            "Sending goal to Nav2: "
            f"({goal.pose.position.x:.2f}, "
            f"{goal.pose.position.y:.2f})."
        )

        try:
            # Nav2 handles planning, control, and recovery behaviour.
            accepted = self.goToPose(goal)

        except Exception as exc:
            self.get_logger().error(f"Failed to send goal to Nav2: {exc}")
            return self._finish_navigation(NavigationOutcome.INTERNAL_ERROR)

        # Treat a Nav2 action rejection as a navigation failure.
        if not accepted:
            self.get_logger().warn("Nav2 rejected the navigation goal.")
            return self._finish_navigation(NavigationOutcome.NAVIGATION_FAILED)

        self._transition_to(NavigationState.NAVIGATING)
        return None

    def _finish_navigation(
        self,
        outcome: NavigationOutcome,
    ) -> NavigationOutcome:
        """Finish the current navigation attempt and return to idle."""
        # Store the outcome for diagnostics and supervisor callers.
        self._last_outcome = outcome

        if outcome == NavigationOutcome.SUCCEEDED:
            self.get_logger().info("Navigation succeeded.")

        elif outcome == NavigationOutcome.CANCELED:
            self.get_logger().info("Navigation canceled.")

        else:
            self.get_logger().warn(
                f"Navigation finished with outcome: {outcome.name}."
            )

        # Clear navigation execution state before accepting another goal.
        self._current_goal = None
        self._cancel_requested = False

        # Return the state machine to IDLE.
        self._transition_to(NavigationState.IDLE)

        # Resolve the action future.
        self._complete_action_outcome(outcome)

        return outcome

    def _complete_action_outcome(
        self,
        outcome: NavigationOutcome,
    ) -> None:
        """Wake the ManagedNavigate callback with a final outcome."""
        if (
            self._action_result_future is not None
            and not self._action_result_future.done()
        ):
            self._action_result_future.set_result(outcome)

    def _finish_managed_action(
        self,
        goal_handle,
        outcome: NavigationOutcome,
    ) -> ManagedNavigate.Result:
        """Convert an internal outcome into a ManagedNavigate result."""
        # Map outcomes to the matching ManagedNavigate result code.
        result_codes = {
            NavigationOutcome.SUCCEEDED: ManagedNavigate.Result.SUCCEEDED,
            NavigationOutcome.BLACKLISTED: ManagedNavigate.Result.BLACKLISTED,
            NavigationOutcome.PLANNING_FAILED: (
                ManagedNavigate.Result.PLANNING_FAILED
            ),
            NavigationOutcome.NAVIGATION_FAILED: (
                ManagedNavigate.Result.NAVIGATION_FAILED
            ),
            NavigationOutcome.STUCK: ManagedNavigate.Result.STUCK,
            NavigationOutcome.CANCELED: ManagedNavigate.Result.CANCELED,
            NavigationOutcome.INVALID_GOAL: ManagedNavigate.Result.INVALID_GOAL,
            NavigationOutcome.INTERNAL_ERROR: (
                ManagedNavigate.Result.INTERNAL_ERROR
            ),
        }

        # Provide a message for each navigation outcome.
        messages = {
            NavigationOutcome.SUCCEEDED: "Navigation succeeded.",
            NavigationOutcome.BLACKLISTED: "Navigation goal was blacklisted.",
            NavigationOutcome.PLANNING_FAILED: "Navigation planning failed.",
            NavigationOutcome.NAVIGATION_FAILED: "Nav2 navigation failed.",
            NavigationOutcome.STUCK: "Robot became stuck during navigation.",
            NavigationOutcome.CANCELED: "Navigation was canceled.",
            NavigationOutcome.INVALID_GOAL: "Navigation goal was invalid.",
            NavigationOutcome.INTERNAL_ERROR: (
                "Navigation Supervisor encountered an internal error."
            ),
        }

        # Populate the action result returned to the client.
        result = ManagedNavigate.Result()
        result.result_code = result_codes[outcome]
        result.message = messages[outcome]
        result.failed_region_recorded = self._failed_region_recorded

        # Mark successful navigation as a successful ROS action state.
        if outcome == NavigationOutcome.SUCCEEDED:
            goal_handle.succeed()

        # Mark cancellation separately.
        elif outcome == NavigationOutcome.CANCELED:
            goal_handle.canceled()

        # Represent all non-success, non-cancellation outcomes as aborted
        # action execution.
        else:
            goal_handle.abort()

        # Release action state so the next ManagedNavigate goal can be accepted.
        self._active_action_goal_handle = None
        self._action_result_future = None
        self._action_goal_reserved = False
        self._action_cancel_pending = False

        return result

    def _validate_goal(
        self,
        goal: PoseStamped,
    ) -> Optional[PoseStamped]:
        """Validate a goal and return a normalised copy."""
        # Require the exact ROS message type expected by Nav2 pose interface.
        if not isinstance(goal, PoseStamped):
            self.get_logger().warn("Navigation goal is not a PoseStamped.")
            return None

        # Reject goals expressed in a frame other than the configured global
        # navigation frame.
        if goal.header.frame_id != self._global_frame:
            self.get_logger().warn(
                f"Goal frame '{goal.header.frame_id}' does not match "
                f"configured global frame '{self._global_frame}'."
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

        # Reject NaN and infinite pose components.
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

        # Reject close to zero quaternions.
        if quaternion_norm < 1.0e-6:
            self.get_logger().warn(
                "Navigation goal has an invalid zero quaternion."
            )
            return None

        validated_goal = copy.deepcopy(goal)
        validated_orientation = validated_goal.pose.orientation

        # Normalise the quaternion to unit length.
        validated_orientation.x /= quaternion_norm
        validated_orientation.y /= quaternion_norm
        validated_orientation.z /= quaternion_norm
        validated_orientation.w /= quaternion_norm

        return validated_goal

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

    def _record_current_goal_failure(self) -> bool:
        """Record the current goal as a failed navigation region."""
        if self._current_goal is None:
            self.get_logger().warn(
                "Navigation failed but no current goal was available."
            )
            return False

        # Use the requested goal position as centre of the failed region.
        position = self._current_goal.pose.position

        # Convert to floating point seconds.
        now_seconds = self.get_clock().now().nanoseconds / 1_000_000_000.0

        # Record generic Nav2 abortion.
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

        # Republish the complete history after every stored or merged failure.
        self._publish_failed_regions()

        return True

    def _publish_failed_regions(self) -> None:
        """Publish the complete current failed-region history."""
        msg = FailedRegionArray()

        # Timestamp and identify the frame used by all stored region centres.
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._global_frame

        # Convert each internal FailedRegionRecord into the public ROS message.
        for record in self._failed_region_manager.get_regions():
            region_msg = FailedRegion()

            # Preserve the manager assigned region identifier.
            region_msg.region_id = record.region_id

            # Store region centres in the configured 2D global navigation frame.
            region_msg.center.x = record.center_x
            region_msg.center.y = record.center_y
            region_msg.center.z = 0.0

            # Include both the region extent and number of failures merged.
            region_msg.radius = record.radius
            region_msg.failure_count = record.failure_count

            # Translate failures to the FailedRegion message constants.
            if record.last_failure_reason == FailureReason.NAV_ABORTED:
                region_msg.last_failure_reason = FailedRegion.NAVIGATION_FAILED

            elif record.last_failure_reason == FailureReason.NO_PROGRESS:
                region_msg.last_failure_reason = FailedRegion.STUCK

            elif record.last_failure_reason == FailureReason.NAV_TIMEOUT:
                region_msg.last_failure_reason = FailedRegion.NAVIGATION_TIMEOUT

            else:
                region_msg.last_failure_reason = FailedRegion.PLANNING_FAILED

            # Split floating-point timestamp into ROS seconds and nanoseconds.
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

            # Add the converted region to the complete history snapshot.
            msg.regions.append(region_msg)

        # Publish one complete array.
        self._failed_regions_publisher.publish(msg)

    def destroy_node(self) -> None:
        """Destroy the action server and Navigation Supervisor."""
        self._action_server.destroy()

        super().destroy_node()


def main(args=None) -> None:
    """Run the Navigation Supervisor node."""
    rclpy.init(args=args)

    supervisor = NavigationSupervisor()

    try:
        while rclpy.ok():
            # Process one batch of callbacks before performing any Simple
            # Commander operations.
            rclpy.spin_once(
                supervisor,
                timeout_sec=0.1,
            )

            supervisor.update()

    except KeyboardInterrupt:
        pass

    finally:
        supervisor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
