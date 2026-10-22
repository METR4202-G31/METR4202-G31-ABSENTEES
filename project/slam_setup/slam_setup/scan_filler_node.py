"""
scan_filler_node: republish /scan with "no hit" beams filled in.

Subscribes : /scan         (sensor_msgs/LaserScan)  raw LiDAR
Publishes  : /scan_filled  (sensor_msgs/LaserScan)  same scan, no-hit beams
                                                    set to range_max - margin

Only slam_toolbox listens to /scan_filled. Nav2's costmaps still use the raw
/scan, so obstacle avoidance is not affected by this node at all.

IMPORTANT: slam_toolbox's `max_laser_range` must be smaller than
range_max - margin (we use 3.0 m vs 3.49 m), otherwise SLAM would draw a fake
wall at the end of every filled beam. See config/slam_toolbox_params.yaml.

Parameters
----------
enabled       (bool,  true)          false = pass scans through unchanged.
                                     Can be changed while running:
                                     ros2 param set /scan_filler enabled false
margin        (float, 0.01)          metres below range_max for filled beams
input_topic   (str,   /scan)
output_topic  (str,   /scan_filled)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from .fill import fill_no_return


class ScanFiller(Node):
    """Republish LaserScans with no-return beams filled for SLAM."""

    def __init__(self):
        super().__init__('scan_filler')

        # Parameters: values that can be changed from a launch file or the
        # command line without editing code.
        self.declare_parameter('enabled', True)
        self.declare_parameter('margin', 0.01)
        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_filled')

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value

        # Subscribe with the "sensor data" QoS (best effort): this matches
        # whatever QoS the LiDAR driver or Gazebo uses.
        self.create_subscription(
            LaserScan, in_topic, self._on_scan, qos_profile_sensor_data)

        # Publish RELIABLE: a reliable publisher can talk to both reliable and
        # best-effort subscribers, so slam_toolbox will receive it either way.
        self._pub = self.create_publisher(
            LaserScan, out_topic,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE))

        # Statistics, logged every 5 s so you can see the node working.
        self._scans = 0
        self._beams = 0
        self._filled = 0
        self.create_timer(5.0, self._log_stats)

        self.get_logger().info(f'Scan filler: {in_topic} -> {out_topic}')

    def _on_scan(self, msg: LaserScan):
        """Run once for every incoming scan (about 5 times per second)."""
        if self.get_parameter('enabled').value:
            margin = self.get_parameter('margin').value
            new_ranges, count = fill_no_return(msg.ranges, msg.range_max, margin)
            msg.ranges = new_ranges
        else:
            count = 0

        # The header (frame and timestamp) is left untouched, so SLAM can
        # still look up where the robot was when this scan was taken.
        self._pub.publish(msg)

        self._scans += 1
        self._beams += len(msg.ranges)
        self._filled += count

    def _log_stats(self):
        if self._scans == 0:
            self.get_logger().warn('No scans received yet - is /scan publishing?')
            return
        pct = 100.0 * self._filled / max(self._beams, 1)
        self.get_logger().info(
            f'{self._scans} scans in last 5 s, {pct:.0f}% of beams were no-hit '
            f'and filled (enabled={self.get_parameter("enabled").value})')
        self._scans = self._beams = self._filled = 0


def main(args=None):
    """Start the node and keep it running until Ctrl+C."""
    rclpy.init(args=args)
    node = ScanFiller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()