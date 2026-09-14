
import rclpy
from rclpy.node import Node
import numpy as np
from aruco_detector_interfaces.msg import ArucoVerdict
from geometry_msgs.msg import Pose, Point, Quaternion
from rclpy.duration import Duration
from .CoordinatePublisher import CoordinatePublisher

import tf2_ros
import tf2_geometry_msgs 
from geometry_msgs.msg import PoseStamped

class MissionControl(Node):

    def __init__(self):
        super().__init__('mission_control')
        
        #dict of marker locations. key is id, value is (PoseStamped, distance) where distance is 
        # the closest distance the image was taken 
        self.markers = {} 
        
        #publisher for publishing the detected markers
        self.coords_pub = CoordinatePublisher(self.markers, "detected aruco markers")
        self.coords_pub.publish_coordinates()


        self.aruco_sub = self.create_subscription(
            ArucoVerdict, 'aruco', self.update_marker_location, 10)
        
        #used for transformation
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        #needs to be communicating to frontier detection
        #needs to be communicating with navigation service

        self.debug_timer = self.create_timer(5.0, self.print_markers)
        
        self.get_logger().info('Mission Control node started.')
        
    def print_markers(self):
        if not self.markers:
            self.get_logger().info("No markers stored yet.")
            return
            
        for marker_id, (pose, dist) in self.markers.items():
            self.get_logger().info(
                f"Marker {marker_id} at Global (X: {pose.position.x:.2f}, Y: {pose.position.y:.2f}) | Distance: {dist:.2f}m"
            )

    
    def dist(self, pos: Point) -> float:
        """
        Given a Point message consisting of x, y, z coordinates, calculates the 
        magnitude of the distance as sqrt(x^2 + y^2 + z^2)
        """
        return np.sqrt(pos.x * pos.x + pos.y * pos.y + pos.z * pos.z)

    def update_marker_location(self, msg: ArucoVerdict):
        """
        Callback for when an aruco_marker is detected on camera. Updates the list of
        markers with the potentially new marker's location, keeping only the closest
        (i.e. most reliable) observation seen so far for each marker ID.
        FLAWED: if it mislabels an id then the correct location could be completely wiped.
        """

        new_dist = self.dist(msg.pose.position)
        if not msg.detected:
            return

        currently_stored_marker, old_dist = self.markers.get(msg.id, (None, float('inf')))

        if currently_stored_marker is None or new_dist < old_dist:
            global_pose = self.transform_to_global(msg.pose, msg.header)
            if global_pose is not None: #paranoid check
                self.markers[msg.id] = (global_pose, new_dist)
        
        self.coords_pub.publish_coordinates()


    def transform_to_global(self, pose, header, target_frame='map'):
        """
        Transforms a Pose (stamped with header, giving its source frame and
        detection time) into target_frame (typically 'map'). Returns None if
        the transform isn't available right now — non-blocking, so this never
        stalls the single-threaded executor.
        """
        stamped = PoseStamped()
        stamped.header = header
        stamped.pose = pose #this is a bit redundant I should probably redo ArucoVerdict

        try:
            transformed = self.tf_buffer.transform(
                stamped, target_frame, timeout=Duration(seconds=1) #timeout for up to 1sec
            )
            return transformed.pose
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f'Transform failed: {e}')
            return None


def main(args=None):
    rclpy.init(args=args)
    node = MissionControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()