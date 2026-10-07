
import rclpy
from rclpy.node import Node
import numpy as np
from aruco_detector_interfaces.msg import ArucoVerdict
from geometry_msgs.msg import Point, Pose
from rclpy.duration import Duration
from .CoordinatePublisher import CoordinatePublisher

import tf2_ros
import tf2_geometry_msgs 
from geometry_msgs.msg import PoseStamped

class MissionControl(Node):

    def __init__(self):
        super().__init__('mission_control')
        
        #dict of marker locations. key is id, value is Pose
        self._markers: dict[int, Pose] = {}

        #private bookkeeping: cumulative weight seen so far per marker id, used to
        #compute the running average in update_marker_location. Not exposed outside
        #this class.
        self._marker_weights: dict[int, float] = {}
        
        #publisher for publishing the detected markers
        self._coords_pub = CoordinatePublisher(node=self, coords_dict=self._markers, topic = "marker", namespace="detected aruco markers")

        #to obtain the aruco information observable from the camera
        self._aruco_sub = self.create_subscription(
            ArucoVerdict, 'aruco', self.update_marker_location, 20)
        
        #allows for transformation between camera frame and global map frame
        self._tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=2)) #each transformation is stored for 2s after its creation to account for lag
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        #prints all detected markers every 5 seconds
        self._debug_timer = self.create_timer(5.0, self.print_markers)
        
        self.get_logger().info('Mission Control node started.')
        
    def print_markers(self):
        """
        Prints information about all stored aruco markers.
        """
        if not self._markers:
            self.get_logger().info("No markers stored yet.")
            return
            
        for marker_id, pose in self._markers.items():
            self.get_logger().info(
            f"Marker {marker_id} at Global (X: {pose.position.x:.2f}, Y: {pose.position.y:.2f})")

    
    def __dist(self, pos: Point) -> float:
        """
        Given a Point message consisting of x, y, z coordinates, calculates the 
        magnitude of the distance as sqrt(x^2 + y^2 + z^2)
        """
        return np.sqrt(pos.x * pos.x + pos.y * pos.y + pos.z * pos.z)

    def update_marker_location(self, msg: ArucoVerdict):
        """
        Callback for when an aruco_marker is detected on camera. Updates the
        marker's stored pose as a running cumulative weighted average of every 
        detection seen so far, weighted so that closer (more reliable) 
        detections contribute more strongly.
        """
        if not msg.detected:
            return

        new_dist = self.__dist(msg.pose.position)
        global_pose = self.transform_to_global(msg.pose, msg.header)
        if global_pose is None:  #check for error
            return

        #gentle inverse-distance weighting: closer detections count more, but
        #even far ones still contribute a little rather than being ignored
        weight = 1.0 / (1.0 + new_dist)

        old_pose = self._markers.get(msg.id)
        old_weight = self._marker_weights.get(msg.id, 0.0)

        if old_pose is None:
            self._markers[msg.id] = global_pose
            self._marker_weights[msg.id] = weight
        else:
            total_weight = old_weight + weight
            blend = weight / total_weight  #how much the new reading shifts the average

            avg_pose = Pose()
            avg_pose.position.x = old_pose.position.x + blend * (global_pose.position.x - old_pose.position.x)
            avg_pose.position.y = old_pose.position.y + blend * (global_pose.position.y - old_pose.position.y)
            avg_pose.position.z = old_pose.position.z + blend * (global_pose.position.z - old_pose.position.z)
            avg_pose.orientation = global_pose.orientation  #orientation not averaged, just kept from latest reading

            self._markers[msg.id] = avg_pose
            self._marker_weights[msg.id] = total_weight

        self._coords_pub.publish_coordinates()


    def transform_to_global(self, pose, header, target_frame='map') -> Pose | None:
        """
        Transforms a Pose (stamped with header, giving its source frame and
        detection time) into target_frame (typically 'map'). Returns None if
        the transform failed for some reason.
        """ 
        stamped = PoseStamped()
        stamped.header = header
        stamped.pose = pose 

        try:
            #convert Pose from camera frame into global map frame
            transformed = self._tf_buffer.transform(
                stamped, target_frame, timeout=Duration(nanoseconds = 10000000) #timeout for up to 10ms
            )
            return transformed.pose              #type: ignore
        #if error occured, display error
        except tf2_ros.TransformException as e:  # type: ignore
            self.get_logger().warn(f'Aruco pose transform failed: {e}')
            return None


def main(args=None):
    rclpy.init(args=args)
    node = MissionControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()