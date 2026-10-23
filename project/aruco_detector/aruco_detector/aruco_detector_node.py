import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from aruco_detector_interfaces.msg import ArucoVerdict


class ArucoDetector(Node):

    def __init__(self):
        super().__init__('aruco_detector')

        #CvBridge allows for easy conversion between camera msg format into raw arrays for opencv
        self.bridge = CvBridge() 
        
        #cv2 aruco detection setup
        self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
        self.arucoParams = cv2.aruco.DetectorParameters_create()

        #camera calibration parameters which will be read live
        self.camera_matrix = None
        self.dist_coeffs = None
        self.marker_length = 0.1  # metres — set to your actual marker size
        
        #variables to ensure aruco msgs only published every 0.1 seconds
        self.debounce_period = 0.1  # seconds
        self.last_process_time = self.get_clock().now()

        
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.analyse_image_callback, 10)

        self.camera_info_sub = self.create_subscription( #used for camera calibration
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)

        self.publisher_ = self.create_publisher(ArucoVerdict, '/aruco', 10)

        self.get_logger().info('Aruco detector node started.')

    """
    Callback for /camera/camera_info message. Reads into current camera calibration information.
    """
    def camera_info_callback(self, msg: CameraInfo):
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)

    """
    Callback for /camera/image_raw message. Runs aruco detection on current camera information.
    """
    def analyse_image_callback(self, msg: Image):
        now = self.get_clock().now()
        elapsed = (now - self.last_process_time).nanoseconds / 1e9  # convert ns -> seconds

        if elapsed < self.debounce_period:
            return  # too soon since last processed frame — skip this one
        self.last_process_time = now
            
        #---------------------------------------------------#
        if self.camera_matrix is None:
            self.get_logger().warn('No camera_info received yet, skipping frame.')
            return

        #turn camera message into array for processing
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

        #run aruco detection
        (corners, ids, rejected) = cv2.aruco.detectMarkers(
            cv_image, self.arucoDict, parameters=self.arucoParams)

        if ids is None:
            # nothing detected this frame — publish nothing
            return

        #run areco pose estimation
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_length, self.camera_matrix, self.dist_coeffs)

        #publish one ArucoVerdict() message to /aruco per marker detected
        for i in range(len(ids)):
            aruco_message = ArucoVerdict()
            aruco_message.header = msg.header
            aruco_message.detected = True
            aruco_message.id = int(ids[i][0])
            aruco_message.pose = self.build_pose(tvecs[i][0], rvecs[i][0])

            self.publisher_.publish(aruco_message)
    
    #turns the pose information into a quarternion for geometry_msgs/Pose format
    def build_pose(self, tvec, rvec) -> Pose:
        pose = Pose()

        pose.position.x = float(tvec[0]) * 2.26 # arbitrary multiplication that performed better in tests
        pose.position.y = float(tvec[1]) * 2.26
        pose.position.z = float(tvec[2]) * 2.26

        rot = Rotation.from_rotvec(rvec)
        quat = rot.as_quat()  # returns [x, y, z, w]

        pose.orientation.x = float(quat[0])
        pose.orientation.y = float(quat[1])
        pose.orientation.z = float(quat[2])
        pose.orientation.w = float(quat[3])

        return pose


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()