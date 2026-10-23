from rclpy.node import Node
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Pose


class CoordinatePublisher:
    """
    Publishes a dictionary of Poses as a single RViz POINTS marker.

    The dictionary is retained by reference so callers can update the set of
    coordinates and publish the new contents without recreating the publisher.
    """

    def __init__(
        self,
        node: Node,
        coords_dict: dict[int, Pose],
        topic: str,
        namespace: str,
        frame_id: str = "map",
        marker_id: int = 0,
        marker_type: int = Marker.POINTS,
        scale_x: float = 0.2,
        scale_y: float = 0.2,
        color: ColorRGBA = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),
    ):
        self._node = node
        self._coords_dict = coords_dict
        self.publisher = node.create_publisher(Marker, topic, 10)

        self._marker = Marker()
        self._marker.header.frame_id = frame_id
        self._marker.ns = namespace
        self._marker.id = marker_id
        self._marker.type = marker_type
        self._marker.action = Marker.ADD
        self._marker.scale.x = scale_x
        self._marker.scale.y = scale_y
        self._marker.color = color

    def publish_coordinates(self):
        self._marker.header.stamp = self._node.get_clock().now().to_msg()
        self._marker.points.clear()

        for pose in self._coords_dict.values():
            self._marker.points.append(pose.position)

        self.publisher.publish(self._marker)
