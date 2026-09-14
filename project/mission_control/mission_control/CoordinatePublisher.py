import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Pose


class CoordinatePublisher():
    """
    Publishes a dictionary of Poses as a single RViz Marker (POINTS-type point
    cloud, by default) on a given topic, refreshing the marker in place on
    every call to publish_coordinates().

    It does not create its own ROS 2 node — it attaches
    its publisher to an existing node passed in by the caller, so it can be
    used inside any node without spinning a second one.

    Because the coords_dict reference is stored (not copied), any external
    mutation of the dict by the caller will be reflected the next time
    publish_coordinates() is called — there is no need to re-construct or
    re-assign the dict for updates to take effect.

    Args:
        node: The ROS 2 node to attach the publisher to. Its clock is used
            to timestamp the marker header.
        coords_dict: Dictionary mapping an arbitrary integer key to a Pose.
            Only the .position of each Pose is used; orientation is ignored,
            as it has no meaning for a POINTS-type marker.
        topic: Name of the topic to publish the Marker on.
        namespace: Marker namespace (Marker.ns). Used by RViz to group and
            distinguish this marker set from others published on the same
            topic.
        frame_id: The coordinate frame all poses in coords_dict are assumed
            to already be expressed in (e.g. "map"). This class does not
            perform any transforms — poses must already be in this frame.
        marker_id: The Marker's id within its namespace. Kept fixed across
            publishes so that RViz treats each publish as an update to the
            same marker rather than creating a new one.
        marker_type: Marker shape/type (e.g. Marker.POINTS, Marker.SPHERE_LIST,
            Marker.CUBE_LIST). Defaults to Marker.POINTS.
        scale_x: Marker scale.x. For POINTS/SPHERE_LIST/CUBE_LIST types this
            is the width of each point/sphere/cube in metres.
        scale_y: Marker scale.y. For POINTS this is the height of each point;
            unused by some marker types.
        color: A single ColorRGBA applied uniformly to every point.
        queue_size: Publisher QoS history depth.

    Example:
        marker_pub = CoordinatePublisher(
            node=self,
            coords_dict=self._markers,
            topic='marker',
            namespace='detected aruco markers',
            frame_id='map',
            color=ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),
        )
        marker_pub.publish_coordinates()
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

        # marker details
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
        """
        Publishes the current contents of coords_dict to the configured
        topic as a single Marker, replacing whatever points were published
        previously (points are cleared and rebuilt on every call).
        """
        self._marker.header.stamp = self._node.get_clock().now().to_msg()
        self._marker.points.clear()  # type:ignore

        # Loop through dictionary and add to marker points
        for id, pose in self._coords_dict.items():
            p = pose.position  # the Point object corresponding to this marker
            self._marker.points.append(p)  # type:ignore

        self.publisher.publish(self._marker)