import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Pose


class CoordinatePublisher(Node):
    def __init__(self, coords_dict: dict[int, tuple[Pose, float]], coords_name: str):
        super().__init__('coordinate_publisher')
        # Create a publisher for the marker topic
        self.publisher = self.create_publisher(Marker, 'marker', 10)
        
        # Your dictionary of coordinates
        self.coords_dict = coords_dict
        self.coords_name = coords_name

    def publish_coordinates(self):
        """
        Publishes all the coordinates in self.coords_dict to the topic /marker 
        """
        marker = Marker()
        marker.header.frame_id = "map"  #frame our coordinates our with respect to
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = self.coords_name
        marker.id = 0
        
        # POINTS type displays a sphere at each coordinate
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        
        # Scale of the points
        marker.scale.x = 0.3  # Width of point
        marker.scale.y = 0.3  # Height of point
        
        # Color of the points (RGBA)
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0) # Red

        # Loop through dictionary and add to marker points
        for id, pose in self.coords_dict.items():
            
            p = pose[0].position #the Point object corresponding to this marker
            marker.points.append(p)
            
        self.publisher.publish(marker)
