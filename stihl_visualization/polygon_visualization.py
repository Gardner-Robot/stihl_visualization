#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header
import yaml
from pyproj import Transformer
from builtin_interfaces.msg import Duration
import os
from ament_index_python.packages import get_package_share_directory
import tf_transformations

class PolygonVisualizationNode(Node):
    def __init__(self):
        super().__init__('polygon_marker_node')

        yaml_file = get_package_share_directory('stihl_visualization') + '/config/stihl.yaml'
        self.mesh_path = 'package://stihl_visualization/meshes/stihl.stl'

        # Load coordinates from YAML
        if not os.path.exists(yaml_file):
            self.get_logger().error(f"YAML file not found: {yaml_file}")
            raise FileNotFoundError(yaml_file)

        with open(yaml_file, 'r') as f:
            data = yaml.safe_load(f)

        coords = data.get('polygon_coordinates', [])
        if len(coords) < 2:
            self.get_logger().error("Need at least 2 coordinates to draw a polygon.")
            raise ValueError("Not enough coordinates.")

        self.origin_lat, self.origin_lon = coords[0]
        self.local_points = self.convert_to_local(coords)

        # Publisher for Marker
        self.marker_pub = self.create_publisher(Marker, 'polygon_marker', 10)
        self.robot_publisher = self.create_publisher(Marker, '/robot_marker', 10)
        self.ground_publisher = self.create_publisher(Marker, '/ground_marker', 10)
        self.robot_coords_sub = self.create_subscription(NavSatFix, '/ublox_gps_node/fix', self.navsat_callback, 10)

        # Timer to publish marker
        self.timer = self.create_timer(1.0, self.publish_marker)
        self.ground_timer = self.create_timer(0.25, self.publish_ground)  # 10 Hz
        self.get_logger().info("PolygonVisualizationNode started.")

    def navsat_callback(self, msg):
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "robot"
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD

        position = self.convert_to_local([(msg.latitude, msg.longitude)])[0]

        marker.pose = Pose()
        marker.pose.position = Point(x=position[0], y=position[1], z=0.0)
        euler_angles = (-1.57, 0.0, -1.57)  # Roll, Pitch, Yaw in radians
        qx, qy, qz, qw = tf_transformations.quaternion_from_euler(*euler_angles)
        marker.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        # Scale of the model (adjust to your mesh size)
        marker.scale.x = 0.001
        marker.scale.y = 0.001
        marker.scale.z = 0.001

        marker.mesh_resource = self.mesh_path
        marker.mesh_use_embedded_materials = True  

        marker.lifetime = Duration(sec=0, nanosec=0)  # 0 = forever

        self.robot_publisher.publish(marker)

    def publish_ground(self):
        # --- Marker 100: Lawn Ground Cube ---
        cube_marker = Marker()
        cube_marker.header.frame_id = 'map'
        cube_marker.ns = 'ground'
        cube_marker.id = 100
        cube_marker.type = Marker.CUBE
        cube_marker.action = Marker.ADD
        cube_marker.pose.position.x = 0.0
        cube_marker.pose.position.y = 0.0
        cube_marker.pose.position.z = -0.01
        cube_marker.pose.orientation.w = 1.0
        cube_marker.scale.x = 70.0
        cube_marker.scale.y = 70.0
        cube_marker.scale.z = 0.01
        cube_marker.color.r = 0.2
        cube_marker.color.g = 0.8
        cube_marker.color.b = 0.2
        cube_marker.color.a = 1.0

        self.ground_publisher.publish(cube_marker)


    def convert_to_local(self, coords):
        """
        Convert lat/lon to local ENU coordinates in meters,
        using the first coordinate as the origin.
        """
        transformer = Transformer.from_crs(
            "epsg:4326",      # WGS84 lat/lon
            "epsg:4978",      # ECEF
            always_xy=True
        )

        # Origin in ECEF
        x0, y0, z0 = transformer.transform(self.origin_lon, self.origin_lat, 0.0)

        # Prepare a local ENU transformer
        # pyproj doesn't have direct ENU but we can use geodetic->ENU with Transformer.from_crs and a proj string
        # Simpler approach: use pyproj.Geod or local projection.
        # We'll use a local tangent plane (Azimuthal Equidistant) centered on origin.
        proj_str = (
            f"+proj=aeqd +lat_0={self.origin_lat} +lon_0={self.origin_lon} "
            "+x_0=0 +y_0=0 +units=m +ellps=WGS84 +type=crs"
        )

        local_transformer = Transformer.from_crs("epsg:4326", proj_str, always_xy=True)

        local_pts = []
        for lat, lon in coords:
            x, y = local_transformer.transform(lon, lat)
            # Map ENU: x -> East, y -> North
            local_pts.append((x, y))
        return local_pts

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "polygon"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1  # Line width in meters
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        for x, y in self.local_points:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.0
            marker.points.append(p)
        marker.points.append(marker.points[0])

        # Close the polygon if desired
        # marker.points.append(marker.points[0])

        self.marker_pub.publish(marker)
        self.get_logger().info(f"Publishing polygon marker: {marker}")

def main(args=None):
    rclpy.init(args=args)
    node = PolygonVisualizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
