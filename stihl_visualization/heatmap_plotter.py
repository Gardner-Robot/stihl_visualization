import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import ReentrantCallbackGroup

import numpy as np
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, Pose
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import NavSatFix
from stihl_vision_msgs.msg import Detection3DArray
from stihl_vision.utils import convertToLocal
from yaml import safe_load
from copy import deepcopy
from collections import Counter
from math import hypot

WEED_TYPES = ['creep_weed', 'leaf_weed', 'circle_weed', 'flower']

class WeedRecord:
    def __init__(self, position):
        self.position = (float(position[0]), float(position[1]))
        self.count = {weed_type: 0 for weed_type in WEED_TYPES}
        self.count['total'] = 0

class HeatmapPlotter(Node):
    """Node that maintains a grid/heatmap and updates it based on detections
    received on the object recognition topic and robot localization.
    Publishes an OccupancyGrid (`stihl_vision/wr/heatmap`) and text markers
    for non-empty cells (`stihl_vision/wr/heatmap_markers`).
    """

    def __init__(self, node_name='heatmap_plotter'):
        super().__init__(node_name)
        self.pkg_path = get_package_share_directory('stihl_visualization')

        # origin lat/lon for local ENU conversion (same as other nodes)
        self.origin_lat_lon = [-29.7864614, -51.1137923]

        # Robot and detection state
        self.current_position = None

        self.weeds = []

        self.loadConfig('heatmap_config.yaml')
        self.make_grid(self.grid_width, self.grid_height, self.grid_resolution)
        self.initRosComm()
        self.publish_heatmap()

        self.get_logger().info('HeatmapPlotter initialized')

    def initRosComm(self):
        self.heatmap_publisher = self.create_publisher(OccupancyGrid, self.heatmap_topic, qos_profile=10)
        self.heatmap_marker_publisher = self.create_publisher(MarkerArray, self.heatmap_markers_topic, qos_profile=10)
        
        cb_group = ReentrantCallbackGroup()
        self.gps_sub = self.create_subscription(NavSatFix, self.gps_topic, self.navsat_callback, 10, callback_group=cb_group)
        self.det_sub = self.create_subscription(Detection3DArray, self.weed_recognition_topic, self.detections_callback, 10, callback_group=cb_group)
        
        for label in self.labels:
            self.__setattr__(label+'heatmap', self.create_publisher(OccupancyGrid, self.heatmap_topic+'/'+label, qos_profile=10))
            self.__setattr__(label+'marker', self.create_publisher(MarkerArray, self.heatmap_markers_topic+'/'+label, qos_profile=10))

        # timer to publish heatmap every 1 second
        self.publish_timer = self.create_timer(1.0, self.publish_heatmap, callback_group=cb_group)

    def navsat_callback(self, msg: NavSatFix):
        try:
            self.current_position = []
            self.current_position = convertToLocal([(msg.latitude, msg.longitude)], self.origin_lat_lon)[0]
        except Exception:
            # keep previous position if conversion fails
            return
        
    def detections_callback(self, msg: Detection3DArray):
        self.get_logger().info('Received Detection3DArray message')
        
        self.get_logger().info(f'Received {len(msg.detections)} detections at position {self.current_position}')
        
        if len(msg.detections) == 0 or not self.current_position:
            return

        new_counts = {wt: 0 for wt in WEED_TYPES}
        new_counts['total'] = len(msg.detections)
        for det in msg.detections:
            if det.label in new_counts:
                new_counts[det.label] += 1

        pos = (float(self.current_position[0]), float(self.current_position[1]))

        match_idx = None
        for i, record in enumerate(self.weeds):
            ex, ey = record.position
            dist = hypot(pos[0] - ex, pos[1] - ey)
            if dist <= self.weed_merge_distance:
                match_idx = i
                break

        if match_idx is None:
            record = WeedRecord(pos)
            record.count = new_counts.copy()
            self.weeds.append(record)
            for label in WEED_TYPES:
                cnt = int(new_counts.get(label, 0))
                if cnt != 0:
                    self.update_heatmap_at_point(pos[0], pos[1], label, delta=cnt)
        else:
            record = self.weeds[match_idx]
            prev = record.count.copy() 
            record.position = pos
            record.count = new_counts.copy()
            for label in WEED_TYPES:
                prev_cnt = int(prev.get(label, 0))
                new_cnt = int(new_counts.get(label, 0))
                delta = new_cnt - prev_cnt
                if delta != 0:
                    self.update_heatmap_at_point(pos[0], pos[1], label, delta=new_cnt - prev_cnt)

        # publish after updating
        self.publish_heatmap(intensity=100/self.heatmap_max)

    def make_grid(self, width=5.0, height=10.0, resolution=1.0):
        if resolution is None or resolution <= 0:
            raise ValueError('resolution must be positive')

        cols = int(np.ceil(float(width) / float(resolution)))
        rows = int(np.ceil(float(height) / float(resolution)))

        self.resolution = float(resolution)
        self.grid_cols = max(cols, 1)
        self.grid_rows = max(rows, 1)
        self.map_width = self.grid_cols * self.resolution
        self.map_height = self.grid_rows * self.resolution
        self.grid = np.zeros((self.grid_rows, self.grid_cols), dtype=int)
        
        self.labels = ['creep_weed', 'leaf_weed', 'circle_weed', 'flower']
        
        self.labels_grid = {}
        for label in self.labels:
            self.labels_grid[label] = deepcopy(self.grid)

        # Precompute poses for each cell center (for marker placement)
        self.grid_poses = []
        for row in range(self.grid_rows):
            pose_row = []
            for col in range(self.grid_cols):
                pose = self.get_cell_center_pose(col, row, z=0.5)
                pose_row.append(pose)
            self.grid_poses.append(pose_row)

    def update_heatmap_at_point(self, x: float, y: float, label:str, delta: int = 1):
        cell = self.get_cell_for_point(x, y)
        if cell is None:
            self.get_logger().warn('Point outside heatmap bounds; skipping')
            return
        col, row = cell
        self.grid[row, col] = max(self.grid[row, col] + delta, 0)
        self.labels_grid[label][row, col] = max(self.labels_grid[label][row, col] + delta, 0)

    def publish_heatmap(self, intensity: int = 1):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = self.resolution
        msg.info.width = self.grid_cols
        msg.info.height = self.grid_rows
        msg.info.origin.position.x = self.begin_x
        msg.info.origin.position.y = self.begin_y

        theta = getattr(self, 'z_rot', 0.0) or 0.0
        half = float(theta) / 2.0
        msg.info.origin.orientation.x = 0.0
        msg.info.origin.orientation.y = 0.0
        msg.info.origin.orientation.z = float(np.sin(half))
        msg.info.origin.orientation.w = float(np.cos(half))

        for label in self.labels:
            msg_label = deepcopy(msg)
            data_label = self.labels_grid[label].flatten().tolist()
            msg_label.data = [min(int(v * intensity), 100) for v in data_label]
            self.__getattribute__(label+'heatmap').publish(msg_label)

            markers_label = MarkerArray()
            for row in range(self.grid_rows):
                for col in range(self.grid_cols):
                    val = self.labels_grid[label][row, col]
                    if val == 0:
                        continue
                    pose = self.grid_poses[row][col]
                    marker = self.make_text_marker(pose, val, id=len(markers_label.markers))
                    markers_label.markers.append(marker)
            self.__getattribute__(label+'marker').publish(markers_label)


        data = self.grid.flatten().tolist()
        # Scale/clamp to occupancy range
        msg.data = [min(int(v * intensity), 100) for v in data]
        self.heatmap_publisher.publish(msg)

        # Publish markers (text) for non-zero cells
        markers = MarkerArray()
        for row in range(self.grid_rows):
            for col in range(self.grid_cols):
                val = self.grid[row, col]
                if val == 0:
                    continue
                pose = self.grid_poses[row][col]
                marker = self.make_text_marker(pose, val, id=len(markers.markers))
                markers.markers.append(marker)
        self.heatmap_marker_publisher.publish(markers)

       

        self.get_logger().info('Published heatmap and markers')


    def make_text_marker(self, pose: Pose, text, rgb=[255, 255, 255], id=0):
        duration = Duration()
        duration.sec = 100000000

        color = np.asarray(rgb) / 255.0

        marker = Marker()
        marker.header.frame_id = 'map'
        marker.action = Marker.ADD
        marker.pose = pose
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = 1.0
        marker.id = id
        marker.ns = 'texts'
        marker.type = Marker.TEXT_VIEW_FACING
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.text = str(text)
        marker.lifetime = duration
        return marker

    def loadConfig(self, config_file):
        # Load grid configuration from a file (if needed)
        with open(self.pkg_path + '/config/' + config_file, 'r') as f:
            config = safe_load(f)
            
            self.begin_x = config['grid'].get('begin_x', -15.0)
            self.begin_y = config['grid'].get('begin_y', -20.0)
            self.z_rot = config['grid'].get('z_rot', -3.14 / 4.0)
            self.grid_width = config['grid'].get('width', 15.0)
            self.grid_height = config['grid'].get('height', 40.0)
            self.grid_resolution = config['grid'].get('resolution', 1.0)
            self.heatmap_topic = config['publishers'].get('heatmap_topic', 'stihl_vision/wr/heatmap')
            self.heatmap_markers_topic = config['publishers'].get('heatmap_markers_topic', 'stihl_vision/wr/heatmap_markers')
            self.weed_recognition_topic = config['subscribers'].get('weed_recognition_topic', 'stihl_vision/wr/weed_recognition')
            self.gps_topic = config['subscribers'].get('gps_topic', '/ublox_gps_node/fix')
            self.heatmap_max = config['grid'].get('max_occupancy', 20)
            self.weed_merge_distance = config['grid'].get('weed_merge_distance', 0.5)
    
    def get_cell_for_point(self, x: float, y: float):
        if x is None or y is None:
            return None

        rx = float(x) - self.begin_x
        ry = float(y) - self.begin_y

        theta = getattr(self, 'z_rot', 0.0) or 0.0
        cos_t = np.cos(-theta)
        sin_t = np.sin(-theta)
        rx_rot = cos_t * rx - sin_t * ry
        ry_rot = sin_t * rx + cos_t * ry

        if rx_rot < 0 or ry_rot < 0 or rx_rot >= self.map_width or ry_rot >= self.map_height:
            return None

        cell_w = self.map_width / float(self.grid_cols)
        cell_h = self.map_height / float(self.grid_rows)

        col = int(np.floor(rx_rot / cell_w))
        row = int(np.floor(ry_rot / cell_h))

        col = min(max(col, 0), self.grid_cols - 1)
        row = min(max(row, 0), self.grid_rows - 1)

        return (col, row)

    def get_cell_center_pose(self, col: int, row: int, z: float = 0.0) -> Pose:
        if not (0 <= col < self.grid_cols and 0 <= row < self.grid_rows):
            raise ValueError('Cell indices out of range')

        local_x = (col + 0.5) * float(self.resolution)
        local_y = (row + 0.5) * float(self.resolution)

        theta = self.z_rot
        cos_t = np.cos(theta)
        sin_t = np.sin(theta)
        rot_x = cos_t * local_x - sin_t * local_y
        rot_y = sin_t * local_x + cos_t * local_y

        world_x = float(self.begin_x) + rot_x
        world_y = float(self.begin_y) + rot_y

        pose = Pose()
        pose.position = Point(x=world_x, y=world_y, z=float(z))

        half = float(theta) / 2.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = float(np.sin(half))
        pose.orientation.w = float(np.cos(half))

        return pose


def main(args=None):
    rclpy.init(args=args)
    node = HeatmapPlotter()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
