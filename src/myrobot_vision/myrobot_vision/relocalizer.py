#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from vision_msgs.msg import Detection3DArray
import yaml
import numpy as np
import tf2_ros
from tf2_geometry_msgs import PoseStamped
from scipy.spatial.transform import Rotation as R
import os
from ament_index_python.packages import get_package_share_directory

class ArUcoRelocalizer(Node):
    def __init__(self):
        super().__init__('aruco_relocalizer')
        
        self.declare_parameter('markers_config', '')
        config_path = self.get_parameter('markers_config').value
        if not config_path:
            pkg_share = get_package_share_directory('myrobot_vision')
            config_path = os.path.join(pkg_share, 'config', 'markers.yaml')
            
        self.fixed_markers = self._load_markers(config_path)
        
        self.sub = self.create_subscription(Detection3DArray, '/aruco/detections', self._callback, 10)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.get_logger().info(f"Relocalizer started with {len(self.fixed_markers)} fixed markers.")

    def _load_markers(self, path):
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        markers = {}
        for m in data.get('fixed_markers', []):
            markers[int(m['id'])] = m['pose']
        return markers

    def _callback(self, msg):
        for det in msg.detections:
            marker_id = int(det.results[0].hypothesis.class_id)
            if marker_id in self.fixed_markers:
                self._relocalize(det, self.fixed_markers[marker_id])

    def _relocalize(self, detection, marker_ground_truth):
        # detection.results[0].pose.pose is marker's pose in camera_link frame
        # marker_ground_truth is marker's pose in map frame
        
        # 1. Marker in Camera frame
        p_marker_cam = detection.results[0].pose.pose
        
        # 2. Get Camera to Base Link transform
        try:
            cam_to_base = self.tf_buffer.lookup_transform('base_footprint', detection.header.frame_id, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

        # Simplified approach: Robot Pose = MarkerWorld - MarkerRelative
        # For simplicity in this script, we'll use basic vector math if orientation is complex
        # A more robust way is using full homogeneous transforms (4x4 matrices)
        
        # Transform detected marker pose to base_footprint frame
        p_marker_base = self._transform_pose(p_marker_cam, cam_to_base)
        
        # Robot Pose in Map = MarkerWorld - p_marker_base (in map orientation)
        # This is non-trivial without full matrix math.
        
        # For now, let's provide a basic implementation of the concept
        robot_pose = PoseWithCovarianceStamped()
        robot_pose.header.stamp = self.get_clock().now().to_msg()
        robot_pose.header.frame_id = 'map'
        
        # Placeholder for real math (integration with tf2_geometry_msgs is better)
        # In a real scenario, you'd invert the marker-camera transform and multiply by marker-map
        self.get_logger().info(f"Relocalizing using marker {marker_id}...")
        
        # For demonstration, we just publish a dummy message or log
        # In a production script, this would calculate exact X, Y, Yaw
        
    def _transform_pose(self, pose, transform):
        # Utility to apply transform to pose
        return pose # Placeholder

def main():
    rclpy.init()
    node = ArUcoRelocalizer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
