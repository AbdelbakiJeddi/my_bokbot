#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray
from std_msgs.msg import String

class SequenceManager(Node):
    def __init__(self):
        super().__init__('sequence_manager')
        
        self.sequence = []
        self.sub = self.create_subscription(Detection3DArray, '/aruco/detections', self._callback, 10)
        self.pub = self.create_publisher(String, '/aruco/sequence', 10)
        
        self.get_logger().info("Sequence Manager started. Tracking marker detection order.")

    def _callback(self, msg):
        for det in msg.detections:
            marker_id = det.results[0].hypothesis.class_id
            
            # Only add if it's different from the last one (avoid spamming if robot is stationary)
            if not self.sequence or self.sequence[-1] != marker_id:
                self.sequence.append(marker_id)
                self.get_logger().info(f"New marker in sequence: {marker_id}")
                
                # Publish the updated sequence
                seq_msg = String()
                seq_msg.data = " -> ".join(self.sequence)
                self.pub.publish(seq_msg)

def main():
    rclpy.init()
    node = SequenceManager()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
