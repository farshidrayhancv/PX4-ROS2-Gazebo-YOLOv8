"""ROS 2 node that subscribes to the depth camera feed and displays it
with a false-color depth map (JET colormap).

The depth sensor publishes 32-bit float images (R_FLOAT32) where each pixel
value is the distance in meters.  This viewer normalizes the values to 0-255
and applies a colormap for visualization.  Closer objects appear red/warm,
farther objects appear blue/cool.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

MAX_DEPTH = 50.0  # meters — clip depths beyond this for better contrast


class DepthViewer(Node):
    def __init__(self):
        super().__init__('depth_viewer')
        self.subscription = self.create_subscription(
            Image,
            'depth_camera',
            self.callback,
            10)
        self.br = CvBridge()
        cv2.namedWindow('Depth Camera', cv2.WINDOW_NORMAL)

    def callback(self, msg):
        self.get_logger().info('Receiving depth frame', throttle_duration_sec=5.0)
        try:
            # Depth images arrive as 32FC1 (32-bit float, single channel)
            frame = self.br.imgmsg_to_cv2(msg, desired_encoding='32FC1')

            # Replace inf/nan with max depth
            frame = np.where(np.isfinite(frame), frame, MAX_DEPTH)

            # Clip and normalize to 0-255
            frame_clipped = np.clip(frame, 0, MAX_DEPTH)
            frame_norm = (frame_clipped / MAX_DEPTH * 255).astype(np.uint8)

            # Apply JET colormap (red=close, blue=far)
            colored = cv2.applyColorMap(frame_norm, cv2.COLORMAP_JET)
            cv2.imshow('Depth Camera', colored)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Failed to process depth frame: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = DepthViewer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
