"""ROS 2 node that subscribes to the thermal camera feed and displays it
with a false-color heatmap (INFERNO colormap).

The thermal sensor publishes 16-bit grayscale images (L16) where each pixel
value encodes temperature in Kelvin * resolution. This viewer normalizes
the values to 0-255 and applies a colormap for visualization.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class ThermalViewer(Node):
    def __init__(self):
        super().__init__('thermal_viewer')
        self.subscription = self.create_subscription(
            Image,
            'thermal_camera',
            self.callback,
            10)
        self.br = CvBridge()
        cv2.namedWindow('Thermal Camera', cv2.WINDOW_NORMAL)

    def callback(self, msg):
        self.get_logger().info('Receiving thermal frame', throttle_duration_sec=5.0)
        try:
            # Thermal images come as 16-bit mono (L16) or 8-bit mono (L8)
            if msg.encoding in ('mono16', '16UC1'):
                frame = self.br.imgmsg_to_cv2(msg, desired_encoding='mono16')
                # Normalize 16-bit to 8-bit for display
                frame_norm = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX)
                frame_8bit = frame_norm.astype(np.uint8)
            else:
                frame = self.br.imgmsg_to_cv2(msg, desired_encoding='mono8')
                frame_8bit = frame

            # Apply INFERNO colormap for thermal visualization
            colored = cv2.applyColorMap(frame_8bit, cv2.COLORMAP_INFERNO)
            cv2.imshow('Thermal Camera', colored)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Failed to process thermal frame: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ThermalViewer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
