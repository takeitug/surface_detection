import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys

class DepthPixelReader(Node):
    def __init__(self, x: int, y: int):
        super().__init__('depth_pixel_reader')
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.target_pixel = (x, y)
        self.get_logger().info(f"Target pixel set to: {self.target_pixel}")

    def listener_callback(self, msg):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            x, y = self.target_pixel
            if 0 <= y < depth_image.shape[0] and 0 <= x < depth_image.shape[1]:
                depth_value = depth_image[y, x]
                self.get_logger().info(f"Depth at pixel {self.target_pixel}: {depth_value} mm")
            else:
                self.get_logger().warn("Pixel out of image bounds.")
        except Exception as e:
            self.get_logger().error(f"Error processing depth image: {e}")

def main(args=None):
    rclpy.init(args=args)
    argv = sys.argv

    # 引数チェック
    if len(argv) < 3:
        print("Usage: ros2 run <package_name> depth_pixel_reader.py <x> <y>")
        return

    try:
        x = int(argv[1])
        y = int(argv[2])
    except ValueError:
        print("Pixel coordinates must be integers.")
        return

    node = DepthPixelReader(x, y)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()