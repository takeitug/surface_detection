import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class ArucoDepthViewer(Node):
    def __init__(self):
        super().__init__('aruco_depth_viewer')
        self.bridge = CvBridge()
        self.depth_image = None
        self.camera_info_received = False

        self.fx = self.fy = self.cx = self.cy = 0

        self.create_subscription(Image, '/camera/camera/color/image_raw', self.rgb_callback, 10)
        self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()

    def camera_info_callback(self, msg):
        if not self.camera_info_received:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.camera_info_received = True
            self.get_logger().info("Camera intrinsics received.")

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def rgb_callback(self, msg):
        if self.depth_image is None or not self.camera_info_received:
            return

        rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        display_image = rgb_image.copy()

        corners, ids, _ = cv2.aruco.detectMarkers(rgb_image, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            for i, corner in enumerate(corners):
                c = corner[0]
                center_x = int(np.mean(c[:, 0]))
                center_y = int(np.mean(c[:, 1]))

                if 0 <= center_y < self.depth_image.shape[0] and 0 <= center_x < self.depth_image.shape[1]:
                    depth = self.depth_image[center_y, center_x]
                    if depth == 0:
                        continue

                    X = (center_x - self.cx) * depth / self.fx
                    Y = (center_y - self.cy) * depth / self.fy
                    Z = depth

                    # 赤い点を描画
                    cv2.circle(display_image, (center_x, center_y), 5, (0, 0, 255), -1)

                    # 3D座標テキストを表示
                    coord_text = f"X:{X:.1f} Y:{Y:.1f} Z:{Z:.1f} mm"
                    cv2.putText(display_image, coord_text, (center_x + 10, center_y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        # ウィンドウに表示
        cv2.imshow("RGB Image with AR Marker", display_image)
        key = cv2.waitKey(1)
        if key == ord('q'):
            self.get_logger().info("Shutting down viewer.")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDepthViewer()
    rclpy.spin(node)
    rclpy.spin_once(node)
    node.destroy_node()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()