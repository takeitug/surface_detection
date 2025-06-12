import rclpy
import rclpy.duration
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Point, Quaternion, TransformStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

class Simple2detect(Node):
    def __init__(self):
        super().__init__('aruco_detect')
        self.bridge = CvBridge()
        self.depth_image = None
        self.camera_info_received = False

        # サブスクライバ
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.rgb_callback, 10)
        self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)

        # パブリッシャ
        self.marker1_pub = self.create_publisher(PoseStamped, '/marker1_pose', 10)
        self.marker2_pub = self.create_publisher(PoseStamped, '/marker2_pose', 10)

        # ArUco設定
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()

        # カメラ内部パラメータ
        self.fx = self.fy = self.cx = self.cy = 0

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
        corners, ids, _ = cv2.aruco.detectMarkers(rgb_image, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            
            marker1_pose=PoseStamped()
            marker2_pose=PoseStamped()
            
            id_count=0
            
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id not in [1,2]:
                    continue
                
                c = corners[i][0]
                center_x = int(np.mean(c[:, 0]))
                center_y = int(np.mean(c[:, 1]))

                if 0 <= center_y < self.depth_image.shape[0] and 0 <= center_x < self.depth_image.shape[1]:
                    depth = self.depth_image[center_y, center_x]
                    if depth == 0:
                        continue

                    x = (center_x - self.cx) * depth / self.fx / 1000.0
                    y = (center_y - self.cy) * depth / self.fy / 1000.0
                    z = depth / 1000.0
                    
                    pose_cam = PoseStamped()
                    pose_cam.header.frame_id = "sample"
                    pose_cam.header.stamp = msg.header.stamp
                    pose_cam.pose.position.x = float(x)
                    pose_cam.pose.position.y = float(y)
                    pose_cam.pose.position.z = float(z)
                    pose_cam.pose.orientation.w = 1.0
                    
                    if i == 0:
                        marker1_pose=pose_cam
                        id_count+=1
                            
                    if i == 1:
                        marker2_pose=pose_cam
                        id_count+=1
                        
                    # 赤点と座標表示
                    cv2.circle(rgb_image, (center_x, center_y), 6, (0, 0, 255), -1)
                    pos_txt = f"({pose_cam.pose.position.x:.2f}, {pose_cam.pose.position.y:.2f}, {pose_cam.pose.position.z:.2f})"
                    cv2.putText(rgb_image, pos_txt, (center_x+10, center_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)

            if id_count==2:
                self.marker1_pub.publish(marker1_pose)
                self.marker2_pub.publish(marker2_pose)
                
                self.get_logger().info(
                    f"Marker ID 1: ({marker1_pose.pose.position.x:.2f}, "
                    f"{marker1_pose.pose.position.y:.2f}, {marker1_pose.pose.position.z:.2f}) [m]"
                )
                self.get_logger().info(
                    f"Marker ID 2: ({marker2_pose.pose.position.x:.2f}, "
                    f"{marker2_pose.pose.position.y:.2f}, {marker2_pose.pose.position.z:.2f}) [m]"
                )
                
        # ウィンドウ表示
        cv2.imshow("AR Marker Detection", rgb_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = Simple2detect()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()