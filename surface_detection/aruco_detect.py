import rclpy
import rclpy.duration
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Point, Quaternion, TransformStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import tf2_ros
import tf2_geometry_msgs
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf_transformations import quaternion_from_euler

import PyKDL

def transform_to_kdl(t):
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.transform.rotation.x, t.transform.rotation.y,
                                                 t.transform.rotation.z, t.transform.rotation.w),
                       PyKDL.Vector(t.transform.translation.x, 
                                    t.transform.translation.y, 
                                    t.transform.translation.z))
    
def do_transform_pose(pose, transform):
    f = transform_to_kdl(transform) * PyKDL.Frame(PyKDL.Rotation.Quaternion(pose.pose.orientation.x, pose.pose.orientation.y,
                                                                          pose.pose.orientation.z, pose.pose.orientation.w),
                                                PyKDL.Vector(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z))
    res = PoseStamped()
    res.pose.position.x = f.p[0]
    res.pose.position.y = f.p[1]
    res.pose.position.z = f.p[2]
    (res.pose.orientation.x, res.pose.orientation.y, res.pose.orientation.z, res.pose.orientation.w) = f.M.GetQuaternion()
    res.header = transform.header
    return res

class ArucoDetect(Node):
    def __init__(self):
        super().__init__('aruco_detect')
        self.bridge = CvBridge()
        self.depth_image = None
        self.camera_info_received = False

        # パラメータ設定
        self.declare_parameter("camera_frame", "camera_link")
        self.declare_parameter("camera_mount_frame", "lbr_link_7")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("camera_to_mount_translation", [0.075, 0.0, 0.025])  # 例: x, y, z
        self.declare_parameter("camera_to_mount_rotation_rpy", [0.0, 0.0, math.pi/2]) # 回転: roll, pitch, yaw [rad]

        self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value
        self.camera_mount_frame = self.get_parameter("camera_mount_frame").get_parameter_value().string_value
        self.world_frame = self.get_parameter("world_frame").get_parameter_value().string_value
        self.translation = self.get_parameter("camera_to_mount_translation").value
        self.rotation_rpy = self.get_parameter("camera_to_mount_rotation_rpy").value

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

        # TFリスナー
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 静的TFブロードキャスター
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.broadcast_static_transform()

    def broadcast_static_transform(self):
        static_tf = TransformStamped()
        static_tf.header.stamp = self.get_clock().now().to_msg()
        static_tf.header.frame_id = self.camera_mount_frame
        static_tf.child_frame_id = self.camera_frame

        static_tf.transform.translation.x = self.translation[0]
        static_tf.transform.translation.y = self.translation[1]
        static_tf.transform.translation.z = self.translation[2]

        q = quaternion_from_euler(*self.rotation_rpy)
        static_tf.transform.rotation.x = q[0]
        static_tf.transform.rotation.y = q[1]
        static_tf.transform.rotation.z = q[2]
        static_tf.transform.rotation.w = q[3]

        self.static_broadcaster.sendTransform(static_tf)
        self.get_logger().info(f"Static TF published from {self.camera_mount_frame} to {self.camera_frame}")

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
                    pose_cam.header.frame_id = self.camera_frame
                    pose_cam.header.stamp = msg.header.stamp
                    pose_cam.pose.position.x = float(x)
                    pose_cam.pose.position.y = float(y)
                    pose_cam.pose.position.z = float(z)
                    pose_cam.pose.orientation.w = 1.0

                    try:
                        tf1 = self.tf_buffer.lookup_transform(
                            self.world_frame,
                            self.camera_mount_frame,
                            rclpy.time.Time()
                        )
                        tf2 = self.tf_buffer.lookup_transform(
                            self.camera_mount_frame,
                            self.camera_frame,
                            rclpy.time.Time()
                        )

                        

                        #pose_mount = tf2_geometry_msgs.do_transform_pose(pose_cam, tf2)
                        pose_mount=do_transform_pose(pose_cam,tf2)
                        #pose_world = tf2_geometry_msgs.tf2_geometry_msgs.do_transform_pose(pose_mount, tf1)
                        pose_world=do_transform_pose(pose_mount,tf1)
                        pose_world.header.frame_id = self.world_frame
                        
                        if i == 0:
                            marker1_pose=pose_world
                            id_count+=1
                            
                        if i == 1:
                            marker2_pose=pose_world
                            id_count+=1
                        

                        # 赤点と座標表示
                        cv2.circle(rgb_image, (center_x, center_y), 6, (0, 0, 255), -1)
                        pos_txt = f"({pose_world.pose.position.x:.2f}, {pose_world.pose.position.y:.2f}, {pose_world.pose.position.z:.2f})"
                        cv2.putText(rgb_image, pos_txt, (center_x+10, center_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)

                    except Exception as e:
                        self.get_logger().warn(f"TF transform failed: {e}")

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
    node = ArucoDetect()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()