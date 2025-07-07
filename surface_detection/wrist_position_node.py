import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import tf2_ros
import PyKDL
from tf_transformations import quaternion_from_euler

def transform_to_kdl(t):
    return PyKDL.Frame(
        PyKDL.Rotation.Quaternion(t.transform.rotation.x, t.transform.rotation.y,
                                  t.transform.rotation.z, t.transform.rotation.w),
        PyKDL.Vector(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z)
    )

def do_transform_pose(pose, transform):
    f = transform_to_kdl(transform) * PyKDL.Frame(
        PyKDL.Rotation.Quaternion(pose.pose.orientation.x, pose.pose.orientation.y,
                                  pose.pose.orientation.z, pose.pose.orientation.w),
        PyKDL.Vector(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
    )
    res = PoseStamped()
    res.pose.position.x = f.p[0]
    res.pose.position.y = f.p[1]
    res.pose.position.z = f.p[2]
    (res.pose.orientation.x, res.pose.orientation.y, res.pose.orientation.z, res.pose.orientation.w) = f.M.GetQuaternion()
    res.header = transform.header
    return res

class LeftWristPoseNode(Node):
    def __init__(self):
        super().__init__('wrist_position_node')
        self.bridge = CvBridge()
        self.depth_image = None
        self.camera_info_received = False

        self.declare_parameter("camera_mount_frame", "camera_mount_frame")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("translation", [-0.9, 0.2, 0.75])  # 原点
        self.declare_parameter("rotation_rpy", [-np.pi/2, np.pi, -np.pi/2.8]) # 回転なし

        self.camera_mount_frame = self.get_parameter("camera_mount_frame").get_parameter_value().string_value
        self.world_frame = self.get_parameter("world_frame").get_parameter_value().string_value
        self.translation = self.get_parameter("translation").value
        self.rotation_rpy = self.get_parameter("rotation_rpy").value

        # サブスクライバ
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.rgb_callback, 10)
        self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)

        self.wrist_pub = self.create_publisher(Float64MultiArray, '/left_wrist_position', 10)

        self.fx = self.fy = self.cx = self.cy = 0

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.broadcast_static_transform()

        self.model = YOLO("/home/isrlab/colcon_ws/yolov8m-pose.pt")

    def broadcast_static_transform(self):
        static_tf = TransformStamped()
        static_tf.header.stamp = self.get_clock().now().to_msg()
        static_tf.header.frame_id = self.world_frame
        static_tf.child_frame_id = self.camera_mount_frame
        static_tf.transform.translation.x = self.translation[0]
        static_tf.transform.translation.y = self.translation[1]
        static_tf.transform.translation.z = self.translation[2]
        q = quaternion_from_euler(*self.rotation_rpy)
        static_tf.transform.rotation.x = q[0]
        static_tf.transform.rotation.y = q[1]
        static_tf.transform.rotation.z = q[2]
        static_tf.transform.rotation.w = q[3]
        self.static_broadcaster.sendTransform(static_tf)
        self.get_logger().info(f"Static TF published from {self.world_frame} to {self.camera_mount_frame}")

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
        results = self.model.predict(rgb_image)
        persons = results[0]

        for person in persons:
            index = 9  # "wrist(L)"
            keypoints = person.keypoints
            kpxy = keypoints.xy.tolist()
            kpconf = keypoints.conf.tolist()
            score = kpconf[0][index]
            x = int(kpxy[0][index][0])
            y = int(kpxy[0][index][1])
            if x >= rgb_image.shape[1]:
                x = rgb_image.shape[1] - 1
            if y >= rgb_image.shape[0]:
                y = rgb_image.shape[0] - 1
            if score < 0.7:
                continue
            depth = self.depth_image[y, x]
            if depth == 0:
                continue
            X = (x - self.cx) * depth / self.fx / 1000.0
            Y = (y - self.cy) * depth / self.fy / 1000.0
            Z = depth / 1000.0
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.world_frame, self.camera_mount_frame, rclpy.time.Time())
                pose_cam = PoseStamped()
                pose_cam.header.frame_id = self.camera_mount_frame
                pose_cam.header.stamp = msg.header.stamp
                pose_cam.pose.position.x = float(X)
                pose_cam.pose.position.y = float(Y)
                pose_cam.pose.position.z = float(Z)
                pose_cam.pose.orientation.w = 1.0
                pose_world = do_transform_pose(pose_cam, tf)
                pose_world.header.frame_id = self.world_frame
                array = Float64MultiArray()
                array.data = [
                    pose_world.pose.position.x,
                    pose_world.pose.position.y,
                    pose_world.pose.position.z
                ]
                self.wrist_pub.publish(array)
                # オプション：表示
                cv2.circle(rgb_image, (x, y), 10, (255, 0, 0), -1)
                pos_txt = f"({array.data[0]:.2f}, {array.data[1]:.2f}, {array.data[2]:.2f})"
                cv2.putText(rgb_image, pos_txt, (x+10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
            except Exception as e:
                self.get_logger().warn(f"TF transform failed: {e}")
        cv2.imshow("Left Wrist Detection", rgb_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = LeftWristPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
