import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, Bool
import numpy as np
from geometry_msgs.msg import PoseStamped, Point, Quaternion, TransformStamped
from tf2_ros import Buffer, TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf_transformations import quaternion_from_euler
from tf2_kdl import transform_to_kdl
import PyKDL

def do_transform_point(point: Point, tf1, tf2):
    pose_cam = PoseStamped()
    pose_cam.header.frame_id = tf2.child_frame_id
    pose_cam.header.stamp = rclpy.time.Time().to_msg()
    pose_cam.pose.position = point
    pose_cam.pose.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
    pose_mount = do_transform_pose(pose_cam, tf2)
    pose_world = do_transform_pose(pose_mount, tf1)
    return pose_world.pose.position

def do_transform_pose(pose: PoseStamped, transform) -> PoseStamped:
    t = transform_to_kdl(transform)
    p = PyKDL.Frame(
        PyKDL.Rotation.Quaternion(
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w
        ),
        PyKDL.Vector(
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z
        )
    )
    f = t * p
    res = PoseStamped()
    res.header = transform.header
    res.pose.position.x = f.p[0]
    res.pose.position.y = f.p[1]
    res.pose.position.z = f.p[2]
    q = f.M.GetQuaternion()
    res.pose.orientation.x = q[0]
    res.pose.orientation.y = q[1]
    res.pose.orientation.z = q[2]
    res.pose.orientation.w = q[3]
    return res

def is_in_capsule(points_np, p1, p2, radius):
    v = p2 - p1
    w = points_np - p1
    v_norm = np.dot(v, v)
    t = np.clip(np.dot(w, v) / v_norm, 0.0, 1.0)
    proj = p1 + np.outer(t, v)
    dists = np.linalg.norm(points_np - proj, axis=1)
    return dists < radius

class MarkerCloudFilter(Node):
    def __init__(self):
        super().__init__('marker_cloud_filter')
        self.marker1_pose_cam = None
        self.marker2_pose_cam = None
        self.radius = 0.1

        self.create_subscription(PointCloud2, '/camera/camera/depth/color/points', self.cloud_callback, 10)
        self.create_subscription(PoseStamped, '/marker1_pose', self.marker1_callback, 10)
        self.create_subscription(PoseStamped, '/marker2_pose', self.marker2_callback, 10)

        self.capsule_cloud_pub = self.create_publisher(PointCloud2, '/capsule_cloud_transformed', 10)
        self.marker1_pub = self.create_publisher(PoseStamped, '/marker1_transformed', 10)
        self.marker2_pub = self.create_publisher(PoseStamped, '/marker2_transformed', 10)

        self.declare_parameter("camera_frame", "camera_link")
        self.declare_parameter("camera_mount_frame", "lbr_link_7")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("camera_to_mount_translation", [0.075, 0.0, 0.025])
        self.declare_parameter("camera_to_mount_rotation_rpy", [0.0, 0.0, np.pi/2])

        self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value
        self.camera_mount_frame = self.get_parameter("camera_mount_frame").get_parameter_value().string_value
        self.world_frame = self.get_parameter("world_frame").get_parameter_value().string_value
        self.translation = self.get_parameter("camera_to_mount_translation").value
        self.rotation_rpy = self.get_parameter("camera_to_mount_rotation_rpy").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.broadcast_static_transform()

        self.create_subscription(Bool, '/pointcloud_acquired', self.stop_callback, 10)

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

    def marker1_callback(self, msg):
        self.marker1_pose_cam = msg.pose

    def marker2_callback(self, msg):
        self.marker2_pose_cam = msg.pose

    def cloud_callback(self, msg: PointCloud2):
        if self.marker1_pose_cam is None or self.marker2_pose_cam is None:
            return
        try:
            tf1 = self.tf_buffer.lookup_transform(self.world_frame, self.camera_mount_frame, rclpy.time.Time())
            tf2 = self.tf_buffer.lookup_transform(self.camera_mount_frame, self.camera_frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

        #---【1】カメラ座標系でのカプセル抽出 ---
        cloud_points = list(point_cloud2.read_points(msg, skip_nans=True, field_names=("x", "y", "z")))
        cloud_np = np.array(cloud_points)
        if cloud_np.dtype.names:  # 構造化配列の場合は通常配列に変換
            cloud_np = np.stack([cloud_np['x'], cloud_np['y'], cloud_np['z']], axis=-1)
        cloud_np = cloud_np.astype(np.float64)
        marker1_np = np.array([self.marker1_pose_cam.position.x, self.marker1_pose_cam.position.y, self.marker1_pose_cam.position.z])
        marker2_np = np.array([self.marker2_pose_cam.position.x, self.marker2_pose_cam.position.y, self.marker2_pose_cam.position.z])
        mask = is_in_capsule(cloud_np, marker1_np, marker2_np, self.radius)
        capsule_points_camera = cloud_np[mask]

        #---【2】抽出点のみワールド座標変換 ---
        capsule_points_world = []
        for pt in capsule_points_camera:
            point = Point(x=float(pt[0]), y=float(pt[1]), z=float(pt[2]))
            try:
                transformed = do_transform_point(point, tf1, tf2)
                capsule_points_world.append([transformed.x, transformed.y, transformed.z])
            except Exception:
                continue

        capsule_np = np.array(capsule_points_world)

        #---【3】マーカー位置もワールド座標でパブリッシュ ---
        marker1_world = do_transform_point(self.marker1_pose_cam.position, tf1, tf2)
        marker2_world = do_transform_point(self.marker2_pose_cam.position, tf1, tf2)
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self.world_frame
        self.capsule_cloud_pub.publish(self.create_cloud(header, capsule_np))
        self.marker1_pub.publish(self.create_pose_stamped(marker1_world, header))
        self.marker2_pub.publish(self.create_pose_stamped(marker2_world, header))

    def create_cloud(self, header: Header, points):
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        return point_cloud2.create_cloud(header, fields, points)

    def create_pose_stamped(self, point: Point, header: Header):
        pose = PoseStamped()
        pose.header = header
        pose.pose.position = point
        pose.pose.orientation = Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
        return pose
    
    def stop_callback(msg):
        if msg.data:
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = MarkerCloudFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()