import math
from pathlib import Path

import cv2
import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from .euroc_utils import pair_stereo, read_sensor_yaml
from .odom_utils import nearest_pose, read_standard_csv
from .tf_utils import make_transform_msg, pose_to_matrix


def stamp_from_ns(stamp_ns):
    msg = Time()
    msg.sec = int(stamp_ns // 1_000_000_000)
    msg.nanosec = int(stamp_ns % 1_000_000_000)
    return msg


def camera_info(stamp, frame_id, width, height, K, P):
    msg = CameraInfo()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.width = int(width)
    msg.height = int(height)
    msg.distortion_model = "plumb_bob"
    msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    msg.k = [float(x) for x in K.reshape(-1)]
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.p = [float(x) for x in P.reshape(-1)]
    return msg


class ReplayNode(Node):
    def __init__(self):
        super().__init__("hno_rtabmap_replay")
        self.declare_parameter("euroc_mav0", "/home/sharpa/datasets/euroc/ASL/V1_01_easy/mav0")
        self.declare_parameter("odom_csv", "")
        self.declare_parameter("max_odom_time_diff_sec", 0.03)
        self.declare_parameter("replay_rate_hz", 20.0)
        self.declare_parameter("max_duration_sec", 0.0)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("left_camera_frame", "cam0_rect")
        self.declare_parameter("right_camera_frame", "cam1_rect")

        self.euroc_mav0 = Path(self.get_parameter("euroc_mav0").value)
        self.odom_csv = Path(self.get_parameter("odom_csv").value)
        self.max_diff_ns = int(float(self.get_parameter("max_odom_time_diff_sec").value) * 1e9)
        self.rate_hz = float(self.get_parameter("replay_rate_hz").value)
        self.max_duration_sec = float(self.get_parameter("max_duration_sec").value)
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.left_frame = self.get_parameter("left_camera_frame").value
        self.right_frame = self.get_parameter("right_camera_frame").value

        self.bridge = CvBridge()
        self.pub_left = self.create_publisher(Image, "/cam0/image_rect", 10)
        self.pub_right = self.create_publisher(Image, "/cam1/image_rect", 10)
        self.pub_left_info = self.create_publisher(CameraInfo, "/cam0/camera_info", 10)
        self.pub_right_info = self.create_publisher(CameraInfo, "/cam1/camera_info", 10)
        self.pub_odom = self.create_publisher(Odometry, "/hno_vio/odom", 50)
        self.pub_clock = self.create_publisher(Clock, "/clock", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        self.left_sensor = read_sensor_yaml(self.euroc_mav0 / "cam0" / "sensor.yaml")
        self.right_sensor = read_sensor_yaml(self.euroc_mav0 / "cam1" / "sensor.yaml")
        self.poses = read_standard_csv(self.odom_csv)
        self.stereo = pair_stereo(self.euroc_mav0 / "cam0" / "data.csv", self.euroc_mav0 / "cam1" / "data.csv")
        if not self.stereo:
            raise RuntimeError("no stereo frames found")
        if not self.poses:
            raise RuntimeError("no odom poses found")

        self._prepare_rectification()
        self._publish_static_tf()
        self.index = 0
        self.skipped = 0
        self.published = 0
        self.start_stamp_ns = None
        self.finished = False
        period = 1.0 / max(0.1, self.rate_hz)
        self.timer = self.create_timer(period, self._tick)
        self.get_logger().info(f"Replay ready: {len(self.stereo)} stereo frames, {len(self.poses)} odom poses")

    def _prepare_rectification(self):
        l, r = self.left_sensor, self.right_sensor
        width, height = l["width"], l["height"]
        image_size = (width, height)
        T_B_L = l["T_BS"]
        T_B_R = r["T_BS"]
        T_L_B = np.linalg.inv(T_B_L)
        T_L_R = T_L_B @ T_B_R
        R = T_L_R[:3, :3]
        t = T_L_R[:3, 3]
        R1, R2, P1, P2, _Q, _roi1, _roi2 = cv2.stereoRectify(
            l["K"], l["D"], r["K"], r["D"], image_size, R, t, flags=cv2.CALIB_ZERO_DISPARITY, alpha=0
        )
        if P2[0, 3] > 0:
            P2[0, 3] = -P2[0, 3]
        self.map_l = cv2.initUndistortRectifyMap(l["K"], l["D"], R1, P1[:3, :3], image_size, cv2.CV_16SC2)
        self.map_r = cv2.initUndistortRectifyMap(r["K"], r["D"], R2, P2[:3, :3], image_size, cv2.CV_16SC2)
        self.P1, self.P2 = P1, P2
        self.K1_rect = P1[:3, :3]
        self.K2_rect = P2[:3, :3]
        T_B_L_rect = T_B_L.copy()
        T_B_R_rect = T_B_R.copy()
        T_B_L_rect[:3, :3] = T_B_L[:3, :3] @ R1.T
        T_B_R_rect[:3, :3] = T_B_R[:3, :3] @ R2.T
        self.T_B_L_rect = T_B_L_rect
        self.T_B_R_rect = T_B_R_rect
        baseline_tf = float(np.linalg.norm(T_B_R[:3, 3] - T_B_L[:3, 3]))
        baseline_p = abs(float(P2[0, 3] / P2[0, 0])) if abs(P2[0, 0]) > 1e-9 else 0.0
        self.get_logger().info(f"T_base_cam0_rect:\n{T_B_L_rect}")
        self.get_logger().info(f"T_base_cam1_rect:\n{T_B_R_rect}")
        self.get_logger().info(f"baseline_tf {baseline_tf:.6f} baseline_from_P {baseline_p:.6f}")
        if not (0.05 <= baseline_tf <= 0.20 and 0.05 <= baseline_p <= 0.20):
            raise RuntimeError(f"invalid stereo baseline: tf={baseline_tf}, P={baseline_p}")

    def _publish_static_tf(self):
        stamp = stamp_from_ns(self.stereo[0][0])
        transforms = [
            make_transform_msg(self.base_frame, self.left_frame, stamp, self.T_B_L_rect),
            make_transform_msg(self.base_frame, self.right_frame, stamp, self.T_B_R_rect),
        ]
        self.static_tf_broadcaster.sendTransform(transforms)

    def _odom_msg(self, stamp, pose):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = float(pose.p[0])
        msg.pose.pose.position.y = float(pose.p[1])
        msg.pose.pose.position.z = float(pose.p[2])
        msg.pose.pose.orientation.x = float(pose.q[0])
        msg.pose.pose.orientation.y = float(pose.q[1])
        msg.pose.pose.orientation.z = float(pose.q[2])
        msg.pose.pose.orientation.w = float(pose.q[3])
        msg.pose.covariance[0] = 0.01
        msg.pose.covariance[7] = 0.01
        msg.pose.covariance[14] = 0.01
        msg.pose.covariance[21] = 0.01
        msg.pose.covariance[28] = 0.01
        msg.pose.covariance[35] = 0.01
        return msg

    def _tick(self):
        if self.index >= len(self.stereo):
            self.get_logger().info(f"Replay finished: published={self.published} skipped={self.skipped}")
            self.finished = True
            self.timer.cancel()
            return
        stamp_ns, left_fn, right_fn = self.stereo[self.index]
        self.index += 1
        if self.start_stamp_ns is None:
            self.start_stamp_ns = stamp_ns
        if self.max_duration_sec > 0 and (stamp_ns - self.start_stamp_ns) * 1e-9 > self.max_duration_sec:
            self.get_logger().info(f"Replay reached max_duration_sec={self.max_duration_sec}")
            self.finished = True
            self.timer.cancel()
            return

        pose, diff = nearest_pose(self.poses, stamp_ns, self.max_diff_ns)
        if pose is None:
            self.skipped += 1
            if self.skipped <= 5 or self.skipped % 100 == 0:
                self.get_logger().warn(f"skip frame {stamp_ns}: nearest odom diff ns={diff}")
            return

        stamp = stamp_from_ns(stamp_ns)
        left = cv2.imread(str(self.euroc_mav0 / "cam0" / "data" / left_fn), cv2.IMREAD_GRAYSCALE)
        right = cv2.imread(str(self.euroc_mav0 / "cam1" / "data" / right_fn), cv2.IMREAD_GRAYSCALE)
        if left is None or right is None:
            self.get_logger().warn(f"missing image at {stamp_ns}")
            self.skipped += 1
            return
        left_rect = cv2.remap(left, self.map_l[0], self.map_l[1], cv2.INTER_LINEAR)
        right_rect = cv2.remap(right, self.map_r[0], self.map_r[1], cv2.INTER_LINEAR)

        clock = Clock()
        clock.clock = stamp
        self.pub_clock.publish(clock)

        left_msg = self.bridge.cv2_to_imgmsg(left_rect, encoding="mono8")
        right_msg = self.bridge.cv2_to_imgmsg(right_rect, encoding="mono8")
        left_msg.header.stamp = stamp
        right_msg.header.stamp = stamp
        left_msg.header.frame_id = self.left_frame
        right_msg.header.frame_id = self.right_frame
        self.pub_left.publish(left_msg)
        self.pub_right.publish(right_msg)
        self.pub_left_info.publish(camera_info(stamp, self.left_frame, self.left_sensor["width"], self.left_sensor["height"], self.K1_rect, self.P1))
        self.pub_right_info.publish(camera_info(stamp, self.right_frame, self.right_sensor["width"], self.right_sensor["height"], self.K2_rect, self.P2))

        odom_msg = self._odom_msg(stamp, pose)
        self.pub_odom.publish(odom_msg)
        self.tf_broadcaster.sendTransform(make_transform_msg(self.odom_frame, self.base_frame, stamp, pose_to_matrix(pose.p, pose.q)))
        self.published += 1


def main():
    rclpy.init()
    node = ReplayNode()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
