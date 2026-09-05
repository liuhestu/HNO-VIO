#!/usr/bin/python3
"""Publish rectified stereo topics for RTAB-Map from raw EuRoC stereo images."""

from pathlib import Path

import cv2
import message_filters
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
import yaml
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import StaticTransformBroadcaster


def load_yaml(path):
    text = Path(path).read_text(encoding="utf-8")
    lines = text.splitlines()
    if lines and lines[0].startswith("%YAML"):
        text = "\n".join(lines[1:])
    return yaml.safe_load(text)


def sensor_from_kalibr(data, name):
    cam = data[name]
    fx, fy, cx, cy = [float(v) for v in cam["intrinsics"]]
    dist = [float(v) for v in cam["distortion_coeffs"]]
    width, height = [int(v) for v in cam["resolution"]]
    return {
        "T_BS": np.array(cam["T_imu_cam"], dtype=float),
        "K": np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=float),
        "D": np.array(dist, dtype=float),
        "width": width,
        "height": height,
    }


def quat_from_rot(R):
    tr = float(np.trace(R))
    if tr > 0.0:
        s = np.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s
    q = np.array([qx, qy, qz, qw], dtype=float)
    q /= np.linalg.norm(q)
    return q


def make_transform(parent, child, stamp, T):
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.transform.translation.x = float(T[0, 3])
    msg.transform.translation.y = float(T[1, 3])
    msg.transform.translation.z = float(T[2, 3])
    q = quat_from_rot(T[:3, :3])
    msg.transform.rotation.x = float(q[0])
    msg.transform.rotation.y = float(q[1])
    msg.transform.rotation.z = float(q[2])
    msg.transform.rotation.w = float(q[3])
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


class RtabmapPreprocess(Node):
    def __init__(self):
        super().__init__("rtabmap_preprocess")
        self.declare_parameter("camera_config", "/home/he/hno_vio_ws/src/hno_vio/config/euroc_mav/kalibr_imucam_chain.yaml")
        self.declare_parameter("left_topic", "/cam0/image_raw")
        self.declare_parameter("right_topic", "/cam1/image_raw")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("left_camera_frame", "cam0_rect")
        self.declare_parameter("right_camera_frame", "cam1_rect")
        self.declare_parameter("approx_sync_max_interval_sec", 0.002)

        self.camera_config = Path(self.get_parameter("camera_config").value)
        self.base_frame = self.get_parameter("base_frame").value
        self.left_frame = self.get_parameter("left_camera_frame").value
        self.right_frame = self.get_parameter("right_camera_frame").value
        self.bridge = CvBridge()

        self.pub_left = self.create_publisher(Image, "/cam0/image_rect", 10)
        self.pub_right = self.create_publisher(Image, "/cam1/image_rect", 10)
        self.pub_left_info = self.create_publisher(CameraInfo, "/cam0/camera_info", 10)
        self.pub_right_info = self.create_publisher(CameraInfo, "/cam1/camera_info", 10)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        data = load_yaml(self.camera_config)
        self.left_sensor = sensor_from_kalibr(data, "cam0")
        self.right_sensor = sensor_from_kalibr(data, "cam1")
        self._prepare_rectification()
        self._publish_static_tf(Time(sec=0, nanosec=0))

        left_topic = self.get_parameter("left_topic").value
        right_topic = self.get_parameter("right_topic").value
        self.sub_left = message_filters.Subscriber(self, Image, left_topic)
        self.sub_right = message_filters.Subscriber(self, Image, right_topic)
        max_interval = float(self.get_parameter("approx_sync_max_interval_sec").value)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.sub_left, self.sub_right], queue_size=30, slop=max_interval
        )
        self.sync.registerCallback(self._callback)
        self.get_logger().info(f"RTAB preprocess ready: {left_topic}, {right_topic}, config={self.camera_config}")

    def _prepare_rectification(self):
        l, r = self.left_sensor, self.right_sensor
        width, height = l["width"], l["height"]
        image_size = (width, height)
        T_B_L = l["T_BS"]
        T_B_R = r["T_BS"]
        T_R_L = np.linalg.inv(T_B_R) @ T_B_L
        R = T_R_L[:3, :3]
        t = T_R_L[:3, 3]
        R1, R2, P1, P2, _Q, _roi1, _roi2 = cv2.stereoRectify(
            l["K"], l["D"], r["K"], r["D"], image_size, R, t, flags=cv2.CALIB_ZERO_DISPARITY, alpha=0
        )
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
        self.get_logger().info(f"baseline_tf={baseline_tf:.6f} baseline_from_P={baseline_p:.6f}")
        if not (0.05 <= baseline_tf <= 0.20 and 0.05 <= baseline_p <= 0.20):
            raise RuntimeError(f"invalid stereo baseline: tf={baseline_tf}, P={baseline_p}")

    def _publish_static_tf(self, stamp):
        self.static_tf_broadcaster.sendTransform([
            make_transform(self.base_frame, self.left_frame, stamp, self.T_B_L_rect),
            make_transform(self.base_frame, self.right_frame, stamp, self.T_B_R_rect),
        ])

    def _callback(self, left_msg, right_msg):
        stamp = left_msg.header.stamp
        left = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding="mono8")
        right = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding="mono8")
        left_rect = cv2.remap(left, self.map_l[0], self.map_l[1], cv2.INTER_LINEAR)
        right_rect = cv2.remap(right, self.map_r[0], self.map_r[1], cv2.INTER_LINEAR)

        left_out = self.bridge.cv2_to_imgmsg(left_rect, encoding="mono8")
        right_out = self.bridge.cv2_to_imgmsg(right_rect, encoding="mono8")
        left_out.header.stamp = stamp
        right_out.header.stamp = stamp
        left_out.header.frame_id = self.left_frame
        right_out.header.frame_id = self.right_frame

        self.pub_left.publish(left_out)
        self.pub_right.publish(right_out)
        self.pub_left_info.publish(camera_info(stamp, self.left_frame, self.left_sensor["width"], self.left_sensor["height"], self.K1_rect, self.P1))
        self.pub_right_info.publish(camera_info(stamp, self.right_frame, self.right_sensor["width"], self.right_sensor["height"], self.K2_rect, self.P2))


def main():
    rclpy.init()
    node = RtabmapPreprocess()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
