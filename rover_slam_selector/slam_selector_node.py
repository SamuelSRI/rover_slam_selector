#!/usr/bin/env python3

import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String
from tf2_ros import TransformBroadcaster


def normalize_quaternion(q: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return q / norm


def quaternion_to_matrix(q: np.ndarray) -> np.ndarray:
    x, y, z, w = normalize_quaternion(q)
    return np.array([
        [1.0 - 2.0 * (y*y + z*z), 2.0 * (x*y - z*w),       2.0 * (x*z + y*w)],
        [2.0 * (x*y + z*w),       1.0 - 2.0 * (x*x + z*z), 2.0 * (y*z - x*w)],
        [2.0 * (x*z - y*w),       2.0 * (y*z + x*w),       1.0 - 2.0 * (x*x + y*y)],
    ], dtype=float)


def matrix_to_quaternion(m: np.ndarray) -> np.ndarray:
    trace = float(np.trace(m))

    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s

    return normalize_quaternion(np.array([x, y, z, w], dtype=float))


def pose_to_matrix(position, orientation) -> np.ndarray:
    t = np.eye(4, dtype=float)
    t[:3, :3] = quaternion_to_matrix(np.array([
        orientation.x, orientation.y, orientation.z, orientation.w
    ], dtype=float))
    t[:3, 3] = np.array([position.x, position.y, position.z], dtype=float)
    return t


class SlamSelectorNode(Node):
    """
    Selects between an indoor LiDAR-SLAM pose and an exterior visual-SLAM pose.

    Inputs:
      /env/exterior             std_msgs/Bool
      indoor_pose_topic         geometry_msgs/PoseWithCovarianceStamped
      exterior_pose_topic       geometry_msgs/PoseWithCovarianceStamped
      odom_topic                nav_msgs/Odometry

    Expected pose convention:
      Both SLAM topics represent T_map_base, even if their raw map origins differ.
      A continuity offset is computed whenever the active source changes.

    Output:
      TF map_frame -> odom_frame
      active_source_topic       std_msgs/String
    """

    def __init__(self) -> None:
        super().__init__("slam_selector")

        self.declare_parameter("indoor_pose_topic", "/slam_toolbox/pose")
        self.declare_parameter("exterior_pose_topic", "/visual_slam/pose")
        self.declare_parameter("odom_topic", "/odometry/filtered")
        self.declare_parameter("environment_topic", "/env/exterior")
        self.declare_parameter("active_source_topic", "/localization/active_source")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("source_timeout_sec", 0.5)
        self.declare_parameter("keep_last_transform_on_timeout", True)
        self.declare_parameter("preserve_continuity_on_switch", True)
        self.declare_parameter("force_planar_output", False)

        self.indoor_topic = str(self.get_parameter("indoor_pose_topic").value)
        self.exterior_topic = str(self.get_parameter("exterior_pose_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.environment_topic = str(self.get_parameter("environment_topic").value)
        self.active_source_topic = str(self.get_parameter("active_source_topic").value)

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)

        self.source_timeout = float(self.get_parameter("source_timeout_sec").value)
        self.keep_last = bool(
            self.get_parameter("keep_last_transform_on_timeout").value
        )
        self.preserve_continuity = bool(
            self.get_parameter("preserve_continuity_on_switch").value
        )
        self.force_planar = bool(
            self.get_parameter("force_planar_output").value
        )

        self.exterior_requested = False
        self.active_source: Optional[str] = None
        self.pending_switch = True

        self.indoor_pose: Optional[np.ndarray] = None
        self.exterior_pose: Optional[np.ndarray] = None
        self.odom_base: Optional[np.ndarray] = None

        self.indoor_stamp_ns: Optional[int] = None
        self.exterior_stamp_ns: Optional[int] = None
        self.odom_stamp_ns: Optional[int] = None

        # Applied to the selected raw SLAM pose:
        # T_map_base_corrected = T_switch_offset @ T_map_base_raw
        self.switch_offset = np.eye(4, dtype=float)

        self.last_map_base_output: Optional[np.ndarray] = None
        self.last_map_odom_output: Optional[np.ndarray] = None

        self.tf_broadcaster = TransformBroadcaster(self)
        self.source_pub = self.create_publisher(
            String, self.active_source_topic, 10
        )

        self.create_subscription(
            Bool, self.environment_topic, self.environment_callback, 10
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.indoor_topic,
            self.indoor_pose_callback,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.exterior_topic,
            self.exterior_pose_callback,
            10,
        )
        self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 20
        )

        rate = float(self.get_parameter("publish_rate_hz").value)
        if rate <= 0.0:
            raise ValueError("publish_rate_hz must be greater than zero")

        self.timer = self.create_timer(1.0 / rate, self.publish_transform)

        self.get_logger().info(
            f"Indoor={self.indoor_topic}, exterior={self.exterior_topic}, "
            f"environment={self.environment_topic}, odom={self.odom_topic}"
        )
        self.get_logger().info(
            f"Publishing the unique TF {self.map_frame} -> {self.odom_frame}"
        )

    def now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    @staticmethod
    def stamp_to_ns(stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def environment_callback(self, msg: Bool) -> None:
        requested = bool(msg.data)

        if requested != self.exterior_requested:
            self.exterior_requested = requested
            self.pending_switch = True
            wanted = "exterior" if requested else "indoor"
            self.get_logger().info(f"Environment changed: requesting {wanted} SLAM")

    def indoor_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        self.indoor_pose = pose_to_matrix(
            msg.pose.pose.position, msg.pose.pose.orientation
        )
        self.indoor_stamp_ns = self.stamp_to_ns(msg.header.stamp)

    def exterior_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        self.exterior_pose = pose_to_matrix(
            msg.pose.pose.position, msg.pose.pose.orientation
        )
        self.exterior_stamp_ns = self.stamp_to_ns(msg.header.stamp)

    def odom_callback(self, msg: Odometry) -> None:
        self.odom_base = pose_to_matrix(
            msg.pose.pose.position, msg.pose.pose.orientation
        )
        self.odom_stamp_ns = self.stamp_to_ns(msg.header.stamp)

    def source_is_fresh(self, stamp_ns: Optional[int]) -> bool:
        if stamp_ns is None:
            return False
        age_sec = (self.now_ns() - stamp_ns) / 1e9
        return 0.0 <= age_sec <= self.source_timeout

    def requested_source(self) -> Tuple[str, Optional[np.ndarray], Optional[int]]:
        if self.exterior_requested:
            return "exterior", self.exterior_pose, self.exterior_stamp_ns
        return "indoor", self.indoor_pose, self.indoor_stamp_ns

    def apply_planar_constraint(self, transform: np.ndarray) -> np.ndarray:
        if not self.force_planar:
            return transform

        output = transform.copy()
        yaw = math.atan2(output[1, 0], output[0, 0])
        c = math.cos(yaw)
        s = math.sin(yaw)

        output[:3, :3] = np.array([
            [c, -s, 0.0],
            [s,  c, 0.0],
            [0.0, 0.0, 1.0],
        ])
        output[2, 3] = 0.0
        return output

    def publish_transform(self) -> None:
        source_name, raw_map_base, source_stamp = self.requested_source()

        odom_fresh = self.source_is_fresh(self.odom_stamp_ns)
        source_fresh = self.source_is_fresh(source_stamp)

        if self.odom_base is None or not odom_fresh:
            self.publish_last_if_allowed("odometry unavailable or stale")
            return

        if raw_map_base is None or not source_fresh:
            self.publish_last_if_allowed(f"{source_name} SLAM unavailable or stale")
            return

        if self.pending_switch or self.active_source != source_name:
            if self.preserve_continuity and self.last_map_base_output is not None:
                self.switch_offset = (
                    self.last_map_base_output @ np.linalg.inv(raw_map_base)
                )
                self.get_logger().info(
                    f"Switching to {source_name} SLAM with continuity offset"
                )
            else:
                self.switch_offset = np.eye(4, dtype=float)
                self.get_logger().info(
                    f"Switching to {source_name} SLAM without continuity offset"
                )

            self.active_source = source_name
            self.pending_switch = False

        corrected_map_base = self.switch_offset @ raw_map_base
        corrected_map_base = self.apply_planar_constraint(corrected_map_base)

        # T_map_odom = T_map_base * inverse(T_odom_base)
        map_odom = corrected_map_base @ np.linalg.inv(self.odom_base)
        map_odom = self.apply_planar_constraint(map_odom)

        self.last_map_base_output = corrected_map_base
        self.last_map_odom_output = map_odom

        self.broadcast_matrix(map_odom)

        source_msg = String()
        source_msg.data = self.active_source or "none"
        self.source_pub.publish(source_msg)

    def publish_last_if_allowed(self, reason: str) -> None:
        if self.keep_last and self.last_map_odom_output is not None:
            self.broadcast_matrix(self.last_map_odom_output)
            return

        self.get_logger().warning(
            f"TF not published: {reason}",
            throttle_duration_sec=2.0,
        )

    def broadcast_matrix(self, transform: np.ndarray) -> None:
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.odom_frame

        msg.transform.translation.x = float(transform[0, 3])
        msg.transform.translation.y = float(transform[1, 3])
        msg.transform.translation.z = float(transform[2, 3])

        q = matrix_to_quaternion(transform[:3, :3])
        msg.transform.rotation.x = float(q[0])
        msg.transform.rotation.y = float(q[1])
        msg.transform.rotation.z = float(q[2])
        msg.transform.rotation.w = float(q[3])

        self.tf_broadcaster.sendTransform(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SlamSelectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
