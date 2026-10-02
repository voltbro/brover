#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from std_srvs.srv import Empty
from tf_transformations import euler_from_quaternion


class OdomPose2D(Node):
    def __init__(self):
        super().__init__("odom_pose2d_node")

        self.declare_parameter("imu_topic", "/bhi360/imu")
        self.declare_parameter("odom_topic", "/odom_pose2d")
        self.declare_parameter("wheel_odom_topic_prefix", "/m_odom")
        self.declare_parameter("wheel_radius", 0.0625)
        self.declare_parameter("publish_period", 0.02)
        self.declare_parameter("max_dt", 0.2)

        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.max_dt = self.get_parameter("max_dt").value

        self.wheel_omegas = [0.0] * 6
        self.x = 0.0
        self.y = 0.0
        self.imu_yaw = 0.0
        self.yaw_offset = 0.0
        self.last_time = self.get_clock().now()

        self.create_subscription(
            Imu,
            self.get_parameter("imu_topic").value,
            self.imu_callback,
            20,
        )

        wheel_prefix = self.get_parameter("wheel_odom_topic_prefix").value
        for idx in range(6):
            self.create_subscription(
                Float32,
                f"{wheel_prefix}{idx + 1}",
                lambda msg, wheel_idx=idx: self.wheel_callback(wheel_idx, msg),
                10,
            )

        self.pub_pose = self.create_publisher(
            Pose2D,
            self.get_parameter("odom_topic").value,
            20,
        )
        self.create_timer(
            self.get_parameter("publish_period").value,
            self.update_odometry,
        )
        self.create_service(Empty, "/odom/reset", self.handle_reset_odom)

        self.get_logger().info("Pose2D node started with reset service")

    def handle_reset_odom(self, request, response):
        self.get_logger().info("Odometry reset requested")
        self.x = 0.0
        self.y = 0.0
        self.yaw_offset = self.imu_yaw
        self.last_time = self.get_clock().now()
        return response

    def wheel_callback(self, idx, msg):
        try:
            self.wheel_omegas[idx] = float(msg.data)
        except (TypeError, ValueError) as exc:
            self.get_logger().warning(f"Invalid wheel msg on idx {idx}: {exc}")

    def imu_callback(self, msg):
        quaternion = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        ]
        self.imu_yaw = euler_from_quaternion(quaternion)[2]

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def update_odometry(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if dt <= 0.0 or dt > self.max_dt:
            return

        left_omega = sum(self.wheel_omegas[:3]) / 3.0
        right_omega = -sum(self.wheel_omegas[3:]) / 3.0

        left_velocity = left_omega * self.wheel_radius
        right_velocity = right_omega * self.wheel_radius
        linear_velocity = (left_velocity + right_velocity) / 2.0
        yaw = self.normalize_angle(self.imu_yaw - self.yaw_offset)

        self.x += linear_velocity * math.cos(yaw) * dt
        self.y += linear_velocity * math.sin(yaw) * dt

        pose = Pose2D()
        pose.x = float(self.x)
        pose.y = float(self.y)
        pose.theta = float(yaw)
        self.pub_pose.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = OdomPose2D()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
