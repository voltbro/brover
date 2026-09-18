import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32


class RobotMover(Node):
    def __init__(self):
        super().__init__("move_publisher")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("wheel_velocity_topic_prefix", "/m_vel")

        self.declare_parameter("max_wheel_velocity", 4.8)
        self.declare_parameter("min_wheel_velocity", 0.1)

        self.declare_parameter("publish_period", 0.05)
        self.declare_parameter("cmd_vel_timeout", 0.25)

        self.max_wheel_velocity = self.get_parameter(
            "max_wheel_velocity"
        ).value

        self.min_wheel_velocity = self.get_parameter(
            "min_wheel_velocity"
        ).value

        self.cmd_vel_timeout = self.get_parameter(
            "cmd_vel_timeout"
        ).value

        self.check_parameters()

        topic_prefix = self.get_parameter(
            "wheel_velocity_topic_prefix"
        ).value

        self.wheel_publishers = [
            self.create_publisher(Float32, f"{topic_prefix}{idx}", 10)
            for idx in range(1, 7)
        ]

        self.left_velocity = Float32()
        self.right_velocity = Float32()

        self.last_cmd_time = self.get_clock().now()

        self.create_subscription(
            Twist,
            self.get_parameter("cmd_vel_topic").value,
            self.cmd_vel_callback,
            10,
        )

        self.create_timer(
            self.get_parameter("publish_period").value,
            self.timer_callback,
        )

        self.get_logger().info(
            "Move controller started: "
            f"cmd_vel=-1.0...1.0, "
            f"wheel velocity=-{self.max_wheel_velocity:.1f}"
            f"...{self.max_wheel_velocity:.1f} rad/s"
        )

    def check_parameters(self):
        if self.min_wheel_velocity < 0.0:
            raise ValueError(
                "min_wheel_velocity must be non-negative"
            )

        if self.max_wheel_velocity <= 0.0:
            raise ValueError(
                "max_wheel_velocity must be positive"
            )

        if self.min_wheel_velocity > self.max_wheel_velocity:
            raise ValueError(
                "min_wheel_velocity must not be greater than "
                "max_wheel_velocity"
            )

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = self.get_clock().now()

        linear = float(msg.linear.x)
        angular = float(msg.angular.z)

        if not math.isfinite(linear) or not math.isfinite(angular):
            self.get_logger().warning(
                "Non-finite cmd_vel received, stopping"
            )
            self.set_wheel_velocities(0.0, 0.0)
            return

        linear = self.clamp(linear, -1.0, 1.0)
        angular = self.clamp(angular, -1.0, 1.0)

        left = linear - angular
        right = linear + angular

        maximum_requested = max(abs(left), abs(right))

        if maximum_requested > 1.0:
            left /= maximum_requested
            right /= maximum_requested

        left = self.normalized_to_wheel_velocity(left)
        right = self.normalized_to_wheel_velocity(right)

        self.set_wheel_velocities(left, right)

    def normalized_to_wheel_velocity(self, value):
        value = self.clamp(value, -1.0, 1.0)

        velocity = value * self.max_wheel_velocity

        if abs(velocity) < self.min_wheel_velocity:
            return 0.0

        return velocity

    def timer_callback(self):
        if self.is_cmd_vel_stale():
            self.set_wheel_velocities(0.0, 0.0)

        self.publish_wheel_velocities()

    def publish_wheel_velocities(self):
        for idx, publisher in enumerate(self.wheel_publishers):
            if idx < 3:
                publisher.publish(self.left_velocity)
            else:
                publisher.publish(self.right_velocity)

    def is_cmd_vel_stale(self):
        elapsed = (
            self.get_clock().now() - self.last_cmd_time
        ).nanoseconds / 1e9

        return elapsed > self.cmd_vel_timeout

    def set_wheel_velocities(self, left, right):
        self.left_velocity.data = float(left)
        self.right_velocity.data = float(right)

    @staticmethod
    def clamp(value, minimum, maximum):
        return max(minimum, min(maximum, value))


def main(args=None):
    rclpy.init(args=args)
    robot_mover = RobotMover()

    try:
        rclpy.spin(robot_mover)
    except KeyboardInterrupt:
        pass
    finally:
        robot_mover.set_wheel_velocities(0.0, 0.0)
        robot_mover.publish_wheel_velocities()
        robot_mover.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()