import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8


class RadiolinkController(Node):
    def __init__(self):
        super().__init__("radiolink")

        self.declare_parameter("joy_topic", "joy")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("rele_topic", "/rele")
        self.declare_parameter("light_topic", "/light")

        self.declare_parameter("publish_period", 0.05)
        self.declare_parameter("joystick_timeout", 0.5)
        self.declare_parameter("light_toggle_timeout", 0.5)

        self.declare_parameter("slow_speed", 0.25)
        self.declare_parameter("normal_speed", 0.5)
        self.declare_parameter("fast_speed", 1.0)

        self.declare_parameter("linear_axis", 1)
        self.declare_parameter("angular_axis", 3)
        self.declare_parameter("speed_axis", 6)

        self.declare_parameter("rele_axis", 7)
        self.declare_parameter("light_axis", 5)

        self.declare_parameter("safety_axis_left", 2)
        self.declare_parameter("safety_axis_right", 4)
        self.declare_parameter("safety_threshold", 0.8)

        self.joystick_timeout = self.get_parameter(
            "joystick_timeout"
        ).value

        self.light_toggle_timeout = self.get_parameter(
            "light_toggle_timeout"
        ).value

        self.slow_speed = self.get_parameter("slow_speed").value
        self.normal_speed = self.get_parameter("normal_speed").value
        self.fast_speed = self.get_parameter("fast_speed").value

        self.linear_axis = self.get_parameter("linear_axis").value
        self.angular_axis = self.get_parameter("angular_axis").value
        self.speed_axis = self.get_parameter("speed_axis").value

        self.rele_axis = self.get_parameter("rele_axis").value
        self.light_axis = self.get_parameter("light_axis").value

        self.safety_axis_left = self.get_parameter(
            "safety_axis_left"
        ).value

        self.safety_axis_right = self.get_parameter(
            "safety_axis_right"
        ).value

        self.safety_threshold = self.get_parameter(
            "safety_threshold"
        ).value

        self.cmd_vel_msg = Twist()

        self.rele_msg = Int8()
        self.rele_msg.data = 0

        self.light_msg = Int8()
        self.light_msg.data = 0

        self.enabled = False
        self.last_joy_time = None

        self.light_was_pressed = False
        self.last_light_toggle_time = None

        self.create_subscription(
            Joy,
            self.get_parameter("joy_topic").value,
            self.joy_callback,
            10,
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.get_parameter("cmd_vel_topic").value,
            10,
        )

        self.rele_pub = self.create_publisher(
            Int8,
            self.get_parameter("rele_topic").value,
            10,
        )

        self.light_pub = self.create_publisher(
            Int8,
            self.get_parameter("light_topic").value,
            10,
        )

        self.create_timer(
            self.get_parameter("publish_period").value,
            self.timer_callback,
        )

    def joy_callback(self, msg):
        self.last_joy_time = self.get_clock().now()

        if not self.has_required_axes(msg):
            self.get_logger().warning(
                "Joy message has too few axes, stopping"
            )
            self.enabled = False
            self.stop()
            return

        if msg.axes[self.rele_axis] > 0:
            self.rele_msg.data = 1
        else:
            self.rele_msg.data = 0

        self.update_light(
            msg.axes[self.light_axis]
        )

        self.enabled = self.is_safety_enabled(msg)

        if not self.enabled:
            self.stop()
            return

        speed = self.select_speed(
            msg.axes[self.speed_axis]
        )

        linear_axis = self.normalize_axis(
            msg.axes[self.linear_axis]
        )

        angular_axis = self.normalize_axis(
            msg.axes[self.angular_axis]
        )

        self.cmd_vel_msg.linear.x = speed * linear_axis
        self.cmd_vel_msg.angular.z = speed * angular_axis

    def update_light(self, light_axis_value):
        light_pressed = light_axis_value < 0

        if (
            light_pressed
            and not self.light_was_pressed
            and self.light_toggle_allowed()
        ):
            self.light_msg.data = 1 - self.light_msg.data
            self.last_light_toggle_time = self.get_clock().now()

        self.light_was_pressed = light_pressed

    def light_toggle_allowed(self):
        if self.last_light_toggle_time is None:
            return True

        elapsed = (
            self.get_clock().now()
            - self.last_light_toggle_time
        ).nanoseconds / 1e9

        return elapsed >= self.light_toggle_timeout

    def has_required_axes(self, msg):
        max_axis = max(
            self.linear_axis,
            self.angular_axis,
            self.speed_axis,
            self.rele_axis,
            self.light_axis,
            self.safety_axis_left,
            self.safety_axis_right,
        )

        return len(msg.axes) > max_axis

    def is_safety_enabled(self, msg):
        return (
            msg.axes[self.safety_axis_left] < 1.0
            and msg.axes[self.safety_axis_right] < 1.0
            and msg.axes[self.safety_axis_right]
            >= self.safety_threshold
        )

    def select_speed(self, speed_axis_value):
        if speed_axis_value > 0.5:
            return self.slow_speed

        if speed_axis_value < -0.5:
            return self.fast_speed

        return self.normal_speed

    def normalize_axis(self, value):
        value = float(value)

        if not math.isfinite(value):
            return 0.0

        return max(-1.0, min(1.0, value))

    def timer_callback(self):
        self.rele_pub.publish(self.rele_msg)
        self.light_pub.publish(self.light_msg)

        if self.is_joystick_stale():
            self.enabled = False
            self.stop()
            return

        if self.enabled:
            self.cmd_vel_pub.publish(self.cmd_vel_msg)

    def is_joystick_stale(self):
        if self.last_joy_time is None:
            return False

        elapsed = (
            self.get_clock().now()
            - self.last_joy_time
        ).nanoseconds / 1e9

        return elapsed > self.joystick_timeout

    def stop(self):
        self.cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(self.cmd_vel_msg)


def main(args=None):
    rclpy.init(args=args)
    radiolink = RadiolinkController()

    try:
        rclpy.spin(radiolink)
    except KeyboardInterrupt:
        pass
    finally:
        radiolink.stop()
        radiolink.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()