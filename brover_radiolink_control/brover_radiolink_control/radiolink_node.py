import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy


class RadiolinkController(Node):
    def __init__(self):
        super().__init__("radiolink")

        self.declare_parameter("joy_topic", "joy")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("publish_period", 0.05)
        self.declare_parameter("joystick_timeout", 0.5)

        self.declare_parameter("slow_speed", 0.09375)
        self.declare_parameter("normal_speed", 0.1875)
        self.declare_parameter("fast_speed", 0.375)
        self.declare_parameter("rotation_scale", 2.5)
        self.declare_parameter("linear_direction", -1.0)
        self.declare_parameter("angular_direction", -1.0)

        self.declare_parameter("linear_axis", 1)
        self.declare_parameter("angular_axis", 3)
        self.declare_parameter("speed_axis", 6)

        self.declare_parameter("safety_axis", 4)
        self.declare_parameter("safety_threshold", -0.8)

        self.joystick_timeout = self.get_parameter("joystick_timeout").value
        self.slow_speed = self.get_parameter("slow_speed").value
        self.normal_speed = self.get_parameter("normal_speed").value
        self.fast_speed = self.get_parameter("fast_speed").value
        self.rotation_scale = self.get_parameter("rotation_scale").value
        self.linear_direction = self.get_parameter("linear_direction").value
        self.angular_direction = self.get_parameter("angular_direction").value
        self.linear_axis = self.get_parameter("linear_axis").value
        self.angular_axis = self.get_parameter("angular_axis").value
        self.speed_axis = self.get_parameter("speed_axis").value
        self.safety_axis = self.get_parameter("safety_axis").value
        self.safety_threshold = self.get_parameter("safety_threshold").value

        self.cmd_vel_msg = Twist()
        self.enabled = False
        self.last_joy_time = None
        self.initial_stop_sent = False

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
        self.create_timer(
            self.get_parameter("publish_period").value,
            self.timer_callback,
        )

    def joy_callback(self, msg):
        self.last_joy_time = self.get_clock().now()

        if not self.has_required_axes(msg):
            self.get_logger().warning(
                "В сообщении Joy недостаточно осей, остановка"
            )
            self.enabled = False
            self.stop()
            return

        was_enabled = self.enabled
        self.enabled = self.is_safety_enabled(msg)
        if not self.enabled:
            if was_enabled:
                self.stop()
            return

        speed = self.select_speed(msg.axes[self.speed_axis])
        self.cmd_vel_msg.linear.x = (
            speed * self.linear_direction * msg.axes[self.linear_axis]
        )
        self.cmd_vel_msg.angular.z = (
            speed
            * self.rotation_scale
            * self.angular_direction
            * msg.axes[self.angular_axis]
        )

    def has_required_axes(self, msg):
        max_axis = max(
            self.linear_axis,
            self.angular_axis,
            self.speed_axis,
            self.safety_axis,
        )
        return len(msg.axes) > max_axis

    def is_safety_enabled(self, msg):
        return msg.axes[self.safety_axis] <= self.safety_threshold

    def select_speed(self, speed_axis_value):
        if speed_axis_value > 0.5:
            return self.slow_speed
        if speed_axis_value < -0.5:
            return self.fast_speed
        return self.normal_speed

    def timer_callback(self):
        if not self.initial_stop_sent:
            self.stop()
            self.initial_stop_sent = True
            return

        if self.is_joystick_stale():
            if self.enabled:
                self.enabled = False
                self.stop()
            return

        if self.enabled:
            self.cmd_vel_pub.publish(self.cmd_vel_msg)

    def is_joystick_stale(self):
        if self.last_joy_time is None:
            return True
        elapsed = (
            self.get_clock().now() - self.last_joy_time
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
