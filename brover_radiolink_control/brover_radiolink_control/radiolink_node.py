import math
from time import monotonic

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy

from .arm_control import ArmCan, ArmControls, HOLD_RATES, switch_position


MODE_SETTLE_SECONDS = 0.3


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
        self.declare_parameter("arm_enable_axis", 5)
        self.declare_parameter("arm_enable_button", -1)
        self.declare_parameter("arm_enable_direction", 1.0)
        self.declare_parameter("arm_left_vertical_axis", 2)
        self.declare_parameter("arm_right_horizontal_axis", 0)
        self.declare_parameter("arm_deadzone", 0.12)

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
        self.home_sent = False
        self.arm_command = (False, HOLD_RATES)
        self.mode = None
        self.mode_since = 0.0
        self.mode_ready = False
        self.drive_ready = False
        self.next_can_error_log = 0.0

        self.left_vertical = self.get_parameter("arm_left_vertical_axis").value
        self.right_horizontal = self.get_parameter("arm_right_horizontal_axis").value
        self.arm_can = ArmCan()
        self.arm = ArmControls(self.get_parameter("arm_deadzone").value)
        self.arm_enable_axis = self.get_parameter("arm_enable_axis").value
        self.arm_enable_button = self.get_parameter("arm_enable_button").value
        self.arm_enable_direction = self.get_parameter("arm_enable_direction").value
        if not -1.0 <= self.safety_threshold < -0.2:
            raise ValueError("safety_threshold must be in [-1, -0.2)")
        if self.arm_enable_direction not in (-1.0, 1.0):
            raise ValueError("arm_enable_direction must be -1 or 1")

        self.create_subscription(
            Joy,
            self.get_parameter("joy_topic").value,
            self.joy_callback,
            1,
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
        if self.is_joystick_stale():
            self.reset_modes()
        self.last_joy_time = monotonic()

        if not self.has_required_axes(msg):
            self.reset_modes()
            self.enabled = False
            self.stop()
            return

        was_enabled = self.enabled
        self.update_modes(msg)
        if not self.enabled:
            if was_enabled:
                self.stop()
            return

        speed = self.select_speed(msg.axes[self.speed_axis])
        linear = msg.axes[self.linear_axis]
        angular = msg.axes[self.angular_axis]
        linear = 0.0 if abs(linear) <= self.arm.deadzone else linear
        angular = 0.0 if abs(angular) <= self.arm.deadzone else angular
        self.cmd_vel_msg.linear.x = (
            speed * self.linear_direction * linear
        )
        self.cmd_vel_msg.angular.z = (
            speed
            * self.rotation_scale
            * self.angular_direction
            * angular
        )

    def has_required_axes(self, msg):
        axes = [
            self.linear_axis,
            self.angular_axis,
            self.speed_axis,
            self.safety_axis,
        ]
        if (0 <= self.safety_axis < len(msg.axes)
                and msg.axes[self.safety_axis] >= -self.safety_threshold):
            axes.extend((self.left_vertical, self.right_horizontal))
        return all(
            0 <= axis < len(msg.axes)
            and math.isfinite(msg.axes[axis])
            and -1.0 <= msg.axes[axis] <= 1.0
            for axis in axes
        )

    def reset_modes(self):
        self.mode = None
        self.mode_ready = False
        self.drive_ready = False
        self.arm.reset()
        self.arm_command = (False, HOLD_RATES)

    def update_modes(self, msg):
        mode = switch_position(msg.axes[self.safety_axis], -self.safety_threshold)
        if mode != self.mode:
            self.mode = mode
            self.mode_since = monotonic()
            self.drive_ready = False
            self.arm.reset()
        if mode == 0:
            self.mode_ready = True

        left = msg.axes[self.angular_axis]
        right = msg.axes[self.linear_axis]
        mode_settled = monotonic() - self.mode_since >= MODE_SETTLE_SECONDS
        if self.mode_ready and mode == -1 and mode_settled:
            self.drive_ready |= max(abs(left), abs(right)) <= self.arm.deadzone
        self.enabled = self.drive_ready and mode == -1
        self.arm_command = (False, HOLD_RATES)
        if self.mode_ready and mode == 1 and mode_settled:
            bank = switch_position(msg.axes[self.speed_axis])
            pressed, released = self.arm_button(msg)
            self.arm_command = self.arm.command(
                bank, pressed, released, left, msg.axes[self.left_vertical],
                msg.axes[self.right_horizontal], right,
            )

    def arm_button(self, msg):
        if self.arm_enable_button >= 0:
            index = self.arm_enable_button
            if index >= len(msg.buttons):
                return False, False
            return msg.buttons[index] == 1, msg.buttons[index] == 0
        if not 0 <= self.arm_enable_axis < len(msg.axes):
            return False, False
        value = self.arm_enable_direction * msg.axes[self.arm_enable_axis]
        if not -1.0 <= value <= 1.0:
            return False, False
        return value >= 0.8, value <= -0.8

    def select_speed(self, speed_axis_value):
        if speed_axis_value > 0.5:
            return self.slow_speed
        if speed_axis_value < -0.5:
            return self.fast_speed
        return self.normal_speed

    def timer_callback(self):
        stale = self.is_joystick_stale()
        if stale:
            self.reset_modes()
        self.send_arm_command()

        if not self.initial_stop_sent:
            self.stop()
            self.initial_stop_sent = True
            return

        if stale:
            if self.enabled:
                self.enabled = False
                self.stop()
            return

        if self.enabled:
            self.cmd_vel_pub.publish(self.cmd_vel_msg)

    def is_joystick_stale(self):
        if self.last_joy_time is None:
            return True
        return monotonic() - self.last_joy_time > self.joystick_timeout

    def send_arm_command(self):
        try:
            home = self.arm_command[0] == 2
            if home and not self.home_sent:
                # Home requires a recent Hold after a CAN timeout.
                self.arm_can.send(False, HOLD_RATES)
            self.arm_can.send(*self.arm_command)
            self.home_sent = home
        except (OSError, AttributeError) as error:
            self.home_sent = False
            self.arm.reset()
            self.arm_command = (False, HOLD_RATES)
            now = monotonic()
            if now >= self.next_can_error_log:
                self.get_logger().warning(f"ServoBoard CAN: {error}")
                self.next_can_error_log = now + 5.0

    def destroy_node(self):
        self.arm_command = (False, HOLD_RATES)
        self.send_arm_command()
        self.arm_can.close()
        return super().destroy_node()

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
