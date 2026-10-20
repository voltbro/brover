#!/usr/bin/env python3
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool


class OutputService:
    """Send the latest desired value without blocking joystick callbacks."""

    def __init__(self, node, service_name):
        self.node = node
        self.service_name = service_name
        self.client = node.create_client(SetBool, service_name)
        self.timeout = float(node.get_parameter("output_service_timeout").value)
        self.retry_period = float(node.get_parameter("output_retry_period").value)
        if any(not math.isfinite(v) or v <= 0 for v in (self.timeout, self.retry_period)):
            raise ValueError("Output service timeout and retry period must be positive")

        # No command exists until the operator uses this control.
        self.desired = None
        self.last_successful = None
        self.pending = None
        self.deadline = 0.0
        self.retry_after = 0.0

    def update(self):
        if self.desired is None:
            return
        now = time.monotonic()
        if self.pending is not None:
            if now < self.deadline:
                return
            self.cancel_pending()
            self.failed("response timeout")
            return

        if self.desired == self.last_successful or now < self.retry_after:
            return
        if not self.client.service_is_ready():
            self.retry_after = now + self.retry_period
            return

        value = bool(self.desired)
        request = SetBool.Request()
        request.data = value
        try:
            future = self.client.call_async(request)
        except Exception as exc:
            self.failed(str(exc))
            return

        # Once sent, a missing reply leaves the remote state uncertain.
        self.last_successful = None
        self.pending = future
        self.deadline = now + self.timeout
        future.add_done_callback(
            lambda completed: self.on_response(completed, value)
        )

    def on_response(self, future, value):
        # Ignore responses from requests already removed after a timeout.
        if future is not self.pending:
            return
        self.pending = None
        try:
            response = future.result()
            if response is None:
                self.failed("empty ROS service response")
            elif response.success:
                self.last_successful = value
                self.retry_after = 0.0
            else:
                self.failed(response.message)
        except Exception as exc:
            self.failed(str(exc))

    def failed(self, message):
        self.retry_after = time.monotonic() + self.retry_period
        self.node.get_logger().warning(
            f"{self.service_name} request failed: {message}"
        )

    def cancel_pending(self):
        future = self.pending
        self.pending = None
        if future is not None:
            self.client.remove_pending_request(future)
            future.cancel()


class RadiolinkController(Node):
    def __init__(self):
        super().__init__("radiolink")

        self.declare_parameter("joy_topic", "joy")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("rele_service", "/rele")
        self.declare_parameter("light_service", "/light")

        self.declare_parameter("publish_period", 0.05)
        self.declare_parameter("joystick_timeout", 0.5)
        self.declare_parameter("light_toggle_timeout", 0.5)
        self.declare_parameter("output_service_timeout", 5.0)
        self.declare_parameter("output_retry_period", 1.0)

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

        self.rele_output = OutputService(
            self, self.get_parameter("rele_service").value,
        )
        self.light_output = OutputService(
            self, self.get_parameter("light_service").value,
        )

        self.enabled = False
        self.last_joy_time = None

        self.rele_was_pressed = None
        self.light_was_pressed = None
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

        rele_pressed = msg.axes[self.rele_axis] > 0
        # The first Joy message establishes the initial switch position.
        # Only a subsequent change creates a relay command.
        if (
            self.rele_was_pressed is not None
            and rele_pressed != self.rele_was_pressed
        ):
            self.rele_output.desired = rele_pressed
        self.rele_was_pressed = rele_pressed

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
        if self.light_was_pressed is None:
            self.light_was_pressed = light_pressed
            return

        if (
            light_pressed
            and not self.light_was_pressed
            and self.light_toggle_allowed()
        ):
            self.light_output.desired = not self.light_output.desired
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
        self.rele_output.update()
        self.light_output.update()

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
        radiolink.rele_output.cancel_pending()
        radiolink.light_output.cancel_pending()
        radiolink.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()