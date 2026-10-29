"""Публикация событий Linux joystick в формате sensor_msgs/Joy без SDL."""

import glob
import os
import select
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


JOYSTICK_PATTERN = "/dev/input/by-id/*Universal_RC_Joystick*-joystick"
EVENT = struct.Struct("<IhBB")
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80
AXIS_SCALE = 32767.0


class DirectJoystick(Node):
    def __init__(self):
        super().__init__("joy")
        self.declare_parameter("device_pattern", JOYSTICK_PATTERN)
        self.declare_parameter("publish_period", 0.05)
        self.device_pattern = self.get_parameter("device_pattern").value
        self.device_fd = None
        self.axes = [0.0] * 8
        self.buttons = [0]
        self.last_connect_attempt = 0.0
        self.received_control_input = False
        self.waiting_for_device = False
        self.publisher = self.create_publisher(Joy, "/joy", 10)
        self.create_timer(
            self.get_parameter("publish_period").value,
            self.update,
        )

    def update(self):
        self.open_device()
        if self.device_fd is None:
            return

        try:
            self.read_events()
        except OSError as error:
            self.get_logger().warning(f"Джойстик отключён: {error}")
            self.close_device()
            return

        # Linux передаёт положение стиков только при его изменении. После
        # первого события передатчика повторяем состояние, как joy_node.
        if not self.received_control_input:
            return

        message = Joy()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "joy"
        message.axes = self.axes
        message.buttons = self.buttons
        self.publisher.publish(message)

    def open_device(self):
        if self.device_fd is not None:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_connect_attempt < 1.0:
            return
        self.last_connect_attempt = now

        try:
            device_path = self.find_device_path()
            if device_path is None:
                raise FileNotFoundError
            self.device_fd = os.open(device_path, os.O_RDONLY | os.O_NONBLOCK)
            self.received_control_input = False
            self.get_logger().info(f"Джойстик подключён: {device_path}")
            self.waiting_for_device = False
        except OSError:
            if not self.waiting_for_device:
                self.get_logger().warning(
                    f"Ожидание джойстика: {self.device_pattern}"
                )
                self.waiting_for_device = True

    def find_device_path(self):
        paths = sorted(
            path
            for path in glob.glob(self.device_pattern)
            if not path.endswith("-event-joystick")
        )
        return paths[0] if paths else None

    def read_events(self):
        while select.select([self.device_fd], [], [], 0.0)[0]:
            event = os.read(self.device_fd, EVENT.size)
            if len(event) != EVENT.size:
                raise OSError("неполное событие джойстика")
            _, value, event_type, number = EVENT.unpack(event)
            is_initial = bool(event_type & JS_EVENT_INIT)
            event_type &= ~JS_EVENT_INIT

            if event_type == JS_EVENT_AXIS:
                self.ensure_axis(number)
                self.axes[number] = max(-1.0, min(1.0, value / AXIS_SCALE))
            elif event_type == JS_EVENT_BUTTON:
                self.ensure_button(number)
                self.buttons[number] = value

            if not is_initial:
                self.received_control_input = True

    def ensure_axis(self, number):
        if number >= len(self.axes):
            self.axes.extend([0.0] * (number + 1 - len(self.axes)))

    def ensure_button(self, number):
        if number >= len(self.buttons):
            self.buttons.extend([0] * (number + 1 - len(self.buttons)))

    def close_device(self):
        if self.device_fd is not None:
            os.close(self.device_fd)
            self.device_fd = None
        self.received_control_input = False

    def destroy_node(self):
        self.close_device()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DirectJoystick()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
