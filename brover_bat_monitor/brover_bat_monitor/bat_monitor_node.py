#!/usr/bin/env python3
import rclpy
from cyphal_ros2_bridge.srv import CallHMILed
from rclpy.node import Node
from sensor_msgs.msg import BatteryState


class LowBatteryLedBlinker(Node):
    def __init__(self):
        super().__init__("bat_monitor")

        self.declare_parameter("battery_topics", ["/bat_1", "/bat_2"])
        self.declare_parameter("led_service", "/hmi/led")
        self.declare_parameter("voltage_threshold", 19.5)
        self.declare_parameter("blink_period", 0.25)
        self.declare_parameter("led_interface", 1)

        self.battery_topics = self.get_parameter("battery_topics").value
        self.voltage_threshold = self.get_parameter("voltage_threshold").value
        self.led_interface = self.get_parameter("led_interface").value

        self.voltages = {
            topic: None for topic in self.battery_topics
        }

        self.battery_alarm = False
        self.led_state = False
        self.pending_led_call = None
        self.last_led = None

        for topic in self.battery_topics:
            self.create_subscription(
                BatteryState,
                topic,
                lambda msg, battery_topic=topic: self.battery_callback(
                    msg, battery_topic
                ),
                10,
            )

        self.led_client = self.create_client(
            CallHMILed,
            self.get_parameter("led_service").value,
        )

        self.create_timer(
            self.get_parameter("blink_period").value,
            self.timer_callback,
        )

        self.get_logger().info(
            "Battery monitor started: "
            f"{self.battery_topics}, threshold={self.voltage_threshold:.2f} V"
        )

    def battery_callback(self, msg, battery_topic):
        self.voltages[battery_topic] = msg.voltage

        if any(voltage is None for voltage in self.voltages.values()):
            return

        previous_alarm = self.battery_alarm

        self.battery_alarm = all(
            voltage < self.voltage_threshold
            for voltage in self.voltages.values()
        )

        if self.battery_alarm and not previous_alarm:
            self.get_logger().warning(
                "Battery alarm: "
                + ", ".join(
                    f"{topic}={voltage:.2f} V"
                    for topic, voltage in self.voltages.items()
                )
            )

        elif previous_alarm and not self.battery_alarm:
            self.get_logger().info(
                "Battery alarm cleared: "
                + ", ".join(
                    f"{topic}={voltage:.2f} V"
                    for topic, voltage in self.voltages.items()
                )
            )
            self.led_state = False

    def timer_callback(self):
        if (
            self.pending_led_call is not None
            and not self.pending_led_call.done()
        ):
            return

        if self.battery_alarm:
            self.led_state = not self.led_state

            if self.led_state:
                self.send_led(255, 0, 0)
            else:
                self.send_led(0, 0, 0)

            return

        self.send_led(0, 255, 0)

    def send_led(self, r, g, b):
        desired_led = (r, g, b)

        if desired_led == self.last_led and not self.battery_alarm:
            return

        if (
            self.pending_led_call is not None
            and not self.pending_led_call.done()
        ):
            return

        if not self.led_client.service_is_ready():
            self.get_logger().warning("/hmi/led service is not ready")
            return

        request = CallHMILed.Request()
        request.led.r = r
        request.led.g = g
        request.led.b = b
        request.led.interface = self.led_interface

        self.last_led = desired_led
        self.pending_led_call = self.led_client.call_async(request)
        self.pending_led_call.add_done_callback(
            self.service_response_callback
        )

    def service_response_callback(self, future):
        try:
            future.result()
        except Exception as exc:
            self.last_led = None
            self.get_logger().error(f"LED service call failed: {exc}")
        finally:
            self.pending_led_call = None


def main(args=None):
    rclpy.init(args=args)
    node = LowBatteryLedBlinker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_led(0, 0, 0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
