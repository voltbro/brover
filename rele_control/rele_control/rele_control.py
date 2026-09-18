#!/usr/bin/env python3
import asyncio

import pycyphal.application
import rclpy
import uavcan.node
import uavcan.primitive.array
from rclpy.node import Node
from std_msgs.msg import Int8


CYPHAL_PORT_ID = 3000


class RelayLightCyphalNode(Node):
    def __init__(self, cyphal_publisher):
        super().__init__("rele_control")

        self.declare_parameter("publish_period", 0.05)

        publish_period = float(
            self.get_parameter("publish_period").value
        )

        self.rele = 0
        self.light = 0

        self.create_subscription(
            Int8,
            "/rele",
            self.rele_callback,
            10,
        )

        self.create_subscription(
            Int8,
            "/light",
            self.light_callback,
            10,
        )

        self.cyphal_publisher = cyphal_publisher

        self.create_timer(
            publish_period,
            self.publish_command,
        )

    def rele_callback(self, msg):
        self.rele = msg.data

    def light_callback(self, msg):
        self.light = msg.data

    def publish_command(self):
        command = uavcan.primitive.array.Integer8_1_0(
            value=[
                self.rele,
                self.light,
            ]
        )

        self.cyphal_publisher.publish_soon(command)


async def main_async(args=None):
    rclpy.init(args=args)

    cyphal_node = None
    ros_node = None

    try:
        node_info = uavcan.node.GetInfo_1_0.Response(
            software_version=uavcan.node.Version_1_0(
                major=1,
                minor=0,
            ),
            name="org.vbcores.relay_light_cyphal",
        )

        cyphal_node = pycyphal.application.make_node(
            node_info
        )

        cyphal_publisher = cyphal_node.make_publisher(
            uavcan.primitive.array.Integer8_1_0,
            CYPHAL_PORT_ID,
        )

        cyphal_node.start()

        ros_node = RelayLightCyphalNode(
            cyphal_publisher
        )

        while rclpy.ok():
            rclpy.spin_once(
                ros_node,
                timeout_sec=0.01,
            )

            await asyncio.sleep(0.001)

    finally:
        if ros_node is not None:
            ros_node.destroy_node()

        if cyphal_node is not None:
            cyphal_node.close()

        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()