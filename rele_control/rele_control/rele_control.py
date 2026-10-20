#!/usr/bin/env python3
"""Forward ROS relay/light service requests to Cyphal register Access."""

import asyncio
import json
import math

import pycyphal.application
import pycyphal.dsdl
import rclpy
import uavcan.node
import uavcan.register as register
from rclpy.node import Node
from rclpy.task import Future
from std_srvs.srv import SetBool, Trigger


CYPHAL_SERVER_NODE_ID = 7
CYPHAL_SERVICE_ID = 384
CYPHAL_RESPONSE_TIMEOUT = 2.0


class RelayLightCyphalNode(Node):
    """Return remote register responses without caching hardware state."""

    def __init__(self, cyphal_node):
        super().__init__('rele_control')
        self.cyphal_node = cyphal_node
        self.pending_calls = set()
        self.declare_parameter('cyphal_server_node_id', CYPHAL_SERVER_NODE_ID)
        self.declare_parameter('cyphal_response_timeout', CYPHAL_RESPONSE_TIMEOUT)
        self.server_node_id = self.get_parameter('cyphal_server_node_id').value
        self.response_timeout = float(
            self.get_parameter('cyphal_response_timeout').value
        )
        if not math.isfinite(self.response_timeout) or self.response_timeout <= 0:
            raise ValueError('cyphal_response_timeout must be finite and positive')

        self.create_service(SetBool, '/light', self.light_callback)
        self.create_service(SetBool, '/rele', self.rele_callback)
        self.create_service(Trigger, '/rele_control/state', self.state_callback)
        self.get_logger().info(
            'Relay/light services started; forwarding register Access '
            f'to Cyphal node {self.server_node_id}, service {CYPHAL_SERVICE_ID}'
        )

    async def light_callback(self, request, response):
        """Write natural8 [0/1] to output.light and return the remote reply."""
        return await self.forward('output.light', int(request.data), response)

    async def rele_callback(self, request, response):
        """Write natural8 [0/1] to output.rele and return the remote reply."""
        return await self.forward('output.rele', int(request.data), response)

    async def state_callback(self, request, response):
        """Read output.states using an empty register value."""
        return await self.forward('output.states', None, response)

    def forward(self, register_name, value, response):
        """Bridge an asyncio transport task to a ROS executor Future."""
        # rclpy drives service coroutines itself. Awaiting an asyncio network
        # future directly inside them would mix two incompatible schedulers.
        ros_future = Future()
        task = asyncio.create_task(self.call_register(register_name, value, response))
        self.pending_calls.add(task)

        def complete(completed):
            self.pending_calls.discard(completed)
            if completed.cancelled():
                ros_future.cancel()
                return
            try:
                result = completed.result()
            except Exception as exc:
                ros_future.set_exception(exc)
            else:
                if not ros_future.cancelled():
                    ros_future.set_result(result)

        task.add_done_callback(complete)
        return ros_future

    async def call_register(self, register_name, value, response):
        """Make one Access call; serialize the full response to ROS message."""
        client = None
        try:
            # Use the same nested fields as the Yakut requests. Empty means
            # read-only; natural8 means write, followed by a readback.
            payload = {'empty': {}} if value is None else {
                'natural8': {'value': [value]},
            }
            request = pycyphal.dsdl.update_from_builtin(
                register.Access_1_0.Request(),
                {'name': {'name': register_name}, 'value': payload},
            )
            # Each asyncio task owns its client proxy, as required by PyCyphal.
            client = self.cyphal_node.make_client(
                register.Access_1_0,
                self.server_node_id,
                CYPHAL_SERVICE_ID,
            )
            client.response_timeout = self.response_timeout
            result = await client.call(request)
            if result is None:
                response.success = False
                response.message = json.dumps({
                    'error': 'Cyphal response timeout',
                    'node_id': self.server_node_id,
                    'register': register_name,
                })
                self.get_logger().warning(response.message)
            else:
                cyphal_response, _metadata = result
                response.message = json.dumps(
                    pycyphal.dsdl.to_builtin(cyphal_response),
                    ensure_ascii=False,
                )
                response.success = True
        except Exception as exc:
            response.success = False
            response.message = json.dumps({
                'error': str(exc),
                'node_id': self.server_node_id,
                'register': register_name,
            }, ensure_ascii=False)
            self.get_logger().error(response.message)
        finally:
            if client is not None:
                client.close()
        return response

    async def stop_pending_calls(self):
        """Finish transport cleanup before closing the Cyphal node."""
        tasks = list(self.pending_calls)
        for task in tasks:
            task.cancel()
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)
        self.pending_calls.clear()


async def main_async(args=None):
    """Give both the ROS executor and Cyphal asyncio transport time to run."""
    rclpy.init(args=args)
    cyphal_node = None
    ros_node = None
    try:
        node_info = uavcan.node.GetInfo_1_0.Response(
            software_version=uavcan.node.Version_1_0(major=1, minor=0),
            name='org.vbcores.relay_light_cyphal',
        )
        cyphal_node = pycyphal.application.make_node(node_info)
        cyphal_node.start()
        ros_node = RelayLightCyphalNode(cyphal_node)

        while rclpy.ok():
            rclpy.spin_once(ros_node, timeout_sec=0.01)
            await asyncio.sleep(0.001)
    finally:
        if ros_node is not None:
            await ros_node.stop_pending_calls()
            ros_node.destroy_node()
        if cyphal_node is not None:
            cyphal_node.close()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    """Run until interrupted."""
    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
