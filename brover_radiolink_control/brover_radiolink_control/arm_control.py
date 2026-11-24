"""Radiolink arm mode and its SocketCAN command format."""

import math
import socket
import struct


ARM_COMMAND_ID = 0x520
HOLD_RATES = (0,) * 6
CAN_FRAME = struct.Struct("=IB3x8s")
PAYLOAD = struct.Struct("=BB6b")


def arm_frame(active, rates):
    """Pack one standard 11-bit Classical CAN frame for the ServoBoard."""
    payload = PAYLOAD.pack(0xA2, int(active), *rates)
    return CAN_FRAME.pack(ARM_COMMAND_ID, len(payload), payload)


def switch_position(value, end=0.8):
    """Return -1, 0, or +1 for a settled switch, or None between detents."""
    if value <= -end:
        return -1
    if value >= end:
        return 1
    if abs(value) <= 0.2:
        return 0
    return None


class ArmControls:
    """Use CH7 to select a right-stick mapping and CH6 to request the startup pose."""

    def __init__(self, deadzone):
        if not 0.0 < deadzone < 1.0:
            raise ValueError("arm_deadzone must be between 0 and 1")
        self.deadzone = deadzone
        self.reset()

    def reset(self):
        self.ready = False

    def command(self, bank, pressed, released, lx, ly, rx, ry):
        if bank not in (-1, 1):
            # Keep the board armed at zero speed while CH7 crosses the middle.
            return self.ready, HOLD_RATES

        base, shoulder, horizontal, vertical = map(self.rate, (lx, ly, rx, ry))
        neutral = not any((base, shoulder, horizontal, vertical))
        if not self.ready:
            self.ready = released and neutral
            return False, HOLD_RATES
        if pressed and neutral:
            return 2, HOLD_RATES
        if not released:
            self.ready = False
            return False, HOLD_RATES

        if bank == -1:
            return True, (base, shoulder, vertical, horizontal, 0, 0)
        return True, (base, shoulder, 0, 0, horizontal, vertical)

    def rate(self, value):
        magnitude = max(0.0, abs(value) - self.deadzone)
        return round(math.copysign(100 * magnitude / (1 - self.deadzone), value))


class ArmCan:
    """Send commands without blocking the ROS executor."""

    def __init__(self):
        self.socket = None

    def send(self, active, rates):
        if self.socket is None:
            self.socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            try:
                self.socket.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, b"")
                self.socket.setblocking(False)
                self.socket.bind(("can1",))
            except OSError:
                self.close()
                raise
        try:
            self.socket.send(arm_frame(active, rates))
        except OSError:
            self.close()
            raise

    def close(self):
        if self.socket is not None:
            self.socket.close()
            self.socket = None
