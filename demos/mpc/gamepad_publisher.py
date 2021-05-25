"""Simple example showing how to get gamepad events."""

import numpy as np

np.set_printoptions(suppress=True, precision=3)

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from mim_msgs.msg import Vector

from inputs import get_gamepad


class GamepadPublisher(Node):
    """
    Publishes the values from the gamepad in a single vector.
    Following this drawing for naming:
       https://en.wikipedia.org/wiki/Xbox_360_controller#/media/File:360_controller.svg

       0 & 1: left stick
       2 & 3: right sitck
       4 & 5: direcional pad
       6:     left trigger
       7:     right trigger
    """

    def __init__(self):
        super().__init__("gamepad_publisher")
        self.publisher_ = self.create_publisher(Vector, "gamepad_axes", 10)
        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.values = 8 * [0.0]

    def process_gamepad_events(self):
        events = get_gamepad()
        for event in events:
            if event.ev_type == "Absolute":
                if event.code == "ABS_X":
                    self.values[0] = event.state / 32768.0
                elif event.code == "ABS_Y":
                    self.values[1] = event.state / -32768.0
                if event.code == "ABS_RX":
                    self.values[2] = event.state / 32768.0
                elif event.code == "ABS_RY":
                    self.values[3] = event.state / -32768.0
                elif event.code == "ABS_HAT0X":
                    self.values[4] = float(event.state)
                elif event.code == "ABS_HAT0Y":
                    self.values[5] = float(event.state * -1)
                elif event.code == "ABS_Z":
                    self.values[6] = event.state / 1024.0
                elif event.code == "ABS_RZ":
                    self.values[7] = event.state / 1024.0

    def timer_callback(self):
        self.process_gamepad_events()

        msg = Vector()
        msg.data = self.values
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = GamepadPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
