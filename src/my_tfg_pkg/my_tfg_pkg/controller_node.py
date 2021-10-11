#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from my_tfg_interfaces.msg import Sensor


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller")  # Node name
        self.subscriber_ = self.create_subscription(
            Sensor, "sensor_data", self.callback_sensor_data, 10)
        self.get_logger().info("Controller has been started.")

    def callback_sensor_data(self, msg):
        self.get_logger().info("Name " + str(msg.name))
        self.get_logger().info("Temperature " + str(msg.temperature))
        self.get_logger().info("Humidity " + str(msg.humidity))


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
