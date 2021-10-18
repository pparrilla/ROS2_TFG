#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from custom_node_message.msg import FloatDataNode, StatusNode


class WindowNode(Node):
    def __init__(self):
        super().__init__("window_31")
        self.declare_parameter("pos_x", 0.0)
        self.declare_parameter("pos_y", 0.0)

        self.status_node_ = StatusNode()
        self.pos_x = self.get_parameter("pos_x").value
        self.pos_y = self.get_parameter("pos_y").value
        self.status_node_.device_id = 31
        self.status_node_.is_working = False

        self.temperature_to_act_ = 20.0

        self.temperature_subscriber_ = self.create_subscription(
            FloatDataNode, "temperature", self.callback_sensor_data, 10)
        self.get_logger().info("Windows has been started.")

    def callback_sensor_data(self, msg):
        self.get_logger().info("Device id: " + str(msg.device_id) + " Temperature: " + str(msg.data))
        if msg.data >= self.temperature_to_act_ and not(self.status_node_.is_working):
            self.status_node_.is_working = True
            self.get_logger().info("window_" + str(self.status_node_.device_id) + " is openning")
        elif msg.data < self.temperature_to_act_ and self.status_node_.is_working:
            self.status_node_.is_working = False
            self.get_logger().info("window_" + str(self.status_node_.device_id) + " is closing")

    def publish_status(self):
        msg = self.sensor_data_
        self.sensor_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WindowNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
