#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from my_tfg_interfaces.msg import FloatDataNode, StatusNode


class WindowNode(Node):
    def __init__(self):
        super().__init__("window_31")
        self.declare_parameter("pos_x", 0.0)
        self.declare_parameter("pos_y", 0.0)

        self.status_node_ = StatusNode()
        self.status_node_.device_id = 31
        self.status_node_.work_status = 0
        self.status_node_.device_type = "window"
        self.status_node_.position.x = self.get_parameter("pos_x").value
        self.status_node_.position.y = self.get_parameter("pos_y").value

        self.temperature_to_act_ = 20.0

        self.temperature_subscriber_ = self.create_subscription(
            FloatDataNode, "temperature", self.callback_sensor_data, 10)
        self.status_publisher_ = self.create_publisher(
            StatusNode, "status_actuator", 10)
        self.status_timer_ = self.create_timer(20, self.publish_status)
        self.get_logger().info("Window_" + str(self.status_node_.device_id) + " has been started.")

    def callback_sensor_data(self, msg):
        self.get_logger().info("Device id: " + str(msg.device_id) + " Temperature: " + str(msg.data))
        if abs(msg.position.x - self.status_node_.position.x) < 3.0 and abs(msg.position.y - self.status_node_.position.y) < 3.0:
            if msg.data >= self.temperature_to_act_ and self.status_node_.work_status == 0:
                self.status_node_.work_status = 1
                self.publish_status()
                self.get_logger().info("window_" + str(self.status_node_.device_id) + " is openning")
            elif msg.data < self.temperature_to_act_ and self.status_node_.work_status != 0:
                self.status_node_.work_status = 0
                self.publish_status()
                self.get_logger().info("window_" + str(self.status_node_.device_id) + " is closing")

    def publish_status(self):
        msg = self.status_node_
        self.status_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WindowNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
