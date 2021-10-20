#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import random

from my_tfg_interfaces.msg import FloatDataNode

class TemperatureHumidityNode(Node):
    def __init__(self):
        super().__init__("sensor_15")
        self.declare_parameter("pos_x", 0.0)
        self.declare_parameter("pos_y", 0.0)

        self.temperature_msg_ = FloatDataNode()
        self.temperature_msg_.device_id = 15
        self.temperature_msg_.data = 18.4
        self.temperature_msg_.position.x == self.get_parameter("pos_x").value
        self.temperature_msg_.position.y == self.get_parameter("pos_y").value

        self.humidity_msg_ = FloatDataNode()
        self.humidity_msg_.device_id = 15
        self.humidity_msg_.data = 62.3
        self.humidity_msg_.position.x == self.get_parameter("pos_x").value
        self.humidity_msg_.position.y == self.get_parameter("pos_y").value

        self.temperature_publisher_ = self.create_publisher(FloatDataNode, "temperature", 10)
        self.humidity_publisher_ = self.create_publisher(FloatDataNode, "humidity", 10)

        self.sensor_timer_ = self.create_timer(60.0, self.publish_temperature_data)
        self.sensor_timer_ = self.create_timer(60.0, self.publish_humidity_data)

        self.get_logger().info("Temp_Hum_Sensor_" + str(self.temperature_msg_.device_id) + " has been started")


    def publish_temperature_data(self):
        msg = self.temperature_msg_
        self.temperature_publisher_.publish(msg)

    def publish_humidity_data(self):
        msg = self.humidity_msg_
        self.humidity_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TemperatureHumidityNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()