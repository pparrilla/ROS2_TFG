#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from my_tfg_interfaces.msg import Sensor

class SensorNode(Node):
    def __init__(self):
        super().__init__("py_test")
        self.declare_parameter("name", "default_sensor_name")
        self.declare_parameter("temperature", 24.0)
        self.declare_parameter("humidity", 60.0)

        self.sensor_data_ = Sensor()
        self.sensor_data_.name = self.get_parameter("name").value
        self.sensor_data_.temperature = self.get_parameter("temperature").value
        self.sensor_data_.humidity = self.get_parameter("humidity").value
        self.sensor_publisher_ = self.create_publisher(Sensor, "sensor_data", 10)
        self.sensor_timer_ = self.create_timer(2.0, self.publish_sensor_data)
        self.get_logger().info("Sensor node called " + self.sensor_data_.name + " has been started")

    def publish_sensor_data(self):
        msg = self.sensor_data_
        self.sensor_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()