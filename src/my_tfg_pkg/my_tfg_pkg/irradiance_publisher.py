#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import random

from my_tfg_interfaces.msg import FloatDataNode

class IrradianceNode(Node):
    def __init__(self):
        super().__init__("sensor_14")
        self.declare_parameter("device_id")
        self.declare_parameter("pos_x", 0.0)
        self.declare_parameter("pos_y", 0.0)

        self.irradiance_data_ = FloatDataNode()
        self.irradiance_data_.device_id = self.get_parameter("device_id").value
        self.irradiance_data_.data = float(random.randint(600,760))
        self.irradiance_data_.position.x == self.get_parameter("pos_x").value
        self.irradiance_data_.position.y == self.get_parameter("pos_y").value

        self.sensor_publisher_ = self.create_publisher(FloatDataNode, "irradiance", 10)
        self.sensor_timer_ = self.create_timer(10.0, self.publish_sensor_data)
        self.get_logger().info("irradiance_" + str(self.irradiance_data_.device_id) + " has been started")

    def publish_sensor_data(self):
        self.irradiance_data_.data = random.uniform(500.2,764.3)
        msg = self.irradiance_data_
        self.sensor_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = IrradianceNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()