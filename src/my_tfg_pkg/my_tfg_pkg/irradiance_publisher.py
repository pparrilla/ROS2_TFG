#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import random

from custom_node_message.msg import FloatDataNode

class IrradianceNode(Node):
    def __init__(self):
        super().__init__("sensor_14")
        self.declare_parameter("pos_x", 0.0)
        self.declare_parameter("pos_y", 0.0)

        self.irrigation_data_ = FloatDataNode()
        self.irrigation_data_.device_id = 14
        self.irrigation_data_.data = float(random.randint(600,760))
        self.irrigation_data_.position.x == self.get_parameter("pos_x").value
        self.irrigation_data_.position.y == self.get_parameter("pos_y").value

        self.sensor_publisher_ = self.create_publisher(FloatDataNode, "irrigation", 10)
        self.sensor_timer_ = self.create_timer(30.0, self.publish_sensor_data)
        self.get_logger().info("Irrigation_" + str(self.irrigation_data_.device_id) + " has been started")

    def publish_sensor_data(self):
        self.irrigation_data_.data = random.random(0,760)
        msg = self.irrigation_data_
        self.sensor_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = IrradianceNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()