#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from my_tfg_interfaces.srv import SensorArea

class HeaterNode(Node):
    def __init__(self):
        super().__init__("heater_node")
        self.declare_parameter("pos_x", 0.0)
        self.declare_parameter("pos_y", 0.0)
        self.pos_x = self.get_parameter("pos_x").value
        self.pos_y = self.get_parameter("pos_y").value
        self.server_ = self.create_service(SensorArea, "sensor_area", self.callback_heater)
        self.get_logger().info("Heater server has been started.")

    def callback_heater(self, request, response):
        # Implement busy
        self.pos_x = request.pos_x
        self.pos_y = request.pos_y
        response.success = True
        return response

def main(args=None):
    rclpy.init(args=args)
    node = HeaterNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()