#!/usr/bin/env python3
from functools import partial
import rclpy
from rclpy.node import Node

from my_tfg_interfaces.msg import Sensor
from my_tfg_interfaces.srv import SensorArea


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller")  # Node name
        self.subscriber_ = self.create_subscription(
            Sensor, "sensor_data", self.callback_sensor_data, 10)
        self.get_logger().info("Controller has been started.")

    def callback_sensor_data(self, msg):
        self.get_logger().info("Name " + str(msg.name))

        if msg.temperature >= 12.0:
            self.get_logger().info("Warning low temperature ")
            self.call_heater(msg.pos_x, msg.pos_y)

        self.get_logger().info("Temperature " + str(msg.temperature))
        self.get_logger().info("Humidity " + str(msg.humidity))

    def call_heater(self, pos_x, pos_y):
        client = self.create_client(SensorArea, "sensor_area")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for heater")

        request = SensorArea.Request()
        request.pos_x = pos_x
        request.pos_y = pos_y

        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_heater, pos_x=pos_x, pos_y=pos_y))

    def callback_call_heater(self, future, pos_x, pos_y):
        try:
            response = future.result()
            self.get_logger().info("Heat sensor pos x:" + str(pos_x) + " pos y:" + str(pos_y))

        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
