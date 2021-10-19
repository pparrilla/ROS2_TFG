#!/usr/bin/env python3
from functools import partial
import json
import time
import rclpy
from rclpy.node import Node

from custom_node_message.msg import FloatDataNode, StatusNode
from my_tfg_interfaces.srv import SensorArea


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_1")  # Node name
        self.status_controller_ = StatusNode()

        self.status_controller_.device_id = 1
        self.status_controller_.work_status = 1
        self.status_controller_.position.x = 0.0
        self.status_controller_.position.y = 0.0
        self.data_nodes_ = { "temperature": [], "humidity": [], "irrigation": [], "status": []}
        self.status_nodes_ = []
        self.position_nodes_ = {}
        self.len_position_nodes = 0
        self.json_file_name_ = "data_" + str(self.status_controller_.device_id) + ".json"


        # Create all subscribers
        self.temperature_subscriber_ = self.create_subscription(
            FloatDataNode, "temperature", self.callback_temperature, 10)

        self.humidity_subscriber_ = self.create_subscription(
            FloatDataNode, "humidity", self.callback_humidity, 10)

        self.irrigation_subscriber_ = self.create_subscription(
            FloatDataNode, "irrigation", self.callback_irrigation, 10)

        self.status_subscriber_ = self.create_subscription(
            FloatDataNode, "status_actuator", self.callback_status, 10)

        self.save_timer_ = self.create_timer(60, self.save_data)

        # Needed to create some publishers and timer
        self.get_logger().info("Controller_" + str(self.status_controller_.device_id) + " has been started.")

    def callback_temperature(self, msg):
        self.get_logger().info("Temperature: " + str(msg.data))
        self.add_data_to_list("temperature", msg)

    def callback_humidity(self, msg):
        self.get_logger().info("Humidity: " + str(msg.data))
        self.add_data_to_list("humidity", msg)

    def callback_irrigation(self, msg):
        self.get_logger().info("Irrigaiton: " + str(msg.data))
        self.add_data_to_list("irrigation", msg)

    def callback_status(self, msg):
        self.add_data_to_list("status", msg)

    def add_data_to_list(self, type, msg):
        list_data = {
            "device_id" : msg.device_id,
            "value" : msg.data,
            "timestamp" : time.time()
        }
        self.data_nodes_[type].append(list_data)

    def save_data(self):
        self.get_logger().info("Saving in " + self.json_file_name_ )
        with open(self.json_file_name_, 'w') as outfile:
            json.dump(self.data_nodes_, outfile)

    def clean_data(self):
        self.data_nodes_ = { "temperature": [], "humidity": [], "irrigation": [], "status": []}
        self.status_nodes_ = []



def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
