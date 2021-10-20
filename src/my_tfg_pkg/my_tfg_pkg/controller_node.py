#!/usr/bin/env python3
from functools import partial
import json
import time
import rclpy
import os
from rclpy.node import Node

from my_tfg_interfaces.msg import FloatDataNode, StatusNode
from my_tfg_interfaces.srv import UploadFile


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_1")  # Node name
        self.status_controller_ = StatusNode()

        self.status_controller_.device_id = 1
        self.status_controller_.work_status = 1
        self.status_controller_.position.x = 0.0
        self.status_controller_.position.y = 0.0
        self.data_nodes_ = {}
        self.status_nodes_ = {}
        self.position_nodes_ = {}
        self.len_position_nodes = 0
        self.json_filename_ = "data_" + \
            self.get_name() + ".json"
        self.init_data()

        # Create all subscribers
        self.temperature_subscriber_ = self.create_subscription(
            FloatDataNode, "temperature", self.callback_temperature, 10)

        self.humidity_subscriber_ = self.create_subscription(
            FloatDataNode, "humidity", self.callback_humidity, 10)

        self.irrigation_subscriber_ = self.create_subscription(
            FloatDataNode, "irrigation", self.callback_irrigation, 10)

        self.status_subscriber_ = self.create_subscription(
            StatusNode, "status_actuator", self.callback_status, 10)

        self.save_timer_ = self.create_timer(60, self.save_data)

        # Needed to create some publishers and timer
        self.get_logger().info(
            "Controller_" + str(self.status_controller_.device_id) + " has been started.")

    def callback_temperature(self, msg):
        self.get_logger().info("Temperature: " + str(msg.data))
        self.add_data_to_list("temperature", msg)

    def callback_humidity(self, msg):
        self.get_logger().info("Humidity: " + str(msg.data))
        self.add_data_to_list("humidity", msg)

    def callback_irrigation(self, msg):
        self.get_logger().info("Irrigation: " + str(msg.data))
        self.add_data_to_list("irrigation", msg)

    def callback_status(self, msg):
        self.get_logger().info(msg.device_type + ": " + str(msg.work_status))
        list_status = {
            "device_id": msg.device_id,
            "work_status": msg.work_status,
            "timestamp": time.time()
        }
        self.status_nodes_[msg.device_type].append(list_status)

    def add_data_to_list(self, type, msg):
        list_data = {
            "device_id": msg.device_id,
            "value": msg.data,
            "timestamp": time.time()
        }
        self.data_nodes_[type].append(list_data)

    def save_data(self):
        self.get_logger().info("Saving in " + self.json_filename_)
        json_dir_path = os.getenv('JSON_ROS_DIR')
        json_file_path = json_dir_path + self.json_filename_
        self.data_nodes_.update(self.status_nodes_)
        with open(json_file_path, 'w') as outfile:
            json.dump(self.data_nodes_, outfile)
        self.call_upload_file()
        self.init_data()

    def call_upload_file(self):
        client = self.create_client(UploadFile, "upload_file")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Firebase Service...")

        request = UploadFile.Request()
        request.device_id = self.status_controller_.device_id
        request.filename = self.json_filename_

        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_upload_file,
                    device_id=self.status_controller_.device_id,
                    filename=self.json_filename_))

    def callback_upload_file(self, future, device_id, filename):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Upload to Firestore done.")
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

    def init_data(self):
        self.data_nodes_ = {"temperature": [],
                            "humidity": [], "irrigation": []}
        self.status_nodes_ = {"heater": [], "window": []}


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
