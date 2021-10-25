#!/usr/bin/env python3
from functools import partial
import json
import datetime
import rclpy
import os
from rclpy.node import Node

from my_tfg_interfaces.msg import FloatDataNode, StatusNode
from my_tfg_interfaces.srv import UploadFile


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_1")  # Node name
        self.declare_parameter("device_id")
        self.declare_parameter("pos_x", 0.0)
        self.declare_parameter("pos_y", 0.0)

        self.status_controller_ = StatusNode()

        self.status_controller_.device_id = self.get_parameter(
            "device_id").value
        self.status_controller_.work_status = 1
        self.status_controller_.position.x = self.get_parameter("pos_x").value
        self.status_controller_.position.y = self.get_parameter("pos_y").value

        self.data_nodes_ = {}
        self.data_nodes_firebase_ = {}

        self.status_nodes_ = {}
        self.position_nodes_ = {}
        self.len_position_nodes = 0
        self.json_filename_ = "data_" + \
            self.get_name() + ".json"
        self.json_filename_firebase_ = "data_" + \
            self.get_name() + "_firebase.json"
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
        # self.save_timer_firebase_ = self.create_timer(120, self.save_data_firebase)
        # Needed to create some publishers and timer
        self.get_logger().info(
            "Controller_" + str(self.status_controller_.device_id) + " has been started.")

    def callback_temperature(self, msg):
        # self.get_logger().info("Temperature: " + str(msg.data))
        self.add_data_to_list("temperature", msg)

    def callback_humidity(self, msg):
        # self.get_logger().info("Humidity: " + str(msg.data))
        self.add_data_to_list("humidity", msg)

    def callback_irrigation(self, msg):
        # self.get_logger().info("Irrigation: " + str(msg.data))
        self.add_data_to_list("irrigation", msg)

    def callback_status(self, msg):
        # self.get_logger().info(msg.device_type + ": " + str(msg.work_status))
        list_status = {
            "device_id": msg.device_id,
            "value": msg.work_status,
            "timestamp": datetime.datetime.now().strftime("%x, %X")
        }
        self.status_nodes_[msg.device_type].append(list_status)

    def add_data_to_list(self, type, msg):
        list_data = {
            "device_id": msg.device_id,
            "value": msg.data,
            "timestamp": datetime.datetime.now().strftime("%x, %X")
        }
        self.data_nodes_[type].append(list_data)

    def save_data(self):
        self.save_data_firebase()
        self.get_logger().info("Saving in " + self.json_filename_)
        json_dir_path = os.getenv('JSON_ROS_DIR')
        json_file_path = json_dir_path + self.json_filename_
        # self.data_nodes_.update(self.status_nodes_)
        with open(json_file_path, 'w') as outfile:
            json.dump(self.data_nodes_, outfile)
        self.call_upload_file("upload_sqlite", self.json_filename_)
        self.init_data()

    def save_data_firebase(self):
        self.get_logger().info("Saving in " + self.json_filename_firebase_ + "for firebase")
        json_dir_path = os.getenv('JSON_ROS_DIR')
        json_file_path = json_dir_path + self.json_filename_firebase_

        # self.data_nodes_firebase_ = self.data_nodes_
        self.data_nodes_.update(self.status_nodes_)

        timestamp = datetime.datetime.now().strftime("%x, %X")
        for type_data in self.data_nodes_:
            devices_id = []
            for data in self.data_nodes_[type_data]:
                if not data["device_id"] in devices_id:
                    devices_id.append(data["device_id"])
                    changed_data_time = data
                    changed_data_time["timestamp"] = timestamp
                    self.data_nodes_firebase_[
                        type_data].append(changed_data_time)

        with open(json_file_path, 'w') as outfile:
            json.dump(self.data_nodes_firebase_, outfile)
        self.call_upload_file("upload_firebase", self.json_filename_firebase_)
        # self.init_data()

    def call_upload_file(self, db, json_name):
        client = self.create_client(UploadFile, db)
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for " + db + " Service...")

        request = UploadFile.Request()
        request.device_id = self.status_controller_.device_id
        request.filename = json_name

        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_upload_file,
                    device_id=self.status_controller_.device_id,
                    filename=json_name))

    def callback_upload_file(self, future, device_id, filename):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Upload to Database done.")
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

    def init_data(self):
        self.data_nodes_ = {"temperature": [],
                            "humidity": [], "irrigation": []}
        self.data_nodes_firebase_ = {"temperature": [],
                                     "humidity": [], "irrigation": [], "heater": [], "window": []}
        self.status_nodes_ = {"heater": [], "window": []}


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
