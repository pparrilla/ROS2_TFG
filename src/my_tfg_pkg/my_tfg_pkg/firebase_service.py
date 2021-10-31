
#!/usr/bin/env python3
import firebase_admin
from firebase_admin import credentials
from firebase_admin import firestore
import json
import os
import rclpy
from rclpy.node import Node

from my_tfg_interfaces.srv import UploadFile


class FirebaseServerNode(Node):
    def __init__(self):
        super().__init__("firebase_service")
        self.main_node_to_listen_ = 1
        self.server_ = self.create_service(
            UploadFile, "upload_firebase", self.callback_upload_file)
        self.server_nodes_info_ = self.create_service(
            UploadFile, "upload_nodes_info", self.callback_upload_nodes_info)
        self.get_logger().info("Firebase service has been started.")

    def callback_upload_file(self, request, response):
        if request.device_id == self.main_node_to_listen_:
            self.get_logger().info("Main node contact to server")
            response.success = self.upload_json(request.filename)
        else:
            response.success = False
        return response

    def callback_upload_nodes_info(self, request, response):
        if request.device_id == self.main_node_to_listen_:
            self.get_logger().info("Main node contact to server")
            response.success = self.upload_json_nodes_info(request.filename)
        else:
            response.success = False
        return response

    def upload_json(self, json_filename):
        json_dir_path = os.getenv('JSON_ROS_DIR')
        cred_file_path = os.getenv('DB_CREDENTIALS')
        collection_name = 'invernadero_1'
        json_file_path = json_dir_path + json_filename

        with open(json_file_path, 'r') as j:
            contents = json.loads(j.read())

        if not firebase_admin._apps:
            cred = credentials.Certificate(cred_file_path)
            firebase_admin.initialize_app(cred, {
                'projectId': "ros2-tfg",
            })

        db = firestore.client()
        self.get_logger().info("Uploading data to Firestore.")
        for key, list in contents.items():
            for i in list:
                doc_ref = db.collection('AgriCloudMeasures').document(
                    key).collection(collection_name).add(i)

        if os.path.exists(json_file_path):
            os.remove(json_file_path)
        else:
            self.get_logger().info("File not exist.")


    def upload_json_nodes_info(self, json_filename):
        json_dir_path = os.getenv('JSON_ROS_DIR')
        cred_file_path = os.getenv('DB_CREDENTIALS')
        collection_name = 'invernadero_1'
        json_file_path = json_dir_path + json_filename

        with open(json_file_path, 'r') as j:
            contents = json.loads(j.read())

        if not firebase_admin._apps:
            cred = credentials.Certificate(cred_file_path)
            firebase_admin.initialize_app(cred, {
                'projectId': "ros2-tfg",
            })

        db = firestore.client()
        self.get_logger().info("Uploading Nodes Info to Firestore.")
        for key, device_id in contents.items():
            for id_key, data in device_id.items():
                self.get_logger().info("data " + data["type"][0])
                new_dict = {}
                for type, value in data.items():
                    self.get_logger().info("Type " + type)
                    new_data = { type : value }
                    new_dict.update(new_data)

                db.collection('AgriCloudMeasures').document(key).collection(collection_name).document(id_key).set(new_dict)

        if os.path.exists(json_file_path):
            os.remove(json_file_path)
        else:
            self.get_logger().info("File not exist.")

        return True


def main(args=None):
    rclpy.init(args=args)
    node = FirebaseServerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
