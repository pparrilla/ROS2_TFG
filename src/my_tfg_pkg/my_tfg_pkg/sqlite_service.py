
#!/usr/bin/env python3
import sqlite3
from sqlite3 import Error
import json
import os
import rclpy
from rclpy.node import Node

from my_tfg_interfaces.srv import UploadFile

# TO DO
class SqliteServerNode(Node):
    def __init__(self):
        super().__init__("sqlite_service")
        self.main_node_to_listen_ = 1
        self.db_name_ = 'agricloud.db'
        self.server_ = self.create_service(
            UploadFile, "upload_sqlite", self.callback_upload_file)
        self.last_timestamp_ = ""
        self.get_logger().info("Sqlite service has been started.")

    def callback_upload_file(self, request, response):
        if request.device_id == self.main_node_to_listen_:
            self.get_logger().info("Main node contact to server")
            response.success = self.upload_sqlite(request.filename)
        else:
            response.success = False
        return response

    def create_connection(self, db_file):
        conn = None
        try:
            conn = sqlite3.connect(db_file)
            print(sqlite3.version)
        except Error as e:
            print(e)

        return conn

    def upload_sqlite(self, json_filename):
        json_dir_path = os.getenv('JSON_ROS_DIR')
        json_file_path = json_dir_path + json_filename

        with open(json_file_path, 'r') as j:
            contents = json.loads(j.read())

        db = self.create_connection(self.db_name_)
        cursorObj = db.cursor()

        self.get_logger().info("Saving in Sqlite3.")

        for key, list in contents.items():
            for i in list:
                entities = (i["device_id"], i["value"], i["timestamp"])
                cursorObj.execute("INSERT INTO " + key + "(id, value, timestamp) VALUES(?, ?, ?)", entities)

        db.commit()


        if os.path.exists(json_file_path):
            os.remove(json_file_path)
        else:
            self.get_logger().info("File not exist.")

        db.close()
        return True


def main(args=None):
    rclpy.init(args=args)
    node = SqliteServerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
