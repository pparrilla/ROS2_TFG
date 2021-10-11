from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    sensor_data_nodes = []

    temperatures = [24.0, 19.0, 30.0]

    for i in range(3):
        sensor_data_nodes.append(Node(
            package="my_tfg_pkg",
            executable="sensor_node",
            name="sensor_data_" + str(i),
            parameters=[
                {"name":"sensor_data_" + str(i)},
                {"temperature": temperatures[i]},
                {"humidity": 40.0}
            ]
        ))

    controller_node = Node(
        package="my_tfg_pkg",
        executable="controller_node",
        name="controller"
    )

    for node in sensor_data_nodes:
        ld.add_action(node)
    ld.add_action(controller_node)
    return ld