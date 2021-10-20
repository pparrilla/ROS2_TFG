from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Sensors

    temp_hum_sensor = []
    temp_hum_sensor_id = [13, 14]
    temperatures = [25.0, 19.0]
    humidities = [12.0, 36.0]
    temp_hum_pos = [[7.5,1.25], [2.5, 8.75]]

    for i in range (2) :
        temp_hum_sensor.append(Node(
            package="my_tfg_pkg",
            executable="temp_hum_publisher",
            name="sensor_" + str(temp_hum_sensor_id[i]),
            parameters=[
                {"device_id": temp_hum_sensor_id[i]},
                {"temperature": temperatures[i]},
                {"humidity": humidities[i]},
                {"pos_x": temp_hum_pos[i][0]},
                {"pos_y": temp_hum_pos[i][1]}
            ]
        ))

    irrigation_sensor = []
    irrigation_sensor_id = [15, 16]
    irrigation_pos = [[6.0,7.0], [2.0, 2.0]]

    for i in range (2) :
        irrigation_sensor.append(Node(
            package="my_tfg_pkg",
            executable="irrigation_publisher",
            name="irrigation_sensor_" + str(irrigation_sensor_id[i]),
            parameters=[
                {"device_id": irrigation_sensor_id[i]},
                {"pos_x": irrigation_pos[i][0]},
                {"pos_y": irrigation_pos[i][1]}
            ]
        ))

    irrigation_sensor = []
    irrigation_sensor_id = [15, 16]
    irrigation_pos = [[6.0,7.0], [2.0, 2.0]]

    for i in range (2) :
        irrigation_sensor.append(Node(
            package="my_tfg_pkg",
            executable="irradiance_publisher",
            name="irrigation_sensor_" + str(irrigation_sensor_id[i]),
            parameters=[
                {"device_id": irrigation_sensor_id[i]},
                {"pos_x": irrigation_pos[i][0]},
                {"pos_y": irrigation_pos[i][1]}
            ]
        ))
    # Actuators

    window_actuator = []
    window_actuator_id = [31, 32, 33, 34]
    window_pos = [[2.5,8.75], [7.5, 8.75], [2.5, 3.75], [7.5, 3.75]]

    for i in range (4) :
        window_actuator.append(Node(
            package="my_tfg_pkg",
            executable="window_node",
            name="window_actuator_" + str(window_actuator_id[i]),
            parameters=[
                {"device_id": window_actuator_id[i]},
                {"pos_x": window_pos[i][0]},
                {"pos_y": window_pos[i][1]}
            ]
        ))

    heater_actuator = []
    heater_actuator_id = [35, 36]
    heater_pos = [[2.5, 1.25], [7.5, 1.25]]

    for i in range (2) :
        heater_actuator.append(Node(
            package="my_tfg_pkg",
            executable="heater_node",
            name="heater_actuator_" + str(heater_actuator_id[i]),
            parameters=[
                {"device_id": heater_actuator_id[i]},
                {"pos_x": heater_pos[i][0]},
                {"pos_y": heater_pos[i][1]}
            ]
        ))


    # Controllers

    controller = []
    controller_id = [1, 2]
    controller_pos = [[5.0, 7.5], [5.0, 2.5]]

    for i in range (2) :
        controller.append(Node(
            package="my_tfg_pkg",
            executable="controller_node",
            name="controller_" + str(controller_id[i]),
            parameters=[
                {"device_id": controller_id[i]},
                {"pos_x": controller_pos[i][0]},
                {"pos_y": controller_pos[i][1]}
            ]
        ))

    # Firebase

    firebase_service = Node(
        package="my_tfg_pkg",
        executable="firebase_service",
        name="firebase_service"
    )

    for node in temp_hum_sensor:
        ld.add_action(node)

    for node in irrigation_sensor:
        ld.add_action(node)

    for node in window_actuator:
        ld.add_action(node)

    for node in heater_actuator:
        ld.add_action(node)

    for node in controller:
        ld.add_action(node)

    ld.add_action(firebase_service)

    return ld