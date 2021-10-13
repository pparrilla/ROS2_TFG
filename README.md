# Diseño y desarrollo de un sistema con ROS2

## Introducción

En este repositorio se va a publicar un proyecto utilizando como tecnología a destacar ROS2.
Este va a constar de diferentes nodos (o robots) que se iran comunicando para pasarse información sobre
datos recopilados, y ordenar a otros actuar en consecuencia.

## Desarrollo

En principio este trabajo se iba a realizar utilizando un único ordenador con diferentes máquinas virtuales
o servicios ejecutando, pero tras hablarlo con el tutor y gracias al material disponible, se va a desplegar
en varios ordenadores, incluyendo alguna Raspberry Pi, además de intentar abaratar costos con el uso de
microcontroladores como el ESP32. Para este último es necesario el uso de MicroPython y MicroROS. \\
Por último me gustaría programar un dashboard donde poder visualizar estos datos con gráficas, gracias
a Angular y alguna base de datos como Firebase.

## Requisitos e instalación

Para la ejecución del proyecto en el estado actual, es necesario:
- [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation.html)

Es necesario también instalar algunas extensiones de colcon:

```
# Para Debian/Ubuntu
sudo apt install python3-colcon-common-extensions

# Añadir al .bashrc o ejecutarlo
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
```

Tras esto, podemos construir el proyecto

```
# Dentro del repositorio
colcon build

# Para tenerlo en el comando ros2
source /dir_repositorio/install/setup.bash

# Ejecutarlo en diferentes terminales

ros2 run my_tfg_pkg controller_node
ros2 run my_tfg_pkg sensor_node
```

Gracias a la incorporación del launchfile, se pueden lanzar 3 sensor_node y 1 controller_node desde una terminal

```
ros2 launch my_tfg_bringup my_tfg.launch.py
```

## To do

- [ ] Implementar en ESP32
- [ ] Análisis de todo el trabajo previo y comparación con otras tecnologías
- [ ] Realizar diagramas de comunicación de todo el entorno de trabajo
- [ ] Aplicar nuevos nodos en función de proyectos de otras empresas.
- [ ] Crear interfaz de usuario para la visualización de datos
- [ ] Mejorar documentación, añadiendo referencias, enlaces, imágenes, tablas..
- [ ] Probar diferentes middleware de dds y hacer comparativas haciendo uso de algún benchmark

## Bibliografía

- [ROS Wiki](wiki.ros.org)
- [ROS2 Foxy](https://docs.ros.org/en/foxy/Docs-Guide.html)
- [ROS2 + DDS Interoperation, Youtube, Real-Time Innovations](https://www.youtube.com/watch?v=GGqcrccWfeE)
- [ROS2-Performance, Github, irobot-ros](https://github.com/irobot-ros/ros2-performance)
- [iFarm](https://ifarms.me/)
- [Nazaríes](https://www.tecnologia-agricola.com/nosotros/)
- [Agroespia](https://agroespia.com/)
- [The Robotics Back-End](https://roboticsbackend.com/)
- [Espressif](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html)
- [Raspberry Pi Pico C/C++ SDK](https://datasheets.raspberrypi.org/pico/raspberry-pi-pico-c-sdk.pdf)
- [Ros-Industrial Consortium Europe](https://rosindustrial.org/news/tag/ROS2)
- [Red teaming the Robot Operating System in industry, Cybersecurity and Robotics, Victor Mayoral Vilches](https://cybersecurityrobotics.net/red-teaming-the-ros-in-industry/)
- [Open Robotics Meets the Open Road, Youtube, The Decoded Show](https://www.youtube.com/watch?v=fAUTJa7Nrdc)
- [The Robotic Edge, Roger S. Barga](https://static1.squarespace.com/static/51df34b1e4b08840dcfd2841/t/5e74ded67fdc5c0278f0c60d/1584717530256/RSBarga+ROS+Industrial+Americas+2020.pdf)