#!/bin/bash

# Activar entorno ROS
source /opt/ros/kinetic/setup.bash
source /home/robotnik/catkin_ws/devel/setup.bash

# Ejecutar la GUI desde el directorio del proyecto
cd "$(dirname "$0")/gui"
python main.py
