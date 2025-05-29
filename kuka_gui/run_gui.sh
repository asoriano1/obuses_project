#!/bin/bash

# Activar entorno ROS
source /opt/ros/kinetic/setup.bash
#source /home/robotnik/catkin_ws/devel/setup.bash
source /home/robotnik/obuses_2025/devel/setup.bash
export ROS_MASTER_URI="http://192.168.1.10:11311"
export ROS_IP="192.168.1.11"
# Ejecutar la GUI desde el directorio del proyecto
cd "$(dirname "$0")/gui"
python main.py
