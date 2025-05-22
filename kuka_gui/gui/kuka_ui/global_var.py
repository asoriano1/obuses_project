# global_var.py
# -*- coding: utf-8 -*-
import QtCore
from std_msgs.msg import Bool, Float64, Float32
from robotnik_msgs.msg import RobotnikMotorsStatus
from geometry_msgs.msg import Pose, Point, Quaternion

#signals
do_callback_motor_status = QtCore.pyqtSignal(RobotnikMotorsStatus)
do_callback_horiz_force = QtCore.pyqtSignal(Float64)
do_callback_current = QtCore.pyqtSignal(Float32)
do_callback_tool_weight = QtCore.pyqtSignal(Float64)
do_callback_moving = QtCore.pyqtSignal(Bool)

#Prepick Pose # tf.transformations.quaternion_from_euler(0, 0, th)
#Prepick_Pose=Pose(Point(100, 100, 100), Quaternion(0, 0, 0, 1))
#rosservice call /kuka_robot/setKukaAbs "{x: 1707.69, y: 235.42, z: 1435.39, A: -59.39, B: 0, C: -174}" 
#CAJA NEGRA
Preplace_Pose_x=1707.69
Preplace_Pose_y=235.42
Preplace_Pose_z=1435.39
Preplace_Pose_a_left=-71 #left
Preplace_Pose_a_right=-71 + 180 #+180 because Preplace_Pose_a_left<0 otherwise -180
Preplace_Pose_b=0#-0.21
Preplace_Pose_c=179#178.41

Preplace_angle_limit=20


#Preplace Pose
#Preplace_Pose=Pose(Point(400, 400, 100), Quaternion(0, 0, 0, 1))
#rosservice call /kuka_robot/setKukaAbs "{x: 255.69, y: 1704.42, z: 1475.39, A: -14.39, B: 0, C: 174}" 
#CAJA GRIS
Prepick_Pose_x=255.49
Prepick_Pose_y=1704.49
Prepick_Pose_z=1542.38
Prepick_Pose_a_left=-18#15.2
Prepick_Pose_a_right=Prepick_Pose_a_left+180 #+180 because Preplace_Pose_a_left<0 otherwise -180
Prepick_Pose_b=0.0#-0.12
Prepick_Pose_c=179.0#178.73

Prepick_angle_limit=90

#Homming Pose
Homming_Pose_x=1260.41
Homming_Pose_y=1284.82
Homming_Pose_z=1455.99
Homming_Pose_a=-120.03 # es importante que no este entre los limites de pick y place [20,90] si no, podria rotar en el sentido erroneo.
Homming_Pose_b=0.0
Homming_Pose_c=179.0#178.73

#RollerBench Pose
RollerBench_Pose=Pose(Point(200, 200, 100), Quaternion(0, 0, 0, 1))
pos_x_kuka=0.0
pos_y_kuka=0.0
pos_z_kuka=0.0
pos_a_kuka=0.0
pos_b_kuka=0.0
pos_c_kuka=0.0
weight_read=0.0
weight_empty=0.0
weight_reads=[0, 0, 0, 0, 0]
weight_expected_min = 9999
weight_expected_max = 9999
horiz_force_read=0.0
horiz_force_empty=0.0

#Current limits
current_limit_0 = 2
current_limit_1 = 3
current_limit_2 = 4
current_limit_cont = 5
current_limit_3 = 7
current_limit_4 = 8
current_limit_picked = 2
#Obus already placed
#Hueveras de 2
Place_Obus_2_1=False
Place_Obus_2_2=False

#Hueveras de 4
Place_Obus_4_1=False
Place_Obus_4_2=False
Place_Obus_4_3=False
Place_Obus_4_4=False

#Hueveras de 8
Place_Obus_8_1=False
Place_Obus_8_2=False
Place_Obus_8_3=False
Place_Obus_8_4=False
Place_Obus_8_5=False
Place_Obus_8_6=False
Place_Obus_8_7=False
Place_Obus_8_8=False

#Hueveras de 16
Place_Obus_16_1=False
Place_Obus_16_2=False
Place_Obus_16_3=False
Place_Obus_16_4=False
Place_Obus_16_5=False
Place_Obus_16_6=False
Place_Obus_16_7=False
Place_Obus_16_8=False
Place_Obus_16_9=False
Place_Obus_16_10=False
Place_Obus_16_11=False
Place_Obus_16_12=False
Place_Obus_16_13=False
Place_Obus_16_14=False
Place_Obus_16_15=False
Place_Obus_16_16=False

#Pick Positions

#Obus already placed
#Hueveras de 2
Pick_Obus_2_1=False
Pick_Obus_2_2=False
Pick_Obus_2_3=False
Pick_Obus_2_4=False
#Hueveras de 4
Pick_Obus_4_1=False
Pick_Obus_4_2=False
Pick_Obus_4_3=False
Pick_Obus_4_4=False
Pick_Obus_4_5=False

#Hueveras de 8
Pick_Obus_8_1=False
Pick_Obus_8_2=False
Pick_Obus_8_3=False
Pick_Obus_8_4=False
Pick_Obus_8_5=False
Pick_Obus_8_6=False
Pick_Obus_8_7=False
Pick_Obus_8_8=False
Pick_Obus_8_9=False
Pick_Obus_8_10=False
Pick_Obus_8_11=False
Pick_Obus_8_12=False
Pick_Obus_8_13=False
Pick_Obus_8_14=False

#Hueveras de 16
Pick_Obus_16_1=False
Pick_Obus_16_2=False
Pick_Obus_16_3=False
Pick_Obus_16_4=False
Pick_Obus_16_5=False
Pick_Obus_16_6=False
Pick_Obus_16_7=False
Pick_Obus_16_8=False
Pick_Obus_16_9=False
Pick_Obus_16_10=False
Pick_Obus_16_11=False
Pick_Obus_16_12=False
Pick_Obus_16_13=False
Pick_Obus_16_14=False
Pick_Obus_16_15=False
Pick_Obus_16_16=False
Pick_Obus_16_17=False
Pick_Obus_16_18=False
Pick_Obus_16_19=False
Pick_Obus_16_20=False