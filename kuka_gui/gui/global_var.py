# config.py
# -*- coding: utf-8 -*-
import os

# Paths
GUI_PATH = os.path.dirname(os.path.abspath(__file__)) + "/"
UI_PATH = os.path.join(GUI_PATH, "resource", "RqtKuka.ui")
IMG_PATH = os.path.join(GUI_PATH, "resource", "images") + "/"
SCRIPTS_PATH = os.path.join(GUI_PATH, "scripts") + "/"

#images
#[0] original - blanca   [1] resaltada - verde    [2] seleccionada - roja
imgObus16izq = [IMG_PATH+'/obus_izq_19x51_0.png', IMG_PATH+'/obus_izq_19x51_1.png', IMG_PATH+'/obus_izq_19x51_2.png']
imgObus16der = [IMG_PATH+'/obus_der_19x51_0.png', IMG_PATH+'/obus_der_19x51_1.png', IMG_PATH+'/obus_der_19x51_2.png']
imgObus8izq = [IMG_PATH+'/obus_izq_26x71_0.png', IMG_PATH+'/obus_izq_26x71_1.png', IMG_PATH+'/obus_izq_26x71_2.png']
imgObus8der = [IMG_PATH+'/obus_der_26x71_0.png', IMG_PATH+'/obus_der_26x71_1.png', IMG_PATH+'/obus_der_26x71_2.png']
imgObus4 = [IMG_PATH+'/obus_der_37x101_0.png', IMG_PATH+'/obus_der_37x101_1.png', IMG_PATH+'/obus_der_37x101_2.png']
imgObus2 = [IMG_PATH+'/obus_der_41x111_0.png', IMG_PATH+'/obus_der_41x111_1.png', IMG_PATH+'/obus_der_41x111_2.png']

# states
CURRENT_STATE=0
STATE_IDLE=0
STATE_MOVING_TO_PREPICK=1
STATE_DOING_PICK_TEST=2
STATE_PICKED=3
STATE_MOVING_TO_PLACE=4
STATE_PLACED=5
STATE_HOMING=6

#init global vars
finger_type=0
angle_tool=0

#cuadrantes
# 2 4
# 1 3
tool_current=0
x_tool=0 #variable para leer la posición x de la herramienta

#ROS service names:
srv_name_move_abs_fast='/kuka_robot/setKukaAbsFast'
srv_name_move_abs_slow='/kuka_robot/setKukaAbs'
srv_name_move_rel_fast='/kuka_robot/setKukaRelFast'
srv_name_move_rel_slow='/kuka_robot/setKukaRel'
srv_tool_homing='/kuka_tool_finger_node/home'
srv_finger_set_pose='/kuka_tool_finger_node/set_odometry' #robotnik_msgs.set.odometry
srv_digital_io='/kuka_tool/robotnik_base_hw/set_digital_output'
srv_limit_cont_current='/kuka_tool/robotnik_base_hw/set_continuous_current_limit'
srv_limit_peak_current='/kuka_tool/robotnik_base_hw/set_peak_current_limit'
srv_angle_mode='kuka_tool_finger_node/set_angle_mode'
srv_move_A1_A6='/kuka_robot/setKukaA1A6'
srv_deadman='kuka_tool_finger_node/set_deadMan_mode'
srv_rel_tool='/kuka_robot/setMoveRelTool'
srv_tare_gauges = '/tare_weight_gauges'

#ROS topic names:
topic_cart_pose_kuka='/kuka_robot/cartesian_pos_kuka'
topic_kuka_moving='/kuka_robot/kuka_moving'
topic_tool_weight='/phidget_load/load_mean'
topic_current='/kuka_tool/robotnik_base_hw/current0'
topic_horiz_force='/phidget_load/vertical_force'
topic_motor_status='/kuka_tool/robotnik_base_hw/status'
topic_tool_state='/kuka_tool/joint_states'
topic_door_state='/phidgets_vint_hub/io'
topic_tool_moving='/kuka_tool/finger_moving'
topic_tool_homed='/kuka_tool/finger_homed'
topic_tool_auto='/kuka_tool/finger_auto'

#CAJA GRIS
Prepick_Pose_z=1542.38
Preplace_Pose_z=1542.38

#current Kuka Position
pos_x_kuka=0.0
pos_y_kuka=0.0
pos_z_kuka=0.0
pos_a_kuka=0.0
pos_b_kuka=0.0
pos_c_kuka=0.0

#gauges var
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

#Turning radius
radius_threshold = 1600
min_radius_threshold = 1400