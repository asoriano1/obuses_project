# config.py
# -*- coding: utf-8 -*-
import os

# Paths
PLUGIN_PATH = os.path.dirname(os.path.abspath(__file__)) + "/"
UI_PATH = os.path.join(PLUGIN_PATH, "ui", "RqtKuka.ui")
IMG_PATH = os.path.join(PLUGIN_PATH, "ui", "images") + "/"

# Estados FSM
STATE_IDLE = 0
STATE_MOVING_TO_PREPICK = 1
STATE_DOING_PICK_TEST = 2
STATE_PICKED = 3
STATE_MOVING_TO_PLACE = 4
STATE_PLACED = 5
STATE_HOMING = 6

# Otros parámetros
PREPLACE_ANGLE_LIMIT = 20
PREPICK_ANGLE_LIMIT = 90

#flags
TOOL_HOMED=False
KUKA_AUT=False
finger_type=0
under_voltage_tool=False
first_time_enabled=False
angle_mode=True
angle_tool=0
origin_pick=0
tool_current=0
first_time_moving_kuka=False
rob_connected = False

#topic names:
topic_cart_pose_kuka='/kuka_robot/cartesian_pos_kuka'
topic_kuka_moving='/kuka_robot/kuka_moving'
topic_tool_weight='/phidget_load/load_mean'
topic_current='/kuka_tool/robotnik_base_hw/current0'
topic_horiz_force='/phidget_load/vertical_force'
topic_motor_status='/kuka_tool/robotnik_base_hw/status'
topic_tool_state='/kuka_tool/joint_states'
topic_door_state='/phidgets_vint_hub/io'


#service names:
srv_name_move_abs_fast='/kuka_robot/setKukaAbsFast'
srv_name_move_abs_slow='/kuka_robot/setKukaAbs'
srv_name_move_rel_fast='/kuka_robot/setKukaRelFast'
srv_name_move_rel_slow='/kuka_robot/setKukaRel'
srv_tool_homing='/kuka_tool/robotnik_base_hw/home'
srv_finger_set_pose='/kuka_tool_finger_node/set_odometry'
srv_digital_io='/kuka_tool/robotnik_base_hw/set_digital_output'
srv_limit_cont_current='/kuka_tool/robotnik_base_hw/set_continuous_current_limit'
srv_limit_peak_current='/kuka_tool/robotnik_base_hw/set_peak_current_limit'
srv_angle_mode='kuka_tool_finger_node/set_angle_mode'
srv_move_A1_A6='/kuka_robot/setKukaA1A6'
srv_deadman='kuka_tool_finger_node/set_deadMan_mode'
srv_rel_tool='/kuka_robot/setMoveRelTool'
srv_tare_gauges = '/tare_weight_gauges'


