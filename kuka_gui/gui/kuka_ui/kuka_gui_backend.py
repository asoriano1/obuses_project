#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import os
from std_msgs.msg import String, Bool
from python_qt_binding import QtCore
from kuka_ui import config
from kuka_ui import global_var

class KukaGuiBackend(QtCore.QObject):
    
    def __init__(self, widget):
        
        super(KukaGuiBackend, self).__init__()
        self.widget = widget
        
        #Joysticks management with multiplexor
        rospy.loginfo("[KukaGuiBackend] Launching joysticks management with multiplexor...")
        command_string = "rosrun topic_tools mux /kuka_pad/joy /kuka_pad/ps4_joy /kuka_pad/itowa_joy mux:=mux_joy __name:=joy_mux_node &"        
        os.system(command_string)        
        #Selecting PS4 by default
        command_string = "rosrun topic_tools mux_select mux_joy /kuka_pad/ps4_joy"
        os.system(command_string)
        
        #Kill screens and launch MAIN nodes
        rospy.loginfo("[KukaGuiBackend] Killing screens... Launching bringup_standalone.launch")
        command_string = "killall screen; sleep 1; screen -S bringup -d -m roslaunch kuka_robot_bringup kuka_robot_bringup_standalone.launch"
        os.system(command_string)
        
        self.init_ros_subscribers()
        
        self.connect_buttons()

    def init_ros_subscribers(self):
        
        #subscriber to robot state
        self.sub_kuka_moving = rospy.Subscriber(topic_kuka_moving, Bool, self.callback_kuka_moving_sub)
        self.do_callback_kuka_moving.connect(self.callback_kuka_moving)

        #subscriber to robot pose
        self.sub_robot_pose = rospy.Subscriber(topic_cart_pose_kuka, Cartesian_Euler_pose, self.callback_robot_pose_sub)  
        self.do_callback_robot_pose.connect(self.callback_robot_pose)

        #subscriber to tool weight detected
        self.sub_tool_weight = rospy.Subscriber(topic_tool_weight, Float64, self.callback_tool_weight_sub)
        self.do_callback_tool_weight.connect(self.callback_tool_weight)     

        #subscriber to tool current
        self.sub_tool_current = rospy.Subscriber(topic_current, Float32, self.callback_current_sub)
        self.do_callback_current.connect(self.callback_current) 
                
        #subscriber to horiz force
        self.sub_tool_force = rospy.Subscriber(topic_horiz_force, Float64, self.callback_horiz_force_sub)
        self.do_callback_horiz_force.connect(self.callback_horiz_force)
        
        #subscriber to motor status of the tool
        self.sub_tool_status = rospy.Subscriber(topic_motor_status, RobotnikMotorsStatus, self.callback_motor_status_sub)
        self.do_callback_motor_status.connect(self.callback_motor_status)
        
        #subscriber to tool state
        self.sub_tool_state = rospy.Subscriber(topic_tool_state, JointState,  self.callback_tool_state_sub)
        self.do_callback_tool_state.connect(self.callback_tool_state)
        
        #subscriber to door state
        self.sub_door_status = rospy.Subscriber(topic_door_state, inputs_outputs, self.callback_door_state_sub)
        self.do_callback_door_state.connect(self.callback_door_state)
        
        rospy.loginfo("[KukaGuiBackend] Suscripciones ROS inicializadas")

    def connect_buttons(self):
        if hasattr(self.widget, 'Gripper_Homing_Button'):
            self.widget.Gripper_Homing_Button.pressed.connect(self.press_gripper_homing)

        if hasattr(self.widget, 'resetPositions_Button_pick'):
            self.widget.resetPositions_Button_pick.pressed.connect(self.reset_pick_positions)

    def press_gripper_homing(self):
        rospy.loginfo("[KukaGuiBackend] Botón Gripper Homing pulsado")
        # Aquí iría la llamada a un servicio o acción real

    def reset_pick_positions(self):
        rospy.loginfo("[KukaGuiBackend] Botón Reset Positions Pick pulsado")
        # Lógica de reseteo

    def pick_status_cb(self, msg):
        rospy.loginfo("[KukaGuiBackend] pick_status_cb: {}".format(msg.data))

    def place_status_cb(self, msg):
        rospy.loginfo("[KukaGuiBackend] place_status_cb: {}".format(msg.data))

    def gui_enable_cb(self, msg):
        rospy.loginfo("[KukaGuiBackend] gui_enable_cb: {}".format(msg.data))
        # Habilitar/deshabilitar botones visualmente aquí


    #relación de callbacks a señales
    #en el original no trabajaban con señales ni kuka_moving, ni robot_poser, ni tool_state, ni door_state
    def callback_tool_weight_sub(self,data):
        self.do_callback_tool_weight.emit(data)
        
    def callback_current_sub(self,data):
        self.do_callback_current.emit(data)
        
    def callback_horiz_force_sub(self,data):
        self.do_callback_horiz_force.emit(data)
        
    def callback_motor_status_sub(self,data):
        self.do_callback_motor_status.emit(data)
        
    def callback_kuka_moving_sub(self,data):
        self.do_callback_kuka_moving.emit(data)
        
    def callback_robot_pose_sub(self,data):
        self.do_callback_robot_pose.emit(data)
        
    def callback_tool_state_sub(self,data):
        self.do_callback_tool_state.emit(data)
    
    def callback_door_state_sub(self,data):
        self.do_callback_door_state.emit(data)
    
    
    