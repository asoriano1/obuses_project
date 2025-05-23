#!/usr/bin/python
# -*- coding: utf-8 -*-
import os
import inspect

import rospy
import rospkg
import time
import datetime 
import xacro
import subprocess
import sys
import QtCore
import QtGui
import numpy
from obuses_poses import *

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QDialog, QFileDialog, QMessageBox, QTableWidgetItem, QApplication, QGroupBox, QCheckBox
from std_msgs.msg import Bool, Float64, Float32
from std_srvs.srv import SetBool
from sensor_msgs.msg import JointState
from robotnik_msgs.srv import home, set_odometry, set_CartesianEuler_pose, set_digital_output, set_float_value
from robotnik_msgs.msg import Cartesian_Euler_pose, RobotnikMotorsStatus, MotorStatus, inputs_outputs
from geometry_msgs.msg import Pose, Point, Quaternion
from kuka_rsi_cartesian_hw_interface.srv import set_A1_A6

from global_var import *
from global_flags import *
from obus_manager import ObusManager

class KukaGUI(QWidget):
        
    do_callback_motor_status = QtCore.pyqtSignal(RobotnikMotorsStatus)
    do_callback_horiz_force = QtCore.pyqtSignal(Float64)
    do_callback_current = QtCore.pyqtSignal(Float32)
    do_callback_tool_weight = QtCore.pyqtSignal(Float64)
    do_callback_moving = QtCore.pyqtSignal(Bool)

    def __init__(self, parent=None):
        
        super(KukaGUI, self).__init__(parent)
        
        # Give QObjects reasonable names
        self.setObjectName('KukaGUI')

        loadUi(UI_PATH, self)
        # Give QObjects reasonable names
        self.setObjectName('RqtKukaUi')
        
        print '__Checking background processes__'   
        #Joysticks management with multiplexor        
        #command_string = "screen -S mux -d -m rosrun topic_tools mux /kuka_pad/joy /kuka_pad/ps4_joy /kuka_pad/itowa_joy mux:=mux_joy __name:=joy_mux_node &"
        command_string = "rosrun topic_tools mux /kuka_pad/joy /kuka_pad/ps4_joy /kuka_pad/itowa_joy mux:=mux_joy __name:=joy_mux_node &"
        os.system(command_string)
        #PS4 by default
        command_string = "rosrun topic_tools mux_select mux_joy /kuka_pad/ps4_joy"
        os.system(command_string)
        
        ###launch MAIN nodes: Joysticks and main
        command_string = "killall screen; sleep 1; screen -S bringup -d -m roslaunch kuka_robot_bringup kuka_robot_bringup_standalone.launch"
        #command_string = "rosnode kill /kuka_pad/ps4_joystick; sleep 1; rosnode kill /kuka_pad/itowa_safe_joystick; sleep 1;rosnode kill /kuka_pad/robotnik_trajectory_pad_node; sleep 1; rosnode kill /kuka_robot/kuka_cartesian_hardware_interface; sleep 1; roslaunch kuka_robot_bringup kuka_robot_bringup_standalone.launch &"        
        ###command_string = "rosnode kill /kuka_pad/ps4_joystick; sleep 1; rosnode kill /kuka_pad/itowa_safe_joystick; sleep 1;rosnode kill /kuka_pad/robotnik_trajectory_pad_node; sleep 1; rosnode kill /kuka_robot/kuka_cartesian_hardware_interface; sleep 1;"        
        os.system(command_string)
                    
        # add signals/slots
        #select obus calibre
        self.calibre_comboBox.currentIndexChanged.connect(self.calibre_selected)
        self.joy_comboBox.currentIndexChanged.connect(self.joy_selected)
        self.mode_label.setText("NOT CONNECTED")
        #Buttons
        self.Finger_Adjust_Button.pressed.connect(self.press_finger_adjust_button)
        self.Tare_Button.pressed.connect(self.press_tare_button)
        self.Tare_Reset_Button.pressed.connect(self.press_tare_reset_button)
        self.Reset_Ext_Button.pressed.connect(self.press_reset_external_pc_button)
        self.Reset_Robot_Button.pressed.connect(self.press_reset_robot_button)
        #self.Reset_Robot_Button.hide()
        self.MoveToTable_Button.pressed.connect(self.press_homming_button)#self.press_move_to_rotation_table_button) 
        self.PickTest_Button.pressed.connect(self.press_picktest_button)
        self.Gripper_Homing_Button.pressed.connect(self.press_tool_homming)
        self.Led_On_Button.pressed.connect(self.press_led_on_button)
        self.Led_Off_Button.pressed.connect(self.press_led_off_button)
        self.Light_On_Button.pressed.connect(self.press_light_on_button)
        self.Light_Off_Button.pressed.connect(self.press_light_off_button)
        self.resetPositions_Button_place.pressed.connect(self.press_reset_positions_button_place)
        self.resetPositions_Button_pick.pressed.connect(self.press_reset_positions_button_pick)
        self.undoPositions_Button_pick.pressed.connect(self.press_undo_positions_button_pick)
        self.undoPositions_Button_place.pressed.connect(self.press_undo_positions_button_place)
        self.press_Button.pressed.connect(self.aut_press_tool)
        
        #Checkboxes of robot settings
        self.deadMan_check.clicked.connect(self.deadMan_state_changed)
        self.toolAngle_check.clicked.connect(self.toolAngle_state_changed)
        self.toolOrientation_check.clicked.connect(self.toolOrientation_state_changed)
        
        self.Led_On_Button.setStyleSheet("color: rgb(80, 170, 80)")
        self.Led_Off_Button.setStyleSheet("color: rgb(170, 80, 80)")
        self.Light_On_Button.setStyleSheet("color: rgb(80, 170, 80)")
        self.Light_Off_Button.setStyleSheet("color: rgb(170, 80, 80)")
        
        pixmap = QtGui.QPixmap(IMG_PATH+"/fondo_huevera_0.png")
        self.background_plate.setPixmap(pixmap)
        self.background_plate_pick.setPixmap(pixmap)
        #self.Home_Button.setEnabled(False)
        self.Finger_Adjust_Button.setEnabled(False)
        self.MoveToTable_Button.setEnabled(False)
        self.weightProgressBar_2.setMinimum(0)
        self.weightProgressBar_2.setMaximum(15)
        
        # si es -1 fuerza a que se seleccione un pick para habilitar un place lo cual quizá no siempre sea lo mejor
        self.origin_pick_quad = 0
        
        self.state_dict = self.init_state_dict()
        self.obus_manager = ObusManager(self)
        
        
        #subscriber to robot state
        self.sub_robot_moving = rospy.Subscriber(topic_kuka_moving, Bool, self.callback_moving)
        #self.do_callback_moving.connect(self.callback_moving)

        #subscriber to robot pose
        self.sub_robot_pose = rospy.Subscriber(topic_cart_pose_kuka, Cartesian_Euler_pose, self.callback_robot_pose)     

        #subscriber to tool weight detected
        self.sub_tool_weight = rospy.Subscriber(topic_tool_weight, Float64, self.callback_tool_weight2)
        self.do_callback_tool_weight.connect(self.callback_tool_weight)     

        #subscriber to tool current
        self.sub_tool_current = rospy.Subscriber(topic_current, Float32, self.callback_current2)
        self.do_callback_current.connect(self.callback_current) 
                
        #subscriber to horiz force
        self.sub_tool_force = rospy.Subscriber(topic_horiz_force, Float64, self.callback_horiz_force2)
        self.do_callback_horiz_force.connect(self.callback_horiz_force)
        
        #subscriber to motor status of the tool
        self.sub_tool_status = rospy.Subscriber(topic_motor_status, RobotnikMotorsStatus, self.callback_motor_status2)
        self.do_callback_motor_status.connect(self.callback_motor_status)
        
        #subscriber to tool state
        self.sub_tool_state = rospy.Subscriber(topic_tool_state, JointState,  self.callback_tool_state)
        
        #subscriber to door state
        self.sub_door_status = rospy.Subscriber(topic_door_state, inputs_outputs, self.callback_door_state)
        
        #Robot settings initialization
        #Default: deadman activated
        try:
                deadman_service=rospy.ServiceProxy(srv_deadman, SetBool)
                ret = deadman_service(True)
        except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                #ret=QMessageBox.critical(self, "WARNING!", 'Deadman service not available', QMessageBox.Ok)
                
        #Default: angle activated
        try:
                angle_mode_service=rospy.ServiceProxy(srv_angle_mode, SetBool)
                ret = angle_mode_service(True)
        except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                #ret=QMessageBox.critical(self, "WARNING!", 'Angle Mode service not available', QMessageBox.Ok)
        #Default: tool orientation reference deactivated
        try:
                toolOrientation_service=rospy.ServiceProxy(srv_rel_tool, SetBool)
                ret = toolOrientation_service(False)
        except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                #ret=QMessageBox.critical(self, "WARNING!", 'Tool Orientation service not available', QMessageBox.Ok)

        
        self.setWindowTitle(" ")           
        self._name = "RqtKuka"
                
        #Variable para almacenar el ultimo obus seleccionado
        self.last_obus_selected_pick = -1
        self.last_obus_selected_place = -1


    #filtro para detectar el raton y ponerlo verde si esta el cursor encima o dejarlo blanco si no
    def eventFilter(self, object, event):               
        if not KUKA_AUT:
            # huevera 16
            if finger_type == 1:        
                # obuses parte izquierda (1-8 pick/place)
                # PlaceObusX_Y corresponde a los nombres de los botones que se le han dado en la .ui
                if (
                    (object == self.PlaceObusButton16_1 and not self.state_dict.get(('place', 16, 1), False)) or
                    (object == self.PlaceObusButton16_2 and not self.state_dict.get(('place', 16, 2), False)) or
                    (object == self.PlaceObusButton16_3 and not self.state_dict.get(('place', 16, 3), False)) or
                    (object == self.PlaceObusButton16_4 and not self.state_dict.get(('place', 16, 4), False)) or
                    (object == self.PlaceObusButton16_5 and not self.state_dict.get(('place', 16, 5), False)) or
                    (object == self.PlaceObusButton16_6 and not self.state_dict.get(('place', 16, 6), False)) or
                    (object == self.PlaceObusButton16_7 and not self.state_dict.get(('place', 16, 7), False)) or
                    (object == self.PlaceObusButton16_8 and not self.state_dict.get(('place', 16, 8), False)) or
                    (object == self.PickObusButton16_1 and not self.state_dict.get(('pick', 16, 1), False)) or
                    (object == self.PickObusButton16_2 and not self.state_dict.get(('pick', 16, 2), False)) or
                    (object == self.PickObusButton16_3 and not self.state_dict.get(('pick', 16, 3), False)) or
                    (object == self.PickObusButton16_4 and not self.state_dict.get(('pick', 16, 4), False)) or
                    (object == self.PickObusButton16_5 and not self.state_dict.get(('pick', 16, 5), False)) or
                    (object == self.PickObusButton16_6 and not self.state_dict.get(('pick', 16, 6), False)) or
                    (object == self.PickObusButton16_7 and not self.state_dict.get(('pick', 16, 7), False)) or
                    (object == self.PickObusButton16_8 and not self.state_dict.get(('pick', 16, 8), False)) or
                    (object == self.PickObusButton16_9 and not self.state_dict.get(('pick', 16, 9), False)) or
                    (object == self.PickObusButton16_10 and not self.state_dict.get(('pick', 16, 10), False))
                ):
                    if event.type() == QtCore.QEvent.HoverEnter:                        
                        object.setIcon(QtGui.QIcon(imgObus16izq[1]))
                    elif event.type() == QtCore.QEvent.HoverLeave:
                        object.setIcon(QtGui.QIcon(imgObus16izq[0]))

                # obuses parte derecha (9-16 pick/place, 11-20 pick)
                elif (
                    (object == self.PlaceObusButton16_9 and not self.state_dict.get(('place', 16, 9), False)) or
                    (object == self.PlaceObusButton16_10 and not self.state_dict.get(('place', 16, 10), False)) or
                    (object == self.PlaceObusButton16_11 and not self.state_dict.get(('place', 16, 11), False)) or
                    (object == self.PlaceObusButton16_12 and not self.state_dict.get(('place', 16, 12), False)) or
                    (object == self.PlaceObusButton16_13 and not self.state_dict.get(('place', 16, 13), False)) or
                    (object == self.PlaceObusButton16_14 and not self.state_dict.get(('place', 16, 14), False)) or
                    (object == self.PlaceObusButton16_15 and not self.state_dict.get(('place', 16, 15), False)) or
                    (object == self.PlaceObusButton16_16 and not self.state_dict.get(('place', 16, 16), False)) or
                    (object == self.PickObusButton16_11 and not self.state_dict.get(('pick', 16, 11), False)) or
                    (object == self.PickObusButton16_12 and not self.state_dict.get(('pick', 16, 12), False)) or
                    (object == self.PickObusButton16_13 and not self.state_dict.get(('pick', 16, 13), False)) or
                    (object == self.PickObusButton16_14 and not self.state_dict.get(('pick', 16, 14), False)) or
                    (object == self.PickObusButton16_15 and not self.state_dict.get(('pick', 16, 15), False)) or
                    (object == self.PickObusButton16_16 and not self.state_dict.get(('pick', 16, 16), False)) or
                    (object == self.PickObusButton16_17 and not self.state_dict.get(('pick', 16, 17), False)) or
                    (object == self.PickObusButton16_18 and not self.state_dict.get(('pick', 16, 18), False)) or
                    (object == self.PickObusButton16_19 and not self.state_dict.get(('pick', 16, 19), False)) or
                    (object == self.PickObusButton16_20 and not self.state_dict.get(('pick', 16, 20), False))
                ):
                    if event.type() == QtCore.QEvent.HoverEnter:                        
                        object.setIcon(QtGui.QIcon(imgObus16der[1]))
                    elif event.type() == QtCore.QEvent.HoverLeave:
                        object.setIcon(QtGui.QIcon(imgObus16der[0]))

            # huevera 8
            if finger_type == 2:                
                # obuses parte izquierda (1-4 pick/place, 1-7 pick)
                if (
                    (object == self.PlaceObusButton8_1 and not self.state_dict.get(('place', 8, 1), False)) or
                    (object == self.PlaceObusButton8_2 and not self.state_dict.get(('place', 8, 2), False)) or
                    (object == self.PlaceObusButton8_3 and not self.state_dict.get(('place', 8, 3), False)) or
                    (object == self.PlaceObusButton8_4 and not self.state_dict.get(('place', 8, 4), False)) or
                    (object == self.PickObusButton8_1 and not self.state_dict.get(('pick', 8, 1), False)) or
                    (object == self.PickObusButton8_2 and not self.state_dict.get(('pick', 8, 2), False)) or
                    (object == self.PickObusButton8_3 and not self.state_dict.get(('pick', 8, 3), False)) or
                    (object == self.PickObusButton8_4 and not self.state_dict.get(('pick', 8, 4), False)) or
                    (object == self.PickObusButton8_5 and not self.state_dict.get(('pick', 8, 5), False)) or
                    (object == self.PickObusButton8_6 and not self.state_dict.get(('pick', 8, 6), False)) or
                    (object == self.PickObusButton8_7 and not self.state_dict.get(('pick', 8, 7), False))
                ):
                    if event.type() == QtCore.QEvent.HoverEnter:
                        object.setIcon(QtGui.QIcon(imgObus8izq[1]))
                    elif event.type() == QtCore.QEvent.HoverLeave:
                        object.setIcon(QtGui.QIcon(imgObus8izq[0]))
                # obuses parte derecha (5-8 pick/place, 8-14 pick)
                elif (
                    (object == self.PlaceObusButton8_5 and not self.state_dict.get(('place', 8, 5), False)) or
                    (object == self.PlaceObusButton8_6 and not self.state_dict.get(('place', 8, 6), False)) or
                    (object == self.PlaceObusButton8_7 and not self.state_dict.get(('place', 8, 7), False)) or
                    (object == self.PlaceObusButton8_8 and not self.state_dict.get(('place', 8, 8), False)) or
                    (object == self.PickObusButton8_8 and not self.state_dict.get(('pick', 8, 8), False)) or
                    (object == self.PickObusButton8_9 and not self.state_dict.get(('pick', 8, 9), False)) or
                    (object == self.PickObusButton8_10 and not self.state_dict.get(('pick', 8, 10), False)) or
                    (object == self.PickObusButton8_11 and not self.state_dict.get(('pick', 8, 11), False)) or
                    (object == self.PickObusButton8_12 and not self.state_dict.get(('pick', 8, 12), False)) or
                    (object == self.PickObusButton8_13 and not self.state_dict.get(('pick', 8, 13), False)) or
                    (object == self.PickObusButton8_14 and not self.state_dict.get(('pick', 8, 14), False))
                ):
                    if event.type() == QtCore.QEvent.HoverEnter:
                        object.setIcon(QtGui.QIcon(imgObus8der[1]))
                    elif event.type() == QtCore.QEvent.HoverLeave:
                        object.setIcon(QtGui.QIcon(imgObus8der[0]))

            # huevera 4
            if finger_type == 3:
                if (
                    (object == self.PlaceObusButton4_1 and not self.state_dict.get(('place', 4, 1), False)) or
                    (object == self.PlaceObusButton4_2 and not self.state_dict.get(('place', 4, 2), False)) or
                    (object == self.PlaceObusButton4_3 and not self.state_dict.get(('place', 4, 3), False)) or
                    (object == self.PlaceObusButton4_4 and not self.state_dict.get(('place', 4, 4), False)) or
                    (object == self.PickObusButton4_1 and not self.state_dict.get(('pick', 4, 1), False)) or
                    (object == self.PickObusButton4_2 and not self.state_dict.get(('pick', 4, 2), False)) or
                    (object == self.PickObusButton4_3 and not self.state_dict.get(('pick', 4, 3), False)) or
                    (object == self.PickObusButton4_4 and not self.state_dict.get(('pick', 4, 4), False)) or
                    (object == self.PickObusButton4_5 and not self.state_dict.get(('pick', 4, 5), False))
                ):
                    if event.type() == QtCore.QEvent.HoverEnter:
                        object.setIcon(QtGui.QIcon(imgObus4[1]))
                    elif event.type() == QtCore.QEvent.HoverLeave:
                        object.setIcon(QtGui.QIcon(imgObus4[0]))

            # huevera 2
            if finger_type == 4:
                if (
                    (object == self.PlaceObusButton2_1 and not self.state_dict.get(('place', 2, 1), False)) or
                    (object == self.PlaceObusButton2_2 and not self.state_dict.get(('place', 2, 2), False)) or
                    (object == self.PickObusButton2_1 and not self.state_dict.get(('pick', 2, 1), False)) or
                    (object == self.PickObusButton2_2 and not self.state_dict.get(('pick', 2, 2), False)) or
                    (object == self.PickObusButton2_3 and not self.state_dict.get(('pick', 2, 3), False)) or
                    (object == self.PickObusButton2_4 and not self.state_dict.get(('pick', 2, 4), False))
                ):
                    if event.type() == QtCore.QEvent.HoverEnter:
                        object.setIcon(QtGui.QIcon(imgObus2[1]))
                    elif event.type() == QtCore.QEvent.HoverLeave:
                        object.setIcon(QtGui.QIcon(imgObus2[0]))

        return False

    
    #inicialización del estado de los obuses para state_dict['tipo', grupo, idx]
    def init_state_dict(self):
        d = {}
        # Place
        for grupo, n in [(2,2), (4,4), (8,8), (16,16)]:
            for idx in xrange(1, n+1):
                d[('place', grupo, idx)] = False
        # Pick
        for grupo, n in [(2,4), (4,5), (8,14), (16,20)]:
            for idx in xrange(1, n+1):
                d[('pick', grupo, idx)] = False
        return d


    def desactivate_buttons(self):

        self.PickTest_Button.setEnabled(False)
        self.Gripper_Homing_Button.setEnabled(False)
        self.MoveToTable_Button.setEnabled(False)
        self.resetPositions_Button_place.setEnabled(False)
        self.resetPositions_Button_pick.setEnabled(False)
        self.undoPositions_Button_place.setEnabled(False)
        self.undoPositions_Button_pick.setEnabled(False)
        self.calibre_comboBox.setEnabled(False)
        self.Finger_Adjust_Button.setEnabled(False)
        self.obus_manager.deactivate_buttons('Place', 2, 1, 2)
        self.obus_manager.deactivate_buttons('Place', 4, 1, 4)
        self.obus_manager.deactivate_buttons('Place', 8, 1, 8)
        self.obus_manager.deactivate_buttons('Place', 16, 1, 16)
        self.obus_manager.deactivate_buttons('Pick', 2, 1, 4)
        self.obus_manager.deactivate_buttons('Pick', 4, 1, 5)
        self.obus_manager.deactivate_buttons('Pick', 8, 1, 14)
        self.obus_manager.deactivate_buttons('Pick', 16, 1, 20)
        
    def activate_buttons(self):        
        self.PickTest_Button.setEnabled(True)
        self.Gripper_Homing_Button.setEnabled(True)
        self.Finger_Adjust_Button.setEnabled(True)
        self.MoveToTable_Button.setEnabled(True)
        self.resetPositions_Button_place.setEnabled(True)
        self.resetPositions_Button_pick.setEnabled(True)
        self.undoPositions_Button_place.setEnabled(True)
        self.undoPositions_Button_pick.setEnabled(True)
        self.calibre_comboBox.setEnabled(True)
        self.joy_comboBox.setEnabled(True)
        
        #activa todos los de pick
        self.obus_manager.activate_buttons('Pick', 2, 1, 4)
        self.obus_manager.activate_buttons('Pick', 4, 1, 5)
        self.obus_manager.activate_buttons('Pick', 8, 1, 14)
        self.obus_manager.activate_buttons('Pick', 16, 1, 20)
        
        #no hay ninguno pulsado, activa todos los places
        if(self.origin_pick_quad==0):
            print("Activa todos los places")
            self.obus_manager.activate_buttons('Place', 16, 1, 16)
            self.obus_manager.activate_buttons('Place', 2, 1, 2)
            self.obus_manager.activate_buttons('Place', 4, 1, 4)
            self.obus_manager.activate_buttons('Place', 8, 1, 8)
        #seleccionado alguno de la parte superior de la caja y dedos 3 y 4               
        elif((finger_type==3 or finger_type==4) and (self.origin_pick_quad==2 or self.origin_pick_quad==4)):
            self.obus_manager.activate_buttons('Place', 4, 3, 4)
            self.obus_manager.activate_buttons('Place', 2, 2, 2)
        #seleccionado alguno de la parte inferior de la caja y dedos 3 y 4  
        elif((finger_type==3 or finger_type==4) and (self.origin_pick_quad==1 or self.origin_pick_quad==3)):
            self.obus_manager.activate_buttons('Place', 4, 1, 2)
            self.obus_manager.activate_buttons('Place', 2, 1, 1)         
        elif(self.origin_pick_quad==2 or self.origin_pick_quad==3):
            self.obus_manager.activate_buttons('Place', 16, 5, 12)
            self.obus_manager.activate_buttons('Place', 8, 3, 6)  
        elif(self.origin_pick_quad==1 or self.origin_pick_quad==4):
            self.obus_manager.activate_buttons('Place', 16, 1, 4)
            self.obus_manager.activate_buttons('Place', 16, 13, 16)
            self.obus_manager.activate_buttons('Place', 8, 1, 2) 
            self.obus_manager.activate_buttons('Place', 8, 7, 8) 
 
        ##PLACE
        for group, last in [(16,16), (8,8), (4,4), (2,2)]:
            self.obus_manager.remove_event_filters('Place', group, 1, last)
        ##PICK
        for group, last in [(16,20), (8,14), (4,5), (2,4)]:
            self.obus_manager.remove_event_filters('Pick', group, 1, last)
    
    def press_reset_positions_button_place(self):
        self.obus_manager.reset_positions_place()
        self.desactivate_buttons()
        self.activate_buttons()
    
    def press_reset_positions_button_pick(self):
        self.obus_manager.reset_positions_pick()        

    def select_icon(self,operation, obus_id, state):        
        #operation es "pick" o "place"
        #obus_id es un vector donde la posicion [0] contiene el calibre y posicion [1] la posicion
        #state=0 normal - blanco
        #state=1 resaltado - verde
        #state=2 seleccionado - rojo
        calibre = int(obus_id[0])
        num = int(obus_id[1])
        
        if calibre == 2:
            return imgObus2[state]
        elif calibre == 4:
            return imgObus4[state]
        elif calibre == 8:
            if operation == 'pick':
                if num < 8:
                    return imgObus8izq[state]                    
                if num >= 8:
                    return imgObus8der[state]
            elif operation == 'place':
                if num < 5:
                    return imgObus8izq[state]
                if num >= 5:
                    return imgObus8der[state]
        elif calibre == 16:
            if operation == 'pick':
                if num < 11:
                    return imgObus16izq[state]
                elif num >= 11:
                    return imgObus16der[state]
            elif operation == 'place':
                if num < 9:
                    return imgObus16izq[state]
                elif num >= 9:
                    return imgObus16der[state]

    def press_undo_positions_button_pick(self):
        print("Undo pick button pressed")
        if self.last_obus_selected_pick == -1:
            return

        # last_obus_selected_pick formato 'YY_XX'
        group, idx = map(int, self.last_obus_selected_pick.split("_"))
        self.state_dict[('pick',group, idx)] = False

        path = self.select_icon('pick', [group, idx], 0)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(path), QtGui.QIcon.Disabled)
        btn_name = 'PickObusButton%d_%d' % (group, idx)
        btn = getattr(self, btn_name, None)
        if btn:
            btn.setIcon(icon)
            btn.installEventFilter(self)

        self.last_obus_selected_pick = -1
        self.origin_pick_quad = 0        


    def press_undo_positions_button_place(self):
        if self.last_obus_selected_place == -1:
            return

        # last_obus_selected_place formato 'YY_XX'
        group, idx = map(int, self.last_obus_selected_place.split("_"))
        self.state_dict[('place',group, idx)] = False

        path = self.select_icon('place', [group, idx], 0)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(path), QtGui.QIcon.Disabled)
        btn_name = 'PlaceObusButton%d_%d' % (group, idx)
        btn = getattr(self, btn_name, None)
        if btn:
            btn.setIcon(icon)
            btn.installEventFilter(self)

        self.last_obus_selected_place = -1


    def callback_moving(self, data):
        global KUKA_AUT, first_time_moving_kuka
        #print 'CB:moving_received:',data.data
        if data.data == True :
            if not KUKA_AUT:
                KUKA_AUT=True
            if first_time_moving_kuka:
                    self.mode_label.setText("AUTOMATIC")
                    self.desactivate_buttons()
                    first_time_moving_kuka = False
                            
        else:
            if KUKA_AUT:    
                KUKA_AUT=False
            if first_time_moving_kuka==False:
                self.mode_label.setText("MANUAL")
                self.activate_buttons()
                first_time_moving_kuka = True

    def callback_moving2(self,data):
            self.do_callback_moving.emit(data)
            
    def callback_motor_status(self,data):

        global under_voltage_tool, first_time_enabled, weight_empty, weight_read
        motor1=data.motor_status[1]
        driveflags_1=numpy.array(map(int,motor1.driveflags))
        under_voltage_1=driveflags_1[12]
        if(under_voltage_1==1):
            under_voltage_tool=True
            pixmap = QtGui.QPixmap(IMG_PATH+"/pinza_roja_peq2.png")
            self.under_voltage_tool.setPixmap(pixmap)
            #print 'undervoltage'
        else:
            under_voltage_tool=False
            pixmap = QtGui.QPixmap(IMG_PATH+"/pinza_verde_peq2.png")
            self.under_voltage_tool.setPixmap(pixmap)
        if(motor1.status=="OPERATION_ENABLED" and first_time_enabled):
            #if(weight_read-weight_empty<-10):
            first_time_enabled=False
            #print 'Warninng of weight should be here'				
            ret = QMessageBox.information(self, "WARNING!", 'Tool enabled', QMessageBox.Ok)
        if(motor1.status=="FAULT"):
            first_time_enabled=True
            #print first_time_enabled
            
    def callback_door_state(self, data):
        
        if data.digital_inputs[0]:
            pixmap =QtGui.QPixmap(IMG_PATH+"/puerta_roja_peq.png")
            self.label_door.setPixmap(pixmap)
        else :
            pixmap =QtGui.QPixmap(IMG_PATH+"/puerta_verde_peq.png")
            self.label_door.setPixmap(pixmap)
        if data.digital_inputs[1]:
            pixmap =QtGui.QPixmap(IMG_PATH+"/emergency_verde_peq.png")
            self.label_6.setPixmap(pixmap)
        else :
            pixmap =QtGui.QPixmap(IMG_PATH+"/emergency_roja_peq.png")
            self.label_6.setPixmap(pixmap)
                        
    def callback_motor_status2(self,data):
            self.do_callback_motor_status.emit(data)

    def callback_robot_pose(self, data):
        global pos_x_kuka, pos_y_kuka, pos_z_kuka, pos_a_kuka, pos_b_kuka, pos_c_kuka, rob_connected#, elapsed_time_gauges#, gauges_failure
        if not rob_connected :
             rob_connected = True
             self.mode_label.setText("MANUAL")
        #print 'CB:robot_pose_received',data
        pos_x_kuka=data.x
        pos_y_kuka=data.y
        pos_z_kuka=data.z
        pos_a_kuka=data.A
        pos_b_kuka=data.B
        pos_c_kuka=data.C
        #elapsed_time_gauges=time.time()-start_time_gauges
        #print 'time between robot callback and gauges' ,elapsed_time_gauges
        #if (elapsed_time_gauges>=2):
            #gauges_failure=True
    def callback_horiz_force(self, data):
        global horiz_force_read, horiz_force_empty
        #print 'force_received:',data.data
        horiz_force_read = data.data
        self.vertforce_lcdNumber.setDigitCount(4)
        self.vertforce_lcdNumber.display(round((data.data-horiz_force_empty)*0.19,1))
    def callback_horiz_force2(self,data):
            self.do_callback_horiz_force.emit(data)
        
    def callback_tool_weight(self, data):
        global weight_empty, weight_reads#, gauges_failure, start_time_gauges
        #start_time_gauges=time.time()
        #gauges_failure=False
        self.weight_lcdNumber.setDigitCount(4)
        palette = self.weight_lcdNumber.palette()
        progressBar_palette = self.weightProgressBar.palette()   
        #print 'CB:tool_weight_received',data
        weight_read=data.data
        weight_no_tool=data.data#-weight_empty
        weight_reads[0]=weight_no_tool
        for i in range(1, 5):
            weight_no_tool=weight_no_tool+weight_reads[i]
        weight_no_tool=weight_no_tool/5
        for i in range(1, 5):
            weight_reads[i]=weight_reads[i-1]
        self.weight_lcdNumber.setDecMode()
        #self.weight_lcdNumber.setNumDigits(3)
        self.weight_lcdNumber.display(round(weight_no_tool,1))
        if (-weight_no_tool)<weight_expected_min:
            palette.setColor(palette.WindowText, QtGui.QColor(10, 10, 10))
            self.weightProgressBar.setStyleSheet("""QProgressBar::chunk { background: grey; }""")
            self.weightProgressBar_2.setStyleSheet("""QProgressBar::chunk { background: grey; }""")
            if(weight_no_tool>5):
                    self.weightProgressBar_2.setStyleSheet("""QProgressBar::chunk { background: red; }""")
        elif (-weight_no_tool)<weight_expected_max:
            palette.setColor(palette.WindowText, QtGui.QColor(20, 230, 20))
            self.weightProgressBar.setStyleSheet("""QProgressBar::chunk { background: green; }""")
        else:
            palette.setColor(palette.WindowText, QtGui.QColor(255, 50, 50))
            self.weightProgressBar.setStyleSheet("""QProgressBar::chunk { background: red; }""")
        self.weight_lcdNumber.setPalette(palette)
        if(weight_no_tool<0):
                self.weightProgressBar_2.setValue(0)
                self.weightProgressBar.setValue(-int(round(weight_no_tool)))
        else: 
                self.weightProgressBar_2.setValue(int(round(weight_no_tool)))
                self.weightProgressBar.setValue(0)
        
    def callback_tool_weight2(self,data):
            self.do_callback_tool_weight.emit(data)

    def callback_current(self, data):
        #print 'CB:current_received',data
        global tool_current
        tool_current = data.data
        self.tool_force_lcdNumber.setDigitCount(4)
        self.tool_force_lcdNumber.display(round(data.data,1))
    def callback_current2(self,data):
            self.do_callback_current.emit(data)
    
    def press_move_to_rotation_table_button(self):
        ret = QMessageBox.warning(self, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            try:
                placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=placed_rel_service(0, 0, pose_z_safe-pos_z_kuka, 0, 0, 0)
                #KUKA_AUT=True
                self.sleep_loop(2)
                while KUKA_AUT: self.sleep_loop(0.3)
                placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)
                ret=placed_abs_service(H2O1_Pose_x, H2O1_Pose_y, H2O1_Pose_z, table_pose_a, H2O1_Pose_b, H2O1_Pose_c)
                #KUKA_AUT=True
                self.sleep_loop(2)
                while KUKA_AUT: self.sleep_loop(0.3)
                placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)                
                ret = placed_abs_service(table_pose_x, table_pose_y, table_pose_z, table_pose_a, table_pose_b, table_pose_c)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
                ret=QMessageBox.critical(self, "WARNING!", 'Movement Service not available.', QMessageBox.Ok)    


    #Pick buttons obus 2
    def press_pick_obus2_1_button(self):
        self.obus_manager.press_obus_button('pick', 2, 1,
            side='left',
            pre_z=Prepick_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(pick_A1, pick_left_A6)
        )

    def press_pick_obus2_2_button(self):
        self.obus_manager.press_obus_button('pick', 2, 2,
            side='left',
            pre_z=Prepick_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(pick_A1, pick_left_A6)
        )

    def press_pick_obus2_3_button(self):
        self.obus_manager.press_obus_button('pick', 2, 3,
            side='right',
            pre_z=Prepick_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(pick_A1, pick_right_A6)
        )        

    def press_pick_obus2_4_button(self):
        self.obus_manager.press_obus_button('pick', 2, 4,
            side='right',
            pre_z=Prepick_Pose_z,
            axis_service=srv_move_A1_A6, 
            axis_args=(pick_A1, pick_right_A6)
        )
                
    #Pick buttons obus 4
    def press_pick_obus4_1_button(self):
        self.obus_manager.press_obus_button('pick', 4, 1, 
            side='left',
            pre_z=Prepick_Pose_z, 
            axis_service=srv_move_A1_A6, 
            axis_args=(pick_A1, pick_left_A6)
        )
        
    def press_pick_obus4_2_button(self):
        self.obus_manager.press_obus_button('pick', 4, 2, 
            side='left',
            pre_z=Prepick_Pose_z, 
            axis_service=srv_move_A1_A6, 
            axis_args=(pick_A1, pick_left_A6)
        )        

    def press_pick_obus4_3_button(self):
        self.obus_manager.press_obus_button('pick', 4, 3, 
            side='left',
            pre_z=Prepick_Pose_z, 
            axis_service=srv_move_A1_A6, 
            axis_args=(pick_A1, pick_left_A6)
        )  

    def press_pick_obus4_4_button(self):
        self.obus_manager.press_obus_button('pick', 4, 4, 
            side='right',
            pre_z=Prepick_Pose_z, 
            axis_service=srv_move_A1_A6, 
            axis_args=(pick_A1, pick_right_A6)
        )
        

    def press_pick_obus4_5_button(self):
        self.obus_manager.press_obus_button('pick', 4, 5, 
            side='right',
            pre_z=Prepick_Pose_z, 
            axis_service=srv_move_A1_A6, 
            axis_args=(pick_A1, pick_right_A6)
        )        

    #Pick buttons obus 8
    def press_pick_obus8_1_button(self):
        self.obus_manager.press_obus_button('pick', 8, 1, 
            side='left',
            pre_z=Prepick_Pose_z, 
            axis_service=srv_move_A1_A6, 
            axis_args=(pick_A1, pick_left_A6)
        )        

    def press_pick_obus8_2_button(self):
        self.obus_manager.press_obus_button('pick', 8, 2, 
            side='left',
            pre_z=Prepick_Pose_z, 
            axis_service=srv_move_A1_A6, 
            axis_args=(pick_A1, pick_left_A6)
        )

    def press_pick_obus8_3_button(self):
        self.obus_manager.press_obus_button('pick', 8, 3, 
            side='left',
            pre_z=Prepick_Pose_z, 
            axis_service=srv_move_A1_A6, 
            axis_args=(pick_A1, pick_left_A6)
        )        

    def press_pick_obus8_4_button(self):
        self.obus_manager.press_obus_button('pick', 8, 4, 
            side='left',
            pre_z=Prepick_Pose_z, 
            axis_service=srv_move_A1_A6, 
            axis_args=(pick_A1, pick_left_A6)
        )        

    def press_pick_obus8_5_button(self):
        self.obus_manager.press_obus_button('pick', 8, 5, 
            side='right',
            pre_z=Prepick_Pose_z, 
            axis_service=srv_move_A1_A6, 
            axis_args=(pick_A1, pick_right_A6)
        )        

    def press_pick_obus8_6_button(self):
        self.obus_manager.press_obus_button('pick', 8, 6, 
            side='right',
            pre_z=Prepick_Pose_z, 
            axis_service=srv_move_A1_A6, 
            axis_args=(pick_A1, pick_right_A6)
        )        

    def press_pick_obus8_7_button(self):
        self.obus_manager.press_obus_button('pick', 8, 7, 
            side='right',
            pre_z=Prepick_Pose_z, 
            axis_service=srv_move_A1_A6, 
            axis_args=(pick_A1, pick_right_A6)
        )         

    def press_pick_obus8_8_button(self):
        self.obus_manager.press_obus_button('pick', 8, 8, 
            side='left',
            pre_z=Prepick_Pose_z, 
            axis_service=srv_move_A1_A6, 
            axis_args=(pick_A1, pick_left_A6)
        )          
                
    def press_pick_obus8_9_button(self):
        self.obus_manager.press_obus_button('pick', 8, 9, 
            side='left',
            pre_z=Prepick_Pose_z, 
            axis_service=srv_move_A1_A6, 
            axis_args=(pick_A1, pick_left_A6)
        )          

    def press_pick_obus8_10_button(self):
        self.obus_manager.press_obus_button('pick', 8, 10, 
            side='left',
            pre_z=Prepick_Pose_z, 
            axis_service=srv_move_A1_A6, 
            axis_args=(pick_A1, pick_left_A6)
        )         
                
    def press_pick_obus8_11_button(self):
        self.obus_manager.press_obus_button('pick', 8, 11, 
            side='left',
            pre_z=Prepick_Pose_z, 
            axis_service=srv_move_A1_A6, 
            axis_args=(pick_A1, pick_left_A6)
        )          

    def press_pick_obus8_12_button(self):
        self.obus_manager.press_obus_button('pick', 8, 12, 
            side='right',
            pre_z=Prepick_Pose_z, 
            axis_service=srv_move_A1_A6, 
            axis_args=(pick_A1, pick_right_A6)
        )        

    def press_pick_obus8_13_button(self):
        self.obus_manager.press_obus_button('pick', 8, 13, 
            side='right',
            pre_z=Prepick_Pose_z, 
            axis_service=srv_move_A1_A6, 
            axis_args=(pick_A1, pick_right_A6)
        )             

    def press_pick_obus8_14_button(self):
        self.obus_manager.press_obus_button('pick', 8, 14, 
            side='right',
            pre_z=Prepick_Pose_z, 
            axis_service=srv_move_A1_A6, 
            axis_args=(pick_A1, pick_right_A6)
        )        

    #Pick buttons obus 16
    def press_pick_obus16_1_button(self):
        self.obus_manager.press_obus_button(
            'pick', 16, 1,
            side='left',
            pre_z=Prepick_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(pick_A1, pick_left_A6)
        )

    def press_pick_obus16_2_button(self):
        self.obus_manager.press_obus_button(
            'pick', 16, 2,
            side='left',
            pre_z=Prepick_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(pick_A1, pick_left_A6)
        )

    def press_pick_obus16_3_button(self):
        self.obus_manager.press_obus_button(
            'pick', 16, 3,
            side='left',
            pre_z=Prepick_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(pick_A1, pick_left_A6)
        )

    def press_pick_obus16_4_button(self):
        self.obus_manager.press_obus_button(
            'pick', 16, 4,
            side='left',
            pre_z=Prepick_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(pick_A1, pick_left_A6)
        )

    def press_pick_obus16_5_button(self):
        self.obus_manager.press_obus_button(
            'pick', 16, 5,
            side='left',
            pre_z=Prepick_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(pick_A1, pick_left_A6)
        )

    def press_pick_obus16_6_button(self):
        self.obus_manager.press_obus_button(
            'pick', 16, 6,
            side='right',
            pre_z=Prepick_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(pick_A1, pick_right_A6)
        )

    def press_pick_obus16_7_button(self):
        self.obus_manager.press_obus_button(
            'pick', 16, 7,
            side='right',
            pre_z=Prepick_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(pick_A1, pick_right_A6)
        )

    def press_pick_obus16_8_button(self):
        self.obus_manager.press_obus_button(
            'pick', 16, 8,
            side='right',
            pre_z=Prepick_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(pick_A1, pick_right_A6)
        )

    def press_pick_obus16_9_button(self):
        self.obus_manager.press_obus_button(
            'pick', 16, 9,
            side='right',
            pre_z=Prepick_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(pick_A1, pick_right_A6)
        )

    def press_pick_obus16_10_button(self):
        self.obus_manager.press_obus_button(
            'pick', 16, 10,
            side='right',
            pre_z=Prepick_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(pick_A1, pick_right_A6)
        )

    def press_pick_obus16_11_button(self):
        self.obus_manager.press_obus_button(
            'pick', 16, 11,
            side='left',
            pre_z=Prepick_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(pick_A1, pick_left_A6)
        )

    def press_pick_obus16_12_button(self):
        self.obus_manager.press_obus_button(
            'pick', 16, 12,
            side='left',
            pre_z=Prepick_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(pick_A1, pick_left_A6)
        )

    def press_pick_obus16_13_button(self):
        self.obus_manager.press_obus_button(
            'pick', 16, 13,
            side='left',
            pre_z=Prepick_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(pick_A1, pick_left_A6)
        )

    def press_pick_obus16_14_button(self):
        self.obus_manager.press_obus_button(
            'pick', 16, 14,
            side='left',
            pre_z=Prepick_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(pick_A1, pick_left_A6)
        )

    def press_pick_obus16_15_button(self):
        self.obus_manager.press_obus_button(
            'pick', 16, 15,
            side='left',
            pre_z=Prepick_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(pick_A1, pick_left_A6)
        )

    def press_pick_obus16_16_button(self):
        self.obus_manager.press_obus_button(
            'pick', 16, 16,
            side='right',
            pre_z=Prepick_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(pick_A1, pick_right_A6)
        )

    def press_pick_obus16_17_button(self):
        self.obus_manager.press_obus_button(
            'pick', 16, 17,
            side='right',
            pre_z=Prepick_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(pick_A1, pick_right_A6)
        )

    def press_pick_obus16_18_button(self):
        self.obus_manager.press_obus_button(
            'pick', 16, 18,
            side='right',
            pre_z=Prepick_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(pick_A1, pick_right_A6)
        )

    def press_pick_obus16_19_button(self):
        self.obus_manager.press_obus_button(
            'pick', 16, 19,
            side='right',
            pre_z=Prepick_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(pick_A1, pick_right_A6)
        )

    def press_pick_obus16_20_button(self):
        self.obus_manager.press_obus_button(
            'pick', 16, 20,
            side='right',
            pre_z=Prepick_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(pick_A1, pick_right_A6)
        )


    #pressing obuses para place
    #obus1
    def press_place_obus2_1_button(self):
        self.obus_manager.press_obus_button(
            'place', 2, 1,
            side='left',
            pre_z=Preplace_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(place_A1, place_left_A6)
        )

    def press_place_obus2_2_button(self):
        self.obus_manager.press_obus_button(
            'place', 2, 2,
            side='right',
            pre_z=Preplace_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(place_A1, place_right_A6)
        )

    def press_place_obus4_1_button(self):
        self.obus_manager.press_obus_button(
            'place', 4, 1,
            side='left',
            pre_z=Preplace_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(place_A1, place_left_A6)
        )

    def press_place_obus4_2_button(self):
        self.obus_manager.press_obus_button(
            'place', 4, 2,
            side='left',
            pre_z=Preplace_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(place_A1, place_left_A6)
        )

    def press_place_obus4_3_button(self):
        self.obus_manager.press_obus_button(
            'place', 4, 3,
            side='right',
            pre_z=Preplace_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(place_A1, place_right_A6)
        )

    def press_place_obus4_4_button(self):
        self.obus_manager.press_obus_button(
            'place', 4, 4,
            side='right',
            pre_z=Preplace_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(place_A1, place_right_A6)
        )

    def press_place_obus8_1_button(self):
        self.obus_manager.press_obus_button(
            'place', 8, 1,
            side='left',
            pre_z=Preplace_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(place_A1, place_left_A6)
        )

    def press_place_obus8_2_button(self):
        self.obus_manager.press_obus_button(
            'place', 8, 2,
            side='left',
            pre_z=Preplace_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(place_A1, place_left_A6)
        )

    def press_place_obus8_3_button(self):
        self.obus_manager.press_obus_button(
            'place', 8, 3,
            side='right',
            pre_z=Preplace_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(place_A1, place_right_A6)
        )

    def press_place_obus8_4_button(self):
        self.obus_manager.press_obus_button(
            'place', 8, 4,
            side='right',
            pre_z=Preplace_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(place_A1, place_right_A6)
        )

    def press_place_obus8_5_button(self):
        self.obus_manager.press_obus_button(
            'place', 8, 5,
            side='left',
            pre_z=Preplace_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(place_A1, place_left_A6)
        )

    def press_place_obus8_6_button(self):
        self.obus_manager.press_obus_button(
            'place', 8, 6,
            side='left',
            pre_z=Preplace_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(place_A1, place_left_A6)
        )

    def press_place_obus8_7_button(self):
        self.obus_manager.press_obus_button(
            'place', 8, 7,
            side='right',
            pre_z=Preplace_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(place_A1, place_right_A6)
        )

    def press_place_obus8_8_button(self):
        self.obus_manager.press_obus_button(
            'place', 8, 8,
            side='right',
            pre_z=Preplace_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(place_A1, place_right_A6)
        )

    # ---- OBUS 16 (1 al 16) ----

    def press_place_obus16_1_button(self):
        self.obus_manager.press_obus_button(
            'place', 16, 1,
            side='left',
            pre_z=Preplace_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(place_A1, place_left_A6)
        )

    def press_place_obus16_2_button(self):
        self.obus_manager.press_obus_button(
            'place', 16, 2,
            side='left',
            pre_z=Preplace_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(place_A1, place_left_A6)
        )

    def press_place_obus16_3_button(self):
        self.obus_manager.press_obus_button(
            'place', 16, 3,
            side='left',
            pre_z=Preplace_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(place_A1, place_left_A6)
        )

    def press_place_obus16_4_button(self):
        self.obus_manager.press_obus_button(
            'place', 16, 4,
            side='left',
            pre_z=Preplace_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(place_A1, place_left_A6)
        )

    def press_place_obus16_5_button(self):
        self.obus_manager.press_obus_button(
            'place', 16, 5,
            side='right',
            pre_z=Preplace_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(place_A1, place_right_A6)
        )

    def press_place_obus16_6_button(self):
        self.obus_manager.press_obus_button(
            'place', 16, 6,
            side='right',
            pre_z=Preplace_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(place_A1, place_right_A6)
        )

    def press_place_obus16_7_button(self):
        self.obus_manager.press_obus_button(
            'place', 16, 7,
            side='right',
            pre_z=Preplace_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(place_A1, place_right_A6)
        )

    def press_place_obus16_8_button(self):
        self.obus_manager.press_obus_button(
            'place', 16, 8,
            side='right',
            pre_z=Preplace_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(place_A1, place_right_A6)
        )

    def press_place_obus16_9_button(self):
        self.obus_manager.press_obus_button(
            'place', 16, 9,
            side='left',
            pre_z=Preplace_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(place_A1, place_left_A6)
        )

    def press_place_obus16_10_button(self):
        self.obus_manager.press_obus_button(
            'place', 16, 10,
            side='left',
            pre_z=Preplace_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(place_A1, place_left_A6)
        )

    def press_place_obus16_11_button(self):
        self.obus_manager.press_obus_button(
            'place', 16, 11,
            side='left',
            pre_z=Preplace_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(place_A1, place_left_A6)
        )

    def press_place_obus16_12_button(self):
        self.obus_manager.press_obus_button(
            'place', 16, 12,
            side='left',
            pre_z=Preplace_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(place_A1, place_left_A6)
        )

    def press_place_obus16_13_button(self):
        self.obus_manager.press_obus_button(
            'place', 16, 13,
            side='right',
            pre_z=Preplace_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(place_A1, place_right_A6)
        )

    def press_place_obus16_14_button(self):
        self.obus_manager.press_obus_button(
            'place', 16, 14,
            side='right',
            pre_z=Preplace_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(place_A1, place_right_A6)
        )

    def press_place_obus16_15_button(self):
        self.obus_manager.press_obus_button(
            'place', 16, 15,
            side='right',
            pre_z=Preplace_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(place_A1, place_right_A6)
        )

    def press_place_obus16_16_button(self):
        self.obus_manager.press_obus_button(
            'place', 16, 16,
            side='right',
            pre_z=Preplace_Pose_z,
            axis_service=srv_move_A1_A6,
            axis_args=(place_A1, place_right_A6)
        )

                    
    def press_tool_homming(self):
        ret = QMessageBox.warning(self, "WARNING!", 'Are you sure? \nBe sure there is no obus picked', QMessageBox.Ok, QMessageBox.Cancel)
        #ret = QMessageBox.critical(self, "WARNING!", 'The tool is activated and there is some weight \ndetected by the gauges!', QMessageBox.Ok)
        if ret == QMessageBox.Ok:
            limit_cont_current_service=rospy.ServiceProxy(srv_limit_cont_current, set_float_value)
            limit_peak_current_service=rospy.ServiceProxy(srv_limit_peak_current, set_float_value)
            limit_cont_current_service(current_limit_0)
            limit_peak_current_service(current_limit_0)
            #Call tool homing method
            global weight_empty, weight_read, TOOL_HOMED
            try:
                gripper_move_service = rospy.ServiceProxy(srv_finger_set_pose,set_odometry)
                homing_service = rospy.ServiceProxy(srv_tool_homing, home)           
                ret = homing_service()
                TOOL_HOMED=True 
                #weight_empty=weight_read
                #gripper_move_service(0.02,0,0,-0.15)
                if ret == True:
                    TOOL_HOMED=True                 
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            #set current again
            if finger_type == 0:
                limit_cont_current_service(current_limit_0)
                limit_peak_current_service(current_limit_0)
            elif finger_type == 1:
                limit_cont_current_service(current_limit_1)
                limit_peak_current_service(current_limit_1)
            elif finger_type == 2:
                limit_cont_current_service(current_limit_2)
                limit_peak_current_service(current_limit_2)
            elif finger_type == 3:
                limit_cont_current_service(current_limit_cont)
                limit_peak_current_service(current_limit_3)
            elif finger_type == 4:
                limit_cont_current_service(current_limit_cont)
                limit_peak_current_service(current_limit_4)
                
    def press_finger_adjust_button(self):
        if(not TOOL_HOMED):
                QMessageBox.warning(self, "WARNING!", 'Are you sure? \nHoming of the tool should be done first', QMessageBox.Ok, QMessageBox.Cancel)
        ret = QMessageBox.warning(self, "WARNING!", 'Are you sure? \nBe sure there is no obus picked', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            
            gripper_trasl_service = rospy.ServiceProxy(srv_finger_set_pose,set_odometry)
            if finger_type == 0:
                print 'No gripper selected'
            elif finger_type == 1:
                print 'Set gripper to 100mm'
                tras_from_homing=0.2-0.1;               
                ret=gripper_trasl_service(tras_from_homing,0,0,0)
            elif finger_type == 2:
                print 'Set gripper to 140mm'
                tras_from_homing=0.2-0.14;
                ret=gripper_trasl_service(tras_from_homing,0,0,0)
            elif finger_type == 3:
                print 'Set gripper to 160mm'
                tras_from_homing=0.2-0.16;
                ret=gripper_trasl_service(tras_from_homing,0,0,0)
            elif finger_type == 4:
                print 'Set gripper to 270mm'
                tras_from_homing=0.03;
                ret=gripper_trasl_service(tras_from_homing,0,0,0)
            
    def press_led_on_button(self):
        try:
            led_service = rospy.ServiceProxy(srv_digital_io, set_digital_output)
            ret = led_service(6,False)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


    def press_led_off_button(self):
        try:
            led_service = rospy.ServiceProxy(srv_digital_io, set_digital_output)
            ret = led_service(6,True)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
    def deadMan_state_changed(self):
        ret_q = QMessageBox.warning(self, "WARNING!", 'Changes to the current configuration will be applied', QMessageBox.Ok, QMessageBox.Cancel)
        if(ret_q==QMessageBox.Ok):
                    if(self.deadMan_check.isChecked()):
                                    try:
                                        deadman_service=rospy.ServiceProxy(srv_deadman, SetBool)
                                        ret = deadman_service(True)
                                    except rospy.ServiceException, e:
                                        print "Service call failed: %s"%e
                    elif(self.deadMan_check.isChecked()==False):
                                    try:
                                        deadman_service=rospy.ServiceProxy(srv_deadman, SetBool)
                                        ret = deadman_service(False)
                                    except rospy.ServiceException, e:
                                        print "Service call failed: %s"%e
        else : 
                self.deadMan_check.nextCheckState()
                
                
    def toolAngle_state_changed(self):
        ret_q = QMessageBox.warning(self, "WARNING!", 'Changes to the current configuration will be applied', QMessageBox.Ok, QMessageBox.Cancel)
        if(ret_q==QMessageBox.Ok):
                        if(self.toolAngle_check.isChecked()):
                                    try:
                                        angle_mode_service=rospy.ServiceProxy(srv_angle_mode, SetBool)
                                        ret = angle_mode_service(True)
                                    except rospy.ServiceException, e:
                                        print "Service call failed: %s"%e
                        elif(self.toolAngle_check.isChecked()==False):
                                    try:
                                        angle_mode_service=rospy.ServiceProxy(srv_angle_mode, SetBool)
                                        ret = angle_mode_service(False)
                                    except rospy.ServiceException, e:
                                        print "Service call failed: %s"%e
        else:
                self.toolAngle_check.nextCheckState()

    def toolOrientation_state_changed(self):
        ret_q = QMessageBox.warning(self, "WARNING!", 'Changes to the current configuration will be applied', QMessageBox.Ok, QMessageBox.Cancel)
        if(ret_q==QMessageBox.Ok):
                if(self.toolOrientation_check.isChecked()):
                            try:
                                toolOrientation_service=rospy.ServiceProxy(srv_rel_tool, SetBool)
                                ret = toolOrientation_service(True)
                            except rospy.ServiceException, e:
                                print "Service call failed: %s"%e
                elif(self.toolOrientation_check.isChecked()==False):
                            try:
                                toolOrientation_service=rospy.ServiceProxy(srv_rel_tool, SetBool)
                                ret = toolOrientation_service(False)
                            except rospy.ServiceException, e:
                                print "Service call failed: %s"%e
        else:
                self.toolOrientation_check.nextCheckState()
			
    def press_light_on_button(self):
        try:
            led_service = rospy.ServiceProxy(srv_digital_io, set_digital_output)
            ret = led_service(4,False)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    
    def press_light_off_button(self):
        try:
            led_service = rospy.ServiceProxy(srv_digital_io, set_digital_output)
            ret = led_service(4,True)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            

    def press_homming_button(self):     
        global KUKA_AUT, pos_z_kuka, pos_a_kuka, CURRENT_STATE
        ret = QMessageBox.warning(self, "WARNING!", 
                                'Are you sure? \nRobot is going to move autonomously', 
                                QMessageBox.Ok, QMessageBox.Cancel)
        if ret != QMessageBox.Ok:
            print("[press_homming_button] Acción cancelada por el usuario.")
            return

        #self.origin_pick_quad = 0 #SEGURO? por qué hacer homing es quitar la restricción de los botones de places?
        print("[press_homming_button] Iniciando homming...")

        try:
            print("[press_homming_button] Moviendo en Z de forma relativa (pre-homing).")
            homming_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
            ret_rel = homming_rel_service(0, 0, pose_z_safe - pos_z_kuka, 0, 0, 0)
            print("[press_homming_button] Movimiento relativo Z ejecutado, esperando...")
            self.sleep_loop(2)
            while KUKA_AUT:
                print("[press_homming_button] Esperando a que KUKA_AUT sea False...")
                self.sleep_loop(0.3)

            print("[press_homming_button] Llamando a home_A1_A6_service...")
            home_A1_A6_service = rospy.ServiceProxy(srv_move_A1_A6, set_A1_A6)
            ret_a1a6 = home_A1_A6_service(0.0, 177)
            print("[press_homming_button] Movimiento home_A1_A6 ejecutado, esperando...")
            self.sleep_loop(2)
            while KUKA_AUT:
                print("[press_homming_button] Esperando a que KUKA_AUT sea False...")
                self.sleep_loop(0.3)

            print("[press_homming_button] Moviendo a posición absoluta en la mesa.")
            placed_abs_service = rospy.ServiceProxy(srv_name_move_abs_slow, set_CartesianEuler_pose)
            ret_abs = placed_abs_service(table_pose_x, table_pose_y, table_pose_z, table_pose_a, table_pose_b, table_pose_c)
            if ret_abs == True:
                CURRENT_STATE = STATE_MOVING_TO_PLACE
                print("[press_homming_button] Homming completado y estado actualizado.")
            else:
                print("[press_homming_button] Advertencia: La llamada a placed_abs_service no devolvió True.")
        except rospy.ServiceException, e:
            print("[press_homming_button] FALLO EN LA LLAMADA AL SERVICIO: %s" % e)
        except Exception, e:
            print("[press_homming_button] ERROR INESPERADO: %s" % e)

            
    def press_picktest_button(self):
        global KUKA_AUT
        ret = QMessageBox.warning(self, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            try:
                placed_rel_service = rospy.ServiceProxy(srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=placed_rel_service(0, 0, 20, 0, 0, 0)
                #KUKA_AUT=True
                #while KUKA_AUT: time.sleep(0.1)
                if ret_rel == True:
                        CURRENT_STATE=STATE_DOING_PICK_TEST
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

    
    def press_tare_button(self):
        tare_service = rospy.ServiceProxy(srv_tare_gauges, SetBool)
        ret = tare_service(True)

    def press_tare_reset_button(self):
        tare_service = rospy.ServiceProxy(srv_tare_gauges, SetBool)
        ret = tare_service(False)

    #################################################JOY SELECTION
    def joy_selected(self, index):
        if index == 0:
            print 'PS4 selected'
            command_string = "rosrun topic_tools mux_select mux_joy /kuka_pad/ps4_joy"
            os.system(command_string)
        elif index == 1:            
            print 'ITOWA selected'
            command_string = "rosrun topic_tools mux_select mux_joy /kuka_pad/itowa_joy"
            os.system(command_string)

    
    #################################################CALIBRE SELECTION
    def calibre_selected(self, index):
        global finger_type, weight_expected_min, weight_expected_max, current_limit_picked
        print 'Selected:',index
        finger_type = index
        if index == 0:
            print 'No gripper selected'
            self.desactivate_buttons()
            str_weight_expected = "[N/A, N/A]"
            self.weight_limited.setText("N/A");
            self.weight_label_expected_var.setText(str_weight_expected)
            pixmap = QtGui.QPixmap(IMG_PATH+"/fondo_huevera_0.png")
            self.background_plate.setPixmap(pixmap)
            self.background_plate_pick.setPixmap(pixmap)
            self.PlaceObusButton2_1.hide()
            self.PlaceObusButton2_2.hide()            
            self.PlaceObusButton4_1.hide()
            self.PlaceObusButton4_2.hide()
            self.PlaceObusButton4_3.hide()
            self.PlaceObusButton4_4.hide()
            self.PlaceObusButton8_1.hide()
            self.PlaceObusButton8_2.hide()
            self.PlaceObusButton8_3.hide()
            self.PlaceObusButton8_4.hide()
            self.PlaceObusButton8_5.hide()
            self.PlaceObusButton8_6.hide()
            self.PlaceObusButton8_7.hide()
            self.PlaceObusButton8_8.hide()            
            self.PlaceObusButton16_1.hide()
            self.PlaceObusButton16_2.hide()
            self.PlaceObusButton16_3.hide()
            self.PlaceObusButton16_4.hide()
            self.PlaceObusButton16_5.hide()
            self.PlaceObusButton16_6.hide()
            self.PlaceObusButton16_7.hide()
            self.PlaceObusButton16_8.hide()  
            self.PlaceObusButton16_9.hide()
            self.PlaceObusButton16_10.hide()
            self.PlaceObusButton16_11.hide()
            self.PlaceObusButton16_12.hide()
            self.PlaceObusButton16_13.hide()
            self.PlaceObusButton16_14.hide()
            self.PlaceObusButton16_15.hide()
            self.PlaceObusButton16_16.hide()   
            for i in range(1, 21):
                name_method='PickObusButton16'+'_'+str(i)
                test_method=getattr(self, name_method)
                test_method.hide()
            for i in range(1, 15):
                name_method='PickObusButton8'+'_'+str(i)
                test_method=getattr(self, name_method)
                test_method.hide()
            for i in range(1, 6):
                name_method='PickObusButton4'+'_'+str(i)
                test_method=getattr(self, name_method)
                test_method.hide()
            for i in range(1, 5):
                name_method='PickObusButton2'+'_'+str(i)
                test_method=getattr(self, name_method)
                test_method.hide()
        else:
            #TODO: check if the gripper is empty. If there is some load not allow to move autonomously
            self.activate_buttons()            
        if index == 1:            
            try:
                rospy.delete_param('robot_description')
            except KeyError:
                print "value not set"
            self.load_robot_description(index)
            weight_expected_min = 4
            weight_expected_max = 9
            str_weight_expected = "["+str(weight_expected_min)+ ", "+ str(weight_expected_max)+ "]"
            self.weight_label_expected_var.setText(str_weight_expected)
            self.weight_limited.setText("3");
            limit_cont_current_service=rospy.ServiceProxy(srv_limit_cont_current, set_float_value)
            limit_peak_current_service=rospy.ServiceProxy(srv_limit_peak_current, set_float_value)
            try:
                limit_cont_current_service(current_limit_1)
                limit_peak_current_service(current_limit_1)
            except (rospy.ServiceException, rospy.ROSException), e:
                rospy.logerr("Service call failed: %s" % (e,))
                
            current_limit_picked = current_limit_1

            pixmap = QtGui.QPixmap(IMG_PATH+"/rotated-fondo_huevera_16.png")
            self.background_plate.setPixmap(pixmap)
            pixmap_pick = QtGui.QPixmap(IMG_PATH+"/BoxPick_3.png")
            self.background_plate_pick.setPixmap(pixmap_pick)
            self.PlaceObusButton16_1.show()
            self.PlaceObusButton16_2.show()
            self.PlaceObusButton16_3.show()
            self.PlaceObusButton16_4.show()
            self.PlaceObusButton16_5.show()
            self.PlaceObusButton16_6.show()
            self.PlaceObusButton16_7.show()
            self.PlaceObusButton16_8.show()  
            self.PlaceObusButton16_9.show()
            self.PlaceObusButton16_10.show()
            self.PlaceObusButton16_11.show()
            self.PlaceObusButton16_12.show()
            self.PlaceObusButton16_13.show()
            self.PlaceObusButton16_14.show()
            self.PlaceObusButton16_15.show()
            self.PlaceObusButton16_16.show()   
            self.PlaceObusButton2_1.hide()
            self.PlaceObusButton2_2.hide()
            self.PlaceObusButton4_1.hide()
            self.PlaceObusButton4_2.hide()
            self.PlaceObusButton4_3.hide()
            self.PlaceObusButton4_4.hide()            
            self.PlaceObusButton8_1.hide()
            self.PlaceObusButton8_2.hide()
            self.PlaceObusButton8_3.hide()
            self.PlaceObusButton8_4.hide()
            self.PlaceObusButton8_5.hide()
            self.PlaceObusButton8_6.hide()
            self.PlaceObusButton8_7.hide()
            self.PlaceObusButton8_8.hide()
            for i in range(1, 21):
                name_method='PickObusButton16'+'_'+str(i)
                test_method=getattr(self, name_method)
                test_method.show()
            for i in range(1, 15):
                name_method='PickObusButton8'+'_'+str(i)
                test_method=getattr(self, name_method)
                test_method.hide()
            for i in range(1, 6):
                name_method='PickObusButton4'+'_'+str(i)
                test_method=getattr(self, name_method)
                test_method.hide()
            for i in range(1, 5):
                name_method='PickObusButton2'+'_'+str(i)
                test_method=getattr(self, name_method)
                test_method.hide()

        elif index == 2:
            try:
                rospy.delete_param('robot_description')
            except KeyError:
                print "value not set"
            self.load_robot_description(index+1)
            weight_expected_min = 13
            weight_expected_max = 18
            str_weight_expected = "["+str(weight_expected_min)+ ", "+ str(weight_expected_max)+ "]"
            self.weight_label_expected_var.setText(str_weight_expected)

            self.weight_limited.setText("4");
            limit_cont_current_service=rospy.ServiceProxy(srv_limit_cont_current, set_float_value)
            limit_peak_current_service=rospy.ServiceProxy(srv_limit_peak_current, set_float_value)
            try:
                limit_cont_current_service(current_limit_2)
                limit_peak_current_service(current_limit_2)
            except (rospy.ServiceException, rospy.ROSException), e:
                rospy.logerr("Service call failed: %s" % (e,))

            current_limit_picked = current_limit_2


            pixmap = QtGui.QPixmap(IMG_PATH+"/rotated-fondo_huevera_8.png")
            self.background_plate.setPixmap(pixmap)
            pixmap_pick = QtGui.QPixmap(IMG_PATH+"/BoxPick_3.png")
            self.background_plate_pick.setPixmap(pixmap_pick)
            self.PlaceObusButton2_1.hide()
            self.PlaceObusButton2_2.hide()
            self.PlaceObusButton4_1.hide()
            self.PlaceObusButton4_2.hide()
            self.PlaceObusButton4_3.hide()
            self.PlaceObusButton4_4.hide()                        
            self.PlaceObusButton8_1.show()
            self.PlaceObusButton8_2.show()
            self.PlaceObusButton8_3.show()
            self.PlaceObusButton8_4.show()
            self.PlaceObusButton8_5.show()
            self.PlaceObusButton8_6.show()
            self.PlaceObusButton8_7.show()
            self.PlaceObusButton8_8.show()
            self.PlaceObusButton16_1.hide()
            self.PlaceObusButton16_2.hide()
            self.PlaceObusButton16_3.hide()
            self.PlaceObusButton16_4.hide()
            self.PlaceObusButton16_5.hide()
            self.PlaceObusButton16_6.hide()
            self.PlaceObusButton16_7.hide()
            self.PlaceObusButton16_8.hide()  
            self.PlaceObusButton16_9.hide()
            self.PlaceObusButton16_10.hide()
            self.PlaceObusButton16_11.hide()
            self.PlaceObusButton16_12.hide()
            self.PlaceObusButton16_13.hide()
            self.PlaceObusButton16_14.hide()
            self.PlaceObusButton16_15.hide()
            self.PlaceObusButton16_16.hide() 
            for i in range(1, 21):
                name_method='PickObusButton16'+'_'+str(i)
                test_method=getattr(self, name_method)
                test_method.hide()
            for i in range(1, 15):
                name_method='PickObusButton8'+'_'+str(i)
                test_method=getattr(self, name_method)
                test_method.show()
            for i in range(1, 6):
                name_method='PickObusButton4'+'_'+str(i)
                test_method=getattr(self, name_method)
                test_method.hide()
            for i in range(1, 5):
                name_method='PickObusButton2'+'_'+str(i)
                test_method=getattr(self, name_method)
                test_method.hide()  

        elif index == 3:
            try:
                rospy.delete_param('robot_description')
            except KeyError:
                print "value not set"
            self.load_robot_description(index+1)
            weight_expected_min = 18
            weight_expected_max = 46
            str_weight_expected = "["+str(weight_expected_min)+ ", "+ str(weight_expected_max)+ "]"
            self.weight_label_expected_var.setText(str_weight_expected)

            self.weight_limited.setText("7");
            limit_cont_current_service=rospy.ServiceProxy(srv_limit_cont_current, set_float_value)
            limit_peak_current_service=rospy.ServiceProxy(srv_limit_peak_current, set_float_value)
            try:
                limit_cont_current_service(current_limit_cont)
                limit_peak_current_service(current_limit_3)
            except (rospy.ServiceException, rospy.ROSException), e:
                rospy.logerr("Service call failed: %s" % (e,))

            current_limit_picked = current_limit_3

            pixmap = QtGui.QPixmap(IMG_PATH+"/rotated-fondo_huevera_4.png")
            self.background_plate.setPixmap(pixmap)
            pixmap_pick = QtGui.QPixmap(IMG_PATH+"/BoxPick_3.png")
            self.background_plate_pick.setPixmap(pixmap_pick)
            self.PlaceObusButton4_1.show()
            self.PlaceObusButton4_2.show()
            self.PlaceObusButton4_3.show()
            self.PlaceObusButton4_4.show()
            self.PlaceObusButton2_1.hide()
            self.PlaceObusButton2_2.hide()
            self.PlaceObusButton8_1.hide()
            self.PlaceObusButton8_2.hide()
            self.PlaceObusButton8_3.hide()
            self.PlaceObusButton8_4.hide()
            self.PlaceObusButton8_5.hide()
            self.PlaceObusButton8_6.hide()
            self.PlaceObusButton8_7.hide()
            self.PlaceObusButton8_8.hide()              
            self.PlaceObusButton16_1.hide()
            self.PlaceObusButton16_2.hide()
            self.PlaceObusButton16_3.hide()
            self.PlaceObusButton16_4.hide()
            self.PlaceObusButton16_5.hide()
            self.PlaceObusButton16_6.hide()
            self.PlaceObusButton16_7.hide()
            self.PlaceObusButton16_8.hide()  
            self.PlaceObusButton16_9.hide()
            self.PlaceObusButton16_10.hide()
            self.PlaceObusButton16_11.hide()
            self.PlaceObusButton16_12.hide()
            self.PlaceObusButton16_13.hide()
            self.PlaceObusButton16_14.hide()
            self.PlaceObusButton16_15.hide()
            self.PlaceObusButton16_16.hide()
            for i in range(1, 21):
                name_method='PickObusButton16'+'_'+str(i)
                test_method=getattr(self, name_method)
                test_method.hide()
            for i in range(1, 15):
                name_method='PickObusButton8'+'_'+str(i)
                test_method=getattr(self, name_method)
                test_method.hide()
            for i in range(1, 6):
                name_method='PickObusButton4'+'_'+str(i)
                test_method=getattr(self, name_method)
                test_method.show()
            for i in range(1, 5):
                name_method='PickObusButton2'+'_'+str(i)
                test_method=getattr(self, name_method)
                test_method.hide() 

        elif index == 4:
            try:
                rospy.delete_param('robot_description')
            except KeyError:
                print "value not set"
            self.load_robot_description(index+1)
            weight_expected_min = 110
            weight_expected_max = 130
            str_weight_expected = "["+str(weight_expected_min)+ ", "+ str(weight_expected_max)+ "]"
            self.weight_label_expected_var.setText(str_weight_expected)
            self.weight_limited.setText("8");

            limit_cont_current_service=rospy.ServiceProxy(srv_limit_cont_current, set_float_value)
            limit_peak_current_service=rospy.ServiceProxy(srv_limit_peak_current, set_float_value)
            try:
                limit_cont_current_service(current_limit_cont)
                limit_peak_current_service(current_limit_4)
            except (rospy.ServiceException, rospy.ROSException), e:
                rospy.logerr("Service call failed: %s" % (e,))

            current_limit_picked = current_limit_4
            
            pixmap = QtGui.QPixmap(IMG_PATH+"/rotated-fondo_huevera_2.png")
            self.background_plate.setPixmap(pixmap)
            pixmap_pick = QtGui.QPixmap(IMG_PATH+"/BoxPick_3.png")
            self.background_plate_pick.setPixmap(pixmap_pick)
            self.PlaceObusButton2_1.show()
            self.PlaceObusButton2_2.show()
            self.PlaceObusButton4_1.hide()
            self.PlaceObusButton4_2.hide()
            self.PlaceObusButton4_3.hide()
            self.PlaceObusButton4_4.hide()
            self.PlaceObusButton8_1.hide()
            self.PlaceObusButton8_2.hide()
            self.PlaceObusButton8_3.hide()
            self.PlaceObusButton8_4.hide()
            self.PlaceObusButton8_5.hide()
            self.PlaceObusButton8_6.hide()
            self.PlaceObusButton8_7.hide()
            self.PlaceObusButton8_8.hide()              
            self.PlaceObusButton16_1.hide()
            self.PlaceObusButton16_2.hide()
            self.PlaceObusButton16_3.hide()
            self.PlaceObusButton16_4.hide()
            self.PlaceObusButton16_5.hide()
            self.PlaceObusButton16_6.hide()
            self.PlaceObusButton16_7.hide()
            self.PlaceObusButton16_8.hide()  
            self.PlaceObusButton16_9.hide()
            self.PlaceObusButton16_10.hide()
            self.PlaceObusButton16_11.hide()
            self.PlaceObusButton16_12.hide()
            self.PlaceObusButton16_13.hide()
            self.PlaceObusButton16_14.hide()
            self.PlaceObusButton16_15.hide()
            self.PlaceObusButton16_16.hide()
            for i in range(1, 21):
                name_method='PickObusButton16'+'_'+str(i)
                test_method=getattr(self, name_method)
                test_method.hide()
            for i in range(1, 15):
                name_method='PickObusButton8'+'_'+str(i)
                test_method=getattr(self, name_method)
                test_method.hide()
            for i in range(1, 6):
                name_method='PickObusButton4'+'_'+str(i)
                test_method=getattr(self, name_method)
                test_method.hide()
            for i in range(1, 5):
                name_method='PickObusButton2'+'_'+str(i)
                test_method=getattr(self, name_method)
                test_method.show()
        #weight progress bar
        self.weightProgressBar.setMaximum(weight_expected_max*1.3)


    def load_robot_description(self, gripper_model):
        command_string = "rosparam load ~/kuka_catkin_ws/src/kuka_experimental/kuka_robot_bringup/robot/bin/kr120toolv%d.urdf /robot_description" % gripper_model
        os.system(command_string)
        
    def press_reset_external_pc_button(self):
        ret = QMessageBox.warning(self, "WARNING!", 'Are you sure? \nExternal PC is going to reset.\n Wait 10 sec and restart the GUI.', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            #command_string = "ssh robotnik@192.168.1.10 sudo -S <<< \"R0b0tn1K\" reboot \n"
            command_string = "~/kuka_catkin_ws/src/rqt_kuka/scripts/reboot.sh"
            print command_string
            os.system(command_string)

    ###TEST APRIETE AUTOMATICO: si el nodo de las galgas falla se va  a liar
    def aut_press_tool(self):
        global angle_tool
        gripper_move_service = rospy.ServiceProxy(srv_finger_set_pose, set_odometry)
        press_counter = 0
        old_pos_x = 0
        self.desactivate_buttons()
        
        print("[aut_press_tool] Iniciando ciclo de cierre de herramienta")

        # Primer bucle: Cerrar la herramienta (ajustar ángulo)
        while press_counter < 5:
            print("press_counter:", press_counter)
            print("angle_tool:", angle_tool)
            try:
                gripper_move_service(x_tool, 0, 0, angle_tool - 0.01)
            except rospy.ServiceException as e:
                print("[aut_press_tool] Error llamando al servicio de pinza (cerrando):", str(e))
                self.activate_buttons()
                return
            except Exception as e:
                print("[aut_press_tool] Error inesperado llamando al servicio (cerrando):", str(e))
                self.activate_buttons()
                return
            self.sleep_loop(0.15)
            if tool_current > current_limit_cont:
                press_counter += 1
            else:
                press_counter = 0
            print("Δx_tool:", abs(x_tool - old_pos_x))
            print("current:", tool_current)
            print("counter:", press_counter)

        press_counter = 0
        # Segundo bucle: Abrir ligeramente la herramienta para afinar posición
        print("[aut_press_tool] Iniciando ciclo de apertura fina")
        while press_counter < 5:
            print("press_counter:", press_counter)
            try:
                gripper_move_service(x_tool + 0.02, 0, 0, angle_tool)
            except rospy.ServiceException as e:
                print("[aut_press_tool] Error llamando al servicio de pinza (abriendo):", str(e))
                self.activate_buttons()
                return
            except Exception as e:
                print("[aut_press_tool] Error inesperado llamando al servicio (abriendo):", str(e))
                self.activate_buttons()
                return
            self.sleep_loop(0.15)
            if tool_current > current_limit_cont or abs(x_tool - old_pos_x) < 0.002:
                press_counter += 1
            else:
                press_counter = 0
                old_pos_x = x_tool
            print("current:", tool_current)
            print("counter:", press_counter)
        
        self.sleep_loop(0.5)
        print("[aut_press_tool] Finalizado, reactivando botones")
        self.activate_buttons() 
        
    def callback_tool_state(self, data):
        global x_tool, angle_tool
        x_tool = data.position[2]
        angle_tool = data.position[3]
    
    def press_reset_robot_button(self):
        global rob_connected
        #command_string = "rosnode kill /kuka_pad/ps4_joystick; sleep 1; rosnode kill /kuka_pad/itowa_safe_joystick; sleep 1; rosnode kill /kuka_pad/robotnik_trajectory_pad_node; sleep 1; rosnode kill /kuka_robot/kuka_cartesian_hardware_interface; sleep 1; roslaunch kuka_robot_bringup kuka_robot_bringup_standalone.launch &"
        command_string = "killall screen; sleep 1; screen -S bringup -d -m roslaunch kuka_robot_bringup kuka_robot_bringup_standalone.launch"
        #command_string = "rosnode kill /kuka_robot/kuka_cartesian_hardware_interface; sleep 1; ROS_NAMESPACE=kuka_robot roslaunch kuka_rsi_cartesian_hw_interface test_hardware_interface.launch &"
        os.system(command_string)
        self.mode_label.setText("NOT CONNECTED")
        rob_connected = False
        
    
    def sleep_loop(self,delay):
        loop = QtCore.QEventLoop()
        timer = QtCore.QTimer()
        timer.setInterval(delay*1000)
        timer.setSingleShot(True)
        timer.timeout.connect(loop.quit)
        timer.start()
        loop.exec_()
