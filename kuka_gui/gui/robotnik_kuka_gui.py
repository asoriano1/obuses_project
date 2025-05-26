#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
robotnik_kuka_gui.py

Este módulo define la clase principal `KukaGUI`, una interfaz gráfica basada en PyQt
para el control de un robot KUKA en tareas de pick and place con obuses.
Incluye conexión con ROS (topics, servicios), configuración visual, y control de botones.
"""
import logging
from color_logger import ColorFormatter

logger = logging.getLogger('robotnik_kuka_gui')
logger.setLevel(logging.DEBUG)

if not logger.handlers:
    stream_handler = logging.StreamHandler()
    stream_handler.setLevel(logging.DEBUG)
    color_formatter = ColorFormatter('%(asctime)s %(levelname)s [%(name)s]: %(message)s')
    stream_handler.setFormatter(color_formatter)
    logger.addHandler(stream_handler)

    file_handler = logging.handlers.RotatingFileHandler(
        '/tmp/robotnik_kuka_gui.log', maxBytes=5*1024*1024, backupCount=2
    )
    file_handler.setFormatter(logging.Formatter('%(asctime)s %(levelname)s [%(name)s]: %(message)s'))
    logger.addHandler(file_handler)

import os
import inspect
import logging
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
import obuses_poses

from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QMessageBox
from std_msgs.msg import Bool, Float64, Float32
from std_srvs.srv import SetBool
from sensor_msgs.msg import JointState
from robotnik_msgs.srv import home, set_odometry, set_CartesianEuler_pose, set_digital_output, set_float_value
from robotnik_msgs.msg import Cartesian_Euler_pose, RobotnikMotorsStatus, MotorStatus, inputs_outputs
from geometry_msgs.msg import Pose, Point, Quaternion
from kuka_rsi_cartesian_hw_interface.srv import set_A1_A6

import global_var
import global_flags
from obus_manager import ObusManager
from widgets_management import WidgetsManagement
from calibres_config import *
#from ros_callbacks import KukaCallbacks


class KukaGUI(QWidget, WidgetsManagement):
    """
    Interfaz principal para la GUI de Robotnik KUKA.
    Se encarga de la inicialización de la GUI, conexión ROS, y gestión de eventos.
    """
    # Señales personalizadas (Qt Signals)        
    do_callback_motor_status = QtCore.pyqtSignal(RobotnikMotorsStatus)
    do_callback_horiz_force = QtCore.pyqtSignal(Float64)
    do_callback_current = QtCore.pyqtSignal(Float32)
    do_callback_tool_weight = QtCore.pyqtSignal(Float64)
    do_callback_robot_moving = QtCore.pyqtSignal(Bool)
    do_callback_door_state = QtCore.pyqtSignal(inputs_outputs)
    
    def __init__(self, parent=None):
        """
        Constructor de la clase KukaGUI. Inicializa la interfaz y se conecta con ROS.
        """
        super(KukaGUI, self).__init__(parent)
        self.setObjectName('KukaGUI')
        loadUi(global_var.UI_PATH, self)
        logger.info("Inicializando GUI principal Robotnik Obuses")
        
        # Lanzar comandos externos para preparar ROS
        self._run_external_commands()        

        # Conexión de elementos gráficos
        self._connect_widgets()
        self._configure_styles()
        self._init_gui_state()
        
        # Inicialización lógica interna y subscripciones ROS
        self.state_dict = self.init_state_dict()
        self.obus_manager = ObusManager(self)

        logger.info("Creando suscripciones ROS")
        self._init_ros_subscribers()
        self._init_ros_services()

        self.setWindowTitle("ROBOTNIK OBUSES GUI")
        self.resize(620, 1100)
        self._name = "RqtKuka"
        self.last_obus_selected_pick = -1
        self.last_obus_selected_place = -1        

    def _run_external_commands(self):
        """Ejecuta comandos necesarios para preparar el entorno ROS."""
        def _run_and_log(cmd, desc):
            logger.info("Ejecutando: %s", desc)
            os.system(cmd)

        _run_and_log("rosrun topic_tools mux /kuka_pad/joy /kuka_pad/ps4_joy /kuka_pad/itowa_joy mux:=mux_joy __name:=joy_mux_node &", "Multiplexor de Joysticks")
        _run_and_log("rosrun topic_tools mux_select mux_joy /kuka_pad/ps4_joy", "Selección por defecto PS4")
        _run_and_log("killall screen; sleep 1; screen -S bringup -d -m roslaunch kuka_robot_bringup kuka_robot_bringup_standalone.launch", "Lanzar bringup principal")
    
    def _connect_widgets(self):
        """Conecta los widgets de la UI con sus respectivas funciones."""
        self.calibre_comboBox.currentIndexChanged.connect(self.calibre_selected)
        self.joy_comboBox.currentIndexChanged.connect(self.joy_selected)
        self.mode_label.setText("NOT CONNECTED")

        # Botones principales
        self.Finger_Adjust_Button.pressed.connect(self.press_finger_adjust_button)
        self.Tare_Button.pressed.connect(self.press_tare_button)
        self.Tare_Reset_Button.pressed.connect(self.press_tare_reset_button)
        self.Reset_Ext_Button.pressed.connect(self.press_reset_external_pc_button)
        self.Reset_Robot_Button.pressed.connect(self.press_reset_robot_button)
        self.MoveToTable_Button.pressed.connect(self.press_homming_button)
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

        # Checkboxes
        self.deadMan_check.clicked.connect(self.deadMan_state_changed)
        self.toolAngle_check.clicked.connect(self.toolAngle_state_changed)
        self.toolOrientation_check.clicked.connect(self.toolOrientation_state_changed)

    def _configure_styles(self):
        """Configura los estilos visuales iniciales."""
        self.Led_On_Button.setStyleSheet("color: rgb(80, 170, 80)")
        self.Led_Off_Button.setStyleSheet("color: rgb(170, 80, 80)")
        self.Light_On_Button.setStyleSheet("color: rgb(80, 170, 80)")
        self.Light_Off_Button.setStyleSheet("color: rgb(170, 80, 80)")

    def _init_gui_state(self):
        """Inicializa algunos valores de estado visuales y lógicos."""
        pixmap = QtGui.QPixmap(global_var.IMG_PATH + "/fondo_huevera_0.png")
        self.background_plate.setPixmap(pixmap)
        self.background_plate_pick.setPixmap(pixmap)
        self.Finger_Adjust_Button.setEnabled(False)
        self.MoveToTable_Button.setEnabled(False)
        self.weightProgressBar_2.setMinimum(0)
        self.weightProgressBar_2.setMaximum(15)
        self.origin_pick_quad = 0

    def _init_ros_subscribers(self):
        """Inicializa las suscripciones a topics de ROS y conecta señales Qt seguras para GUI."""
        # Movimiento del robot
        self.sub_robot_moving = rospy.Subscriber(global_var.topic_kuka_moving, Bool, self.callback_robot_moving)
        self.do_callback_robot_moving.connect(self.callback_robot_moving_signal)
        #self.sub_robot_moving = rospy.Subscriber(global_var.topic_kuka_moving, Bool, self.callback_robot_moving)
        # Pose del robot
        self.sub_robot_pose = rospy.Subscriber(global_var.topic_cart_pose_kuka, Cartesian_Euler_pose, self.callback_robot_pose)
        # Peso de la herramienta
        self.sub_tool_weight = rospy.Subscriber(global_var.topic_tool_weight, Float64, self.callback_tool_weight)
        self.do_callback_tool_weight.connect(self.callback_tool_weight_signal)
        # Corriente
        self.sub_tool_current = rospy.Subscriber(global_var.topic_current, Float32, self.callback_current)
        self.do_callback_current.connect(self.callback_current_signal)
        # Fuerza horizontal
        self.sub_tool_force = rospy.Subscriber(global_var.topic_horiz_force, Float64, self.callback_horiz_force)
        self.do_callback_horiz_force.connect(self.callback_horiz_force_signal)
        # Estado del motor
        self.sub_tool_status = rospy.Subscriber(global_var.topic_motor_status, RobotnikMotorsStatus, self.callback_motor_status)
        self.do_callback_motor_status.connect(self.callback_motor_status_signal)
        # Estado de la herramienta (joints)
        self.sub_tool_state = rospy.Subscriber(global_var.topic_tool_state, JointState, self.callback_tool_state)
        # Estado de la puerta
        self.sub_door_status = rospy.Subscriber(global_var.topic_door_state, inputs_outputs, self.callback_door_state)
        self.do_callback_door_state.connect(self.callback_door_state_signal)

    def _init_ros_services(self):
        """Inicializa los servicios ROS requeridos para control de herramienta."""
        try:
            deadman_service = rospy.ServiceProxy(global_var.srv_deadman, SetBool)
            ret = deadman_service(True)
        except rospy.ServiceException as e:
            logger.error("Service call failed: %s", e)
            QMessageBox.critical(self, "Error", "Service call failed: %s" % e)

        try:
            angle_mode_service = rospy.ServiceProxy(global_var.srv_angle_mode, SetBool)
            ret = angle_mode_service(True)
        except rospy.ServiceException as e:
            logger.error("Service call failed: %s", e)
            QMessageBox.critical(self, "Error", "Service call failed: %s" % e)

        try:
            toolOrientation_service = rospy.ServiceProxy(global_var.srv_rel_tool, SetBool)
            ret = toolOrientation_service(False)
        except rospy.ServiceException as e:
            logger.error("Service call failed: %s", e)
            QMessageBox.critical(self, "Error", "Service call failed: %s" % e)
    
    #inicialización del estado de los obuses para state_dict['tipo', grupo, idx]
    def init_state_dict(self):
        """Inicializa el diccionario que contiene el estado (activo/inactivo) de cada obús."""
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

    #filtro para detectar el raton y ponerlo verde si esta el cursor encima o dejarlo blanco si no
    def eventFilter(self, object, event):               
        """
        Filtro de eventos de Qt: detecta el cursor sobre los botones y cambia
        su icono visual para indicar que puede ser seleccionado.
        """
        if not global_flags.KUKA_AUT:
            # huevera 16
            if global_var.finger_type == 1:        
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
                        object.setIcon(QtGui.QIcon(global_var.imgObus16izq[1]))
                    elif event.type() == QtCore.QEvent.HoverLeave:
                        object.setIcon(QtGui.QIcon(global_var.imgObus16izq[0]))

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
                        object.setIcon(QtGui.QIcon(global_var.imgObus16der[1]))
                    elif event.type() == QtCore.QEvent.HoverLeave:
                        object.setIcon(QtGui.QIcon(global_var.imgObus16der[0]))

            # huevera 8
            if global_var.finger_type == 2:                
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
                        object.setIcon(QtGui.QIcon(global_var.imgObus8izq[1]))
                    elif event.type() == QtCore.QEvent.HoverLeave:
                        object.setIcon(QtGui.QIcon(global_var.imgObus8izq[0]))
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
                        object.setIcon(QtGui.QIcon(global_var.imgObus8der[1]))
                    elif event.type() == QtCore.QEvent.HoverLeave:
                        object.setIcon(QtGui.QIcon(global_var.imgObus8der[0]))

            # huevera 4
            if global_var.finger_type == 3:
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
                        object.setIcon(QtGui.QIcon(global_var.imgObus4[1]))
                    elif event.type() == QtCore.QEvent.HoverLeave:
                        object.setIcon(QtGui.QIcon(global_var.imgObus4[0]))

            # huevera 2
            if global_var.finger_type == 4:
                if (
                    (object == self.PlaceObusButton2_1 and not self.state_dict.get(('place', 2, 1), False)) or
                    (object == self.PlaceObusButton2_2 and not self.state_dict.get(('place', 2, 2), False)) or
                    (object == self.PickObusButton2_1 and not self.state_dict.get(('pick', 2, 1), False)) or
                    (object == self.PickObusButton2_2 and not self.state_dict.get(('pick', 2, 2), False)) or
                    (object == self.PickObusButton2_3 and not self.state_dict.get(('pick', 2, 3), False)) or
                    (object == self.PickObusButton2_4 and not self.state_dict.get(('pick', 2, 4), False))
                ):
                    if event.type() == QtCore.QEvent.HoverEnter:
                        object.setIcon(QtGui.QIcon(global_var.imgObus2[1]))
                    elif event.type() == QtCore.QEvent.HoverLeave:
                        object.setIcon(QtGui.QIcon(global_var.imgObus2[0]))

        return False

# Función: Desactivate buttons.
    def desactivate_buttons(self):
        """Desactiva todos los botones de la interfaz gráfica relacionados con pick/place."""
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
        
# Función: Activate buttons.
    def activate_buttons(self):
        """
        Activa todos los botones según el cuadrante de origen del pick y el tipo de dedo.
        También reactiva event filters y botones de control principales.
        """      
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
            self.obus_manager.activate_buttons('Place', 16, 1, 16)
            self.obus_manager.activate_buttons('Place', 2, 1, 2)
            self.obus_manager.activate_buttons('Place', 4, 1, 4)
            self.obus_manager.activate_buttons('Place', 8, 1, 8)
        #seleccionado alguno de la parte superior de la caja y dedos 3 y 4               
        elif((global_var.finger_type==3 or global_var.finger_type==4) and (self.origin_pick_quad==2 or self.origin_pick_quad==4)):
            self.obus_manager.activate_buttons('Place', 4, 3, 4)
            self.obus_manager.activate_buttons('Place', 2, 2, 2)
        #seleccionado alguno de la parte inferior de la caja y dedos 3 y 4  
        elif((global_var.finger_type==3 or global_var.finger_type==4) and (self.origin_pick_quad==1 or self.origin_pick_quad==3)):
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
    
# Gestión de acción de botón: 'Reset positions place'.
    def press_reset_positions_button_place(self):
        """Acción del botón: resetea las posiciones de place."""
        self.obus_manager.reset_positions_place()
        self.desactivate_buttons()
        self.activate_buttons()
    
# Gestión de acción de botón: 'Reset positions pick'.
    def press_reset_positions_button_pick(self):
        """Acción del botón: resetea las posiciones de pick."""
        self.obus_manager.reset_positions_pick()        

# Gestión de acción de botón: 'Undo positions pick'.
    def press_undo_positions_button_pick(self):
        """Acción del botón: deshace la última posición pick marcada."""
        logger.debug("Undo pick button pressed")
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

    # Gestión de acción de botón: 'Undo positions place'.
    def press_undo_positions_button_place(self):
        """Acción del botón: deshace la última posición place marcada."""
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

    # Callback ROS
    def callback_robot_moving(self,data):        
        self.do_callback_robot_moving.emit(data)
    
    # Callback signal
    def callback_robot_moving_signal(self, data):
        """
        Callback ROS: se ejecuta cuando el robot cambia su estado de movimiento.
        Actualiza el modo (AUTOMATIC/MANUAL) y activa/desactiva botones.
        """
        #logger.info("CB:moving_received:"),data.data
        if data.data == True :
            if not global_flags.KUKA_AUT:
                global_flags.KUKA_AUT=True
            if global_flags.first_time_moving_kuka:
                    self.mode_label.setText("AUTOMATIC")
                    self.desactivate_buttons()
                    global_flags.first_time_moving_kuka = False
                            
        else:
            if global_flags.KUKA_AUT:    
                global_flags.KUKA_AUT=False
            if global_flags.first_time_moving_kuka==False:
                self.mode_label.setText("MANUAL")
                self.activate_buttons()
                global_flags.first_time_moving_kuka = True

    
            
    # Callback ROS: gestiona eventos del topic o servicio relacionado.
    def callback_motor_status_signal(self,data):
        """
        Callback ROS: actualiza el estado del voltaje y estado operativo de la herramienta.
        Muestra advertencia si hay fallo o habilitación.
        """
        #global_flags.first_time_enabled
        #motor1=data.motor_status[1]
        #driveflags_1=numpy.array(map(int,motor1.driveflags))
        #under_voltage_1=driveflags_1[12]
        #if(under_voltage_1==1):
        #    global_flags.under_voltage_tool=True
        #    pixmap = QtGui.QPixmap(global_var.IMG_PATH+"/pinza_roja_peq2.png")
        #    self.under_voltage_tool.setPixmap(pixmap)
        #    #logger.info("undervoltage")
        #else:
        #    global_flags.under_voltage_tool=False
        #    pixmap = QtGui.QPixmap(global_var.IMG_PATH+"/pinza_verde_peq2.png")
        #    self.under_voltage_tool.setPixmap(pixmap)
        #if(motor1.status=="OPERATION_ENABLED" and first_time_enabled):
        #    #if(weight_read-weight_empty<-10):
        #    global_flags.first_time_enabled=False
        #    #logger.info("Warninng of weight should be here")				
        #    ret = QMessageBox.information(self, "WARNING!", 'Tool enabled', QMessageBox.Ok)
        #if(motor1.status=="FAULT"):
        #    global_flags.first_time_enabled=True
        #    #logger.info(first_time_enabled)
            

    # Callback ROS
    def callback_door_state(self,data):
            self.do_callback_door_state.emit(data)
    # Callback signal
    def callback_door_state_signal(self, data):
        """Callback ROS: actualiza el estado de la puerta de seguridad."""
        if data.digital_inputs[0]:
            pixmap =QtGui.QPixmap(global_var.IMG_PATH+"/puerta_roja_peq.png")
            self.label_door.setPixmap(pixmap)
        else :
            pixmap =QtGui.QPixmap(global_var.IMG_PATH+"/puerta_verde_peq.png")
            self.label_door.setPixmap(pixmap)
        if data.digital_inputs[1]:
            pixmap =QtGui.QPixmap(global_var.IMG_PATH+"/emergency_verde_peq.png")
            self.label_6.setPixmap(pixmap)
        else :
            pixmap =QtGui.QPixmap(global_var.IMG_PATH+"/emergency_roja_peq.png")
            self.label_6.setPixmap(pixmap)
                        
    # Callback ROS: gestiona eventos del topic o servicio relacionado.
    def callback_motor_status(self,data):
            self.do_callback_motor_status.emit(data)

    # Callback ROS: gestiona eventos del topic o servicio relacionado.
    def callback_robot_pose(self, data):
        if not global_flags.rob_connected :
             global_flags.rob_connected = True
             self.mode_label.setText("MANUAL")
        #logger.info("CB:robot_pose_received"),data
        global_var.pos_x_kuka=data.x
        global_var.pos_y_kuka=data.y
        global_var.pos_z_kuka=data.z
        global_var.pos_a_kuka=data.A
        global_var.pos_b_kuka=data.B
        global_var.pos_c_kuka=data.C
        #elapsed_time_gauges=time.time()-start_time_gauges
        #logger.info("time between robot callback and gauges") ,elapsed_time_gauges
        #if (elapsed_time_gauges>=2):
            #gauges_failure=True
    
    # Callback signal
    def callback_horiz_force_signal(self, data):
        #logger.info("force_received:"),data.data
        global_var.horiz_force_read = data.data
        self.vertforce_lcdNumber.setDigitCount(4)
        self.vertforce_lcdNumber.display(round((data.data-global_var.horiz_force_empty)*0.19,1))
    # Callback ROS
    def callback_horiz_force(self,data):
        self.do_callback_horiz_force.emit(data)
        
    # Callback signal
    def callback_tool_weight_signal(self, data):
        #start_time_gauges=time.time()
        #gauges_failure=False
        self.weight_lcdNumber.setDigitCount(4)
        palette = self.weight_lcdNumber.palette()
        progressBar_palette = self.weightProgressBar.palette()   
        #logger.info("CB:tool_weight_received"),data
        global_var.weight_read=data.data
        weight_no_tool=data.data#-weight_empty
        global_var.weight_reads[0]=weight_no_tool
        for i in range(1, 5):
            weight_no_tool=weight_no_tool+global_var.weight_reads[i]
        weight_no_tool=weight_no_tool/5
        for i in range(1, 5):
            global_var.weight_reads[i]=global_var.weight_reads[i-1]
        self.weight_lcdNumber.setDecMode()
        #self.weight_lcdNumber.setNumDigits(3)
        self.weight_lcdNumber.display(round(weight_no_tool,1))
        if (-weight_no_tool)<global_var.weight_expected_min:
            palette.setColor(palette.WindowText, QtGui.QColor(10, 10, 10))
            self.weightProgressBar.setStyleSheet("""QProgressBar::chunk { background: grey; }""")
            self.weightProgressBar_2.setStyleSheet("""QProgressBar::chunk { background: grey; }""")
            if(weight_no_tool>5):
                    self.weightProgressBar_2.setStyleSheet("""QProgressBar::chunk { background: red; }""")
        elif (-weight_no_tool)<global_var.weight_expected_max:
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
        
    # Callback ROS
    def callback_tool_weight(self,data):
        self.do_callback_tool_weight.emit(data)

    # Callback signal
    def callback_current_signal(self, data):
        #logger.info("CB:current_received"),data
        global_var.tool_current = data.data
        self.tool_force_lcdNumber.setDigitCount(4)
        self.tool_force_lcdNumber.display(round(data.data,1))
    
    # Callback ROS
    def callback_current(self,data):
            self.do_callback_current.emit(data)
    
# Gestión de acción de botón: 'Move to rotation table'.
    def press_move_to_rotation_table_button(self):
        ret = QMessageBox.warning(self, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            try:
                placed_rel_service = rospy.ServiceProxy(global_var.srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=placed_rel_service(0, 0, obuses_poses.pose_z_safe-global_var.pos_z_kuka, 0, 0, 0)
                #global_flags.KUKA_AUT=True
                self.sleep_loop(2)
                while global_flags.KUKA_AUT: self.sleep_loop(0.3)
                placed_abs_service = rospy.ServiceProxy(global_var.srv_name_move_abs_slow, set_CartesianEuler_pose)
                ret=placed_abs_service(obuses_poses.H2O1_Pose_x, obuses_poses.H2O1_Pose_y, obuses_poses.H2O1_Pose_z, obuses_poses.table_pose_a, obuses_poses.H2O1_Pose_b, obuses_poses.H2O1_Pose_c)
                #global_flags.KUKA_AUT=True
                self.sleep_loop(2)
                while global_flags.KUKA_AUT: self.sleep_loop(0.3)
                placed_abs_service = rospy.ServiceProxy(global_var.srv_name_move_abs_slow, set_CartesianEuler_pose)                
                ret = placed_abs_service(obuses_poses.table_pose_x, obuses_poses.table_pose_y, obuses_poses.table_pose_z, obuses_poses.table_pose_a, obuses_poses.table_pose_b, obuses_poses.table_pose_c)
            except rospy.ServiceException as e:
                logger.error("Service call failed: %s", e)
                ret=QMessageBox.critical(self, "ERROR!", 'Movement Service not available.', QMessageBox.Ok)    
                    
# Gestión de acción de botón: 'Tool homming'.
    def press_tool_homming(self):
        ret = QMessageBox.warning(self, "WARNING!", 'Are you sure? \nBe sure there is no obus picked', QMessageBox.Ok, QMessageBox.Cancel)        
        if ret == QMessageBox.Ok:
            #Call tool homing method
            try:
                limit_cont_current_service=rospy.ServiceProxy(global_var.srv_limit_cont_current, set_float_value)
                limit_peak_current_service=rospy.ServiceProxy(global_var.srv_limit_peak_current, set_float_value)
                limit_cont_current_service(global_var.current_limit_0)
                limit_peak_current_service(global_var.current_limit_0)
                #gripper_move_service = rospy.ServiceProxy(global_var.srv_finger_set_pose,set_odometry)
                homing_service = rospy.ServiceProxy(global_var.srv_tool_homing, home)           
                ret = homing_service()
                #TOOL_HOMED=True 
                #weight_empty=weight_read
                #gripper_move_service(0.02,0,0,-0.15)
                if ret == True:
                    global_flags.TOOL_HOMED=True
                else:
                    logger.info("ERROR: Homing service returns false!")
                    #antes se hacía el TOOL_HOMED=True aunque fallase
                #set current again
                if global_var.finger_type == 0:
                    limit_cont_current_service(global_var.current_limit_0)
                    limit_peak_current_service(global_var.current_limit_0)
                elif global_var.finger_type == 1:
                    limit_cont_current_service()
                    limit_peak_current_service(global_var.current_limit_1)
                elif global_var.finger_type == 2:
                    limit_cont_current_service(global_var.current_limit_2)
                    limit_peak_current_service(global_var.current_limit_2)
                elif global_var.finger_type == 3:
                    limit_cont_current_service(global_var.current_limit_cont)
                    limit_peak_current_service(global_var.current_limit_3)
                elif global_var.finger_type == 4:
                    limit_cont_current_service(global_var.current_limit_cont)
                    limit_peak_current_service(global_var.current_limit_4)               
            except rospy.ServiceException as e:
                logger.error("Service call failed: %s", e)
                QMessageBox.critical(self, "Error", "Service call failed: %s" % e)
            
# Gestión de acción de botón: 'Finger adjust'.
    def press_finger_adjust_button(self):
        if(not global_flags.TOOL_HOMED):
                QMessageBox.warning(self, "WARNING!", 'Are you sure? \nHoming of the tool should be done first', QMessageBox.Ok, QMessageBox.Cancel)
        ret = QMessageBox.warning(self, "WARNING!", 'Are you sure? \nBe sure there is no obus picked', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            try:
                gripper_trasl_service = rospy.ServiceProxy(global_var.srv_finger_set_pose,set_odometry)
                if global_var.finger_type == 0:
                    logger.debug("No gripper selected")
                elif global_var.finger_type == 1:
                    logger.debug("Set gripper to 100mm")
                    tras_from_homing=0.2-0.1;               
                    ret=gripper_trasl_service(tras_from_homing,0,0,0)
                elif global_var.finger_type == 2:
                    logger.debug("Set gripper to 140mm")
                    tras_from_homing=0.2-0.14;
                    ret=gripper_trasl_service(tras_from_homing,0,0,0)
                elif global_var.finger_type == 3:
                    logger.debug("Set gripper to 160mm")
                    tras_from_homing=0.2-0.16;
                    ret=gripper_trasl_service(tras_from_homing,0,0,0)
                elif global_var.finger_type == 4:
                    logger.debug("Set gripper to 270mm")
                    tras_from_homing=0.03;
                    ret=gripper_trasl_service(tras_from_homing,0,0,0)
            except rospy.ServiceException as e:
                logger.error("Service call failed: %s", e)
                QMessageBox.critical(self, "Error", "Service call failed: %s" % e)
            
# Gestión de acción de botón: 'Led on'.
    def press_led_on_button(self):
        try:
            led_service = rospy.ServiceProxy(global_var.srv_digital_io, set_digital_output)
            ret = led_service(6,False)
        except rospy.ServiceException as e:
            logger.error("Service call failed: %s", e)
            QMessageBox.critical(self, "Error", "Service call failed: %s" % e)

# Gestión de acción de botón: 'Led off'.
    def press_led_off_button(self):
        try:
            led_service = rospy.ServiceProxy(global_var.srv_digital_io, set_digital_output)
            ret = led_service(6,True)
        except rospy.ServiceException as e:
            logger.error("Service call failed: %s", e)
            QMessageBox.critical(self, "Error", "Service call failed: %s" % e)
        
# Función: Deadman state changed.
    def deadMan_state_changed(self):
        ret_q = QMessageBox.warning(self, "WARNING!", 'Changes to the current configuration will be applied', QMessageBox.Ok, QMessageBox.Cancel)
        if(ret_q==QMessageBox.Ok):
                    if(self.deadMan_check.isChecked()):
                                    try:
                                        deadman_service=rospy.ServiceProxy(global_var.srv_deadman, SetBool)
                                        ret = deadman_service(True)
                                    except rospy.ServiceException as e:
                                        logger.error("Service call failed: %s", e)
                                        QMessageBox.critical(self, "Error", "Service call failed: %s" % e)
                    elif(self.deadMan_check.isChecked()==False):
                                    try:
                                        deadman_service=rospy.ServiceProxy(global_var.srv_deadman, SetBool)
                                        ret = deadman_service(False)
                                    except rospy.ServiceException as e:
                                        logger.error("Service call failed: %s", e)
                                        QMessageBox.critical(self, "Error", "Service call failed: %s" % e)
        else : 
                self.deadMan_check.nextCheckState()
                
# Función: Toolangle state changed.
    def toolAngle_state_changed(self):
        ret_q = QMessageBox.warning(self, "WARNING!", 'Changes to the current configuration will be applied', QMessageBox.Ok, QMessageBox.Cancel)
        if(ret_q==QMessageBox.Ok):
                        if(self.toolAngle_check.isChecked()):
                                    try:
                                        angle_mode_service=rospy.ServiceProxy(global_var.srv_angle_mode, SetBool)
                                        ret = angle_mode_service(True)
                                    except rospy.ServiceException as e:
                                        logger.error("Service call failed: %s", e)
                                        QMessageBox.critical(self, "Error", "Service call failed: %s" % e)
                        elif(self.toolAngle_check.isChecked()==False):
                                    try:
                                        angle_mode_service=rospy.ServiceProxy(global_var.srv_angle_mode, SetBool)
                                        ret = angle_mode_service(False)
                                    except rospy.ServiceException as e:
                                        logger.error("Service call failed: %s", e)
                                        QMessageBox.critical(self, "Error", "Service call failed: %s" % e)
        else:
                self.toolAngle_check.nextCheckState()

# Función: Toolorientation state changed.
    def toolOrientation_state_changed(self):
        ret_q = QMessageBox.warning(self, "WARNING!", 'Changes to the current configuration will be applied', QMessageBox.Ok, QMessageBox.Cancel)
        if(ret_q==QMessageBox.Ok):
                if(self.toolOrientation_check.isChecked()):
                            try:
                                toolOrientation_service=rospy.ServiceProxy(global_var.srv_rel_tool, SetBool)
                                ret = toolOrientation_service(True)
                            except rospy.ServiceException as e:
                                logger.error("Service call failed: %s", e)
                                QMessageBox.critical(self, "Error", "Service call failed: %s" % e)
                elif(self.toolOrientation_check.isChecked()==False):
                            try:
                                toolOrientation_service=rospy.ServiceProxy(global_var.srv_rel_tool, SetBool)
                                ret = toolOrientation_service(False)
                            except rospy.ServiceException as e:
                                logger.error("Service call failed: %s", e)
                                QMessageBox.critical(self, "Error", "Service call failed: %s" % e)
        else:
                self.toolOrientation_check.nextCheckState()
			
# Gestión de acción de botón: 'Light on'.
    def press_light_on_button(self):
        try:
            led_service = rospy.ServiceProxy(global_var.srv_digital_io, set_digital_output)
            ret = led_service(4,False)
        except rospy.ServiceException as e:
            logger.error("Service call failed: %s", e)
            QMessageBox.critical(self, "Error", "Service call failed: %s" % e)
    
# Gestión de acción de botón: 'Light off'.
    def press_light_off_button(self):
        try:
            led_service = rospy.ServiceProxy(global_var.srv_digital_io, set_digital_output)
            ret = led_service(4,True)
        except rospy.ServiceException as e:
            logger.error("Service call failed: %s", e)
            QMessageBox.critical(self, "Error", "Service call failed: %s" % e)
            
# Gestión de acción de botón: 'Homming'.
    def press_homming_button(self):     
        ret = QMessageBox.warning(self, "WARNING!", 
                                'Are you sure? \nRobot is going to move autonomously', 
                                QMessageBox.Ok, QMessageBox.Cancel)
        if ret != QMessageBox.Ok:
            logger.debug("[press_homming_button] Acción cancelada por el usuario.")
            return

        #self.origin_pick_quad = 0 #SEGURO? por qué hacer homing es quitar la restricción de los botones de places?
        logger.info("[press_homming_button] Iniciando homming...")

        try:
            logger.info("[press_homming_button] Moviendo en Z de forma relativa (pre-homing).")
            homming_rel_service = rospy.ServiceProxy(global_var.srv_name_move_rel_slow, set_CartesianEuler_pose)
            ret_rel = homming_rel_service(0, 0, obuses_poses.pose_z_safe - global_var.pos_z_kuka, 0, 0, 0)
            logger.info("[press_homming_button] Movimiento relativo Z ejecutado, esperando...")
            self.sleep_loop(2)
            while global_flags.KUKA_AUT:
                logger.info("[press_homming_button] Esperando a que global_flags.KUKA_AUT sea False...")
                self.sleep_loop(0.3)

            logger.info("[press_homming_button] Llamando a home_A1_A6_service...")
            home_A1_A6_service = rospy.ServiceProxy(global_var.srv_move_A1_A6, set_A1_A6)
            ret_a1a6 = home_A1_A6_service(0.0, 177)
            logger.info("[press_homming_button] Movimiento home_A1_A6 ejecutado, esperando...")
            self.sleep_loop(2)
            logger.info("[press_homming_button] Esperando a que global_flags.KUKA_AUT sea False...")
            while global_flags.KUKA_AUT:                
                self.sleep_loop(0.3)

            logger.info("[press_homming_button] Moviendo a posición absoluta en la mesa.")
            placed_abs_service = rospy.ServiceProxy(global_var.srv_name_move_abs_slow, set_CartesianEuler_pose)
            ret_abs = placed_abs_service(obuses_poses.table_pose_x, obuses_poses.table_pose_y, obuses_poses.table_pose_z, obuses_poses.table_pose_a, obuses_poses.table_pose_b, obuses_poses.table_pose_c)
            if ret_abs == True:
                global_var.CURRENT_STATE = global_var.STATE_MOVING_TO_PLACE
                logger.info("[press_homming_button] Homming completado y estado actualizado.")
            else:
                logger.warning("[press_homming_button] Advertencia: La llamada a placed_abs_service no devolvió True.")
        except rospy.ServiceException as e:
            logger.error("[press_homming_button] FALLO EN LA LLAMADA AL SERVICIO: %s" % e)
            QMessageBox.critical(self, "Error", "Service call failed: %s" % e)
        except Exception as e:
            logger.error("[press_homming_button] ERROR INESPERADO: %s" % e)
            QMessageBox.critical(self, "Error", "Service call failed: %s" % e)

# Gestión de acción de botón: 'Picktest'.
    def press_picktest_button(self):
        ret = QMessageBox.warning(self, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            try:
                placed_rel_service = rospy.ServiceProxy(global_var.srv_name_move_rel_slow, set_CartesianEuler_pose)
                ret_rel=placed_rel_service(0, 0, 20, 0, 0, 0)
                #global_flags.KUKA_AUT=True
                #while global_flags.KUKA_AUT: time.sleep(0.1)
                if ret_rel == True:
                        CURRENT_STATE=global_var.STATE_DOING_PICK_TEST
            except rospy.ServiceException as e:
                logger.error("Service call failed: %s", e)
                QMessageBox.critical(self, "Error", "Service call failed: %s" % e)

# Gestión de acción de botón: 'Tare'.
    def press_tare_button(self):
        try:
            tare_service = rospy.ServiceProxy(global_var.srv_tare_gauges, SetBool)
            ret = tare_service(True)
        except rospy.ServiceException as e:
                logger.error("Service call failed: %s", e)
                QMessageBox.critical(self, "Error", "Service call failed: %s" % e)

# Gestión de acción de botón: 'Tare reset'.
    def press_tare_reset_button(self):
        try:
            tare_service = rospy.ServiceProxy(global_var.srv_tare_gauges, SetBool)
            ret = tare_service(False)
        except rospy.ServiceException as e:
                logger.error("Service call failed: %s", e)
                QMessageBox.critical(self, "Error", "Service call failed: %s" % e)
    #################################################JOY SELECTION
# Función: Joy selected.
    def joy_selected(self, index):
        if index == 0:
            logger.debug("PS4 selected")
            command_string = "rosrun topic_tools mux_select mux_joy /kuka_pad/ps4_joy"
            os.system(command_string)
        elif index == 1:            
            logger.debug("ITOWA selected")
            command_string = "rosrun topic_tools mux_select mux_joy /kuka_pad/itowa_joy"
            os.system(command_string)
    
    #################################################CALIBRE SELECTION
# Gestión de cambio de calibre (selección de tipo de gripper/obus).
    def calibre_selected(self, index):

        logger.debug('Selected:: %s', index)
        global_var.finger_type = index
        calibre = CALIBRES.get(index, CALIBRES[0])

        # Desactivar botones si no hay calibre
        if index == 0:
            self.desactivate_buttons()
            self.weight_limited.setText("N/A")
            self.weight_label_expected_var.setText("[N/A, N/A]")
        else:
            self.activate_buttons()

        # Establecer rangos de peso esperados
        global_var.weight_expected_min = calibre["weight_min"] or 0
        global_var.weight_expected_max = calibre["weight_max"] or 0
        str_weight_expected = "[{}, {}]".format(
            calibre["weight_min"] if calibre["weight_min"] is not None else "N/A",
            calibre["weight_max"] if calibre["weight_max"] is not None else "N/A",
        )
        self.weight_label_expected_var.setText(str_weight_expected)
        self.weight_limited.setText(calibre["weight_limited"])

        # Cambiar imágenes de fondo
        pixmap = QtGui.QPixmap(global_var.IMG_PATH + "/" + calibre["bg_img"])
        self.background_plate.setPixmap(pixmap)
        pixmap_pick = QtGui.QPixmap(global_var.IMG_PATH + "/" + calibre["bg_pick_img"])
        self.background_plate_pick.setPixmap(pixmap_pick)

        # Oculta todos los botones y muestra solo los del calibre elegido
        hide_buttons(self, ALL_BTNS)
        show_buttons(self, calibre["show_place"] + calibre["show_pick"])

        # Cambiar límites de corriente si aplica
        if calibre["current_limit"]:
            limit_cont_current_service = rospy.ServiceProxy(global_var.srv_limit_cont_current, set_float_value)
            limit_peak_current_service = rospy.ServiceProxy(global_var.srv_limit_peak_current, set_float_value)
            limit_value = [calibre["current_limit"]]
            try:
                limit_cont_current_service(limit_value)
                limit_peak_current_service(limit_value)
            except (rospy.ServiceException, rospy.ROSException) as e:
                logger.error("Service call failed: %s" % (e,))
                QMessageBox.critical(self, "Error", "Service call failed: %s" % e)
            global_var.current_limit_picked = limit_value

        # Ajusta el máximo del progressbar
        if calibre["weight_max"]:
            self.weightProgressBar.setMaximum(calibre["weight_max"] * 1.3)
        else:
            self.weightProgressBar.setMaximum(100)

    #def load_robot_description(self, gripper_model):
    #    command_string = "rosparam load ~/kuka_catkin_ws/src/kuka_experimental/kuka_robot_bringup/robot/bin/kr120toolv%d.urdf /robot_description" % gripper_model
    #    os.system(command_string)
        
# Gestión de acción de botón: 'Reset external pc'.
    def press_reset_external_pc_button(self):
        ret = QMessageBox.warning(self, "WARNING!", 'Are you sure? \nExternal PC is going to reset.\n Wait 10 sec and restart the GUI.', QMessageBox.Ok, QMessageBox.Cancel)
        if ret == QMessageBox.Ok:
            #command_string = "ssh robotnik@192.168.1.10 sudo -S <<< \"R0b0tn1K\" reboot \n"
            command_string = global_var.SCRIPTS_PATH + "reboot.sh"
            logger.info(command_string)
            os.system(command_string)

    ###TEST APRIETE AUTOMATICO: si el nodo de las galgas falla se va  a liar
# Función: Aut press tool.
    def aut_press_tool(self):
        gripper_move_service = rospy.ServiceProxy(global_var.srv_finger_set_pose, set_odometry)
        press_counter = 0
        old_pos_x = 0
        self.desactivate_buttons()
        
        logger.info("[aut_press_tool] Iniciando ciclo de cierre de herramienta")

        # Primer bucle: Cerrar la herramienta (ajustar ángulo)
        while press_counter < 5:
            logger.debug("press_counter:: %s", press_counter)
            logger.debug("global_var.angle_tool:: %s", global_var.angle_tool)
            try:
                gripper_move_service(global_var.x_tool, 0, 0, global_var.angle_tool - 0.01)
            except rospy.ServiceException as e:
                logger.error("[aut_press_tool] Error llamando al servicio de pinza (cerrando):: %s", str(e))
                QMessageBox.critical(self, "Error", "Service call failed: %s" % e)
                self.activate_buttons()
                return
            except Exception as e:
                logger.error("[aut_press_tool] Error inesperado llamando al servicio (cerrando):: %s", str(e))
                QMessageBox.critical(self, "Error", "Service call failed: %s" % e)
                self.activate_buttons()
                return
            self.sleep_loop(0.15)
            if global_var.tool_current > global_var.current_limit_cont:
                press_counter += 1
            else:
                press_counter = 0
            logger.debug("Δglobal_var.x_tool:: %s", abs(global_var.x_tool - old_pos_x))
            logger.debug("current:: %s", global_var.tool_current)
            logger.debug("counter:: %s", press_counter)

        press_counter = 0
        # Segundo bucle: Abrir ligeramente la herramienta para afinar posición
        logger.info("[aut_press_tool] Iniciando ciclo de apertura fina")
        while press_counter < 5:
            logger.debug("press_counter:: %s", press_counter)
            try:
                gripper_move_service(global_var.x_tool + 0.02, 0, 0, global_var.angle_tool)
            except rospy.ServiceException as e:
                logger.error("[aut_press_tool] Error llamando al servicio de pinza (abriendo):: %s", str(e))
                QMessageBox.critical(self, "Error", "Service call failed: %s" % e)
                self.activate_buttons()
                return
            except Exception as e:
                logger.error("[aut_press_tool] Error inesperado llamando al servicio (abriendo):: %s", str(e))
                QMessageBox.critical(self, "Error", "Service call failed: %s" % e)
                self.activate_buttons()
                return
            self.sleep_loop(0.15)
            if global_var.tool_current > global_var.current_limit_cont or abs(global_var.x_tool - old_pos_x) < 0.002:
                press_counter += 1
            else:
                press_counter = 0
                old_pos_x = global_var.x_tool
            logger.debug("current:: %s", global_var.tool_current)
            logger.debug("counter:: %s", press_counter)
        
        self.sleep_loop(0.5)
        logger.info("[aut_press_tool] Finalizado, reactivando botones")
        self.activate_buttons() 
        
# Callback ROS: gestiona eventos del topic o servicio relacionado.
    def callback_tool_state(self, data):
        global_var.x_tool = data.position[2]
        global_var.angle_tool = data.position[3]
    
# Gestión de acción de botón: 'Reset robot'.
    def press_reset_robot_button(self):
        #command_string = "rosnode kill /kuka_pad/ps4_joystick; sleep 1; rosnode kill /kuka_pad/itowa_safe_joystick; sleep 1; rosnode kill /kuka_pad/robotnik_trajectory_pad_node; sleep 1; rosnode kill /kuka_robot/kuka_cartesian_hardware_interface; sleep 1; roslaunch kuka_robot_bringup kuka_robot_bringup_standalone.launch &"
        command_string = "killall screen; sleep 1; screen -S bringup -d -m roslaunch kuka_robot_bringup kuka_robot_bringup_standalone.launch"
        #command_string = "rosnode kill /kuka_robot/kuka_cartesian_hardware_interface; sleep 1; ROS_NAMESPACE=kuka_robot roslaunch kuka_rsi_cartesian_hw_interface test_hardware_interface.launch &"
        os.system(command_string)
        self.mode_label.setText("NOT CONNECTED")
        global_flags.rob_connected = False
           
# Función: Sleep loop.
    def sleep_loop(self,delay):
        loop = QtCore.QEventLoop()
        timer = QtCore.QTimer()
        timer.setInterval(delay*1000)
        timer.setSingleShot(True)
        timer.timeout.connect(loop.quit)
        timer.start()
        loop.exec_()
