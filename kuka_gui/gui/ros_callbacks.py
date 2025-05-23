# ros_callbacks.py
# -*- coding: utf-8 -*-

import numpy
import logging
from python_qt_binding.QtWidgets import QMessageBox
from global_flags import *
from global_var import *
from obuses_poses import *
from QtGui import QPixmap  # ajusta esto si usas PyQt5/PySide2

logger = logging.getLogger('ros_callbacks')

class KukaCallbacks(object):
    """
    Clase base que agrupa los callbacks ROS.
    Debe ser heredada por cualquier clase GUI que maneje ROS callbacks.
    """

    def callback_moving(self, data):
        global KUKA_AUT, first_time_moving_kuka
        if data.data:
            if not KUKA_AUT:
                KUKA_AUT = True
            if first_time_moving_kuka:
                self.mode_label.setText("AUTOMATIC")
                self.desactivate_buttons()
                first_time_moving_kuka = False
        else:
            if KUKA_AUT:
                KUKA_AUT = False
            if not first_time_moving_kuka:
                self.mode_label.setText("MANUAL")
                self.activate_buttons()
                first_time_moving_kuka = True

    def callback_motor_status(self, data):
        global under_voltage_tool, first_time_enabled
        motor1 = data.motor_status[1]
        driveflags_1 = numpy.array(map(int, motor1.driveflags))
        under_voltage_1 = driveflags_1[12]

        if under_voltage_1 == 1:
            under_voltage_tool = True
            pixmap = QPixmap(IMG_PATH + "/pinza_roja_peq2.png")
            self.under_voltage_tool.setPixmap(pixmap)
        else:
            under_voltage_tool = False
            pixmap = QPixmap(IMG_PATH + "/pinza_verde_peq2.png")
            self.under_voltage_tool.setPixmap(pixmap)

        if motor1.status == "OPERATION_ENABLED" and first_time_enabled:
            first_time_enabled = False
            QMessageBox.information(self, "WARNING!", 'Tool enabled', QMessageBox.Ok)

        if motor1.status == "FAULT":
            first_time_enabled = True

    def callback_tool_weight(self, data):
        self._tool_weight = data.data

    def callback_current(self, data):
        self._tool_current = data.data

    def callback_horiz_force(self, data):
        self._tool_force = data.data

    def callback_robot_pose(self, data):
        self._robot_pose = data

    def callback_tool_state(self, data):
        self._tool_joint_state = data

    def callback_door_state(self, data):
        self._door_status = data