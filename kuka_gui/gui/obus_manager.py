# obus_manager.py
# -*- coding: utf-8 -*-

from python_qt_binding import QtGui
from python_qt_binding.QtWidgets import QMessageBox
import rospy
import global_var
import global_flags
import obuses_poses
from robotnik_msgs.srv import set_CartesianEuler_pose
from kuka_rsi_cartesian_hw_interface.srv import set_A1_A6
import logging
import os

logger = logging.getLogger('robotnik_kuka_gui')

class ObusManager:
    def __init__(self, parent):
        self.parent = parent
        self.place_positions = self._get_place_positions()
        self.setup_buttons('Pick')
        self.setup_buttons('Place')
        
    def set_obus_state(self, tipo, grupo, idx, state):
        self.parent.state_dict[(tipo, grupo, idx)] = state

    def get_obus_state(self, tipo, grupo, idx):
        return self.parent.state_dict.get((tipo, grupo, idx), False)
        
    def _get_place_positions(self):
        """
        Retorna un diccionario con la posición y tamaño de cada botón Place
        Formato: {'PlaceObusButton2_1': (x, y, w, h), ...}
        """
        positions = {}
        # Huevera 2
        positions['PlaceObusButton2_1'] = (70, 560, 111, 41)
        positions['PlaceObusButton2_2'] = (70, 560-70, 111, 41)
        # Huevera 4
        positions['PlaceObusButton4_1'] = (60, 600, 102, 37)
        positions['PlaceObusButton4_2'] = (60, 600-50, 102, 37)
        positions['PlaceObusButton4_3'] = (60, 600-102, 102, 37)
        positions['PlaceObusButton4_4'] = (60, 600-152, 102, 37)
        # Huevera 8
        positions['PlaceObusButton8_1'] = (60, 610, 71, 26)
        positions['PlaceObusButton8_2'] = (60, 610-38, 71, 26)
        positions['PlaceObusButton8_3'] = (60, 610-86, 71, 26)
        positions['PlaceObusButton8_4'] = (60, 610-132, 71, 26)
        positions['PlaceObusButton8_5'] = (60+51, 610-18, 71, 26)
        positions['PlaceObusButton8_6'] = (60+51, 610-63, 71, 26)
        positions['PlaceObusButton8_7'] = (60+51, 610-109, 71, 26)
        positions['PlaceObusButton8_8'] = (60+51, 610-150, 71, 26)
        # Huevera 16
        positions['PlaceObusButton16_1'] = (60, 610, 51, 19)
        positions['PlaceObusButton16_2'] = (60, 610-22, 51, 19)
        positions['PlaceObusButton16_3'] = (60, 610-42, 51, 19)
        positions['PlaceObusButton16_4'] = (60, 610-63, 51, 19)
        positions['PlaceObusButton16_5'] = (60, 610-86, 51, 19)
        positions['PlaceObusButton16_6'] = (60, 610-109, 51, 19)
        positions['PlaceObusButton16_7'] = (60, 610-132, 51, 19)
        positions['PlaceObusButton16_8'] = (60, 610-152, 51, 19)
        positions['PlaceObusButton16_9'] = (60+74, 610, 51, 19)
        positions['PlaceObusButton16_10'] = (60+74, 610-22, 51, 19)
        positions['PlaceObusButton16_11'] = (60+74, 610-42, 51, 19)
        positions['PlaceObusButton16_12'] = (60+74, 610-63, 51, 19)
        positions['PlaceObusButton16_13'] = (60+74, 610-86, 51, 19)
        positions['PlaceObusButton16_14'] = (60+74, 610-109, 51, 19)
        positions['PlaceObusButton16_15'] = (60+74, 610-132, 51, 19)
        positions['PlaceObusButton16_16'] = (60+74, 610-152, 51, 19)
        
        return positions
    
    def setup_buttons(self, action):
        groups = {2:4, 4:5, 8:15, 16:21} if action == 'Pick' else {2:2, 4:4, 8:8, 16:16}
        for group, count in groups.items():
            for n in range(1, count+1):
                name = '%sObusButton%d_%d' % (action, group, n)
                btn = getattr(self.parent, name, None)
                if btn:
                    handler_name = 'press_%s_obus%d_%d_button' % (action.lower(), group, n)
                    handler = getattr(self.parent, handler_name, None)
                    if handler:
                        btn.clicked.connect(handler)
                    btn.hide()
                    path = self.parent.select_icon(action.lower(), [group, n], 0)
                    mask = QtGui.QPixmap(path)
                    btn.setMask(mask.mask())
                    btn.setMouseTracking(True)
                    btn.installEventFilter(self.parent)
                    # Aplica geometría solo para "Place"
                    if action == 'Place':
                        pos = self.place_positions.get(name)
                        if pos:
                            btn.setGeometry(*pos)

                    
    def activate_buttons(self, action, group, start, end):
        """
        Activa (y muestra) los botones desde 'start' hasta 'end' (ambos inclusive).
        Ejemplo: activate_buttons('Pick', 16, 3, 10) activa PickObus16_3 ... PickObus16_10
        """
        for n in range(start, end+1):
            name = '%sObusButton%d_%d' % (action, group, n)
            btn = getattr(self.parent, name, None)
            if btn:
                btn.setEnabled(True)
                

    def deactivate_buttons(self, action, group, start, end):
        for n in range(start, end+1):
            name = '%sObusButton%d_%d' % (action, group, n)
            btn = getattr(self.parent, name, None)
            if btn:
                btn.setEnabled(False)

                
    def remove_event_filters(self, action, group, start, end):
        """
        Elimina el eventFilter para todos los botones en el rango si su estado es True.
        action: 'Pick' o 'Place'
        group: 2,4,8,16
        start, end: índices a revisar
        """
        tipo = action.lower()
        logger.debug("[remove_event_filters] action=%s, group=%s, rango=%d-%d", action, group, start, end)
        for i in range(start, end+1):
            key = (tipo, group, i)
            state = self.parent.state_dict.get(key, False)
            logger.debug("[remove_event_filters] Revisando %s: estado=%s", key, state)
            if state:
                # Ojo con el nombre del botón, debe ser coherente con cómo lo creaste
                btn_name = '%sObusButton%d_%d' % (action, group, i)  # Ejemplo: PickObus16_1
                btn = getattr(self.parent, btn_name, None)
                if btn:
                    btn.removeEventFilter(self.parent)
                    logger.debug("[remove_event_filters] Eliminado eventFilter en %s", btn_name)
                else:
                    logger.debug("[remove_event_filters] No se encontró el botón: %s", btn_name)
            else:
                logger.debug("[remove_event_filters] No se elimina eventFilter en %s (estado False)", str(key))

                    
    def reset_positions_place(self):
        # Reset estado
        for group, last in [(16,16), (8,8), (4,4), (2,2)]:
            for i in range(1, last+1):
                self.parent.state_dict[('place', group, i)] = False
                btn_name = 'PlaceObusButton%d_%d' % (group, i)
                btn = getattr(self.parent, btn_name, None)
                if btn:
                    icon = QtGui.QIcon()
                    path = self.parent.select_icon('place', [group, i], 0)
                    icon.addPixmap(QtGui.QPixmap(path), QtGui.QIcon.Disabled)
                    btn.setIcon(icon)
                    btn.installEventFilter(self.parent)
        self.parent.last_obus_selected_place = -1        
        
    def reset_positions_pick(self):
        for group, last in [(16,20), (8,14), (4,5), (2,4)]:
            for i in range(1, last+1):
                self.parent.state_dict[('pick', group, i)] = False
                btn_name = 'PickObusButton%d_%d' % (group, i)
                btn = getattr(self.parent, btn_name, None)
                if btn:
                    icon = QtGui.QIcon()
                    path = self.parent.select_icon('pick', [group, i], 0)
                    icon.addPixmap(QtGui.QPixmap(path), QtGui.QIcon.Disabled)
                    btn.setIcon(icon)
                    btn.installEventFilter(self.parent)
        self.parent.last_obus_selected_pick = -1
        self.parent.origin_pick_quad = 0

    def press_obus_button(self, tipo, grupo, idx, 
                        side=None,  # 'left', 'right', etc según la lógica de pick/place
                        pre_z=None,  # Prepick_Pose_z o pose_z_safe, etc
                        icon_state=2,  # ¿qué color de icono quieres poner?
                        axis_service=None,
                        axis_args=None):
        """
        tipo: 'pick' o 'place'
        grupo: 2, 4, 8, 16
        idx: número de obús (1...n)
        side: para elegir A6 izquierdo/derecho, si aplica
        pre_z: valor a usar para el Z del servicio (prepick_pose_z, pose_z_safe, etc)
        icon_state: el color o estado visual (por defecto 2, como en tus funciones)
        axis_service: nombre del servicio para A1_A6 (si es distinto en pick/place)
        axis_args: argumentos que quieres pasarle (lista o tuple)
        """
        
        logger.debug("[Obus_manager] press_obus_button called: tipo=%s, grupo=%s, idx=%s, side=%s, pre_z=%s, icon_state=%s", tipo, grupo, idx, side, pre_z, icon_state)

        # Estado centralizado en el diccionario de la GUI principal
        state_key = (tipo, grupo, idx)
        btn_name = ('PickObusButton' if tipo == 'pick' else 'PlaceObusButton') + '%d_%d' % (grupo, idx)
        last_attr = 'last_obus_selected_' + tipo
        
        logger.debug("[Obus_manager] Button widget name: %s", btn_name)

        # Verifica si ya está ocupado
        if self.parent.state_dict[state_key]:
            logger.debug("[Obus_manager] Position %s already occupied.", state_key)
            ret = QMessageBox.warning(self.parent, "ERROR!", 'Position already occupied.', QMessageBox.Ok)
            return

        ret = QMessageBox.warning(self.parent, "WARNING!", 'Are you sure? \nRobot moves automatically', QMessageBox.Ok, QMessageBox.Cancel)
        if ret != QMessageBox.Ok:
            logger.debug("[Obus_manager] User cancelled action for button %s.", btn_name)
            return

        # Marca como seleccionado
        self.parent.state_dict[state_key] = True
        setattr(self.parent, last_attr, '%d_%d' % (grupo, idx))
        logger.debug("[Obus_manager] Set state %s as selected. last_attr=%s", state_key, last_attr)
        

        # Selección de cuadrante
        if tipo == 'pick':
            if grupo == 2:
                origin_pick_quad = 1 if idx in [1, 2] else 2
            elif grupo == 4:
                origin_pick_quad = 1 if idx in [1, 2, 3] else 2
            elif grupo == 8:
                if idx in range(1, 5):
                    origin_pick_quad = 1
                elif idx in range(5, 9):
                    origin_pick_quad = 2
                elif idx in range(9, 12):
                    origin_pick_quad = 3
                elif idx in range(12, 15):
                    origin_pick_quad = 4
            elif grupo == 16:
                if idx in range(1, 6):
                    origin_pick_quad = 1
                elif idx in range(6, 11):
                    origin_pick_quad = 2
                elif idx in range(11, 16):
                    origin_pick_quad = 3
                elif idx in range(16, 21):
                    origin_pick_quad = 4
            self.parent.origin_pick_quad = origin_pick_quad
            logger.debug("[Obus_manager] origin_pick_quad set to %s", self.parent.origin_pick_quad)

        # Cambia icono
        logger.debug("[Obus_manager] Selecting icon for tipo=%s, grupo=%s, idx=%s, icon_state=%s" , tipo, grupo, idx, icon_state)
        icon = QtGui.QIcon()
        path = self.parent.select_icon(tipo, [grupo, idx], icon_state)
        logger.debug("[Obus_manager] Icon path selected: %s", path)
        if not os.path.isfile(path):
            logger.error("[ERROR] Icon file does not exist: %s", path)
        else:
            logger.debug("[Obus_manager] Icon file exists: %s", path)
        icon.addPixmap(QtGui.QPixmap(path), QtGui.QIcon.Disabled)
        btn = getattr(self.parent, btn_name)
        logger.debug("[Obus_manager] Setting icon on button: %s (%s)", btn_name, btn)
        btn.setIcon(icon)
        logger.debug("[Obus_manager] Icon set successfully on %s", btn_name)
        # Llama a los servicios
        try:
            # Subida lenta en Z (prepick/preplace)
            logger.debug("[Obus_manager] Calling rel_service with pre_z=%s, pos_z_kuka=%s", pre_z, global_var.pos_z_kuka)
            rel_service = rospy.ServiceProxy(global_var.srv_name_move_rel_slow, set_CartesianEuler_pose)
            rel_service(0, 0, pre_z - global_var.pos_z_kuka, 0, 0, 0)
            self.parent.sleep_loop(2)
            while global_flags.KUKA_AUT: self.parent.sleep_loop(0.3)
            
            # Movimiento en ejes A1_A6
            axis_service_proxy = rospy.ServiceProxy(axis_service, set_A1_A6)
            ret_axis = axis_service_proxy(*axis_args)
            if ret_axis is True:
                global CURRENT_STATE
                CURRENT_STATE = global_var.STATE_MOVING_TO_PLACE if tipo == 'pick' else global_var.STATE_MOVING_TO_PREPICK
                        
            pre_x = global_var.pos_x_kuka
            pre_y = global_var.pos_y_kuka
            
            #Movimiento en cartesianas para centrar en la caja (por si se ha cambiado el radio)
            if tipo == 'pick':
                pre_x=obuses_poses.prepick_abs_x
                pre_y=obuses_poses.prepick_abs_y
            elif tipo =='place':
                pre_x=obuses_poses.preplace_abs_x
                pre_y=obuses_poses.preplace_abs_y
            logger.debug("[Obus_manager] Calling abs_service with pre_x=%s, pre_y=%s", pre_x, pre_y)
            abs_service = rospy.ServiceProxy(global_var.srv_name_move_abs_slow, set_CartesianEuler_pose)
            abs_service(pre_x, pre_y, global_var.pos_z_kuka, global_var.pos_a_kuka, global_var.pos_b_kuka, global_var.pos_c_kuka)
            self.parent.sleep_loop(2)
            while global_flags.KUKA_AUT: self.parent.sleep_loop(0.3)    

        except rospy.ServiceException as e:
            logger.error("Service call failed: %s", e)
            QMessageBox.critical(self.parent, "WARNING!", 'Movement Service not available.', QMessageBox.Ok)


