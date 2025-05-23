# widgets_management.py
from python_qt_binding.QtWidgets import QWidget
import QtCore
import QtGui
from global_var import *
from global_flags import *
from obuses_poses import *

class WidgetsManagement(object):

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
