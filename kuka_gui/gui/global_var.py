# config.py
# -*- coding: utf-8 -*-
import os

# Paths
GUI_PATH = os.path.dirname(os.path.abspath(__file__)) + "/"
UI_PATH = os.path.join(GUI_PATH, "resource", "RqtKuka.ui")
IMG_PATH = os.path.join(GUI_PATH, "resource", "images") + "/"

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