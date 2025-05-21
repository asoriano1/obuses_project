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


