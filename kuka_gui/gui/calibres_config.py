# calibres_config.py

CALIBRES = {
    0: {  # Sin gripper
        "weight_min": None,
        "weight_max": None,
        "weight_limited": "N/A",
        "bg_img": "fondo_huevera_0.png",
        "bg_pick_img": "fondo_huevera_0.png",
        "show_place": [],
        "show_pick": [],
        "current_limit": None,
    },
    1: {  # 16 obuses
        "weight_min": 4,
        "weight_max": 9,
        "weight_limited": "3",
        "bg_img": "rotated-fondo_huevera_16.png",
        "bg_pick_img": "BoxPick_3.png",
        "show_place": ["PlaceObusButton16_{}".format(i) for i in range(1, 17)],
        "show_pick": ["PickObusButton16_{}".format(i) for i in range(1, 21)],
        "current_limit": "current_limit_1",
    },
    2: {  # 8 obuses
        "weight_min": 13,
        "weight_max": 18,
        "weight_limited": "4",
        "bg_img": "rotated-fondo_huevera_8.png",
        "bg_pick_img": "BoxPick_3.png",
        "show_place": ["PlaceObusButton8_{}".format(i) for i in range(1, 9)],
        "show_pick": ["PickObusButton8_{}".format(i) for i in range(1, 15)],
        "current_limit": "current_limit_2",
    },
    3: {  # 4 obuses
        "weight_min": 18,
        "weight_max": 46,
        "weight_limited": "7",
        "bg_img": "rotated-fondo_huevera_4.png",
        "bg_pick_img": "BoxPick_3.png",
        "show_place": ["PlaceObusButton4_{}".format(i) for i in range(1, 5)],
        "show_pick": ["PickObusButton4_{}".format(i) for i in range(1, 6)],
        "current_limit": "current_limit_3",
    },
    4: {  # 2 obuses
        "weight_min": 110,
        "weight_max": 130,
        "weight_limited": "8",
        "bg_img": "rotated-fondo_huevera_2.png",
        "bg_pick_img": "BoxPick_3.png",
        "show_place": ["PlaceObusButton2_{}".format(i) for i in range(1, 3)],
        "show_pick": ["PickObusButton2_{}".format(i) for i in range(1, 5)],
        "current_limit": "current_limit_4",
    },
}

ALL_PLACE_BTNS = sum([v["show_place"] for v in CALIBRES.values()], [])
ALL_PICK_BTNS = sum([v["show_pick"] for v in CALIBRES.values()], [])
ALL_BTNS = list(set(ALL_PLACE_BTNS + ALL_PICK_BTNS))

def show_buttons(self, btn_list):
    for name in btn_list:
        btn = getattr(self, name, None)
        if btn:
            btn.show()

def hide_buttons(self, btn_list):
    for name in btn_list:
        btn = getattr(self, name, None)
        if btn:
            btn.hide()
