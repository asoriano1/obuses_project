# -*- coding: utf-8 -*-
import logging
# Configuración básica de logging
logging.basicConfig(
    level=logging.DEBUG,  # Cambia a INFO o WARNING en producción si quieres menos "ruido"
    format='%(asctime)s %(levelname)s [%(name)s]: %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)

import sys
import os
import time
import xmlrpclib
import rospy
from python_qt_binding.QtWidgets import QApplication
from robotnik_kuka_gui import KukaGUI


def wait_for_roscore_loop():
    ros_master_uri = os.environ.get('ROS_MASTER_URI', 'http://localhost:11311')
    master = xmlrpclib.ServerProxy(ros_master_uri)

    print("[INFO] Esperando a que roscore esté disponible (Ctrl+C para salir)...")

    while True:
        try:
            code, msg, val = master.getSystemState('/ros_check')
            if code == 1:
                print("[INFO] roscore está activo.")
                return True
        except:
            print("[WARN] roscore no encontrado. Reintentando en 3 segundos...")
        time.sleep(3)

if __name__ == "__main__":
    
    if not rospy.core.is_initialized():
        wait_for_roscore_loop()
        rospy.init_node('kuka_gui', anonymous=True)

    
    app = QApplication(sys.argv)
    
    print("Widget initialization...")
    widget = KukaGUI()
    print("GUI initialization...")
    widget.show()
    print("GUI launched!")
    sys.exit(app.exec_())