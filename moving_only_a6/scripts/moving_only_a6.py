#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
from std_msgs.msg import Bool
from kuka_rsi_cartesian_hw_interface.srv import set_A1_A6

class KukaA1A6Controller(object):
    """
    Nodo ROS que alterna las articulaciones A1 y A6 de Kuka 
    entre dos valores de A6 configurables (por parámetro),
    siempre con A1=0, llamando al servicio '/setKukaA1A6'
    y esperando a que el robot se detenga (en base a '/kuka_moving').
    """

    def __init__(self):
        # Leer parámetros (con valores por defecto si no están definidos)
        self.a6_min = rospy.get_param("~a6_min", 40.0)  # Valor por defecto = 40
        self.a6_max = rospy.get_param("~a6_max", 60.0)  # Valor por defecto = 60
        self.total_duration = rospy.get_param("~duration", 60.0)  # Por defecto 1 minuto

        rospy.loginfo("Parámetros leídos: a6_min=%.2f, a6_max=%.2f, duration=%.1f s", 
                      self.a6_min, self.a6_max, self.total_duration)

        # Suscribirse al tópico que indica si el robot está moviéndose
        self.robot_moving = False
        rospy.Subscriber("/kuka_moving", Bool, self.moving_callback)

        rospy.loginfo("Esperando a que el servicio '/setKukaA1A6' esté disponible...")
        rospy.wait_for_service("/kuka_robot/setKukaA1A6")
        self.set_kuka_a1_a6_client = rospy.ServiceProxy("/kuka_robot/setKukaA1A6", set_A1_A6)
        rospy.loginfo("Servicio '/setKukaA1A6' encontrado.")

    def moving_callback(self, msg):
        """
        Callback que actualiza el estado del robot (en movimiento o parado).
        """
        self.robot_moving = msg.data

    def wait_until_stopped(self, timeout=30.0, rate_hz=5):
        """
        Espera hasta que el robot se detenga (robot_moving == False).
        :param timeout: Tiempo máximo de espera en segundos (0 o negativo = sin límite).
        :param rate_hz: Frecuencia de chequeo.
        :return: True si se detuvo antes del timeout, False en caso contrario.
        """
        start_time = rospy.Time.now().to_sec()
        rate = rospy.Rate(rate_hz)
        while not rospy.is_shutdown():
            if not self.robot_moving:
                # Robot parado
                return True
            if timeout > 0 and (rospy.Time.now().to_sec() - start_time) >= timeout:
                rospy.logwarn("Tiempo de espera agotado antes de que el robot se detuviera.")
                return False
            rate.sleep()

    def move_a1_a6(self, a1_value, a6_value):
        """
        Llama al servicio para mover A1 y A6 a los valores indicados.
        Retorna True/False según la respuesta 'ret' del servicio.
        """
        rospy.loginfo("Llamando al servicio: A1=%.1f, A6=%.1f", a1_value, a6_value)
        try:
            resp = self.set_kuka_a1_a6_client(a1_value, a6_value)
            if resp.ret:
                rospy.loginfo("Llamada exitosa: movimiento iniciado.")
            else:
                rospy.logwarn("El servicio '/setKukaA1A6' devolvió ret=False. Revisa límites o lógica interna.")
            return resp.ret
        except rospy.ServiceException as e:
            rospy.logerr("Error al llamar a /setKukaA1A6: %s", e)
            return False

def main():
    rospy.init_node("kuka_a1_a6_node", anonymous=True)

    controller = KukaA1A6Controller()

    start_time = time.time()
    # Bucle que alterna a6_min y a6_max
    angles = [controller.a6_min, controller.a6_max]
    i = 0

    rospy.loginfo("Iniciando bucle de %.1f segundos alternando A6 entre %.1f y %.1f (A1=0).",
                  controller.total_duration, controller.a6_min, controller.a6_max)

    while not rospy.is_shutdown():
        elapsed = time.time() - start_time
        if elapsed >= controller.total_duration:
            rospy.loginfo("Ha transcurrido %.1f segundos. Finalizando secuencia.", controller.total_duration)
            break

        # 1) Llamar al servicio con A1=0 y A6 en (a6_min o a6_max)
        a6_target = angles[i % len(angles)]
        success = controller.move_a1_a6(0.0, a6_target)
        if not success:
            rospy.logwarn("No se pudo iniciar el movimiento (ret=False). Saltamos al siguiente o finalizamos.")
            # Podrías hacer un break si no quieres seguir alternando tras fallo
            # break

        # 2) Esperar a que el robot deje de moverse
        stopped_ok = controller.wait_until_stopped(timeout=30.0)
        if not stopped_ok:
            rospy.logwarn("No se detectó la parada del robot antes del timeout. Terminamos.")
            break

        i += 1
        rospy.sleep(10)  # pausa pequeña opcional

    rospy.loginfo("Secuencia completada. Saliendo del nodo.")

if __name__ == "__main__":
    main()
