#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String, Bool
from cobot_controladores.msg import msg_web
import subprocess
import signal
import os


class Control_Launch:
    def __init__(self):
        rospy.init_node('launch_controller', anonymous=False)

        # Ruta al launch principal
        self.launch_file = '/home/dobot/catkin_ws/src/cobot/cobot_controladores/launch/ini_rob_colaborativo.launch'

        # Proceso del launch
        self.launch_process = None

        # Publisher hacia la web
        self.informe_pub = rospy.Publisher('informe_web',String,queue_size=10)

        # Subscriber desde la web
        rospy.Subscriber(
            'control_launch',
            Bool,
            self.control_launch_callback
        )

        rospy.loginfo("Nodo Control_Launch inicializado correctamente")
        
    def publicar_informe(self, mensaje: str, tipo: int):
        msg = msg_web()
        msg.mensaje = mensaje
        msg.tipo = tipo
        self.informe_pub.publish(msg)

    def control_launch_callback(self, msg):
        if msg.data:
            self.start_launch()
        else:
            self.stop_launch()

    def start_launch(self):
        if self.launch_process is None:
            rospy.loginfo("Iniciando el programa principal")

            self.launch_process = subprocess.Popen(
                ['gnome-terminal', '--disable-factory', '--', 'roslaunch', 'cobot_controladores', 'ini_rob_colaborativo.launch'], 
                preexec_fn=os.setpgrp # Crea un nuevo grupo de procesos
            )

            publicar_informe("Programa inicializado",0)

        else:
            publicar_informe("Programa ya estaba en ejecución",-1)

    def stop_launch(self):
        if self.launch_process is not None:
            rospy.loginfo("Finalizando el programa principal")

            os.killpg(self.launch_process.pid, signal.SIGINT)
            self.launch_process.wait()
            self.launch_process = None

            publicar_informe("Programa finalizado",0)

        else:
            publicar_informe("Programa no estaba iniciado",-1)


if __name__ == '__main__':
    try:
        control_launch = Control_Launch()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


