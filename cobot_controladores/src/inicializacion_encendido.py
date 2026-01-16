#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String, Bool
import subprocess
import signal
import os


class Control_Launch:
    def __init__(self):
        rospy.init_node('launch_controller', anonymous=False)

        # Ruta al launch principal
        self.launch_file = (
            '/home/dobot/catkin_ws/src/cobot_niryo/'
            'niryo_controladores/launch/ini_rob_colaborativo.launch'
        )

        # Proceso del launch
        self.launch_process = None

        # Publisher hacia la web
        self.informe_pub = rospy.Publisher(
            'informe_web',
            String,
            queue_size=10
        )

        # Subscriber desde la web
        rospy.Subscriber(
            'control_launch',
            Bool,
            self.control_launch_callback
        )

        rospy.loginfo("Nodo Control_Launch inicializado correctamente")

    def control_launch_callback(self, msg):
        if msg.data:
            self.start_launch()
        else:
            self.stop_launch()

    def start_launch(self):
        if self.launch_process is None:
            rospy.loginfo("Iniciando el programa principal")

            self.launch_process = subprocess.Popen(
                ['roslaunch', 'niryo_controladores', 'ini_rob_colaborativo.launch'],
                preexec_fn=os.setpgrp
            )

            self.informe_pub.publish("Programa inicializado")

        else:
            self.informe_pub.publish("Programa ya estaba en ejecución")

    def stop_launch(self):
        if self.launch_process is not None:
            rospy.loginfo("Finalizando el programa principal")

            os.killpg(self.launch_process.pid, signal.SIGINT)
            self.launch_process.wait()
            self.launch_process = None

            self.informe_pub.publish("Programa finalizado")

        else:
            self.informe_pub.publish("Programa no estaba iniciado")


if __name__ == '__main__':
    try:
        control_launch = Control_Launch()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


