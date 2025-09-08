#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
import subprocess
import signal
import os

# Variable para almacenar el proceso del launch
launch_process = None
launch_file = '/home/dobot_labme/catkin_ws/src/cobot_niryo/niryo_controladores/launch/ini_rob_colaborativo.launch'

def control_launch_callback(msg):
    global launch_process

    if msg.data == 'prender':
        if launch_process is None:  # Solo iniciar si no está corriendo
            rospy.loginfo(f"Iniciando el lanzamiento: {launch_file}")
            # Lanza el archivo launch en una nueva terminal
            launch_process = subprocess.Popen(
                ['gnome-terminal', '--disable-factory', '--', 'roslaunch', 'niryo_controladores', 'ini_rob_colaborativo.launch'], 
                preexec_fn=os.setpgrp # Crea un nuevo grupo de procesos
            )
            rospy.loginfo("Launch iniciado correctamente.")
        else:
            rospy.loginfo("El launch ya está corriendo.")
    
    elif msg.data == 'apagar':
        if launch_process is not None:  # Solo detener si está corriendo
            rospy.loginfo("Deteniendo el proceso del launch...")
            os.killpg(launch_process.pid, signal.SIGINT)   # Termina el grupo de procesos
            launch_process.wait()  # Espera a que termine
            launch_process = None
            rospy.loginfo("Launch detenido y ventana cerrada.")
        else:
            rospy.loginfo("No hay un launch corriendo para detener.")

def launch_controller():
    rospy.init_node('launch_controller', anonymous=True)
    rospy.Subscriber('/control_launch', String, control_launch_callback)
    
    rospy.loginfo("Nodo de control de lanzamiento iniciado.")
    rospy.spin()

if __name__ == '__main__':
    try:
        launch_controller()
    except rospy.ROSInterruptException:
        pass

