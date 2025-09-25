#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Int16MultiArray
from moveit_msgs.msg import DisplayTrajectory
import numpy as np
import time

PAUSA_ENTRE_TRAYECTORIAS = 1.0  # segundos

def radianes_a_bits(rad):
    return [int((r + np.pi) * 4095 / (2 * np.pi)) for r in rad]

def ejecutar_trayectoria(joint_trajectory):
    puntos = joint_trajectory.points
    t0 = time.time()
    for punto in puntos:
        posiciones_bits = radianes_a_bits(punto.positions)
        msg = Int16MultiArray(data=posiciones_bits)
        pub_cord_dy.publish(msg)

        t_obj = punto.time_from_start.to_sec()
        while (time.time() - t0) < t_obj:
            time.sleep(0.001)

def callback_display_trajectory(msg):
    rospy.loginfo("Recibida trayectoria planificada. Ejecutando...")
    for idx, trajectory in enumerate(msg.trajectory):
        ejecutar_trayectoria(trajectory.joint_trajectory)
        if idx < len(msg.trajectory) - 1:
            rospy.loginfo(f"Pausa de {PAUSA_ENTRE_TRAYECTORIAS} segundos entre trayectorias")
            time.sleep(PAUSA_ENTRE_TRAYECTORIAS)

if __name__ == '__main__':
    rospy.init_node('emulador_joint_trajectory_controller')
    pub_cord_dy = rospy.Publisher('cord_dy', Int16MultiArray, queue_size=10)
    rospy.Subscriber('/move_group/display_planned_path', DisplayTrajectory, callback_display_trajectory)
    rospy.loginfo("Nodo emulador_joint_trajectory_controller listo y esperando trayectoria...")
    rospy.spin()