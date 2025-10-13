#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Int16MultiArray
from moveit_msgs.msg import MoveGroupActionResult
import numpy as np
import time

PAUSA_ENTRE_TRAYECTORIAS = 1.0  # segundos

def radianes_a_bits(rad):
    return [int((r + np.pi) * 4095 / (2 * np.pi)) for r in rad]

def ejecutar_trayectoria(joint_trajectory):
    puntos = joint_trajectory.points
    if not puntos:
        rospy.logwarn("⚠️ Trayectoria vacía, no se ejecutará.")
        return

    t0 = time.time()
    for punto in puntos:
        posiciones_bits = radianes_a_bits(punto.positions)
        msg = Int16MultiArray(data=posiciones_bits)
        pub_cord_dy.publish(msg)

        t_obj = punto.time_from_start.to_sec()
        while (time.time() - t0) < t_obj:
            time.sleep(0.001)

def callback_movegroup_result(msg):
    error = msg.result.error_code.val

    if error != 1:
        rospy.logerr(f"❌ Error en planificación o ejecución (código {error}). Plan descartado.")
        return

    rospy.loginfo("✅ Plan recibido correctamente. Ejecutando...")
    planned_traj = msg.result.planned_trajectory.joint_trajectory

    ejecutar_trayectoria(planned_traj)
    rospy.loginfo("✅ Trayectoria finalizada correctamente.")

if __name__ == '__main__':
    rospy.init_node('emulador_joint_trajectory_controller')
    pub_cord_dy = rospy.Publisher('cord_dy', Int16MultiArray, queue_size=10)

    rospy.Subscriber('/move_group/result', MoveGroupActionResult, callback_movegroup_result)

    rospy.loginfo("Nodo listo y esperando resultados de MoveIt (/move_group/result)...")
    rospy.spin()
