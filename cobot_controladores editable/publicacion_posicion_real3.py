#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int16MultiArray, Header
from sensor_msgs.msg import JointState
from moveit_commander import RobotCommander, MoveGroupCommander
import numpy as np
import threading

from Normalizacion_Robot import NormalizacionRobot

norm = NormalizacionRobot()
class PublicacionPosicionReal:
    def __init__(self):
        rospy.init_node('publicacion_posicion_real', anonymous=True)

        self.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        self.joint_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        rospy.Subscriber('pos_dy', Int16MultiArray, self.callback_pos_dy)

        self.positionPos_dy = [2090, 2090, 2090, 2090, 2090, 2090]
        self.lock = threading.Lock()  # para acceso seguro desde ambos hilos

        # Hilo de publicación constante (10 Hz)
        rospy.Timer(rospy.Duration(0.1), self.timer_callback)

        rospy.loginfo("Nodo 'publicacion_posicion_real' iniciado.")
        rospy.spin()
    
    def callback_pos_dy(self, msg):
        if not msg.data or len(msg.data) != 6:
            rospy.logwarn("Mensaje vacío recibido en pos_dy.")
            return

        with self.lock:
            for i in range(6):
                if msg.data[i] != -1 or msg.data[i] != 0:
                    self.positionPos_dy[i] = msg.data[i]

    def timer_callback(self, event):
        with self.lock:
            self.publicar_joint_states(self.positionPos_dy)

    def publicar_joint_states(self, positionPos_dy):
        posiciones_rad = norm.bit_rad(positionPos_dy)
        joint_msg = JointState()
        joint_msg.header.stamp = rospy.Time.now()
        joint_msg.name = self.joint_names
        joint_msg.position = posiciones_rad
        self.joint_pub.publish(joint_msg)
if __name__ == '__main__':
    try:
        PublicacionPosicionReal()
    except rospy.ROSInterruptException:
        pass
