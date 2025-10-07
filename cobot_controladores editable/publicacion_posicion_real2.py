#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int16MultiArray, Header
from sensor_msgs.msg import JointState
import numpy as np

class PublicacionPosicionReal:
    def __init__(self):
        # Inicializa el nodo
        rospy.init_node('publicacion_posicion_real', anonymous=True)

        # Nombres de las articulaciones (ajustalos a los tuyos)
        self.joint_names = [
            "joint_1", "joint_2", "joint_3", 
            "joint_4", "joint_5", "joint_6"
        ]

        # Publicador a joint_states
        self.joint_pub = rospy.Publisher('joint_states', JointState, queue_size=10)

        # Suscriptor al tópico que recibe los valores de Arduino
        rospy.Subscriber('pos_dy', Int16MultiArray, self.callback_pos_dy)

        rospy.loginfo("Nodo 'publicacion_posicion_real' iniciado.")        
        
        self.positionPos_dy = [2090, 2090, 2090, 2090, 2090, 2090]        
        while self.joint_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.1)

        # Ahora sí, publicar la posición inicial
        self.publicar_joint_states(self.positionPos_dy)       
        
        rospy.spin()

    def bit_rad(self, bit):
        rad = [((b * (2 * np.pi) / 4095) - np.pi) for b in bit]
        return np.array(rad, dtype=np.float64)

    def callback_pos_dy(self, msg):
        # Validación
        if not msg.data or len(msg.data) != 6:
            rospy.logwarn("Mensaje vacío recibido en pos_dy.")
            return
        
        for i in range(6):
            if msg.data[i] != -1:
                self.positionPos_dy[i] = msg.data[i]
            #VERRIFICAR si esto es necesario, el indicativo tal ves solo deberia ir en la interfaz
            else:
                rospy.loginfo("Motor %s desconectado o error", i+1)
        
        self.publicar_joint_states(self.positionPos_dy)             
        
    def publicar_joint_states(self, positionPos_dy):  
        # Conversión bit -> radianes     
        posiciones_rad = self.bit_rad(positionPos_dy)

        # Emparejar con nombres de articulaciones (por si hay menos)
        nombres = self.joint_names[:len(posiciones_rad)]

        # Crear y publicar el mensaje JointState
        joint_msg = JointState()
        joint_msg.header = Header()
        joint_msg.header.stamp = rospy.Time.now()
        joint_msg.name = nombres
        joint_msg.position = posiciones_rad

        self.joint_pub.publish(joint_msg)
        rospy.logdebug("Publicada posición real en joint_states: %s", posiciones_rad)

if __name__ == '__main__':
    try:
        PublicacionPosicionReal()
    except rospy.ROSInterruptException:
        pass
