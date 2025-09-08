#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import actionlib
from niryo_controladores.msg import actionSAction, actionSFeedback, actionSResult
import sys
import numpy as np
from moveit_commander import MoveGroupCommander, roscpp_initialize
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16MultiArray
import time

class EjecucionTrayectoriaServer:
    def __init__(self):
        
        roscpp_initialize(sys.argv)  # Inicializa MoveIt
        rospy.init_node('ejecutar_trayectoria_server')
        
        self.server = actionlib.SimpleActionServer('ejecutar_trayectoria_server', actionSAction, self.execute_callback, False)
        self.move_group = MoveGroupCommander("niryo_arm")
        
        # Publicar en el tópico de coordenadas Dynamixel
        self.cord_dy_pub = rospy.Publisher('cord_dy', Int16MultiArray, queue_size=10)
        
        # Publicar en el tópico de coordenadas Dynamixel
        self.cord_dy_pub2 = rospy.Publisher('ErroresRos', Int16MultiArray, queue_size=10)
        
        self.server.start()
        
    def rad_bit(self, rad):
        bit = [(int((r + np.pi) * 4095 / (2 * np.pi))) for r in rad]
        return np.int16(bit)

    def execute_callback(self, goal):
        try:
            # Aquí puedes iniciar la publicación de feedback si es necesario
            # esta publicacion se puede poner en cualquier parte del proceso para enviar un msje al cliente de como se abansa
            feedback = actionSFeedback()
            feedback.is_running = True
            self.server.publish_feedback(feedback)  # si quiero enviar un feeedback ponerlo dentro del parentesis

            # Ejecutar la trayectoria planificada
            self.move_group.execute(goal.traj, wait=True)  # Cambiar a wait=False si es necesario


            start_time = time.time()
        
            for punto in goal.traj.joint_trajectory.points:
                # Convertir posiciones a bits y enviar
                posiciones_bits = self.rad_bit(punto.positions)
                msg = Int16MultiArray(data=posiciones_bits)
                self.cord_dy_pub.publish(msg)

                # Esperar hasta que sea tiempo de enviar el siguiente punto
                tiempo_objetivo = punto.time_from_start.to_sec()
                while time.time() - start_time < tiempo_objetivo:
                    time.sleep(0.001)
    
    
            # Calcular la cantidad de puntos
            cantidad_puntos = len(goal.traj.joint_trajectory.points)

            # Crear el mensaje con esa cantidad en una lista
            msg2 = Int16MultiArray(data=[cantidad_puntos])
            
            self.cord_dy_pub2.publish(msg2)
            
            
            self.server.set_succeeded()   #esto me indica que el proceso termino
        except rospy.ROSException as e:
            rospy.logerr("Error al ejecutar la trayectoria: {}".format(e))
            self.server.set_aborted()

if __name__ == '__main__':
    server = EjecucionTrayectoriaServer()
    rospy.spin()
