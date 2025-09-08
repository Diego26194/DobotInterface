#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import actionlib
from niryo_controladores.msg import actionSAction, actionSFeedback, actionSResult
import sys
from moveit_commander import MoveGroupCommander, roscpp_initialize
from sensor_msgs.msg import JointState

class EjecucionTrayectoriaServer:
    def __init__(self):
        roscpp_initialize(sys.argv)  # Inicializa MoveIt
        rospy.init_node('ejecutar_trayectoria_server')
        
        self.server = actionlib.SimpleActionServer('ejecutar_trayectoria_server', actionSAction, self.execute_callback, False)
        self.move_group = MoveGroupCommander("niryo_arm")
        
        
        self.server.start()

    def execute_callback(self, goal):
    	
        try:
        
            # Aquí puedes iniciar la publicación de feedback si es necesario
            # esta publicacion se puede poner en cualquier parte del proceso para enviar un msje al cliente de como se abansa
            feedback = actionSFeedback()
            feedback.is_running = True
            self.server.publish_feedback(feedback)  # si quiero enviar un feeedback ponerlo dentro del parentesis

            # Ejecutar la trayectoria planificada
            self.move_group.execute(goal.traj, wait=True)  # Cambiar a wait=False si es necesario
            
            self.server.set_succeeded()   #esto me indica que el proceso termino
        except rospy.ROSException as e:
            rospy.logerr("Error al ejecutar la trayectoria: {}".format(e))
            self.server.set_aborted()

if __name__ == '__main__':
    server = EjecucionTrayectoriaServer()
    rospy.spin()
