#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String, Bool, Int16MultiArray, Float64MultiArray
from moveit_commander import MoveGroupCommander, roscpp_initialize
from moveit_msgs.msg import RobotTrajectory, RobotState
from sensor_msgs.msg import JointState
from db_puntos import leer_datos
import numpy as np
import sys

class ControladorRobot:
    def __init__(self):
        rospy.init_node('modo_control')
        
        # Inicializar MoveGroupCommander para el grupo de articulaciones del robot
        self.move_group = MoveGroupCommander("prueba_arm")
        
        # Suscribirse al tópico pos_dy para obtener la realimentación de la posición de los motores
        rospy.Subscriber('pos_dy', Int16MultiArray, self.actualizar_estado_robot)
        
        # Publicar en el tópico de coordenadas Dynamixel
        self.cord_dy_pub = rospy.Publisher('cord_dy', Int16MultiArray, queue_size=10)

        # Suscribirse al tópico "rutina" para recibir señales de inicio y fin de la rutina
        rospy.Subscriber('rutina', Bool, self.ejecutar_rutina)

        self.rutina_pub = rospy.Publisher('rutina', Bool, queue_size=10)

        # Estado actual del robot
        self.estado_actual = None
        
         # Posición de "home" predefinida
        self.home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Inicializar el robot en la posición de "home"
        self.move_group.set_joint_value_target(self.home_position)
        self.move_group.go()

        # Variable para controlar si se está ejecutando una rutina
        self.ejecutando_rutina = False

    def rad_bit(self, rad):
        bit = [(int((r + np.pi) * 4095 / (2 * np.pi))) for r in rad]
        return np.int16(bit)

    #def bit_rad(self, bit):
    #    rad = [(b * 2 * np.pi / 4095 - np.pi) for b in bit]
    #    return rad
        
    #def rad_grados(self, rad):
    #    grad = [(rad  * 180 / (np.pi))]
    #    return grad

    def grados_rad(self, grad):
        rad = [g * np.pi / 180 for g in grad]
        return rad
    
    def ejecutar_trayectoria(self, joint_angles):
        try:
            # Convertir de bits a radianes antes de pasar a MoveGroupCommander
            #joint_angles_rad = self.bit_rad(joint_angles)
            self.move_group.set_joint_value_target(joint_angles)
        
            # Obtener la trayectoria planificada
            traj = self.move_group.plan()
            waypoints = traj.joint_trajectory.points
        
            # Publicar cada punto de la trayectoria en cord_dy
            for point in waypoints:
                joint_state_bits = self.rad_bit(point.positions)
                joint_state_msg = Int16MultiArray(data=joint_state_bits)
                self.cord_dy_pub.publish(joint_state_msg)
                rospy.sleep(0.13)  # Esperar un breve tiempo entre cada punto
            
        except rospy.ROSException as e:
            rospy.logerr("Error al ejecutar la trayectoria: {}".format(e))
            self.ejecutando_rutina = False

    
    def ejecutar_rutina(self, data):
        # Si la rutina se activa, ejecutarla
        F=Bool(False)
        if data.data and not self.ejecutando_rutina :
            rospy.loginfo("Iniciando rutina...")
            self.ejecutando_rutina = True
            
            self.rutina_pub.publish(F)
             
            # Obtener puntos de la base de datos y ejecutarlos uno por uno
            puntos = leer_datos()
            for punto in puntos:
                try:
                    # Crear un mensaje Float64MultiArray con las coordenadas del punto
                    joint_angles = punto['x'], punto['y'], punto['z'], punto['a'], punto['b'], punto['c']

                    #transformar datos de grados a radianes

                    joint_angles_grados=self.grados_rad(joint_angles)
                    
                    # Ejecutar la trayectoria para el punto actual
                    self.ejecutar_trayectoria(joint_angles_grados)
                    rospy.loginfo("Ejecutando punto de la base de datos: {}".format(punto))
                    
                except KeyError as e:
                    rospy.logerr("Error al obtener los datos del punto: {}".format(e))
                    self.ejecutando_rutina = bool(False)
            
            # Finalizar la rutina y publicar en el tópico "rutina"
            rospy.loginfo("Rutina finalizada.")
            self.ejecutando_rutina = bool(False)

    def actualizar_estado_robot(self, data):
        # Actualizar el estado actual del robot con los datos recibidos
        self.estado_actual = data.data

    def ejecutar_cord_ros(self, data):
        # Si se recibe un mensaje de cord_ros, ejecutar la trayectoria con esos datos
        F=Bool(False)
        if not self.ejecutando_rutina :
            
            self.ejecutando_rutina = bool(True)
            self.rutina_pub.publish(F)
            try:
                # Obtener los datos de cord_ros
                joint_angles =self.grados_rad (data.data)
                
                # Ejecutar la trayectoria con los datos de cord_ros
                self.ejecutar_trayectoria(joint_angles)
                rospy.loginfo("Ejecutando trayectoria desde cord_ros: {}".format(joint_angles))
                self.ejecutando_rutina = bool(False)
                
            except Exception as e:
                rospy.logerr("Error al ejecutar trayectoria desde cord_ros: {}".format(e))
                self.ejecutando_rutina = bool(False)


                
    #def publicar_rutina(self, estado):
        # Publicar en el tópico "rutina" para indicar que la rutina ha finalizado
    #    self.ejecutando_rutina = estado
    #    self.rutina_pub.publish(Bool(estado))
      
if __name__ == '__main__':
    try:
        roscpp_initialize(sys.argv)
        controlador = ControladorRobot()
        
        # Suscribirse al tópico cord_ros
        rospy.Subscriber('cord_ros', Float64MultiArray, controlador.ejecutar_cord_ros)
        
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Se ha interrumpido la ejecución del nodo.")
