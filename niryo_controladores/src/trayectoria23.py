#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String, Bool, Int16MultiArray, Float64MultiArray
from moveit_commander import MoveGroupCommander, roscpp_initialize, RobotCommander
from geometry_msgs.msg import Pose, Point
from moveit_msgs.msg import RobotTrajectory, RobotState, MotionSequenceRequest, MotionSequenceItem, MotionPlanRequest, Constraints
from sensor_msgs.msg import JointState
from db_puntos import leer_datos
import numpy as np
import sys

class ControladorRobot:
    def __init__(self):
    
        self.contador_joint_states = 0
    
        rospy.init_node('modo_control')
        
        # Inicializar MoveGroupCommander para el grupo de articulaciones del robot
        self.move_group = MoveGroupCommander("niryo_arm")
        
    
        # Suscribirse al tópico pos_dy para obtener la realimentación de la posición de los motores
        rospy.Subscriber('pos_dy', Int16MultiArray, self.actualizar_estado_robot)
        
        # Publicar en el tópico de coordenadas Dynamixel
        self.cord_dy_pub = rospy.Publisher('cord_dy', Int16MultiArray, queue_size=10)

        # Suscribirse al tópico "rutina" para recibir señales de inicio y fin de la rutina
        rospy.Subscriber('rutina', Bool, self.ejecutar_rutina)

        self.rutina_pub = rospy.Publisher('rutina', Bool, queue_size=10)

        # Suscribirse al tópico joint_states para controlar la publicación en cord_dy
        rospy.Subscriber('joint_states', JointState, self.controlar_publicacion_cord_dy)

        # Estado actual del robot
        self.estado_actual = None
        
        # Posición de "home" predefinida
        self.home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Inicializar el robot en la posición de "home"
        self.move_group.set_joint_value_target(self.home_position)
        self.move_group.go()

        # Variables de control
        self.ejecutando_rutina = False
        self.rutina_corriendo = False


        robot = RobotCommander()


    def rad_bit(self, rad):
        bit = [(int((r + np.pi) * 4095 / (2 * np.pi))) for r in rad]
        return np.int16(bit)

    def grados_rad(self, grad):
        rad = [g * np.pi / 180 for g in grad]
        return rad
    
   
    
    
    
    
    def ejecutar_trayectoria(self, joint_angles):
        try:
            # Activar la bandera rutina_corriendo
            self.rutina_corriendo = True



            self.move_group.set_planner_id("PTP")  
            
            # Convertir de bits a radianes antes de pasar a MoveGroupCommander
            self.move_group.set_joint_value_target(joint_angles)
        
            # Obtener la trayectoria planificada
            success,traj,planning_time,error_code = self.move_group.plan()
            
            #traj = PlanComponentsBuilder.create_ptp()
            #traj = PlanComponentsBuilder.create_lin()
            #radius=0.5
            #traj = PlanComponentsBuilder.create_circ(radius, circ_center)
            
            
            
            
            # Ejecutar la trayectoria planificada
            self.move_group.execute(traj, wait=False)
            #self.move_group.execute(traj)

        except rospy.ROSException as e:
            rospy.logerr("Error al ejecutar la trayectoria: {}".format(e))
            self.ejecutando_rutina = False

        # Desactivar la bandera rutina_corriendo cuando termina
        self.rutina_corriendo = False
    
    def ejecutar_rutina(self, data):
        F = Bool(False)
        if data.data and not self.ejecutando_rutina:
            rospy.loginfo("Iniciando rutina...")
            self.ejecutando_rutina = True
            self.rutina_pub.publish(F)


            waypoints = [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Punto 1
            [90, 15, 0, 0, 0, 0],  # Punto 2
            [130, 30, 0.0, 0, 0, 0],  # Punto 3
            [90, -30, -0, 0, 0, 0],  # Punto 4
            [-90, -50, 0, 0.0, 0, 0.0]   # Punto 5
            ]

            # Crea la solicitud de secuencia
            sequence_request = MotionSequenceRequest()
            
            # Agrega los puntos a la solicitud de secuencia
            for angles in waypoints:
                # Crea un nuevo MotionSequenceItem para cada waypoint
                sequence_item = MotionSequenceItem()

                # Configura el MotionPlanRequest del item
                sequence_item.req = MotionPlanRequest()
                sequence_item.req.group_name = "niryo_arm"  # Nombre del grupo del robot

                # Crea un estado de inicio que refleje la posición actual de las articulaciones
                goal_state = RobotState()
                goal_state.joint_state.position = self.grados_rad(angles)  # Convertimos los ángulos a radianes

                # Establece las restricciones de destino (goal constraints) basadas en la posición deseada
                goal_constraints = Constraints()
                # Aquí podrías definir restricciones específicas, como por ejemplo restricciones de posición o de orientación.

                sequence_item.req.goal_constraints.append(goal_constraints)

                # Configura el estado inicial del robot para el plan
                sequence_item.req.start_state = self.move_group.get_current_state()

                # Establece el radio de mezcla (blend_radius) para suavizar la transición entre puntos
                sequence_item.blend_radius = 0.05

                # Añade este item a la secuencia
                sequence_request.items.append(sequence_item)


            # Envía la solicitud de secuencia
            splan=self.move_group.plan(sequence_request)

            # Ejecuta la secuencia
            self.move_group.execute(splan)

            
             




        
            self.ejecutando_rutina = False

    def actualizar_estado_robot(self, data):
        # Actualizar el estado actual del robot con los datos recibidos
        self.estado_actual = data.data

    def controlar_publicacion_cord_dy(self, data):
        # Solo publicar si la rutina está corriendo
        self.contador_joint_states += 1
        # Publicar uno de cada dos mensajes recibidos
        if self.contador_joint_states % 1 == 0:
            joint_state_bits = self.rad_bit(data.position)
            joint_state_msg = Int16MultiArray(data=joint_state_bits)
            self.cord_dy_pub.publish(joint_state_msg)

    def ejecutar_cord_ros(self, data):
        
        #rospy.loginfo("Tipo de data.data: {}".format(type(data.data)))  # Esto imprimirá el tipo de data.data
        
        F = Bool(False)
        if not self.ejecutando_rutina:
            self.ejecutando_rutina = True
            self.rutina_pub.publish(F)
            try:
                # Obtener los datos de cord_ros
                joint_angles = self.grados_rad(list(data.data)) 
                
                
                
                # Ejecutar la trayectoria con los datos de cord_ros
                self.ejecutar_trayectoria(joint_angles)
                rospy.loginfo("Ejecutando trayectoria desde cord_ros: {}".format(joint_angles))
                self.ejecutando_rutina = False
                
            except Exception as e:
                rospy.logerr("Error al ejecutar trayectoria desde cord_ros: {}".format(e))
                self.ejecutando_rutina = False
      
if __name__ == '__main__':
    try:
        roscpp_initialize(sys.argv)
        controlador = ControladorRobot()
        
        # Suscribirse al tópico cord_ros
        rospy.Subscriber('cord_ros', Float64MultiArray, controlador.ejecutar_cord_ros)
        
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Se ha interrumpido la ejecución del nodo.")
