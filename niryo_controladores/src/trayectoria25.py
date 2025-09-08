#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String, Bool, Int16MultiArray, Float64MultiArray
from moveit_commander import MoveGroupCommander, roscpp_initialize, RobotCommander
from geometry_msgs.msg import Pose, Point
from moveit_msgs.msg import RobotTrajectory, RobotState, MotionSequenceRequest, MotionSequenceItem, MotionPlanRequest,MoveGroupSequenceAction, Constraints, MoveGroupSequenceGoal, GenericTrajectory, Constraints, JointConstraint
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from db_puntos import leer_datos
import numpy as np
import sys
from actionlib import SimpleActionClient
from moveit_msgs.srv import GetMotionPlan

class ControladorRobot:
    def __init__(self):
    
        self.contador_joint_states = 0
    
        rospy.init_node('modo_control')
        
        # Inicializar MoveGroupCommander para el grupo de articulaciones del robot
        self.move_group = MoveGroupCommander("niryo_arm")
        self.move_group.set_planner_id("PTP") 
        
    
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

        # Crear cliente de acción para ejecutar la secuencia de movimiento
        rospy.loginfo("a")
        self.sequence_action_client = SimpleActionClient("/sequence_move_group", MoveGroupSequenceAction)
        rospy.loginfo("b")
        self.sequence_action_client.wait_for_server()
        rospy.loginfo("c")


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
            [90, 0, -0, 0, 0, 0],  # Punto 4
            [-90, 0, 0, 0.0, 0, 0.0]   # Punto 5
            ]

            # Crea la solicitud de secuencia
            sequence_request = MotionSequenceRequest()
            
            # Agrega los puntos a la solicitud de secuencia
            for angles in waypoints:
                item = MotionSequenceItem()
                traj = JointTrajectory()
                traj.joint_names = self.move_group.get_active_joints()
                point = JointTrajectoryPoint()
    #////////comienzo de cambio////////
    
    # Convierte los ángulos a radianes
                point.positions = self.grados_rad(angles)
    
    # Configura la velocidad y aceleración
                point.velocities = [2.0] 
                point.accelerations = [2.0] 
    
    # Calcula el tiempo en base al ángulo que más se mueve
                #tiempo_max = max([abs(ang) for ang in angles]) / 2.0  # Velocidad máxima 2 rad/s
                #point.time_from_start = rospy.Duration(tiempo_max)
    
    # Añadir el estado inicial (posiciones actuales) para el primer punto
                if not sequence_request.items:  # Si es el primer punto de la secuencia
                    current_state = self.move_group.get_current_joint_values()
                    point.positions = current_state
    
    #////////fin de cambio////////
                point.positions = self.grados_rad(angles)
                point.time_from_start = rospy.Duration(1)  # Tiempo para alcanzar el punto
                traj.points.append(point)
                
                generic_traj = GenericTrajectory()
                generic_traj.joint_trajectory.append(traj)
                
                #///////// añadir restricciones////////////
                constraint = Constraints()
                joint_constraints = []
                for i, joint_name in enumerate(traj.joint_names):
                    joint_constraint = JointConstraint()
                    joint_constraint.joint_name= joint_name
                    joint_constraint.position = point.positions[i]
                    joint_constraint.tolerance_below = 0.01
                    joint_constraint.tolerance_above = 0.01
                    joint_constraints.append(joint_constraint)
                # Añadir el constraint a la secuencia
                constraint.joint_constraints = joint_constraints
                item.req.goal_constraints.append(constraint)
                
    
                item.req.reference_trajectories.append(generic_traj)
                item.req.planner_id="PTP"
                item.req.group_name="niryo_arm"
                item.req.max_velocity_scaling_factor=1.0
                item.req.max_acceleration_scaling_factor=1.0


                item.blend_radius = 0.0  # Sin fusión entre puntos

                # Añade este item a la secuencia
                sequence_request.items.append(item)


            # Crear la meta para el cliente de acción
            
            goal = MoveGroupSequenceGoal()
            rospy.loginfo("1")
            #rospy.loginfo("Solicitud de secuencia: %s", sequence_request)
            
            # Asegúrate de que el cliente de acción está conectado al servidor
            if not self.sequence_action_client.wait_for_server(timeout=rospy.Duration(5)):
                rospy.loginfo("El servidor de acción no está disponible.")
                return
            
            goal.request = sequence_request
            rospy.loginfo("2")
            # Cancela metas previas y envía una nueva meta
            self.sequence_action_client.cancel_all_goals()
            self.sequence_action_client.send_goal(goal)
            rospy.loginfo("3")
            # Espera el resultado
            self.sequence_action_client.wait_for_result()
            rospy.loginfo("4")
            # Imprime el estado del resultado
            result = self.sequence_action_client.get_result()
            
            
            #if result is None:
            #    rospy.loginfo("No se recibió ningún resultado de la acción.")
            #else:
            #    rospy.loginfo("Resultado de la acción: %s", result)
                
                
            rospy.loginfo("5")

            
             




        
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
