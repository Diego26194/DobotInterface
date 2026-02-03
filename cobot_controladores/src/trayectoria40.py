#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String, Bool, Int16, Int16MultiArray, Float64MultiArray
from moveit_commander import MoveGroupCommander, roscpp_initialize, RobotCommander
from geometry_msgs.msg import Pose, Point, PoseStamped, Point, Quaternion
from moveit_msgs.msg import RobotTrajectory, RobotState, MotionSequenceRequest, MotionSequenceItem, MotionPlanRequest, MoveGroupSequenceAction, Constraints, MoveGroupSequenceGoal, GenericTrajectory, JointConstraint, DisplayTrajectory
from moveit_msgs.msg import PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from db_puntos6 import  leer_rutina_sin_euler, obtener_rutina, leer_punto_rutina
import numpy as np
import sys
from actionlib import SimpleActionClient 
from moveit_msgs.srv import GetMotionPlan
from cobot_controladores.msg import actionSAction, actionSGoal, actionSResult, punto_correr, waits, trayectorias


# Bounding volume
from shape_msgs.msg import SolidPrimitive, Mesh, MeshTriangle
from geometry_msgs.msg import PointStamped
# Construir bounding volume simple (usaremos solo el solid primitive)
from moveit_msgs.msg import BoundingVolume

from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionFK, GetPositionFKRequest

from Normalizacion_Robot import NormalizacionRobot

norm = NormalizacionRobot()

class ControladorRobot:
    def __init__(self):
        rospy.init_node('modo_control')

        # Inicializar MoveGroupCommander para el grupo de articulaciones del robot
        self.move_group = MoveGroupCommander("cobot_arm")
        self.move_group.set_planner_id("PTP")

        # Suscribirse al tópico pos_dy para obtener la realimentación de la posición de los motores
        rospy.Subscriber('pos_dy', Int16MultiArray, self.actualizar_estado_robot)

        # Publicar en el tópico de coordenadas Dynamixel
        self.cord_dy_pub = rospy.Publisher('cord_dy', Int16MultiArray, queue_size=10)

        # Suscribirse al tópico "rutina" para recibir señales de inicio y fin de la rutina
        rospy.Subscriber('rutina', Bool, self.Ordenar_ejec_rutina)

        self.rutina_pub = rospy.Publisher('rutina', Bool, queue_size=10)

        # Publicar en el tópico de coordenadas Dynamixel alternativo
        self.cord_dy_pub2 = rospy.Publisher('cord_dy2', Int16MultiArray, queue_size=10)
        
        # Publicar en joint_states
        self.joint_pub = rospy.Publisher('joint_states', JointState, queue_size=10) 
        self.pos_dy_pub = rospy.Publisher('pos_dy', Int16MultiArray, queue_size=10) 

        # NUEVOS TOPICOS PARA COMUNICACION CON PLANNER
        self.trayectoria_pub = rospy.Publisher('trayectoria_A_planificacion', Int16, queue_size=10)
        self.rutine_waits = rospy.Publisher('rutine_waits', waits, queue_size=10)
        self.plan_sub = rospy.Subscriber('planificacion_A_trayectoria', Int16, self._planificacion_callback)
        self.rutine_trayectorias = rospy.Publisher('rutine_trayectorias', trayectorias, queue_size=10)

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

        # Variables para comunicacion planner <-> trayectoria
        self._last_plan_msg = None
        self._plan_event = False

        # Cliente action server para secuencia MoveGroup (Pilz)
        rospy.loginfo("Inicializando cliente de secuencia...")
        self.sequence_action_client = SimpleActionClient("/sequence_move_group", MoveGroupSequenceAction)
        self.sequence_action_client.wait_for_server()
        rospy.loginfo("Cliente de secuencia conectado.")

        # ACTION SERVER para enviar trayectorias al controlador real (si lo usas)
        self.client = SimpleActionClient('ejecutar_trayectoria_server', actionSAction)
        rospy.loginfo("Esperando a que el servidor de acciones esté disponible...")
        self.client.wait_for_server()
        rospy.loginfo("Servidor de acciones disponible, listo para enviar goals")

        rospy.Subscriber('/move_group/display_planned_path', DisplayTrajectory, self.anular_ejecucion)
        self.planear_Rutina = False
        
        ## Trayectoria        
        
        self.posicion_Trayectorias=[]
        self.rutina_Trayectorias=[] 
    
    def dict_to_pose(self,pose_dict):
        
        pose = Pose()
        pose.position = Point(
            x=pose_dict["position"]["x"],
            y=pose_dict["position"]["y"],
            z=pose_dict["position"]["z"]
        )
        pose.orientation = Quaternion(
            x=pose_dict["orientation"]["x"],
            y=pose_dict["orientation"]["y"],
            z=pose_dict["orientation"]["z"],
            w=pose_dict["orientation"]["w"]
        )
        return pose

    def anular_ejecucion(self, msg):
        if self.planear_Rutina:
            try:
                if self.sequence_action_client.wait_for_server(timeout=rospy.Duration(1.0)):
                    rospy.sleep(0.1)
                    rospy.loginfo("Cancelando ejecución")
                    #self.sequence_action_client.cancel_all_goals()
                    self.esta_planificando = False
                else:
                    rospy.logwarn("Action server no está disponible para cancelar metas.")
            except rospy.ROSException as e:
                rospy.logerr(f"No se pudo cancelar metas: {e}")
            except Exception as e:
                rospy.logerr(f"Error inesperado al cancelar metas: {e}")

    def ejecutar_trayectoria_position(self, joint_angles,velocidad):
    
        try:
            self.trayectoria_pub.publish(Int16(0)) 
            if not self.esperar_confirmacion(0, timeout=10):
                rospy.logerr("Error ,No comunicado con Controlador.")
                return
            
            self.trayectoria_pub.publish(Int16(-6)) 
            # Activar la bandera rutina_corriendo
            self.rutina_corriendo = True
            self.planear_Rutina = False
            
            rospy.logwarn(1)
            self.move_group.set_max_velocity_scaling_factor(velocidad)

            self.move_group.set_planner_id("PTP")
            
            self.move_group.set_joint_value_target(joint_angles)
            

            success, traj, planning_time, error_code = self.move_group.plan()
            
            # esperar confirmacion 3
            if not self.esperar_confirmacion(3, timeout=10.0):
                rospy.logwarn("No se recibió confirmación final (3), pero finalizando localmente.")

            self.rutina_corriendo = False
            

        except rospy.ROSException as e:
            rospy.logerr("Error al ejecutar la trayectoria: {}".format(e))
            self.ejecutando_rutina = False
        
        
        self.rutina_corriendo = False
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        
    def ejecutar_trayectoria_pose(self, pose,velocidad,plan):
        try:
            self.trayectoria_pub.publish(Int16(0)) 
            if not self.esperar_confirmacion(0, timeout=10):
                rospy.logerr("Error ,No comunicado con Controlador.")
                return
            
            self.trayectoria_pub.publish(Int16(-6)) 
            # Activar la bandera rutina_corriendo
            self.rutina_corriendo = True
            self.planear_Rutina = False
            
            self.move_group.set_max_velocity_scaling_factor(velocidad)

            self.move_group.set_planner_id(plan)
            self.move_group.set_pose_target(pose,self.move_group.get_end_effector_link())

            success, traj, planning_time, error_code = self.move_group.plan()
            
            # esperar confirmacion 3
            if not self.esperar_confirmacion(3, timeout=10.0):
                rospy.logwarn("No se recibió confirmación final (3), pero finalizando localmente.")

            self.rutina_corriendo = False

        except rospy.ROSException as e:
            rospy.logerr("Error al ejecutar la trayectoria: {}".format(e))
            self.ejecutando_rutina = False

        self.rutina_corriendo = False
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    # callback del topico planificacion_A_trayectoria
    def _planificacion_callback(self, msg):
        try:
            self._last_plan_msg = int(msg.data)
            self._plan_event = True
        except Exception:
            pass

    def esperar_confirmacion(self, expected, timeout=10.0):
        """Espera a que planificacion_A_trayectoria publique un valor.
        - expected: valor entero esperado. Si es None, se acepta cualquier valor distinto de None.
        - timeout: segundos a esperar antes de fallar.
        Devuelve True si recibió el esperado, False en caso contrario.
        """        
        self._last_plan_msg = None
        self._plan_event = False
        start = rospy.Time.now()
        rate = rospy.Rate(10)

        while (rospy.Time.now() - start).to_sec() < timeout:
            if self._plan_event:
                val = self._last_plan_msg
                # si expected es None, aceptamos cualquier valor
                if expected is None:
                    return True
                if val == expected:
                    return True
                if val==-1:
                    rospy.logwarn("⚠️ No se recibió feedback en el tiempo establecido.")
                    return False
                if val==-2:
                    rospy.logwarn("❌ Planificación fallida")
                    return False
                # si llega otro valor lo ignoramos y seguimos esperando
                self._plan_event = False
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break
        rospy.logwarn(f"esperar_confirmacion: timeout esperando {expected}")
        self._plan_event = False
        return False

    def pose_to_constraint(self, pose_stamped, ratio):
        """
        Crea un objeto moveit_msgs/Constraints a partir de un PoseStamped.
        
        Esta función define DOS aspectos:
        1. La posición y orientación objetivo (a dónde debe ir el robot).
        2. Las tolerancias espaciales y angulares (qué tan exacto debe ser).
        
        El parámetro 'ratio' se usa actualmente para ambas tolerancias.
        Si en el futuro solo se desea indicar la posición objetivo sin tolerancia,
        se puede comentar la parte que asigna el tamaño de la esfera y los límites angulares.
        """

        constr = Constraints()

        # ======================================================
        #  POSICIÓN OBJETIVO (define el punto a alcanzar)
        # ======================================================
        pos = PositionConstraint()
        pos.header = pose_stamped.header
        pos.link_name = self.move_group.get_end_effector_link()

        # --- Volumen de restricción centrado en la posición del objetivo ---
        bv = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        
        # ======================================================
        #  TOLERANCIA ESPACIAL (puede comentarse si no se quiere usar)
        # ======================================================
        sphere.dimensions = [float(ratio if ratio > 0 else 0.002)]  # <-- tolerancia en metros
        # Si querés eliminar tolerancia, usar simplemente:
        # sphere.dimensions = [0.001]  # o un valor muy pequeño
        
        bv.primitives.append(sphere)
        
        # --- Pose del centro del volumen (posición objetivo) ---
        p = Pose()
        p.position = pose_stamped.pose.position
        p.orientation.w = 1.0  # sin rotación adicional
        bv.primitive_poses.append(p)
        
        pos.constraint_region = bv
        pos.weight = 1.0

        # ======================================================
        #  ORIENTACIÓN OBJETIVO
        # ======================================================
        ori = OrientationConstraint()
        ori.header = pose_stamped.header
        ori.link_name = pos.link_name
        ori.orientation = pose_stamped.pose.orientation

        # ======================================================
        #  TOLERANCIA ANGULAR (puede comentarse si no se quiere usar)
        # ======================================================
        tol = float(ratio if ratio > 0 else 0.05)  # en radianes
        ori.absolute_x_axis_tolerance = tol
        ori.absolute_y_axis_tolerance = tol
        ori.absolute_z_axis_tolerance = tol
        # Si querés una orientación fija, podrías definir tolerancias muy bajas:
        # ori.absolute_x_axis_tolerance = ori.absolute_y_axis_tolerance = ori.absolute_z_axis_tolerance = 1e-3
        
        ori.weight = 1.0

        # ======================================================
        # 5️⃣ COMPILAR CONSTRAINT
        # ======================================================
        constr.position_constraints = [pos]
        constr.orientation_constraints = [ori]
        return constr

    def expandir_rutina(self, lista,nombre):
        """Expande recursivamente las rutinas anidadas en una lista plana de puntos (elem donde elem['rutina'] == False).
        Ignora elementos con id == 'control'.
        """
        nombre_rut=nombre
        
        puntos = []
        for elem in lista:
            if elem.get('id') == 'control':
                continue
            if elem.get('plan')== "Rutina":
                nombre_rut = elem.get('nombre')
                if not nombre_rut:
                    rospy.logwarn("Elemento rutina sin nombre, se omite.")
                    continue
                try:
                    sub = obtener_rutina(nombre_rut)                    
                except Exception as e:
                    rospy.logerr(f"Error al obtener rutina {nombre_rut}: {e}")
                    continue
                # comprobar control en la rutina anidada
                control = next((r for r in sub if r.get('id') == 'control'), None)
                if control and control.get('error', False):
                    rospy.logwarn(f"Rutina anidada {nombre_rut} inválida, se omite.")
                    continue
                sub = [r for r in sub if r.get('id') != 'control']
                # Recursivamente expandir
                puntos.extend(self.expandir_rutina(sub, nombre_rut))
            else:                
                if elem.get('plan')== "Trayectoria":                    
                    self.posicion_Trayectorias.append(elem.get('pos'))
                    self.rutina_Trayectorias.append(nombre_rut)
                    
                puntos.append(elem)
        return puntos
    
    def dividir_en_tramos(self, puntos):
        tramos = []
        tramo_actual = []

        for p in puntos:
            if p.get('plan') == "Trayectoria":
                tramo_actual.append(p)
                tramos.append(tramo_actual)
                tramo_actual = []
            else:
                tramo_actual.append(p)

        if tramo_actual:
            tramos.append(tramo_actual)

        return tramos

    def crear_goal(self, puntos, Pinicial):
        """
        Crea un MoveGroupSequenceGoal para Pilz usando puntos en formato pose.
        Usa el estado inicial (Pinicial) como start_state en lugar de la pose actual del robot.
        
        Parámetros:
        - puntos: lista de diccionarios con la información de cada punto (pose, plan, vel_esc, ratio, etc.)
        - Pinicial: lista [q1, q2, q3, q4, q5, q6] indicando el estado inicial del robot (en radianes)
        """
        sequence_request = MotionSequenceRequest()        

        # ======================================================
        #  Crear RobotState inicial desde Pinicial
        # ======================================================
        start_state = RobotState()
        joint_state = JointState()
        joint_state.name = self.move_group.get_active_joints()  # toma los nombres del grupo
        joint_state.position = Pinicial
        start_state.joint_state = joint_state

        # ======================================================
        #  Crear los items de la secuencia
        # ======================================================
        for idx, p in enumerate(puntos):             
            pose = PoseStamped()
            pose.header.frame_id = "base_link"
            pose.pose = self.dict_to_pose(p.get('coordenadasCQuaterniones'))

            item = MotionSequenceItem()
            
            # Usar Pinicial como start_state en el primer movimiento
            if idx == 0:
                item.req.start_state = start_state
            
            #El proximo punto despues de una trayectoria empezara donde esta termino
            if p.get('plan')== "Trayectoria":
                PTrayectoriaFinal=norm.bit_rad(p.get('puntos')[-1]) 
                #si es trayectoria debo ir al punto iniciar,por eso cambio a PTP
                item.req.planner_id = 'PTP'
            else: 
                #indico el plan comun cuando no es trayectoria
                item.req.planner_id = p.get('plan', 'PTP')  # PTP, LIN o CIRC
            
            item.req.group_name = "cobot_arm"            
            item.req.max_velocity_scaling_factor = float(p.get('vel_esc', 50)) / 100.0
            item.req.max_acceleration_scaling_factor = 1.0                
            
            # Crear constraint hacia el punto objetivo
            ratio = float(p.get('ratio', 0))
            item.req.goal_constraints.append(self.pose_to_constraint(pose, ratio))

            # Radio de blending (si se desea suavizar transiciones)
            item.blend_radius = 0.0

            sequence_request.items.append(item)

        # ======================================================
        #  Construir el Goal final para enviar al ActionServer
        # ======================================================
        goal = MoveGroupSequenceGoal()
        goal.request = sequence_request
        return goal
    
    def Ordenar_ejec_rutina(self, data):
        F = Bool(False)
        self.planear_Rutina = True
        self.rutina_pub.publish(F)
        if not data.data or self.ejecutando_rutina:
            return
        self.ejecutar_rutina("rutina_actual")       


    def ejecutar_rutina(self, rut):

        rospy.loginfo("Iniciando rutina...")
        self.ejecutando_rutina = True

        # Leer rutina desde DB
        try:
            if rut=="rutina_actual":
                rutina = leer_rutina_sin_euler()
            else:
                rutina= obtener_rutina(rut)
        except Exception as e:
            rospy.logerr(f"Error leyendo rutina: {e}")
            self.ejecutando_rutina = False
            return

        # buscar registro control por id
        control = next((r for r in rutina if r.get('id') == 'control'), None)
        if control and control.get('error', True):
            rospy.logwarn("Rutina inválida (control con error=true), abortando.")
            self.ejecutando_rutina = False
            return

        # eliminar registros control
        rutina = [r for r in rutina if r.get('id') != 'control']

        # Expandir rutinas anidadas
        puntos = self.expandir_rutina(rutina,rut)

        rospy.loginfo(f"Puntos expandidos: {len(puntos)}")        
        
        #self.sequence_action_client.wait_for_result()        
        
        # Buscar todos los waits y trayectorias con sus índices 
        indices_wait = []
        valores_wait = []        
        
        indices_Trayectorias = []
                
        tramos = []
        tramo_actual = []
        PTrayectoriaFinal = None

        for i, p in enumerate(puntos):
            wait_val = int(p.get('wait', 0))
            if wait_val != 0:
                indices_wait.append(i) 
                valores_wait.append(wait_val)
                rospy.logwarn(indices_wait)
                rospy.logwarn(valores_wait)
            if p.get('plan')== "Trayectoria":
                indices_Trayectorias.append(i)
                tramo_actual.append(p)
                PTrayectoriaFinal=norm.bit_rad(p.get('puntos')[-1]) 
                tramos.append({
                    "puntos": tramo_actual,
                    "P_final": PTrayectoriaFinal
                })
                tramo_actual = []
            else:
                tramo_actual.append(p)                
        if tramo_actual:
            tramos.append({
                "puntos": tramo_actual,
                "P_final": None
            })

        tiene_wait = len(indices_wait) > 0
        tiene_trayectoria = len(indices_Trayectorias) > 0
            
        if not tiene_trayectoria:
            self.sequence_action_client.cancel_all_goals()
            
            posicion_actual = self.move_group.get_current_joint_values()
            goal = self.crear_goal(puntos,posicion_actual )        
            
            self.trayectoria_pub.publish(Int16(0)) 
            if not self.esperar_confirmacion(0, timeout=10):
                rospy.logerr("Error ,No comunicado con Controlador.")
                return            
            
            rospy.loginfo("Comiendo de planificaciond de rutina: enviando -1 a planificación y mandando goal completo")
            self.trayectoria_pub.publish(Int16(-1)) 
            if not self.sequence_action_client.wait_for_server(timeout=rospy.Duration(5)):
                rospy.logerr("El servidor de secuencia no está disponible.")
                self.ejecutando_rutina = False
                return                    
                          
            self.sequence_action_client.send_goal(goal)
            
            if not self.esperar_confirmacion(1, timeout=10):
                rospy.logerr("Error ,rutina no ejecutable. Abortando.")
                self.ejecutando_rutina = False
                return          
            else:
                rospy.loginfo("rutina ejecutable") 
                  
                
            if not tiene_wait: #NO Tiene trayectoria NI wait
                
                # --- CASO 1: rutina completa sin waits ---
                rospy.loginfo("Rutina sin waits ni trayectorias: enviando -2 a planificación y mandando goal completo")
                self.trayectoria_pub.publish(Int16(-2))
                
            else: # NO Tiene trayectoria pero si wait
                
                # --- CASO 2: rutina segmentada por waits ---
                rospy.loginfo("Rutina con waits: enviando -3 a planificación y esperando confirmación")
                self.trayectoria_pub.publish(Int16(-3))

                if not self.esperar_confirmacion(2, timeout=10.0):
                    rospy.logerr("No se recibió confirmación de planificación (1). Abortando.")
                    self.ejecutando_rutina = False
                    return

                msg_waits = waits()
                msg_waits.indice = indices_wait
                msg_waits.wait = valores_wait

                self.rutine_waits.publish(msg_waits)
                rospy.loginfo(f"📤 Publicado waits: {list(zip(indices_wait, valores_wait))}")
                
        else: #Tiene trayectoria
            
            
            self.sequence_action_client.cancel_all_goals() 
            posicion_actual = self.move_group.get_current_joint_values()
            posicion_real= posicion_actual    
            
            self.trayectoria_pub.publish(Int16(0)) 
            if not self.esperar_confirmacion(0, timeout=10):
                rospy.logerr("Error ,No comunicado con Controlador.")
                return            
                        
            for idx, sub_tramos in enumerate(tramos):
                
                goal = self.crear_goal(sub_tramos["puntos"],posicion_actual )                 
                
                
                rospy.loginfo("Comiendo de planificaciond de rutina: enviando -1 a planificación y mandando goal completo")
                self.trayectoria_pub.publish(Int16(-1)) 
                if not self.sequence_action_client.wait_for_server(timeout=rospy.Duration(5)):
                    rospy.logerr("El servidor de secuencia no está disponible.")
                    self.ejecutando_rutina = False
                    return        
                
                if idx > 0:
                    self.pos_dy_pub.publish(Int16MultiArray(data=norm.rad_bit(posicion_actual)))
                    posiciones_rad = posicion_actual
                    joint_msg = JointState()
                    joint_msg.header.stamp = rospy.Time.now()
                    joint_msg.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
                    joint_msg.position = posiciones_rad
                    self.joint_pub.publish(joint_msg)
                    
                
                    self.sequence_action_client.send_goal(goal) 
                    
                    if not self.esperar_confirmacion(1, timeout=10.0):
                        rospy.logerr("Error ,rutina no ejecutable. Abortando.")
                        self.ejecutando_rutina = False
                        
                        
                        self.pos_dy_pub.publish(Int16MultiArray(data=norm.rad_bit(posicion_real)))
                        posiciones_rad = posicion_real
                        joint_msg.header.stamp = rospy.Time.now()
                        joint_msg.position = posiciones_rad
                        self.joint_pub.publish(joint_msg)   
                        
                        return          
                    else:
                        rospy.loginfo("rutina ejecutable")
                
                    self.pos_dy_pub.publish(Int16MultiArray(data=norm.rad_bit(posicion_real)))
                    posiciones_rad = posicion_real
                    joint_msg.header.stamp = rospy.Time.now()
                    joint_msg.position = posiciones_rad
                    self.joint_pub.publish(joint_msg)   
                
                else:
                    self.sequence_action_client.send_goal(goal)                           
                                
                    if not self.esperar_confirmacion(1, timeout=10.0):
                        rospy.logerr("Error ,rutina no ejecutable. Abortando.")
                        self.ejecutando_rutina = False
                        return          
                    else:
                        rospy.loginfo("rutina ejecutable")  
                        
                posicion_actual= sub_tramos["P_final"]       
                  
                
                
                
                
            
            if not tiene_wait: #Tiene trayectoria pero NO wait
                
                # --- CASO 3: rutina segmentada por trayectoria---
                rospy.loginfo("Rutina con trayectorias: enviando -4 a planificación y esperando confirmación")
                self.trayectoria_pub.publish(Int16(-4))

                if not self.esperar_confirmacion(2, timeout=10.0):
                    rospy.logerr("No se recibió confirmación de planificación (1). Abortando.")
                    self.ejecutando_rutina = False
                    return

                msg_trayectorias = trayectorias()
                msg_trayectorias.indice = indices_Trayectorias
                msg_trayectorias.posicion = self.posicion_Trayectorias
                msg_trayectorias.Rutina = self.rutina_Trayectorias                

                self.rutine_trayectorias.publish(msg_trayectorias)
                rospy.loginfo(f"📤 Publicado trayectorias: {list(zip(indices_Trayectorias, self.posicion_Trayectorias))}")
                
                self.posicion_Trayectorias.clear()
                self.rutina_Trayectorias.clear()
                
            else: #Tiene trayectoria y wait
                
                # --- CASO 4: rutina segmentada por waits y trayectoria ---
                rospy.loginfo("Rutina con waits y trayectorias: enviando -5 a planificación y esperando confirmación")
                self.trayectoria_pub.publish(Int16(-5))

                if not self.esperar_confirmacion(2, timeout=10.0):
                    rospy.logerr("No se recibió confirmación de planificación (1). Abortando.")
                    self.ejecutando_rutina = False
                    return
                                
                msg_trayectorias = trayectorias()
                msg_trayectorias.indice = indices_Trayectorias
                msg_trayectorias.posicion = self.posicion_Trayectorias
                msg_trayectorias.Rutina = self.rutina_Trayectorias
                
                rospy.logwarn(msg_trayectorias)

                self.rutine_trayectorias.publish(msg_trayectorias)
                rospy.loginfo(f"📤 Publicado trayectorias: {list(zip(indices_Trayectorias, self.posicion_Trayectorias))}")
                
                self.posicion_Trayectorias.clear()
                self.rutina_Trayectorias.clear()

                msg_waits = waits()
                msg_waits.indice = indices_wait
                msg_waits.wait = valores_wait

                self.rutine_waits.publish(msg_waits)
                rospy.loginfo(f"📤 Publicado waits: {list(zip(indices_wait, valores_wait))}")

        # esperar confirmacion 3
        if not self.esperar_confirmacion(3, timeout=10.0):
            rospy.logwarn("No se recibió confirmación final (3), pero finalizando localmente.")

        self.ejecutando_rutina = False
        rospy.loginfo("ejecutar_rutina: finalizada.")

    def ejecutar_Trayectoria(self, trayectoria):
        rospy.loginfo(f"Puntos expandidos: {trayectoria}") 
        self.sequence_action_client.cancel_all_goals()     
                  
        self.posicion_Trayectorias.append(trayectoria.get('pos'))
        self.rutina_Trayectorias.append("rutina_actual")   
        
        posicion_actual = self.move_group.get_current_joint_values()
        goal = self.crear_goal([trayectoria],posicion_actual ) 
        
        self.trayectoria_pub.publish(Int16(0)) 
        if not self.esperar_confirmacion(0, timeout=10):
            rospy.logerr("Error ,No comunicado con Controlador.")
            return  
                                              
        rospy.loginfo("Comiendo de planificaciond de rutina: enviando -2 a planificación y mandando goal completo")
        self.trayectoria_pub.publish(Int16(-1)) 
        if not self.sequence_action_client.wait_for_server(timeout=rospy.Duration(5)):
            rospy.logerr("El servidor de secuencia no está disponible.")
            self.ejecutando_rutina = False
            return       
                             
        self.sequence_action_client.send_goal(goal)
        
        if not self.esperar_confirmacion(1, timeout=10.0):
            rospy.logerr("Error ,rutina no ejecutable. Abortando.")
            self.ejecutando_rutina = False
            return          
        else:
            rospy.loginfo("rutina ejecutable")                
        
        indices_Trayectorias = []
        
        indices_Trayectorias.append(0)

        rospy.loginfo("Rutina sin waits: enviando -4 a planificación y esperando confirmación")
        self.trayectoria_pub.publish(Int16(-4))

        if not self.esperar_confirmacion(2, timeout=10.0):
            rospy.logerr("No se recibió confirmación de planificación (1). Abortando.")
            self.ejecutando_rutina = False
            return

        msg_trayectorias = trayectorias()
        msg_trayectorias.indice = indices_Trayectorias
        msg_trayectorias.posicion = self.posicion_Trayectorias
        msg_trayectorias.Rutina = self.rutina_Trayectorias

        self.rutine_trayectorias.publish(msg_trayectorias)
        rospy.loginfo(f"📤 Publicado trayectorias: {list(zip(indices_Trayectorias, self.posicion_Trayectorias))}")
        
        self.posicion_Trayectorias.clear()
        self.rutina_Trayectorias.clear()

        # esperar confirmacion 3
        if not self.esperar_confirmacion(3, timeout=10.0):
            rospy.logwarn("No se recibió confirmación final (3), pero finalizando localmente.")

        self.ejecutando_rutina = False
        rospy.loginfo("ejecutar_rutina: finalizada.")

    def actualizar_estado_robot(self, data):
        # Actualizar el estado actual del robot con los datos recibidos
        self.estado_actual = data.data

    def ejecutar_cord_ros(self, data):
        F = Bool(False)
        #data.descripcion[orgien_de_orden,plan,velocidad,radio]
        rospy.loginfo(data)
        
        if not self.ejecutando_rutina:
            self.ejecutando_rutina = True
            self.rutina_pub.publish(F)
            try:
                if data.descriocion[1]==1:
                    velocidad=data.descriocion[2]/100
                    plan="PTP"
                elif data.descriocion[1]==2:
                    velocidad=data.descriocion[2]/100
                    plan="LIN"
                elif data.descriocion[1]==3:
                    velocidad=data.descriocion[2]/100
                    plan="CIRC"
                elif data.descriocion[1]==4:
                    plan="Rutina"
                elif data.descriocion[1]==5:
                    plan="Trayectoria"
                else:
                    rospy.logerr("Eror,Plan no especificado: {}".format(data.descriocion[1]))                    
                
                pose = None
                if data.descriocion[0] == 1:
                    # Obtener los datos de cord_ros (pose en este caso)
                    joint_angles = norm.grados_rad(list(data.coordenadas))
                    
                    if data.descriocion[1] == 1:  
                        self.ejecutar_trayectoria_position(joint_angles,velocidad)
                    else:
                        
                        #transformar coordenadas angulares a pose                        
                        try:                     
            
                            cord_ang_rad = norm.grados_rad(joint_angles)
                            
                            rospy.wait_for_service('/compute_fk')
                            fk = rospy.ServiceProxy('/compute_fk', GetPositionFK)

                            req = GetPositionFKRequest()
                            req.header.frame_id = self.move_group.get_planning_frame()
                            #req.header.frame_id = "world" 
                            req.fk_link_names = [self.move_group.get_end_effector_link()]

                            # Estado articular
                            robot_state = RobotState()
                            js = JointState()
                            js.name = self.move_group.get_active_joints()
                            js.position = cord_ang_rad
                            
                            print("Joints activos:", self.move_group.get_active_joints())
                            
                            robot_state.joint_state = js
                            req.robot_state = robot_state

                            resp = fk(req)
                            
                            if resp.error_code.val == 1:  # SUCCESS
                                pose=resp.pose_stamped[0].pose
                            else:
                                rospy.logerr(f"FK falló con código {resp.error_code.val}")
                        except Exception as e:
                            rospy.logerr(f"Error en FK: {e}")
                        
                        
                        if data.descriocion[1]==3:
                            radio=data.descriocion[3]/1000
                            self.ejecutar_trayectoria_pose(pose,velocidad,plan,radio)
                        else:
                            self.ejecutar_trayectoria_pose(pose,velocidad,plan)
                            
                elif data.descriocion[0] == 2 or data.descriocion[0] == 3:
                    instruccion=leer_punto_rutina(data.coordenadas[0])
                    if data.descriocion[1]==4:
                        nombre=instruccion.get("nombre", {})
                        self.ejecutar_rutina(nombre)
                    elif data.descriocion[1]==5:
                        rospy.loginfo("Ejecutar trayectoria")
                        self.ejecutar_Trayectoria(instruccion)
                    else:
                        punto = instruccion.get("coordenadasCQuaterniones", {})
                        position = punto.get("position", {})
                        orientation = punto.get("orientation", {})

                        pose = Pose()
                        pose.position.x = position.get("x", 0.0)
                        pose.position.y = position.get("y", 0.0)
                        pose.position.z = position.get("z", 0.0)
                        pose.orientation.x = orientation.get("x", 0.0)
                        pose.orientation.y = orientation.get("y", 0.0)
                        pose.orientation.z = orientation.get("z", 0.0)
                        pose.orientation.w = orientation.get("w", 1.0)
                        
                        if data.descriocion[1]==3:
                            radio=data.descriocion[3]/1000
                            self.ejecutar_trayectoria_pose(pose,velocidad,plan,radio)
                        else:
                            self.ejecutar_trayectoria_pose(pose,velocidad,plan) 
                else:                    
                    rospy.logerr("Eror,en el origen de la instruccion: {}".format(data.descriocion[0]))                       
                    
                rospy.loginfo("Ejecutando trayectoria desde cord_ros: {}".format(data.coordenadas))
                self.ejecutando_rutina = False
            except Exception as e:
                rospy.logerr("Error al ejecutar trayectoria desde cord_ros: {}".format(e))
                self.ejecutando_rutina = False


if __name__ == '__main__':
    try:
        roscpp_initialize(sys.argv)
        controlador = ControladorRobot()
        # Suscribirse al tópico cord_ros
        rospy.Subscriber('cord_ros', punto_correr, controlador.ejecutar_cord_ros, queue_size=1)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Se ha interrumpido la ejecución del nodo.")
