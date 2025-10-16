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
from db_puntos3 import  leer_rutina_sin_euler, obtener_rutina, leer_punto_rutina
import numpy as np
import sys
from actionlib import SimpleActionClient 
from moveit_msgs.srv import GetMotionPlan
from cobot_controladores.msg import actionSAction, actionSGoal, actionSResult, punto_correr, waits


# Bounding volume
from shape_msgs.msg import SolidPrimitive, Mesh, MeshTriangle
from geometry_msgs.msg import PointStamped
# Construir bounding volume simple (usaremos solo el solid primitive)
from moveit_msgs.msg import BoundingVolume

from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionFK, GetPositionFKRequest


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
        rospy.Subscriber('rutina', Bool, self.ejecutar_rutina)

        self.rutina_pub = rospy.Publisher('rutina', Bool, queue_size=10)

        # Publicar en el tópico de coordenadas Dynamixel alternativo
        self.cord_dy_pub2 = rospy.Publisher('cord_dy2', Int16MultiArray, queue_size=10)

        # NUEVOS TOPICOS PARA COMUNICACION CON PLANNER
        self.trayectoria_pub = rospy.Publisher('trayectoria_A_planificacion', Int16, queue_size=10)
        self.rutine_waits = rospy.Publisher('rutine_waits', waits, queue_size=10)
        self.plan_sub = rospy.Subscriber('planificacion_A_trayectoria', Int16, self._planificacion_callback)

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


    def rad_bit(self, rad):
        bit = [(int((r + np.pi) * 4095 / (2 * np.pi))) for r in rad]
        return np.int16(bit)

    def grados_rad(self, grad):
        rad = [g * np.pi / 180 for g in grad]
        return rad
    
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
            self.trayectoria_pub.publish(Int16(-5)) 
            # Activar la bandera rutina_corriendo
            self.rutina_corriendo = True
            self.planear_Rutina = False
            
            rospy.logwarn(velocidad)
            self.move_group.set_max_velocity_scaling_factor(velocidad)

            self.move_group.set_planner_id("PTP")
            
            self.move_group.set_joint_value_target(joint_angles)
            

            success, traj, planning_time, error_code = self.move_group.plan()
            
            
            # esperar confirmacion 3
            if not self.esperar_confirmacion(3, timeout=30.0):
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
            self.trayectoria_pub.publish(Int16(-5)) 
            # Activar la bandera rutina_corriendo
            self.rutina_corriendo = True
            self.planear_Rutina = False
            
            self.move_group.set_max_velocity_scaling_factor(velocidad)

            self.move_group.set_planner_id(plan)
            self.move_group.set_pose_target(pose,self.move_group.get_end_effector_link())

            success, traj, planning_time, error_code = self.move_group.plan()
            
            # esperar confirmacion 3
            if not self.esperar_confirmacion(3, timeout=30.0):
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

    def esperar_confirmacion(self, expected, timeout=30.0):
        """Espera a que planificacion_A_trayectoria publique un valor.
        - expected: valor entero esperado. Si es None, se acepta cualquier valor distinto de None.
        - timeout: segundos a esperar antes de fallar.
        Devuelve True si recibió el esperado, False en caso contrario.
        """
        start = rospy.Time.now()
        rate = rospy.Rate(10)
        self._plan_event = False
        self._last_plan_msg = None

        while (rospy.Time.now() - start).to_sec() < timeout:
            if self._plan_event:
                val = self._last_plan_msg
                # si expected es None, aceptamos cualquier valor
                if expected is None:
                    return True
                if val == expected:
                    return True
                # en caso de error (0) devolvemos False si esperamos otro valor
                if val == 0:
                    return False
                # si llega otro valor lo ignoramos y seguimos esperando
                self._plan_event = False
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break
        rospy.logwarn(f"esperar_confirmacion: timeout esperando {expected}")
        return False

    def pose_to_constraint(self, pose_stamped, ratio):
        """Crea un objeto moveit Constraints a partir de un PoseStamped.
        Usamos PositionConstraint y OrientationConstraint con tolerancias basadas en 'ratio'.
        Se asume que 'ratio' está en metros para posición y en radianes para orientación (si es pequeño).
        """
        constr = Constraints()

        # Posición: bounding volume como una pequeña esfera (usamos SolidPrimitive de tipo SPHERE)
        pos = PositionConstraint()
        pos.header = pose_stamped.header
        pos.link_name = self.move_group.get_end_effector_link()

        # Crear una esfera con radio = ratio
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [float(ratio if ratio > 0 else 0.002)]

        bv = BoundingVolume()
        bv.primitives.append(sphere)
        # Centramos el bounding volume en la posición deseada
        p = Pose()
        p.position = pose_stamped.pose.position
        p.orientation.w = 1.0 
        bv.primitive_poses.append(p)

        pos.constraint_region = bv
        pos.weight = 1.0

        # Orientación
        ori = OrientationConstraint()
        ori.header = pose_stamped.header
        ori.link_name = pos.link_name
        ori.orientation = pose_stamped.pose.orientation
        # tolerancias en radianes (usar ratio como aproximación)
        tol = float(ratio if ratio > 0 else 0.05)
        ori.absolute_x_axis_tolerance = tol
        ori.absolute_y_axis_tolerance = tol
        ori.absolute_z_axis_tolerance = tol
        ori.weight = 1.0

        # Añadir a Constraints
        constr.position_constraints = [pos]
        constr.orientation_constraints = [ori]
        return constr

    def expandir_rutina(self, lista):
        """Expande recursivamente las rutinas anidadas en una lista plana de puntos (elem donde elem['rutina'] == False).
        Ignora elementos con id == 'control'.
        """
        puntos = []
        for elem in lista:
            if elem.get('id') == 'control':
                continue
            if elem.get('rutina', False):
                nombre = elem.get('nombre')
                if not nombre:
                    rospy.logwarn("Elemento rutina sin nombre, se omite.")
                    continue
                try:
                    sub = obtener_rutina(nombre)
                except Exception as e:
                    rospy.logerr(f"Error al obtener rutina {nombre}: {e}")
                    continue
                # comprobar control en la rutina anidada
                control = next((r for r in sub if r.get('id') == 'control'), None)
                if control and control.get('error', False):
                    rospy.logwarn(f"Rutina anidada {nombre} inválida, se omite.")
                    continue
                sub = [r for r in sub if r.get('id') != 'control']
                # Recursivamente expandir
                puntos.extend(self.expandir_rutina(sub))
            else:
                puntos.append(elem)
        return puntos

    def crear_goal(self, puntos):
        """Crea un MoveGroupSequenceGoal para Pilz usando puntos en formato pose.
        Se añade primero un item con la pose actual del robot para asegurar que la secuencia parte desde la pose actual.
        """
        sequence_request = MotionSequenceRequest()

        # Obtener pose actual
        current_pose_stamped = PoseStamped()
        current_pose_stamped.header.frame_id = "base_link"
        current_pose_stamped.pose = self.move_group.get_current_pose().pose

        # Item inicial con pose actual (para asegurar inicio desde la pose real)
        init_item = MotionSequenceItem()
        init_item.req.group_name = "cobot_arm"
        init_item.req.planner_id = "PTP"
        init_item.req.max_velocity_scaling_factor = 0.1
        init_item.req.max_acceleration_scaling_factor = 0.1
        init_item.req.goal_constraints.append(self.pose_to_constraint(current_pose_stamped, 0.01))
        sequence_request.items.append(init_item)

        for p in puntos:
            pose = PoseStamped()
            pose.header.frame_id = "base_link"
                        
            coords = self.dict_to_pose( p.get('coordenadasCQuaterniones') )
            pose.pose = coords

            item = MotionSequenceItem()
            item.req.group_name = "cobot_arm"
            item.req.planner_id = p.get('plan', 'PTP')  # PTP, LIN, CIRC
            item.req.max_velocity_scaling_factor = float(p.get('vel_esc', 100)) / 100.0
            item.req.max_acceleration_scaling_factor = 1.0

            ratio = float(p.get('ratio', 0.01))
            item.req.goal_constraints.append(self.pose_to_constraint(pose, ratio))
            item.blend_radius = 0.0

            sequence_request.items.append(item)

        goal = MoveGroupSequenceGoal()
        goal.request = sequence_request
        return goal

    def ejecutar_rutina(self, data):
        F = Bool(False)
        self.planear_Rutina = True
        if not data.data or self.ejecutando_rutina:
            return

        rospy.loginfo("Iniciando rutina...")
        self.ejecutando_rutina = True
        self.rutina_pub.publish(F)

        # Leer rutina desde DB
        try:
            rutina = leer_rutina_sin_euler()
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
        puntos = self.expandir_rutina(rutina)

        rospy.loginfo(f"Puntos expandidos: {len(puntos)}")
        
        goal = self.crear_goal(puntos)        
        
        rospy.loginfo("Comiendo de planificaciond de rutina: enviando -2 a planificación y mandando goal completo")
        self.trayectoria_pub.publish(Int16(-1)) 
        if not self.sequence_action_client.wait_for_server(timeout=rospy.Duration(5)):
            rospy.logerr("El servidor de secuencia no está disponible.")
            self.ejecutando_rutina = False
            return
        
        
        self.sequence_action_client.cancel_all_goals()              
        self.sequence_action_client.send_goal(goal)
        self.sequence_action_client.wait_for_result()
        
        if not self.esperar_confirmacion(0, timeout=30.0):
            rospy.logerr("Error ,rutina no ejecutable. Abortando.")
            self.ejecutando_rutina = False
            return          
        else:
            rospy.logerr("rutina ejecutable")           
        
        # Buscar todos los waits y sus índices (ajustados por +1)
        indices_wait = []
        valores_wait = []

        for i, p in enumerate(puntos):
            wait_val = int(p.get('wait', 0))
            if wait_val != 0:
                indices_wait.append(i + 1)  # +1 porque crear_goal agrega un punto inicial
                valores_wait.append(wait_val)

        tiene_wait = len(indices_wait) > 0
        
        #comienzo con las ejecuciones

        if not tiene_wait:
            # --- CASO 1: rutina completa sin waits ---
            rospy.loginfo("Rutina sin waits: enviando -2 a planificación y mandando goal completo")
            self.trayectoria_pub.publish(Int16(-2))

            

        else:
            # --- CASO 2: rutina segmentada por waits ---
            rospy.loginfo("Rutina con waits: enviando -3 a planificación y esperando confirmación")
            self.trayectoria_pub.publish(Int16(-3))

            if not self.esperar_confirmacion(1, timeout=30.0):
                rospy.logerr("No se recibió confirmación de planificación (1). Abortando.")
                self.ejecutando_rutina = False
                return

            msg_waits = waits()
            msg_waits.indice = indices_wait
            msg_waits.waits = valores_wait

            self.rutine_waits.publish(msg_waits)
            rospy.loginfo(f"📤 Publicado waits: {list(zip(indices_wait, valores_wait))}")

        # esperar confirmacion 3
        if not self.esperar_confirmacion(3, timeout=30.0):
            rospy.logwarn("No se recibió confirmación final (3), pero finalizando localmente.")

        self.ejecutando_rutina = False
        rospy.loginfo("ejecutar_rutina: finalizada.")

    def actualizar_estado_robot(self, data):
        # Actualizar el estado actual del robot con los datos recibidos
        self.estado_actual = data.data

    def ejecutar_cord_ros(self, data):
        F = Bool(False)
        #data.descripcion[orgien_de_orden,plan,velocidad,radio]
        if not self.ejecutando_rutina:
            self.ejecutando_rutina = True
            self.rutina_pub.publish(F)
            try:
                if data.descriocion[1]==1:
                    plan="PTP"
                elif data.descriocion[1]==2:
                    plan="LIN"
                elif data.descriocion[1]==3:
                    plan="CIRC"
                else:
                    rospy.logerr("Eror,Plan no especificado: {}".format(data.descriocion[1]))
                    
                velocidad=data.descriocion[2]/100
                pose = None
                if data.descriocion[0] == 1:
                    # Obtener los datos de cord_ros (pose en este caso)
                    joint_angles = self.grados_rad(list(data.coordenadas))
                    
                    if data.descriocion[1] == 1:  
                        self.ejecutar_trayectoria_position(joint_angles,velocidad)
                    else:
                        
                        #transformar coordenadas angulares a pose                        
                        try:                     
            
                            cord_ang_rad = self.grados_rad(joint_angles)
                            
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
        rospy.Subscriber('cord_ros', punto_correr, controlador.ejecutar_cord_ros)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Se ha interrumpido la ejecución del nodo.")
