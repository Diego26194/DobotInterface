#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String, Bool, Int16, Int16MultiArray, Float64MultiArray
from moveit_commander import MoveGroupCommander, roscpp_initialize, RobotCommander
from geometry_msgs.msg import Pose, Point, PoseStamped
from moveit_msgs.msg import RobotTrajectory, RobotState, MotionSequenceRequest, MotionSequenceItem, MotionPlanRequest, MoveGroupSequenceAction, Constraints, MoveGroupSequenceGoal, GenericTrajectory, JointConstraint, DisplayTrajectory
from moveit_msgs.msg import PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from db_puntos3 import leer_datos_rutina, leer_rutina_sin_euler, obtener_rutina
import numpy as np
import sys
from actionlib import SimpleActionClient 
from moveit_msgs.srv import GetMotionPlan
from cobot_controladores.msg import actionSAction, actionSGoal, actionSResult


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

    def anular_ejecucion(self, msg):
        if self.planear_Rutina:
            try:
                if self.sequence_action_client.wait_for_server(timeout=rospy.Duration(1.0)):
                    rospy.sleep(0.1)
                    rospy.loginfo("Cancelando ejecución")
                    self.sequence_action_client.cancel_all_goals()
                    self.esta_planificando = False
                else:
                    rospy.logwarn("Action server no está disponible para cancelar metas.")
            except rospy.ROSException as e:
                rospy.logerr(f"No se pudo cancelar metas: {e}")
            except Exception as e:
                rospy.logerr(f"Error inesperado al cancelar metas: {e}")

    def ejecutar_trayectoria(self, joint_angles):
        try:
            # Activar la bandera rutina_corriendo
            self.rutina_corriendo = True
            self.planear_Rutina = False

            self.move_group.set_planner_id("PTP")
            self.move_group.set_joint_value_target(joint_angles)

            success, traj, planning_time, error_code = self.move_group.plan()

            self.rutina_corriendo = False

        except rospy.ROSException as e:
            rospy.logerr("Error al ejecutar la trayectoria: {}".format(e))
            self.ejecutando_rutina = False

        self.rutina_corriendo = False

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
        pos.link_name = "ee_link" if hasattr(self.move_group, 'get_end_effector_link') and self.move_group.get_end_effector_link() else "tool0"

        # Crear una esfera con radio = ratio
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [float(ratio if ratio > 0 else 0.001)]

        # Bounding volume
        from shape_msgs.msg import SolidPrimitive, Mesh, MeshTriangle
        from geometry_msgs.msg import PointStamped
        # Construir bounding volume simple (usaremos solo el solid primitive)
        from moveit_msgs.msg import BoundingVolume
        bv = BoundingVolume()
        bv.primitives.append(sphere)
        # Centramos el bounding volume en la posición deseada
        p = Point()
        p.x = pose_stamped.pose.position.x
        p.y = pose_stamped.pose.position.y
        p.z = pose_stamped.pose.position.z
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
            coords = p.get('coordenadasCQuaterniones')
            if not coords or len(coords) < 7:
                rospy.logwarn(f"Punto con coordenadas inválidas: {p}")
                continue
            pose.pose.position.x = float(coords[0])
            pose.pose.position.y = float(coords[1])
            pose.pose.position.z = float(coords[2])
            pose.pose.orientation.x = float(coords[3])
            pose.pose.orientation.y = float(coords[4])
            pose.pose.orientation.z = float(coords[5])
            pose.pose.orientation.w = float(coords[6])

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
        if control and control.get('error', False):
            rospy.logwarn("Rutina inválida (control con error=true), abortando.")
            self.ejecutando_rutina = False
            return

        # eliminar registros control
        rutina = [r for r in rutina if r.get('id') != 'control']

        # Expandir rutinas anidadas
        puntos = self.expandir_rutina(rutina)

        rospy.loginfo(f"Puntos expandidos: {len(puntos)}")

        # Verificar si hay algún wait != 0
        tiene_wait = any(int(p.get('wait', 0)) != 0 for p in puntos)

        if not tiene_wait:
            # --- CASO 1: rutina completa sin waits ---
            rospy.loginfo("Rutina sin waits: enviando -1 a planificación y mandando goal completo")
            self.trayectoria_pub.publish(Int16(-1))

            goal = self.crear_goal(puntos)
            if not self.sequence_action_client.wait_for_server(timeout=rospy.Duration(5)):
                rospy.logerr("El servidor de secuencia no está disponible.")
                self.ejecutando_rutina = False
                return
            self.sequence_action_client.cancel_all_goals()
            self.sequence_action_client.send_goal(goal)
            self.sequence_action_client.wait_for_result()

        else:
            # --- CASO 2: rutina segmentada por waits ---
            rospy.loginfo("Rutina con waits: enviando -2 a planificación y esperando confirmación")
            self.trayectoria_pub.publish(Int16(-2))

            if not self.esperar_confirmacion(1, timeout=30.0):
                rospy.logerr("No se recibió confirmación de planificación (1). Abortando.")
                self.ejecutando_rutina = False
                return

            bloque = []
            for p in puntos:
                bloque.append(p)
                if int(p.get('wait', 0)) != 0:
                    rospy.loginfo("Creando goal para bloque hasta wait")
                    goal = self.crear_goal(bloque)

                    if not self.sequence_action_client.wait_for_server(timeout=rospy.Duration(5)):
                        rospy.logerr("El servidor de secuencia no está disponible.")
                        self.ejecutando_rutina = False
                        return

                    self.sequence_action_client.cancel_all_goals()
                    self.sequence_action_client.send_goal(goal)
                    self.sequence_action_client.wait_for_result()

                    # enviar wait al otro nodo (decisegundos)
                    wait_val = int(p.get('wait', 0))
                    rospy.loginfo(f"Enviando wait {wait_val} a planificación")
                    self.trayectoria_pub.publish(Int16(wait_val))

                    # esperar confirmacion 2
                    if not self.esperar_confirmacion(2, timeout=30.0):
                        rospy.logerr("No se recibió confirmación de guardado (2). Abortando.")
                        self.ejecutando_rutina = False
                        return

                    bloque = []  # resetear bloque

            # fin de rutina
            rospy.loginfo("Rutina completa: notificando fin con -3")
            self.trayectoria_pub.publish(Int16(-3))

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
        if not self.ejecutando_rutina:
            self.ejecutando_rutina = True
            self.rutina_pub.publish(F)
            try:
                # Obtener los datos de cord_ros (pose en este caso)
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
