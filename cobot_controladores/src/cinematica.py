#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32MultiArray, Int16MultiArray, String, Bool
from db_puntos3 import (
    escribir_datos,
    leer_datos,
    eliminar_todos_datos,
    leer_punto,
    cambiar_punto,
    eliminar_punto,
    agregar_punto_rutina,
    leer_rutina_completa,
    eliminar_todos_datos_rutina,
    eliminar_punto_rutina,
    leer_punto_rutina,
    eliminar_rutina,
    obtener_todas_rutinas,
    obtener_rutina,
    agregar_rutina,
    agregar_rutina_rutina,
    editar_punto_rutina,
    verificar_rutinas_control,
    actualizar_control,
    eliminar_rutina_control,
    leer_rutina_sin_quaterniones,
)
from cobot_controladores.msg import punto_web, nombresPuntos, punto_real
import numpy as np

import tf
from moveit_commander import MoveGroupCommander
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionFK, GetPositionFKRequest
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

from Normalizacion_Robot import NormalizacionRobot

import re


import time

norm = NormalizacionRobot()
class CinematicMode:
    def __init__(self):
        rospy.init_node('cinematic_mode')
        
                
        # Definir el nombre del nuevo tópico
        self.puntos_dy_pub = rospy.Publisher('puntos_dy', Float32MultiArray, queue_size=10)

        #   ////////  PAGINA WEB    ////////

        
        
        # Definir el nombre del nuevo tópico
        self.puntos_web = rospy.Publisher('puntodb', punto_web, queue_size=10)
        
        self.puntos_rutina = rospy.Publisher('puntoRutina', punto_web, queue_size=10)
        
        self.informe_web = rospy.Publisher('informe_web', String, queue_size=10)
        
        self.correr_rutina = rospy.Publisher('rutina', Bool, queue_size=10)
        
        self.nombres_puntos_tabla = rospy.Publisher('lista_puntosdb', nombresPuntos, queue_size=10)
        
        self.nombres_rutinas_tabla = rospy.Publisher('lista_rutinasdb', nombresPuntos, queue_size=10)
        
        self.pub_pos_real = rospy.Publisher('pos_real', punto_real, queue_size=10)
        
        
        self.velocidad=50
        self.ratio=0
        
        # Eliminar todos los datos de la base de datos al inicio
        #################### Eliminar todos los datos de la base de datos ####################
        #eliminar_todos_datos()
        eliminar_todos_datos_rutina()
        #################### Aquí termina la sección de eliminación de la base de datos ####################
        
        self.angulos = [0] * 6
        
        
        #################### Inicializa el conversor de coordenadas para un robot definido en MoveIt. ####################
        self.move_group = MoveGroupCommander("cobot_arm")
        self.group_name = "cobot_arm"
        self.base_frame = self.move_group.get_planning_frame()
        self.move_group.set_planner_id("PTP")     
        
        self.punto_ang = rospy.Publisher('ang', Float32MultiArray, queue_size=10)
        self.punto_cart = rospy.Publisher('cart', Float32MultiArray, queue_size=10)
        self.punto_pose = rospy.Publisher('pose', Float32MultiArray, queue_size=10)
        rospy.Subscriber('ang_cart', Float32MultiArray, self.ang_cart)
        rospy.Subscriber('cart_ang', Float32MultiArray, self.cart_ang)
        
    def ang_cart(self, ang):
        angulos= ang.data
        pose = self.AngulosArticulares_a_pose(angulos)
        pose_list=[ pose.position.x*1000,
                    pose.position.y*1000,
                    pose.position.z*1000,
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                    ]
        msgPose = Float32MultiArray()
        msgPose .data = pose_list
        self.punto_pose.publish(msgPose)
        
        cart=self.pose_a_cartesianasEuler(pose)
                
        msgCart = Float32MultiArray()
        msgCart.data = cart
        self.punto_cart.publish(msgCart)
        
    def cart_ang(self, cart):
        cartesianas=cart.data
        
        pose = self.cartesianasEuler_a_pose(cartesianas)        
        
        if pose:
        
            pose_list=[ pose.position.x*1000,
                        pose.position.y*1000,
                        pose.position.z*1000,
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w,
                        ]
            msgPose = Float32MultiArray()
            msgPose .data = pose_list
            self.punto_pose.publish(msgPose)
            ang= self.pose_a_AngulosArticulares(pose)
            if ang is not None:
        
                msgAng = Float32MultiArray()
                msgAng.data = [float(a) for a in ang]
                self.punto_ang.publish(msgAng)
        else: 
            None        
   
    # ===============================
    # 1. Ángulos articulares -> Pose
    # ===============================
    def AngulosArticulares_a_pose(self, cord_ang_grados):   
        	
        try:                     
            
            cord_ang_rad = norm.grados_rad(cord_ang_grados)
            
            rospy.wait_for_service('/compute_fk')
            fk = rospy.ServiceProxy('/compute_fk', GetPositionFK)

            req = GetPositionFKRequest()
            req.header.frame_id = self.base_frame
            #req.header.frame_id = "world" 
            req.fk_link_names = [self.move_group.get_end_effector_link()]

            # Estado articular
            robot_state = RobotState()
            js = JointState()
            js.name = self.move_group.get_active_joints()
            js.position = cord_ang_rad
            
            #print("Joints activos:", self.move_group.get_active_joints())
            
            robot_state.joint_state = js
            req.robot_state = robot_state

            resp = fk(req)
            
            if resp.error_code.val == 1:  # SUCCESS
                return resp.pose_stamped[0].pose
            else:
                rospy.logerr(f"FK falló con código {resp.error_code.val}")
                return None
        except Exception as e:
            rospy.logerr(f"Error en FK: {e}")
            return None

    # ===============================
    # 2. Pose -> Ángulos articulares
    # ===============================
    def pose_a_AngulosArticulares(self, pose: Pose):
        rospy.wait_for_service('/compute_ik')
        compute_ik = rospy.ServiceProxy('/compute_ik', GetPositionIK)

        self.move_group.set_goal_tolerance(0.001)
        self.move_group.set_goal_position_tolerance(0.001)
        self.move_group.set_goal_orientation_tolerance(0.01)
        
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.base_frame
        pose_stamped.pose = pose
        pose_stamped.header.stamp = rospy.Time.now()

        ik_req = GetPositionIKRequest()
        ik_req.ik_request.group_name = self.group_name
        ik_req.ik_request.pose_stamped = pose_stamped
        ik_req.ik_request.robot_state = self.move_group.get_current_state()
        ik_req.ik_request.avoid_collisions = False

        resp = compute_ik(ik_req)
        
        if resp.error_code.val == 1:  # SUCCESS
            cord_ang_rad = list(resp.solution.joint_state.position)
            #return norm.rad_grados(cord_ang_rad)
            return norm.rad_grados(cord_ang_rad)
        else:
            rospy.logerr(f"IK falló con código: {resp.error_code.val}")
            return None

    # ===============================
    # 3. Cartesianas + Euler -> Pose
    # ===============================
    def cartesianasEuler_a_pose(self, coord):
        # 1. Separar parte cartesiana y angular
        coord_cart = [c / 1000.0 for c in coord[:3]]   # mm → m
        coord_ang  = norm.grados_rad(coord[3:])        # grados → radianes

        # 2. Calcular cuaternión a partir de las coordenadas angulares
        quat = tf.transformations.quaternion_from_euler(
            coord_ang[0], coord_ang[1], coord_ang[2]
        )
        
        # 3. Construir objeto Pose
        pose = Pose()
        pose.position.x = coord_cart[0]
        pose.position.y = coord_cart[1]
        pose.position.z = coord_cart[2]
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
       
        return pose
    
    # ===============================
    # 4. Pose -> Cartesianas + Euler
    # ===============================
    def pose_a_cartesianasEuler(self, pose: Pose):
        quat = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quat)
        
        coordenadas = [
           float(pose.position.x * 1000),
           float(pose.position.y * 1000),
           float(pose.position.z * 1000),
           ] + [float(a) for a in norm.rad_grados([roll, pitch, yaw])
        ]
        return coordenadas        
       
    # ===============================
    # 5. Ángulos articulares -> Cartesianas + Euler
    # ===============================
    def AngulosArticulares_a_cartesianasEuler(self, cord_ang_grados):
        """
        cord_ang_grados: lista [j1..j6] en grados
        return: [x,y,z,roll,pitch,yaw] en grados
        """
        pose = self.AngulosArticulares_a_pose(cord_ang_grados)
        return self.pose_a_cartesianasEuler(pose)

    # ===============================
    # 6. Cartesianas + Euler -> Ángulos articulares
    # ===============================
    def cartesianaEuler_a_AngulosArticulares(self, coord_grados):
        """
        coord_grados: [x,y,z,roll,pitch,yaw] en (m, grados)
        return: lista [j1..j6] en grados o None si falla
        """
        pose = self.cartesianasEuler_a_pose(coord_grados)
        
        if pose:
            return self.pose_a_AngulosArticulares(pose)
        else: 
            None
                        
if __name__ == '__main__':
    try:
        cinematic_mode =CinematicMode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
