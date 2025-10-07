#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String, Bool, Int16MultiArray, Float64MultiArray
from moveit_commander import MoveGroupCommander, roscpp_initialize, RobotCommander
from geometry_msgs.msg import Pose, Point
from moveit_msgs.msg import RobotTrajectory, RobotState, MotionSequenceRequest, MotionSequenceItem, MotionPlanRequest,MoveGroupSequenceAction, Constraints, MoveGroupSequenceGoal, GenericTrajectory, Constraints, JointConstraint, DisplayTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from db_puntos2 import leer_datos_rutina
import numpy as np
import sys
from actionlib import SimpleActionClient 
from moveit_msgs.srv import GetMotionPlan
from cobot_controladores.msg import actionSAction, actionSGoal, actionSResult

import rospy
import tf
from moveit_commander import MoveGroupCommander
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from geometry_msgs.msg import PoseStamped, Pose


# ===============================
# 1. Ángulos articulares -> Pose
# ===============================
def ang_to_pose(cord_ang, group: MoveGroupCommander):
    """
    cord_ang: lista con ángulos articulares [rad]
    group: objeto MoveGroupCommander
    """
    group.set_joint_value_target(cord_ang)
    pose: Pose = group.get_current_pose().pose
    return pose


# ===============================
# 2. Pose -> Ángulos articulares (IK)
# ===============================
def pose_to_ang(pose: Pose, group_name="manipulator"):
    """
    pose: geometry_msgs/Pose
    group_name: nombre del move_group definido en MoveIt
    return: lista de ángulos articulares [rad] o None si falla
    """
    rospy.wait_for_service('/compute_ik')
    compute_ik = rospy.ServiceProxy('/compute_ik', GetPositionIK)

    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = "base_link"  # 👈 ajustar si tu frame base es otro
    pose_stamped.pose = pose

    ik_req = GetPositionIKRequest()
    ik_req.ik_request.group_name = group_name
    ik_req.ik_request.pose_stamped = pose_stamped

    resp = compute_ik(ik_req)

    if resp.error_code.val == 1:  # 1 = SUCCESS
        return resp.solution.joint_state.position
    else:
        rospy.logerr(f"IK falló con código: {resp.error_code.val}")
        return None


# ===============================
# 3. Cuaternion -> Euler
# ===============================
def quat_to_euler(quat):
    """
    quat: [x, y, z, w]
    return: (roll, pitch, yaw) en radianes
    """
    return tf.transformations.euler_from_quaternion(quat)


# ===============================
# 4. Euler -> Cuaternion
# ===============================
def euler_to_quat(roll, pitch, yaw):
    """
    roll, pitch, yaw en radianes
    return: [x, y, z, w]
    """
    return tf.transformations.quaternion_from_euler(roll, pitch, yaw)


# ===============================
# 5. Pose -> [x,y,z,roll,pitch,yaw]
# ===============================
def pose_to_euler_pose(pose: Pose):
    """
    Devuelve una lista [x,y,z,roll,pitch,yaw] en radianes
    """
    quat = [
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ]
    roll, pitch, yaw = quat_to_euler(quat)
    return [
        pose.position.x,
        pose.position.y,
        pose.position.z,
        roll,
        pitch,
        yaw,
    ]
