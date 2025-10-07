#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, roscpp_initialize
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest

def test_ik():
    roscpp_initialize([])
    rospy.init_node("test_ik_node", anonymous=True)

    group = MoveGroupCommander("cobot_arm")  # cambia por tu grupo si es necesario
    planning_frame = group.get_planning_frame()
    rospy.logwarn(f"Planning frame: {planning_frame}")

    # 1. Tomamos la pose actual del robot (debería ser alcanzable sí o sí)
    current_pose = group.get_current_pose().pose
    rospy.logwarn("Current pose:")
    rospy.logwarn(current_pose)

    # 2. Llamamos al servicio /compute_ik
    rospy.wait_for_service('/compute_ik')
    compute_ik = rospy.ServiceProxy('/compute_ik', GetPositionIK)

    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = planning_frame
    pose_stamped.pose = current_pose

    ik_req = GetPositionIKRequest()
    ik_req.ik_request.group_name = "cobot_arm"
    ik_req.ik_request.pose_stamped = pose_stamped
    ik_req.ik_request.avoid_collisions = False

    try:
        resp = compute_ik(ik_req)
        rospy.logwarn("Respuesta del IK:")
        rospy.logwarn(resp)
        if resp.error_code.val == 1:
            print("✅ IK encontró solución")
        else:
            print(f"❌ IK falló con código {resp.error_code.val}")
    except rospy.ServiceException as e:
        print("Error llamando al servicio IK:", e)

if __name__ == "__main__":
    test_ik()

