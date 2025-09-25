#!/usr/bin/env python
import rospy
import moveit_commander
from moveit_msgs.msg import RobotState, DisplayRobotState, DisplayTrajectory
from sensor_msgs.msg import JointState
import sys

# Inicializar ROS y MoveIt
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_robot', anonymous=True)

# Inicializar MoveIt Commander
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "prueba_arm"  # Reemplaza con el nombre real de tu grupo
move_group = moveit_commander.MoveGroupCommander(group_name)

# Crear un objeto de estado del robot
robot_state = RobotState()
robot_state.joint_state.name = move_group.get_active_joints()
robot_state.joint_state.position = move_group.get_current_joint_values()


def move_to_joint_positions(joint_positions):
    move_group.set_joint_value_target(joint_positions)
    plan = move_group.plan()

    
    if not move_group.go(wait=True):
        rospy.logerr("no se puede llegar a la posicion")
    else:
        rospy.loginfo("Movimiento completado exitosamente.")

if __name__ == "__main__":
    
    joint_positions = [0.3,0.5, 0, 0, 0, 0]  

    
    move_to_joint_positions(joint_positions)

    # Limpieza
    moveit_commander.roscpp_shutdown()
