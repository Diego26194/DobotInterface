cmake_minimum_required(VERSION 3.0.2)
project(cobot_controladores)


find_package(catkin REQUIRED COMPONENTS
  moveit_commander
  rospy
  std_msgs
  geometry_msgs
  actionlib_msgs
  message_generation
  moveit_msgs
  actionlib_msgs actionlib
)

add_action_files(
  FILES
  actionS.action  
)
add_message_files(
  FILES
  msg_estados.msg
  msg_web.msg
  punto_web.msg
  nombresPuntos.msg
  punto_real.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
  moveit_msgs 
)

catkin_package(
  CATKIN_DEPENDS message_runtime 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
