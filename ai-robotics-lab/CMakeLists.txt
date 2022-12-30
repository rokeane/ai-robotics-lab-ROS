cmake_minimum_required(VERSION 2.8.3)
project(ai_robotics_lab)

find_package(catkin REQUIRED COMPONENTS
#  move_to
  rospy
  smach_ros
  message_generation
  std_msgs
)

#add_message_files(DIRECTORY msg FILES Floats.msg HeaderString.msg)
add_service_files(DIRECTORY srv FILES FFSolve.srv)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
