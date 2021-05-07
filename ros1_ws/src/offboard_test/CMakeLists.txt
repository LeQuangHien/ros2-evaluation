cmake_minimum_required(VERSION 3.0.2)
project(offboard_test)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   GoalPose.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   OffboardPose.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )


## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(offboard_server src/offboard_server.cpp)
target_link_libraries(offboard_server ${catkin_LIBRARIES})
add_dependencies(offboard_server offboard_test_gencpp)

add_executable(client src/add_three_ints_client.cpp)
target_link_libraries(client ${catkin_LIBRARIES})
add_dependencies(client offboard_test_gencpp)

add_executable(goal_pose_sender src/goal_pose_sender.cpp)
target_link_libraries(goal_pose_sender ${catkin_LIBRARIES})
add_dependencies(goal_pose_sender offboard_test_gencpp)

add_executable(goal_pose_receiver src/goal_pose_receiver.cpp)
target_link_libraries(goal_pose_receiver ${catkin_LIBRARIES})
add_dependencies(goal_pose_receiver offboard_test_gencpp)


