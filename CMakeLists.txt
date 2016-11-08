cmake_minimum_required(VERSION 2.8.3)
project(learning_object_dynamics)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  jaco_msgs
  moveit_msgs
  roscpp
  rospy
  std_msgs
  actionlib_msgs
  actionlib
  moveit_ros_planning_interface
  moveit_utils
  moveit_msgs
  nav_msgs
  move_base_msgs
  segbot_arm_perception
  segbot_arm_manipulation
  pcl_ros
  pcl_conversions
  bwi_msgs
)


## Generate actions in the 'action' folder
 add_action_files(
   DIRECTORY action
   FILES
   LogPerception.action
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES segbot_arm_tasks
#  CATKIN_DEPENDS jaco_msgs moveit_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
   CATKIN_DEPENDS actionlib_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


## executables

add_executable(ex1 src/ex1.cpp)
target_link_libraries(ex1 ${catkin_LIBRARIES} )

add_executable(ex2 src/ex2.cpp)
target_link_libraries(ex2 ${catkin_LIBRARIES} )

add_executable(ex3 src/ex3.cpp)
target_link_libraries(ex3 ${catkin_LIBRARIES} )

add_executable(ex4 src/ex4.cpp)
target_link_libraries(ex4 ${catkin_LIBRARIES} )

add_executable(ex5 src/ex5.cpp)
target_link_libraries(ex5 ${catkin_LIBRARIES} )

add_executable(ex6 src/ex6.cpp)
target_link_libraries(ex6 ${catkin_LIBRARIES} )

add_executable(push_script src/push_script.cpp)
target_link_libraries(push_script ${catkin_LIBRARIES} )

#add_executable(log_perception_action src/log_perception_action.cpp)
#target_link_libraries(log_perception_action ${catkin_LIBRARIES} )

#add_executable(look_behavior src/look_behavior.cpp)
#target_link_libraries(look_behavior ${catkin_LIBRARIES} )

#add_executable(vision_logging_server src/vision_logging_server.cpp)
#target_link_libraries(vision_logging_server ${catkin_LIBRARIES} )



## example task 
#add_executable(object_handover_delivery_task src/object_handover_delivery_task.cpp)
#target_link_libraries(object_handover_delivery_task ${catkin_LIBRARIES} )
