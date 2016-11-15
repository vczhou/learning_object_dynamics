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
  std_srvs
  std_msgs
  actionlib_msgs
  actionlib
  cv_bridge
  message_generation
  moveit_ros_planning_interface
  moveit_utils
  moveit_msgs
  nav_msgs
  move_base_msgs
  sensor_msgs
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

## Generate services in 'srv' folder
 add_service_files(
	FILES
	ProcessVision.srv
	StorePointCloud.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
	DEPENDENCIES
	std_msgs
	std_srvs
	sensor_msgs
	geometry_msgs
	actionlib_msgs
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
   INCLUDE_DIRS include
#  LIBRARIES segbot_arm_tasks
#  CATKIN_DEPENDS jaco_msgs moveit_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
   DEPENDS actionlib_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  	include
	${catkin_INCLUDE_DIRS}
)


## executables

add_executable(push_script src/push_script.cpp)
target_link_libraries(push_script ${catkin_LIBRARIES} )

add_executable(push_var src/push_var.cpp)
target_link_libraries(push_var ${catkin_LIBRARIES} )

add_executable(log_perception_action src/log_perception_action.cpp)
add_dependencies(log_perception_action learning_object_dynamics_gencpp)
target_link_libraries(log_perception_action ${catkin_LIBRARIES} ${Boost_LIBRARIES} )

#add_executable(look_behavior src/look_behavior.cpp)
#target_link_libraries(look_behavior ${catkin_LIBRARIES} )

#add_executable(vision_logging_server src/vision_logging_server.cpp)
#target_link_libraries(vision_logging_server ${catkin_LIBRARIES} )


## example task 
#add_executable(object_handover_delivery_task src/object_handover_delivery_task.cpp)
#target_link_libraries(object_handover_delivery_task ${catkin_LIBRARIES} )
