cmake_minimum_required(VERSION 2.8.3)
project(msg_lidar_shape)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
  sensor_msgs
  #jsk_recognition_msgs
  visualization_msgs
#  message_runtime
)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
DetectedObject.msg
DetectedObjectArray.msg
DTLane.msg
Lane.msg
LaneArray.msg
Waypoint.msg
WaypointState.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  #jsk_recognition_msgs
  visualization_msgs
#message_runtime
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################

## The catkin_package macro generates cmake config files for your package
catkin_package(CATKIN_DEPENDS message_runtime geometry_msgs roscpp rospy std_msgs visualization_msgs)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)



#############
## Install ##
#############


#############
## Testing ##
#############



