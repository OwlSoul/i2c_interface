cmake_minimum_required(VERSION 2.8.3)
project(i2c_interface)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  std_msgs
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
  FILES
  DeviceWriteString.srv
  DeviceReadString.srv
  DeviceWriteBytes.srv
  DeviceReadBytes.srv
  DeviceSetPoll.srv
  DeviceStopPoll.srv
)

generate_messages(DEPENDENCIES std_msgs)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################



###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############



#############
## Testing ##
#############

add_executable(i2c_interface_node src/i2c_interface_node.cpp)
add_dependencies(i2c_interface_node i2c_interface_generate_messages_cpp)
target_link_libraries(i2c_interface_node ${catkin_LIBRARIES})
