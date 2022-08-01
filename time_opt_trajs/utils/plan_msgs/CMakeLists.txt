cmake_minimum_required(VERSION 3.10)
project(plan_msgs)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs)
find_package(Eigen3 REQUIRED)

  
# Generate messages in the 'msg' folder
add_message_files(
  FILES
  PolyTraj.msg
  DataDisp.msg
  Cylinders.msg
  )
 
# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  )


catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  message_runtime
  DEPENDS
  EIGEN3)
  