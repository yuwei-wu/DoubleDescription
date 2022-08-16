cmake_minimum_required(VERSION 3.10)
project(opt_planner)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  kr_mav_msgs
  kr_tracker_msgs
  path_searching
  traj_utils
  cv_bridge
  message_runtime
  path_searching
  decomp_ros_utils
  motion_primitive_library
)

# catkin_package(CATKIN_DEPENDS message_runtime)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES opt_planner
 CATKIN_DEPENDS path_searching traj_utils message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_library(opt_planner
  src/planner_manager.cpp
  src/poly_opt.cpp
  )
target_link_libraries(opt_planner
  ${catkin_LIBRARIES}
  )
add_dependencies(opt_planner
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  )
