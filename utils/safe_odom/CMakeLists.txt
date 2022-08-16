cmake_minimum_required(VERSION 3.10)
project(safe_odom)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  message_filters
)

find_package(Eigen3 REQUIRED)

catkin_package(
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
)

add_executable(safe_odom src/safe_odom.cpp)
  
target_link_libraries(safe_odom
    ${catkin_LIBRARIES}
    ${catkin_INCLUDE_DIRS}
  )
  