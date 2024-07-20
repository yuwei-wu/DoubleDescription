cmake_minimum_required(VERSION 2.8.3)
project(decomp_ros_utils LANGUAGES C CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -Wall -Wno-deprecated-declarations")
find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(catkin_simple REQUIRED)
find_package(cmake_modules)
find_package(Eigen3 REQUIRED)


include_directories(${EIGEN3_INCLUDE_DIRS})


include_directories(${EIGEN3_INCLUDE_DIRS} ${DECOMP_UTIL_INCLUDE_DIRS})

add_definitions(-DQT_NO_KEYWORDS)

## This setting causes Qt's "MOC" generation to happen automatically.
## this does not moc things in include!!!!!!! only in src
set(CMAKE_AUTOMOC ON)

## We'll use the version that rviz used so they are compatible.

message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)




add_library(decomp_ros_utils
  ${MOC_FILES}
  include/
  )
target_link_libraries(decomp_ros_utils
   ${catkin_LIBRARIES})
  
catkin_simple()

cs_install()

cs_export()
