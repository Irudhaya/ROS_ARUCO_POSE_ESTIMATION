cmake_minimum_required(VERSION 2.8.3)
project(pose_aruco)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++1z)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
include_directories(/usr/include/opencv4)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  cv_bridge
)
find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pose_aruco
#  CATKIN_DEPENDS geometry_msgs roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/pose_aruco.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
