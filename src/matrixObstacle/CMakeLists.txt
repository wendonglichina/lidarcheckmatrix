cmake_minimum_required(VERSION 2.8.3)
project(matrixObstacle)

add_compile_options(-std=c++11)

SET(CMAKE_CXX_FLAGS "$ENV{CXXFLAGS} -O2 -fopenmp -w") 

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  geometry_msgs
  Polygons_msg
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_library(YAML_CPP_LIBRARIES yaml-cpp)

include_directories(
	${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  include/
  lib/
  )

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL OpenCV
)

file(GLOB_RECURSE MATRIX_LIB_SRC "${CMAKE_CURRENT_SOURCE_DIR}/lib/*.a*")

#ADD_DEFINITIONS(-D ROS_PROJECT)  #开启或禁止库中包含ros依赖

#add_library(matrixlib ${LIDAR3D_LIB_SRC})

add_executable(pub_matrixobs src/getMatrixObstacle.cpp) 
target_link_libraries(pub_matrixobs ${PCL_LIBRARIES})
target_link_libraries(pub_matrixobs ${catkin_LIBRARIES})
target_link_libraries(pub_matrixobs ${MATRIX_LIB_SRC})