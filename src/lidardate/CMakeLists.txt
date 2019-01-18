cmake_minimum_required(VERSION 2.8.3)
project(lidardate)

add_compile_options(-std=c++11)

SET(CMAKE_CXX_FLAGS "$ENV{CXXFLAGS} -O2 -fopenmp -w") 

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_library(YAML_CPP_LIBRARIES yaml-cpp)

include_directories(
	${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  )

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL OpenCV
)

file(GLOB_RECURSE LIDAR3D_LIB_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/lib/*.c*")

#ADD_DEFINITIONS(-D ROS_PROJECT)  #开启或禁止库中包含ros依赖

#add_library(lidarlib ${LIDAR3D_LIB_SRC})
#target_link_libraries(lidarlib ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES} glog)

add_executable(pub_lidartp src/lidarReceived.cpp) 
target_link_libraries(pub_lidartp ${PCL_LIBRARIES})
target_link_libraries(pub_lidartp ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})