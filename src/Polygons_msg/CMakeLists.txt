cmake_minimum_required(VERSION 2.8.3)
project(Polygons_msg)

add_compile_options(-std=c++11)

SET(CMAKE_CXX_FLAGS "$ENV{CXXFLAGS} -O2 -fopenmp -w") 

find_package(catkin REQUIRED COMPONENTS roscpp rospy message_generation std_msgs geometry_msgs)

add_message_files(
  FILES
  Polygons.msg
)
generate_messages(
 DEPENDENCIES
 geometry_msgs
)
include_directories(
	${catkin_INCLUDE_DIRS} 
  )

catkin_package(
  #CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
  CATKIN_DEPENDS message_runtime
)