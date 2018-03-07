cmake_minimum_required(VERSION 2.8.3)
project(mapclasses)

#set(CMAKE_CXX_FLAGS "-O4 -g -fPIC -std=c++0x -fpermissive")
set (CMAKE_POSITION_INDEPENDENT_CODE TRUE)

# Add the dlib library for svm classification
include(/home/nacho/workspace/dlib-19.6/dlib/cmake)
find_package(dlib 19.06 REQUIRED)
IF(DLIB_FOUND)
  include_directories(${DLIB_INCLUDE_DIR})
  link_directories(${DLIB_LIBRARY_DIRS})
  message("DLIB lib is found:" ${DLIB_INCLUDE_DIR})
ENDIF(DLIB_FOUND)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	pcl_conversions
  	pcl_ros
  	sensor_msgs
  	std_msgs
  	geometry_msgs
  	message_generation
	tf
  	tf_conversions
  	eigen_conversions
)

add_message_files(FILES
	buff.msg
)

generate_messages(DEPENDENCIES
	std_msgs
	sensor_msgs
)

catkin_package(
	DEPENDS
	CATKIN_DEPENDS roscpp
	INCLUDE_DIRS include
	LIBRARIES map scanBuffer dlib
)

set(MAPPINGlibs
  sonarMap
  scanBuffer
  scanFilter
  utils
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(map src/map.cpp)
add_library(scanBuffer src/scanBuffer.cpp)
add_library(utils src/utils.cpp)
add_library(scanFilter src/scanFilter.cpp)

add_dependencies(scanFilter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(scanBuffer dlib ${catkin_LIBRARIES})
target_link_libraries(map scanBuffer utils dlib ${catkin_LIBRARIES})
target_link_libraries(scanFilter ${catkin_LIBRARIES})
