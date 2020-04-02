cmake_minimum_required(VERSION 2.8.3)
project(robot_interface)

## Catkin dependencies
find_package(catkin REQUIRED COMPONENTS
roscpp
std_msgs
geometry_msgs
actionlib
)

## Use C++11 Standard
set(CMAKE_CXX_FLAGS "-std=c++11 -g")

## Declare catkin exports of this package
catkin_package(
INCLUDE_DIRS include
LIBRARIES ${PROJECT_NAME}
CATKIN_DEPENDS
	roscpp
	std_msgs
	geometry_msgs
DEPENDS
)


## Source files for the interface library
set(LIB_SRC_FILES
  src/robot_client.cpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

## Build RVIZ tool library
add_library(${PROJECT_NAME} ${LIB_SRC_FILES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

## install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)


