cmake_minimum_required(VERSION 2.8.3)
project(full_coverage)

set(CMAKE_CXX_FLAGS "-std=c++17 -g")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  costmap_2d
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
  nav_msgs
  robot_interface
  tf
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(OpenCV REQUIRED)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  SquareCell.msg
  CellGrid.msg
  SweepProgress.msg
  FollowStatus.msg
  PlanInfo.msg
  DebugCell.msg
  DebugGrid.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  StopCoverage.srv
  GetCoarseGrid.srv
  GetNavPlan.srv
  GenerateNavPlan.srv
  BoolReq.srv
  ExecutePlan.srv
  MakePlan.srv
  Visit.srv
)

add_action_files(
  DIRECTORY action
  FILES Plan.action Execute.action
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   actionlib_msgs
   std_msgs
   geometry_msgs
   nav_msgs
 )



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES wall_planner_lib
  CATKIN_DEPENDS 
    actionlib
    actionlib_msgs
    roscpp 
    rospy 
    std_msgs 
    message_runtime
    geometry_msgs
    nav_msgs
    robot_interface
    tf
#  DEPENDS Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(wall_planner_lib
  src/wall_planner.cpp
  src/coarse_cell.cpp
  src/robot_cell.cpp
  src/coverage_cell.cpp
  src/coarse_grid.cpp
  src/robot_grid.cpp
  src/coverage_grid.cpp
  src/cell_bounds.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(wall_planner_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wall_planner_lib ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(coarse_grid_converter src/coarse_grid_converter.cpp)
target_link_libraries(coarse_grid_converter ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(coarse_grid_converter ${PROJECT_NAME}_generate_messages_cpp)

add_executable(cover_map src/cover_map.cpp)
target_link_libraries(cover_map ${catkin_LIBRARIES})
add_dependencies(cover_map ${PROJECT_NAME}_generate_messages_cpp robot_interface)

add_executable(safe_drive_pub src/safe_drive_pub.cpp)
target_link_libraries(safe_drive_pub ${catkin_LIBRARIES})
add_dependencies(safe_drive_pub ${PROJECT_NAME}_generate_messages_cpp)

add_executable(wall_planner_node src/wall_planner_node.cpp)
target_link_libraries(wall_planner_node ${catkin_LIBRARIES})
add_dependencies(wall_planner_node ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(wall_planner_node
  wall_planner_lib
)
