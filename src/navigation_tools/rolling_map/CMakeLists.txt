## CREATE PACKAGE ===============================

cmake_minimum_required(VERSION 3.8.0)
project(rolling_map LANGUAGES CXX)
enable_language(CUDA OPTIONAL)

# Compile as C++ 14
set(CMAKE_CXX_FLAGS "-std=c++14")

# Determine if CUDA is available
include(CheckLanguage)
check_language(CUDA)
get_property(languages GLOBAL PROPERTY ENABLED_LANGUAGES)
if(CMAKE_CUDA_COMPILER AND ("CUDA" IN_LIST languages))
  set(CUDA_ENABLE true)
  add_definitions(-DUSE_CUDA)
  message("compiling rolling map with Cuda")
else()
  set(CUDA_ENABLE false)
  message("compiling rolling map without Cuda")
endif()

# -------------------------------------



## REQUIRED PACKAGES ============================

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  pcl_ros
)

# Use boost for shared_mutex
find_package(Boost REQUIRED COMPONENTS thread)

# Use openmp for CPU parallelism
find_package(OpenMP REQUIRED)
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

# -------------------------------------



## GENERATE SERVICE FILES =======================

# Generate services in the 'srv' folder
add_service_files(
  FILES
  Box.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
)

# -------------------------------------


## CATKIN PACKAGE ===============================

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rolling_map
  CATKIN_DEPENDS roscpp tf
  DEPENDS Boost
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# -------------------------------------



## LIBRARIES AND EXECUTABLES ===================

## Set up a C++ library for the rolling map class functionality
set(LIB_SOURCES
  src/rolling_map.cpp
)

if(CUDA_ENABLE)
  set(CUDA_SEPERABLE_COMPILATION ON)
  set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -rdc=true")
  list(APPEND LIB_SOURCES src/rolling_map.cu)
endif()

add_library(${PROJECT_NAME} ${LIB_SOURCES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
set_property(TARGET ${PROJECT_NAME} PROPERTY CUDA_RESOLVE_DEVICE_SYMBOLS ON)

## Create an executable node for creating a rolling map in ROS
add_executable(${PROJECT_NAME}_node src/rolling_map_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})

# -------------------------------------
