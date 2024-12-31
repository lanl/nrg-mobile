## CREATE PACKAGE ===============================

cmake_minimum_required(VERSION 3.18)
project(rolling_map LANGUAGES C CXX)
enable_language(CUDA OPTIONAL)

# Compile as C++ 20, available in ROS2 Humble and newer
set(CMAKE_CXX_STANDARD 20)

# Determine if CUDA is available
include(CheckLanguage)
check_language(CUDA)
get_property(languages GLOBAL PROPERTY ENABLED_LANGUAGES)
if(CMAKE_CUDA_COMPILER AND ("CUDA" IN_LIST languages))
  set(CUDA_ENABLE true)
  set(CMAKE_POSITION_INDEPENDENT_CODE ON)
  add_definitions(-DUSE_CUDA)
  message("compiling rolling map with Cuda")
else()
  set(CUDA_ENABLE false)
  message("compiling rolling map without Cuda")
endif()

# -------------------------------------

## REQUIRED PACKAGES ============================

# Find catkin macros and libraries
find_package(ament_cmake_auto)
ament_auto_find_build_dependencies()

# Used in std::for_each
find_package(TBB REQUIRED)

# Use openmp for CPU parallelism
find_package(OpenMP REQUIRED)
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

find_package(cpp_timer QUIET)
if (NOT cpp_timer_FOUND)
  message("Compiling without timer")
endif()

# -------------------------------------

# Message generation
rosidl_generate_interfaces(
  ${PROJECT_NAME}
    "srv/Box.srv"
  DEPENDENCIES
    geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

# -------------------------------------

## LIBRARIES AND EXECUTABLES ===================

## Set up a C++ library for the rolling map class functionality
set(LIB_SOURCES
  src/rolling_map.cpp
)

if(CUDA_ENABLE)
  set(CUDA_SEPERABLE_COMPILATION ON)
  set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -rdc=true")
  list(APPEND LIB_SOURCES src/cast_rays.cu src/cuda_voxel_grid.cu src/update_position.cu src/rolling_map.cu)
endif()

ament_auto_add_library(${PROJECT_NAME}_lib ${LIB_SOURCES})
set_property(TARGET ${PROJECT_NAME} PROPERTY CUDA_RESOLVE_DEVICE_SYMBOLS ON)

## Create an executable node for creating a rolling map in ROS
ament_auto_add_executable(${PROJECT_NAME}_node src/rolling_map_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${cpp_typesupport_target})

if (cpp_timer_FOUND)
  add_definitions(-DTIMEIT)
  target_link_libraries(${PROJECT_NAME}_node cpp_timer)
endif()

# -------------------------------------

ament_auto_package()
