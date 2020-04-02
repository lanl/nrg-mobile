##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(sweep_gui)

set(CMAKE_CXX_FLAGS "-std=c++11 -g")

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS roscpp full_coverage)

# QT
find_package(Qt5 COMPONENTS Widgets Core REQUIRED)

# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 
catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS roscpp full_coverage
  DEPENDS Qt5Widgets Qt5Core)

set(QT_LIBRARIES Qt5::Widgets Qt5::Core)

include_directories(${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR} include)

qt5_add_resources(RESOURCE_FILES
  resources/images.qrc
)

qt5_wrap_cpp(MOC_FILES
  include/${PROJECT_NAME}/demo_window.hpp
  include/${PROJECT_NAME}/led.hpp
  include/${PROJECT_NAME}/zoom_view.hpp
  include/${PROJECT_NAME}/qnode.hpp
)

qt5_wrap_ui(UI_FILES
  ui/demo_window.ui
)

set(SRC_FILES
  src/demo_window.cpp
  src/led.cpp
  src/qnode.cpp
  src/zoom_view.cpp
  ${MOC_FILES}
  ${UI_FILES}
  ${RESOURCE_FILES}
)

add_library(${PROJECT_NAME}_lib ${SRC_FILES})

add_executable(${PROJECT_NAME} src/main.cpp)

add_dependencies(${PROJECT_NAME}_lib ${${PROJECT_NAME}_lib_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_lib ${QT_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PROJECT_NAME}_lib)

