cmake_minimum_required(VERSION 2.8.3)
project(itia_deflection_control)
add_compile_options(-std=c++11 -funroll-loops -Wall -Ofast)
set(CMAKE_BUILD_TYPE Release)


find_package(catkin REQUIRED COMPONENTS
  controller_interface
  controller_manager
  hardware_interface
  itia_controllers_and_filters
  itia_futils
  itia_nodelet_hw_interface
  itia_rutils
  pluginlib
  roscpp
  sensor_msgs
  std_msgs
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES itia_deflection_control
  CATKIN_DEPENDS controller_interface controller_manager hardware_interface itia_controllers_and_filters itia_futils itia_nodelet_hw_interface itia_rutils pluginlib roscpp sensor_msgs std_msgs
  DEPENDS 
)

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/itia_deflection_control.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
