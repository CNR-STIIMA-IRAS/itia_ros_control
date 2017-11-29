cmake_minimum_required(VERSION 2.8.3)
project(itia_driver_mass_controller)
add_compile_options(-std=c++11 -funroll-loops -Wall -Ofast)
set(CMAKE_BUILD_TYPE Release)


find_package(catkin REQUIRED COMPONENTS
  controller_interface
  controller_manager
  hardware_interface
  itia_futils
  itia_helios
  itia_nodelet_hw_interface
  itia_rutils
  pluginlib
  roscpp
  sensor_msgs
  itia_controllers_and_filters
)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS controller_interface controller_manager hardware_interface itia_futils itia_helios itia_nodelet_hw_interface itia_rutils itia_controllers_and_filters pluginlib roscpp sensor_msgs
 DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/${PROJECT_NAME}.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
