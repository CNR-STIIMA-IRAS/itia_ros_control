cmake_minimum_required(VERSION 2.8.3)
project(itia_helios_controller)

SET(CMAKE_CXX_FLAGS "-std=c++11 -funroll-loops")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -O4")

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  controller_manager
  hardware_interface
  itia_futils
  itia_helios
  itia_rutils
  pluginlib
  roscpp
  sensor_msgs
  itia_nodelet_hw_interface
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES itia_helios_controller
  CATKIN_DEPENDS controller_interface controller_manager hardware_interface itia_futils itia_helios itia_rutils pluginlib roscpp sensor_msgs itia_nodelet_hw_interface
  DEPENDS 
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(itia_helios_controller
  src/${PROJECT_NAME}/itia_helios_controller.cpp
  src/${PROJECT_NAME}/itia_fir_controller.cpp
  src/${PROJECT_NAME}/itia_fir_controller_joint_teleop.cpp
  src/${PROJECT_NAME}/itia_fir_controller_cart_teleop.cpp
  
)
add_dependencies(itia_helios_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(itia_helios_controller
    ${catkin_LIBRARIES}
)
