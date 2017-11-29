cmake_minimum_required(VERSION 2.8.3)
project(itia_model_feedforward)
add_compile_options(-std=c++11 -funroll-loops -Wall -Ofast)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  controller_manager
  hardware_interface
  itia_controllers_and_filters
  itia_dynamics_core
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
  LIBRARIES itia_model_feedforward
  CATKIN_DEPENDS controller_interface controller_manager hardware_interface itia_controllers_and_filters itia_dynamics_core itia_futils itia_nodelet_hw_interface itia_rutils pluginlib roscpp sensor_msgs std_msgs
  DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/itia_model_feedforward.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


# add_executable(${PROJECT_NAME}_node src/itia_model_feedforward_node.cpp)
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
