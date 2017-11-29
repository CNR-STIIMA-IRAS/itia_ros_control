cmake_minimum_required(VERSION 2.8.3)
project(itia_nodelet_hw_interface)
add_compile_options(-std=c++11 -funroll-loops -Wall -Ofast)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  controller_manager
  hardware_interface
  nodelet
  roscpp
  sensor_msgs
  itia_rutils
  diagnostic_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES itia_nodelet_hw_interface
  CATKIN_DEPENDS controller_interface controller_manager hardware_interface nodelet roscpp sensor_msgs itia_rutils diagnostic_msgs
  DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(itia_nodelet_hw_interface
  src/${PROJECT_NAME}/nodelet_hw_interface.cpp
  src/${PROJECT_NAME}/main_nodelet_hw_interface.cpp
)
add_dependencies(itia_nodelet_hw_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(itia_nodelet_hw_interface
  ${catkin_LIBRARIES}
)

# add_executable(itia_nodelet_hw_interface_node src/itia_nodelet_hw_interface_node.cpp)
# add_dependencies(itia_nodelet_hw_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(itia_nodelet_hw_interface_node
#   ${catkin_LIBRARIES}
# )
