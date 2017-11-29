cmake_minimum_required(VERSION 2.8.3)
project(itia_joint_impedance_control)
add_compile_options(-std=c++11 -funroll-loops -Wall -Ofast)
set(CMAKE_BUILD_TYPE Release)


find_package(fourbythree_msgs  )

if (fourbythree_msgs_FOUND)
  add_definitions(-DFOURBYTHREE_MSGS)
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
    fourbythree_msgs
  )
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES itia_joint_impedance_control
    CATKIN_DEPENDS controller_interface controller_manager hardware_interface itia_controllers_and_filters itia_dynamics_core itia_futils itia_nodelet_hw_interface itia_rutils pluginlib roscpp sensor_msgs std_msgs fourbythree_msgs
    DEPENDS 
  )
else()
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
    LIBRARIES itia_joint_impedance_control
    CATKIN_DEPENDS controller_interface controller_manager hardware_interface itia_controllers_and_filters itia_dynamics_core itia_futils itia_nodelet_hw_interface itia_rutils pluginlib roscpp sensor_msgs std_msgs 
    DEPENDS 
  )
  
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  message(WARNING "FourByThreeMsgs do not exist, using standard messages. If you are working on FourByThreeProject, you need the fourbythree_msgs") 
  
endif ()



include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/itia_joint_impedance_control.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)



