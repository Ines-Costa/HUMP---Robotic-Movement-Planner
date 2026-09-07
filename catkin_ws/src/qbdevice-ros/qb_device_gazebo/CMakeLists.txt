cmake_minimum_required(VERSION 3.0.2 FATAL_ERROR)
project(qb_device_gazebo VERSION 3.0.6 LANGUAGES CXX)

# Dependency Settings
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    gazebo_ros_control
    controller_manager
)
find_package(gazebo REQUIRED)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    combined_robot_hw_sim
  CATKIN_DEPENDS
    roscpp
    controller_manager
    gazebo_ros_control
)

# Exported libraries
add_library(combined_robot_hw_sim
  src/combined_robot_hw_sim.cpp
)

target_link_libraries(combined_robot_hw_sim
  ${catkin_LIBRARIES}
)

# C++ Settings
if (CMAKE_VERSION VERSION_LESS "3.1")
  target_compile_options(combined_robot_hw_sim
    PUBLIC
      "-std=c++17"
  )
else ()
  set_property(
    TARGET
      combined_robot_hw_sim
    PROPERTY CXX_STANDARD 17
  )

  set_property(
    TARGET
      combined_robot_hw_sim
    PROPERTY CXX_STANDARD_REQUIRED ON
  )
endif ()

# Installation
install(
  TARGETS combined_robot_hw_sim
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY
    include/qb_device_gazebo/
  DESTINATION
    ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY
    worlds/
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
)

install(
  FILES
    plugin.xml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
