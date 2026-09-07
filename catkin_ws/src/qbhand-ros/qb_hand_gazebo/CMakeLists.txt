cmake_minimum_required(VERSION 3.0.3 FATAL_ERROR)
project(qb_hand_gazebo VERSION 3.0.3 LANGUAGES CXX)

# Dependency Settings
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    gazebo_ros_control
    controller_manager
    qb_device_gazebo
    qb_device_hardware_interface
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
    qb_hand_gazebo_plugin
    qb_hand_gazebo_hardware_interface
  CATKIN_DEPENDS
    roscpp
    controller_manager
    gazebo_ros_control
    qb_device_gazebo
    qb_device_hardware_interface
)

# Exported libraries
add_library(qb_hand_gazebo_plugin
  src/qb_hand_gazebo_plugin.cpp
)

target_link_libraries(qb_hand_gazebo_plugin
  PUBLIC
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
    qb_hand_gazebo_hardware_interface
)

add_dependencies(qb_hand_gazebo_plugin
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_library(qb_hand_gazebo_hardware_interface
  src/qb_hand_gazebo_hardware_interface.cpp
)

target_link_libraries(qb_hand_gazebo_plugin
  PUBLIC
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
)

add_dependencies(qb_hand_gazebo_plugin
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# C++ Settings
if (CMAKE_VERSION VERSION_LESS "3.1")
  target_compile_options(qb_hand_gazebo_plugin
    PUBLIC
      "-std=c++17"
  )
  target_compile_options(qb_hand_gazebo_hardware_interface
    PUBLIC
      "-std=c++17"
  )
else ()
  set_property(
    TARGET
      qb_hand_gazebo_plugin
      qb_hand_gazebo_hardware_interface
    PROPERTY CXX_STANDARD 17
  )

  set_property(
    TARGET
      qb_hand_gazebo_plugin
      qb_hand_gazebo_hardware_interface
    PROPERTY CXX_STANDARD_REQUIRED ON
  )
endif ()

# Installation
install(
  TARGETS qb_hand_gazebo_plugin qb_hand_gazebo_hardware_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY
    include/qb_hand_gazebo/
  DESTINATION
    ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  FILES
    plugin.xml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
