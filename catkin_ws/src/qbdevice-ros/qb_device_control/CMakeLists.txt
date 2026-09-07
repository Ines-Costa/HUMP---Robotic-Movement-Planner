cmake_minimum_required(VERSION 3.0.2 FATAL_ERROR)
project(qb_device_control VERSION 3.0.6 LANGUAGES CXX)

# Dependency Settings
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    actionlib
    combined_robot_hw
    control_msgs
    controller_manager
    qb_device_hardware_interface
    qb_device_utils
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    actionlib
    combined_robot_hw
    control_msgs
    controller_manager
    qb_device_hardware_interface
    qb_device_utils
)

# Build Settings
add_executable(qb_device_default
  src/main.cpp
  src/qb_device_control.cpp
)

target_link_libraries(qb_device_default
  PUBLIC
    ${catkin_LIBRARIES}
)

add_dependencies(qb_device_default
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# Exported libraries
add_library(${PROJECT_NAME}
  src/qb_device_control.cpp
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# C++ Settings
if (CMAKE_VERSION VERSION_LESS "3.1")
  target_compile_options(qb_device_default
    PUBLIC
      "-std=c++14"
  )
  target_compile_options(${PROJECT_NAME}
    PUBLIC
      "-std=c++14"
  )
else ()
  set_property(
    TARGET
      qb_device_default
      ${PROJECT_NAME}
    PROPERTY CXX_STANDARD 14
  )

  set_property(
    TARGET
      qb_device_default
      ${PROJECT_NAME}
    PROPERTY CXX_STANDARD_REQUIRED ON
  )
endif ()

# Installation
install(
  TARGETS qb_device_default
  RUNTIME DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION
    ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY
    launch/
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)