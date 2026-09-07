cmake_minimum_required(VERSION 3.0.2 FATAL_ERROR)
project(qb_device_driver VERSION 3.0.6 LANGUAGES CXX)

# Dependency Settings
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    qb_device_srvs
    qb_device_utils
)
add_subdirectory(api/qbdevice-api-7.x.x)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    qb_device_srvs
    qb_device_utils
)

# Build Settings
add_executable(qb_device_communication_handler
  src/main.cpp
  src/qb_device_communication_handler.cpp
)

target_link_libraries(qb_device_communication_handler
  PRIVATE
    Serial
    qbrobotics_driver
  PUBLIC
    ${catkin_LIBRARIES}
)

add_dependencies(qb_device_communication_handler
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# C++ Settings
if (CMAKE_VERSION VERSION_LESS "3.1")
  target_compile_options(qb_device_communication_handler
    PUBLIC
      "-std=c++14"
  )
else ()
  set_property(
    TARGET
      qb_device_communication_handler
    PROPERTY CXX_STANDARD 14
  )

  set_property(
    TARGET
      qb_device_communication_handler
    PROPERTY CXX_STANDARD_REQUIRED ON
  )
endif ()

# Installation
install(
  TARGETS qb_device_communication_handler
  RUNTIME DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/
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