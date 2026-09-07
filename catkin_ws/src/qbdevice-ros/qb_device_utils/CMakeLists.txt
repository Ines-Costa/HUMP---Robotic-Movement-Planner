cmake_minimum_required(VERSION 3.0.2 FATAL_ERROR)
project(qb_device_utils VERSION 3.0.6 LANGUAGES CXX)

# Dependency Settings
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
)

# Installation
install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION
    ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)