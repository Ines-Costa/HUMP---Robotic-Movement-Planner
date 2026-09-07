cmake_minimum_required(VERSION 3.0.2 FATAL_ERROR)
project(qb_device_description VERSION 3.0.6 LANGUAGES CXX)

# Dependency Settings
find_package(catkin REQUIRED)

catkin_package()

# Installation
install(
  DIRECTORY
    launch/
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)