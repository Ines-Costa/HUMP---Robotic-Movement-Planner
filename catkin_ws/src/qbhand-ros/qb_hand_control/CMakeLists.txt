cmake_minimum_required(VERSION 3.0.3 FATAL_ERROR)
project(qb_hand_control VERSION 3.0.3 LANGUAGES CXX)

# Dependency Settings
find_package(catkin REQUIRED)

catkin_package()

# Installation

install(
  DIRECTORY
    config/
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
install(
  DIRECTORY
    launch/
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
