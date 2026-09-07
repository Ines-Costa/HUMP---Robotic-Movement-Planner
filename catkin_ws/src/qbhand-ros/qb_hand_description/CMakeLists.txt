cmake_minimum_required(VERSION 3.0.3 FATAL_ERROR)
project(qb_hand_description VERSION 3.0.3 LANGUAGES CXX)

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

install(
  DIRECTORY
    meshes/
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
)

install(
  DIRECTORY
    rviz/
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)

install(
  DIRECTORY
    urdf/
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)
