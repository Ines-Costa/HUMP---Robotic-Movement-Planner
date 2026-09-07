cmake_minimum_required(VERSION 3.0.2 FATAL_ERROR)
project(qb_device_srvs VERSION 3.0.6 LANGUAGES CXX)

# Dependency Settings
find_package(catkin REQUIRED
  COMPONENTS
    std_srvs
    qb_device_msgs
    message_generation
)

add_service_files(
  DIRECTORY srv
  FILES
    GetMeasurements.srv
    InitializeDevice.srv
    SetCommands.srv
    SetPID.srv
    Trigger.srv
    SetControlMode.srv
)

generate_messages(
  DEPENDENCIES
    std_srvs
    qb_device_msgs
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    std_srvs
    qb_device_msgs
    message_runtime
)

# Installation
install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION
    ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY
    srv/
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}/srv
)