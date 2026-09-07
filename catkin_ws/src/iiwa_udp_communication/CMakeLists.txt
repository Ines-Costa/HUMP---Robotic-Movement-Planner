cmake_minimum_required(VERSION 3.0.2)
project(iiwa_udp_communication)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/app_enable
  scripts/start_app
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
