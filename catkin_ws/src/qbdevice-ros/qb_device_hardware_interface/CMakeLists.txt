cmake_minimum_required(VERSION 3.0.2 FATAL_ERROR)
project(qb_device_hardware_interface VERSION 3.0.6 LANGUAGES CXX)

# Dependency Settings
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    hardware_interface
    joint_limits_interface
    transmission_interface
    control_msgs
    urdf
    qb_device_msgs
    qb_device_srvs
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
    hardware_interface
    joint_limits_interface
    transmission_interface
    control_msgs
    urdf
    qb_device_msgs
    qb_device_srvs
)

# Exported libraries
add_library(${PROJECT_NAME}
  src/qb_device_hardware_interface.cpp
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
  target_compile_options(${PROJECT_NAME}
    PUBLIC
      "-std=c++14"
  )
else ()
  set_property(
    TARGET
      ${PROJECT_NAME}
    PROPERTY CXX_STANDARD 14
  )

  set_property(
    TARGET
      ${PROJECT_NAME}
    PROPERTY CXX_STANDARD_REQUIRED ON
  )
endif ()

# Installation
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

# Tests
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

#TODO: catkin_add_gmock: http://docs.ros.org/melodic/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-add-gmock

#  # explicit gmock build FIXME: outside sourcing
#  include_directories(/usr/include/gmock /usr/src/gmock)
#  find_library(GMOCK_LIBRARIES libgmock)
#  if(NOT GMOCK_LIBRARIES)
#    add_library(libgmock /usr/src/gmock/src/gmock-all.cc)
#  endif()

  catkin_add_gtest(${PROJECT_NAME}_unit_tests
    src/test/unit_tests.cpp
  )

  target_link_libraries(${PROJECT_NAME}_unit_tests
#    libgmock
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )

  if (CMAKE_VERSION VERSION_LESS "3.1")
    target_compile_options(${PROJECT_NAME}_unit_tests
      PUBLIC
        "-std=c++14"
    )
  else ()
    set_property(
      TARGET
        ${PROJECT_NAME}_unit_tests
      PROPERTY CXX_STANDARD 14
    )

    set_property(
      TARGET
        ${PROJECT_NAME}_unit_tests
      PROPERTY CXX_STANDARD_REQUIRED ON
    )
  endif ()
endif()