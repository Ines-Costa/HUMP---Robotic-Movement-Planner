cmake_minimum_required(VERSION 2.8.3)
set(PROJECT_NAME aros_moveit_planner)
project(${PROJECT_NAME})

SET(CMAKE_CXX_FLAGS "-g -Wall -std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
    COMPONENTS
    roscpp
    moveit_msgs
    moveit_core
    moveit_ros_planning_interface
    cmake_modules
    actionlib
    actionlib_msgs
    pluginlib
    geometric_shapes
)

include_directories(${EIGEN3_INCLUDE_DIR})
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
INCLUDE_DIRS include
LIBRARIES
    humanoid_planner
    visualization_tools
    path_trajectory_planner
    trajectory_planner
    kinematics_helper
CATKIN_DEPENDS 
    moveit_core
    moveit_ros_planning_interface
    moveit_ros_perception
    actionlib
    actionlib_msgs
    geometric_shapes
    interactive_markers
#  DEPENDS system_lib
)

##############################################################################
# Settings
##############################################################################

## Tests
# TEST=0. do not run the tests
# TEST=1, run the tests
set(TEST 1)

## Documentation
# DOC=0. do not make the documentation
# DOC=1, make the documentation
set(DOC 1)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

# sources
##file(GLOB_RECURSE HPP_SOURCES RELATIVE ${PROJECT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.hpp)
##file(GLOB_RECURSE CPP_SOURCES RELATIVE ${PROJECT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)
## Declare a C++ library
##add_library(${PROJECT_NAME}
##    ${HPP_SOURCES}
##    ${CPP_SOURCES}
##)

## Kinematic helper library
file(GLOB_RECURSE KIN_HPP_SOURCES RELATIVE ${PROJECT_SOURCE_DIR} FOLLOW_SYMLINKS
    include/${PROJECT_NAME}/kinematics_helper.hpp
    include/${PROJECT_NAME}/common.hpp
    )
file(GLOB_RECURSE KIN_CPP_SOURCES RELATIVE ${PROJECT_SOURCE_DIR} FOLLOW_SYMLINKS
    src/kinematics_helper.cpp
    src/common.cpp
    )
add_library(kinematics_helper ${KIN_CPP_SOURCES} ${KIN_HPP_SOURCES})
target_link_libraries(kinematics_helper ${catkin_LIBRARIES})

## Trajectory planner library
file(GLOB_RECURSE TRAJ_HPP_SOURCES RELATIVE ${PROJECT_SOURCE_DIR} FOLLOW_SYMLINKS
    include/${PROJECT_NAME}/trajectory_planner.hpp
    include/${PROJECT_NAME}/base_planner.hpp
    ${KIN_HPP_SOURCES}
    )
file(GLOB_RECURSE TRAJ_CPP_SOURCES RELATIVE ${PROJECT_SOURCE_DIR} FOLLOW_SYMLINKS
     src/trajectory_planner.cpp
     ${KIN_CPP_SOURCES}
    )
add_library(trajectory_planner ${TRAJ_CPP_SOURCES} ${TRAJ_HPP_SOURCES})
target_link_libraries(trajectory_planner ${catkin_LIBRARIES})


## Path Trajectory planner library
file(GLOB_RECURSE PTRAJ_HPP_SOURCES RELATIVE ${PROJECT_SOURCE_DIR} FOLLOW_SYMLINKS
     include/${PROJECT_NAME}/path_trajectory_planner.hpp
     ${TRAJ_HPP_SOURCES}
     )
file(GLOB_RECURSE PTRAJ_CPP_SOURCES RELATIVE ${PROJECT_SOURCE_DIR} FOLLOW_SYMLINKS
     src/path_trajectory_planner.cpp
     ${TRAJ_CPP_SOURCES}
    )
add_library(path_trajectory_planner ${PTRAJ_CPP_SOURCES} ${PTRAJ_HPP_SOURCES})
target_link_libraries(path_trajectory_planner ${catkin_LIBRARIES})

## Visualization tools library
file(GLOB_RECURSE VIS_HPP_SOURCES RELATIVE ${PROJECT_SOURCE_DIR} FOLLOW_SYMLINKS
     include/${PROJECT_NAME}/visualization_tools.hpp
     )
file(GLOB_RECURSE VIS_CPP_SOURCES RELATIVE ${PROJECT_SOURCE_DIR} FOLLOW_SYMLINKS
     src/visualization_tools.cpp
    )
add_library(visualization_tools ${VIS_CPP_SOURCES} ${VIS_HPP_SOURCES})
target_link_libraries(visualization_tools ${catkin_LIBRARIES})

## Humanoid MoveIt! Planner Library
file(GLOB_RECURSE HP_HPP_SOURCES RELATIVE ${PROJECT_SOURCE_DIR} FOLLOW_SYMLINKS
     include/${PROJECT_NAME}/humanoid_moveit_planner.hpp
     include/${PROJECT_NAME}/kinematics_helper.hpp
     include/${PROJECT_NAME}/common.hpp
     )
file(GLOB_RECURSE HP_CPP_SOURCES RELATIVE ${PROJECT_SOURCE_DIR} FOLLOW_SYMLINKS
     src/humanoid_moveit_planner.cpp
     src/kinematics_helper.cpp
     src/common.cpp
    )
add_library(humanoid_planner ${HP_CPP_SOURCES} ${HP_HPP_SOURCES})
target_link_libraries(humanoid_planner ${catkin_LIBRARIES})


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(aros_moveit_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(aros_moveit_planner_node src/aros_moveit_planner_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(aros_moveit_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
##target_link_libraries(${PROJECT_NAME}
##   ${catkin_LIBRARIES}
##)


##############################################################################
# Testing
##############################################################################
if(TEST)
    # gtest
    enable_testing ()
    find_package(GTest REQUIRED)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
    include_directories(${GTEST_INCLUDE_DIRS})
    SET( TEST_LIBRARIES ${GTEST_LIBRARIES} )

    ## Kinematics helper tests
    set(PROJECT_KIN_TEST_NAME ${PROJECT_NAME}_kin_helper_test)
    file(GLOB KIN_TEST_CPP_FILES ${PROJECT_SOURCE_DIR}/test/src/kin_helper_test.cpp)
    ##file(GLOB KIN_TEST_HPP_FILES ${PROJECT_SOURCE_DIR}/test/include/kin_helper_test.hpp)
    add_executable(${PROJECT_KIN_TEST_NAME} ${KIN_TEST_CPP_FILES})
    target_link_libraries(${PROJECT_KIN_TEST_NAME} ${catkin_LIBRARIES} ${TEST_LIBRARIES} kinematics_helper)
    add_test (NAME Test_Kinematics_Helper COMMAND ${PROJECT_KIN_TEST_NAME})

    ## Trajectory planner tests
    set(PROJECT_TRAJ_TEST_NAME ${PROJECT_NAME}_traj_planner_test)
    file(GLOB TRAJ_TEST_CPP_FILES ${PROJECT_SOURCE_DIR}/test/src/traj_planner_test.cpp)
    ##file(GLOB TRAJ_TEST_HPP_FILES ${PROJECT_SOURCE_DIR}/test/include/traj_planner_test.hpp)
    add_executable(${PROJECT_TRAJ_TEST_NAME} ${TRAJ_TEST_CPP_FILES})
    target_link_libraries(${PROJECT_TRAJ_TEST_NAME} ${catkin_LIBRARIES} ${TEST_LIBRARIES} trajectory_planner)
    add_test (NAME Test_Trajectory_Planner COMMAND ${PROJECT_TRAJ_TEST_NAME})

    ## Path Trajectory planner tests
    set(PROJECT_PTRAJ_TEST_NAME ${PROJECT_NAME}_path_traj_planner_test)
    file(GLOB PTRAJ_TEST_CPP_FILES ${PROJECT_SOURCE_DIR}/test/src/path_traj_planner_test.cpp)
    ##file(GLOB PTRAJ_TEST_HPP_FILES ${PROJECT_SOURCE_DIR}/test/include/path_traj_planner_test.hpp)
    add_executable(${PROJECT_PTRAJ_TEST_NAME} ${PTRAJ_TEST_CPP_FILES})
    target_link_libraries(${PROJECT_PTRAJ_TEST_NAME} ${catkin_LIBRARIES} ${TEST_LIBRARIES} path_trajectory_planner)
    add_test (NAME Test_Path_Trajectory_Planner COMMAND ${PROJECT_PTRAJ_TEST_NAME})

    ## Visualization tools tests
    set(PROJECT_VIS_TEST_NAME ${PROJECT_NAME}_visualization_test)
    file(GLOB VIS_TEST_CPP_FILES ${PROJECT_SOURCE_DIR}/test/src/visualization_tools_test.cpp)
    ##file(GLOB VIS_TEST_HPP_FILES ${PROJECT_SOURCE_DIR}/test/include/visualization_tools_test.hpp)
    add_executable(${PROJECT_VIS_TEST_NAME} ${VIS_TEST_CPP_FILES})
    target_link_libraries(${PROJECT_VIS_TEST_NAME} ${catkin_LIBRARIES} ${TEST_LIBRARIES}
        trajectory_planner visualization_tools)
    add_test (NAME Test_Visualization_tools COMMAND ${PROJECT_VIS_TEST_NAME})

    ## Humanoid planner tests
    set(PROJECT_HP_TEST_NAME ${PROJECT_NAME}_humanoid_planner_test)
    file(GLOB HP_TEST_CPP_FILES ${PROJECT_SOURCE_DIR}/test/src/humanoid_planner_test.cpp)
    ##file(GLOB HP_TEST_HPP_FILES ${PROJECT_SOURCE_DIR}/test/include/humanoid_planner_test.hpp)
    add_executable(${PROJECT_HP_TEST_NAME} ${HP_TEST_CPP_FILES})
    target_link_libraries(${PROJECT_HP_TEST_NAME} ${catkin_LIBRARIES} ${TEST_LIBRARIES}
        humanoid_planner)
    add_test (NAME Test_Humanoid_planner COMMAND ${PROJECT_HP_TEST_NAME})


endif()




##############################################################################
# Documenting
##############################################################################
if(DOC)
    find_package(Doxygen)
    if(NOT DOXYGEN_FOUND)
        message(WARNING "Doxygen not found, unable to generate documentation")
    else()
        add_custom_target(doc_${PROJECT_NAME} COMMAND ${DOXYGEN_EXECUTABLE} "doxyconfig/doxyfile.ini"
            WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
            COMMENT "Building user's documentation into the doc directory ..."
        )
    endif()
endif()

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS humanoid_planner visualization_tools path_trajectory_planner trajectory_planner kinematics_helper
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.hpp"
   PATTERN ".svn"  EXCLUDE
   PATTERN ".git" EXCLUDE
 )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

