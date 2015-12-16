cmake_minimum_required(VERSION 2.8.3)
project(localization)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf
)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Ofast")
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
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
#   std_msgs
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
#  INCLUDE_DIRS include
#  LIBRARIES localization
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(particle src/util/particle.cpp)
add_library(occupancy_map src/util/occupancy_map.cpp)


add_library(sensor_update src/sensor_update_strategies/sensor_update.cpp)
add_library(likelihood_sensor_update src/sensor_update_strategies/likelihood_sensor_update.cpp)
add_library(correlation_sensor_update src/sensor_update_strategies/correlation_sensor_update.cpp)
target_link_libraries(sensor_update occupancy_map likelihood_sensor_update correlation_sensor_update)

add_library(motion_update src/motion_update_strategies/motion_update.cpp)
add_library(odometry_motion_update src/motion_update_strategies/odometry_motion_update.cpp)
add_library(velocity_motion_update src/motion_update_strategies/velocity_motion_update.cpp)
target_link_libraries(motion_update odometry_motion_update velocity_motion_update)

add_library(sampling src/sampling_strategies/sampling.cpp)
add_library(stochastic_universal_sampling src/sampling_strategies/stochastic_universal_sampling.cpp)
add_library(roulette_wheel_sampling src/sampling_strategies/roulette_wheel_sampling.cpp)
target_link_libraries(sampling stochastic_universal_sampling roulette_wheel_sampling)

add_library(pose_filter src/filtering_strategies/pose_filter.cpp)
target_link_libraries(pose_filter sensor_update motion_update sampling)
add_library(particle_filter src/filtering_strategies/particle_filter.cpp)
target_link_libraries(particle_filter particle pose_filter)

add_library(behaviour src/behaviour/localization_behaviour.cpp)
target_link_libraries(behaviour ${catkin_LIBRARIES} pose_filter)

add_library(filter_parser src/parsers/filter_parser.cpp)
target_link_libraries(filter_parser pose_filter particle_filter)

add_library(motion_update_parser src/parsers/motion_update_parser.cpp)
target_link_libraries(motion_update_parser motion_update)
add_library(sensor_update_parser src/parsers/sensor_update_parser.cpp)
target_link_libraries(sensor_update_parser sensor_update)
add_library(sampling_parser src/parsers/sampling_parser.cpp)
target_link_libraries(sampling_parser sampling)

add_library(ros_parameter_parser src/parsers/ros_parameter_parser.cpp)
target_link_libraries(ros_parameter_parser filter_parser motion_update_parser sensor_update_parser sampling_parser)

add_library(ros_topic_parser src/parsers/ros_topic_parser.cpp)

#target_link_libraries(pose_filter sensor_update motion_update sampling)


## Declare a cpp executable
add_executable(localization src/localization.cpp)
add_executable(scan_remapper src/scan_remapper.cpp)

target_link_libraries(localization ${catkin_LIBRARIES} ros_parameter_parser ros_topic_parser behaviour)
target_link_libraries(scan_remapper ${catkin_LIBRARIES} ros_topic_parser)
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(localization_node localization_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(localization_node
#   ${catkin_LIBRARIES}
# )

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
# install(TARGETS localization localization_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_localization.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
