cmake_minimum_required(VERSION 2.8.3)
project(object_recognition)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Ofast")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  sensor_msgs
  roscpp
  rospy
  std_msgs
)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
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
#  LIBRARIES object_recognition
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(object_recognition
#   src/${PROJECT_NAME}/object_recognition.cpp
# )

## Declare a cpp executable
# add_executable(object_recognition_node src/object_recognition_node.cpp)
#add_library(ros_parameters src/parameters/ros_parameters.cpp)
#add_library(object_recognition_parameters src/parameters/object_recognition_parameters.cpp)
#target_link_libraries(object_recognition_parameters ros_parameters)

add_library(filter src/filtering_strategies/filter.cpp)
add_library(pass_through_filter src/filtering_strategies/pass_through_filter.cpp)
add_library(statistical_filter src/filtering_strategies/statistical_filter.cpp)
add_library(voxel_grid_filter src/filtering_strategies/voxel_grid_filter.cpp)
add_library(extract_filter src/filtering_strategies/extract_filter.cpp)
target_link_libraries(filter ${PCL_LIBRARIES} statistical_filter pass_through_filter voxel_grid_filter extract_filter)

add_library(segmentation src/segmentation_strategies/segmentation.cpp)
target_link_libraries(segmentation extract_filter)
add_library(plane_segmentation src/segmentation_strategies/plane_segmentation.cpp)
target_link_libraries(plane_segmentation extract_filter)

add_library(euclidean_segmentation src/segmentation_strategies/euclidean_segmentation.cpp)
target_link_libraries(euclidean_segmentation ${PCL_LIBRARIES})
add_library(region_growing_segmentation src/segmentation_strategies/region_growing_segmentation.cpp)
target_link_libraries(segmentation ${PCL_LIBRARIES} plane_segmentation euclidean_segmentation region_growing_segmentation)

add_library(spin_image src/descriptors/spin_image.cpp)
target_link_libraries(spin_image ${PCL_LIBRARIES} filter)

add_library(histogram_math src/util/histogram_math.cpp)

add_library(correspondence src/correspondence_strategies/correspondence.cpp)
add_library(correlation_correspondence src/correspondence_strategies/correlation_correspondence.cpp)
add_library(eigenimage_correspondence src/correspondence_strategies/eigenimage_correspondence.cpp)
target_link_libraries(eigenimage_correspondence ${PCL_LIBRARIES})


target_link_libraries(correspondence ${PCL_LIBRARIES} correlation_correspondence eigenimage_correspondence histogram_math)


add_library(point_cloud_listener src/listeners/point_cloud_listener.cpp)
add_library(visualization_marker src/msg/visualization_marker.cpp)

add_library(category src/util/category.cpp)
target_link_libraries(category  ${PCL_LIBRARIES} visualization_marker)

add_library(object_recognition_controller src/controller/object_recognition_controller.cpp)
target_link_libraries(object_recognition_controller 
                        category 
                        filter 
                        segmentation 
                        spin_image
                        correspondence)
                        
add_library(object_recognition_behaviour src/behaviour/object_recognition_behaviour.cpp)
target_link_libraries(object_recognition_behaviour 
                        point_cloud_listener visualization_marker 
                        object_recognition_controller)
  
add_library(training_files_parser src/parsers/training_files_parser.cpp)
add_library(model_parser src/parsers/model_parser.cpp)
target_link_libraries(model_parser training_files_parser)
add_library(ros_parameters_parser src/parsers/ros_parameters_parser.cpp)

target_link_libraries(ros_parameters_parser ${catkin_LIBRARIES} ${PCL_LIBRARIES}
                        category
                        filter 
                        segmentation 
                        spin_image
                        correspondence)

add_executable(spin_recognition src/spin_recognition.cpp)
target_link_libraries(spin_recognition ${catkin_LIBRARIES} ${PCL_LIBRARIES} 
                      object_recognition_behaviour 
                      ros_parameters_parser model_parser training_files_parser)

add_executable(spin_training src/spin_training.cpp)
target_link_libraries(spin_training ${PCL_LIBRARIES} ros_parameters_parser model_parser training_files_parser)

# add_dependencies(talker beginner_tutorials_generate_messages_cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(object_recognition_node object_recognition_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(object_recognition_node
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
# install(TARGETS object_recognition object_recognition_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_object_recognition.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
