cmake_minimum_required(VERSION 3.0.2)
project(image_processing)


if(NOT "${CMAKE_CXX_STANDARD}")
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# add_compile_options(-std=c++11)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  roscpp
  std_srvs
  image_transport
  message_generation
)

add_service_files(
  FILES
  Snapshot.srv
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
  sensor_msgs
  roscpp
  std_srvs
  image_transport
  message_generation
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(image_snapshot src/image_snapshot.cpp)

target_link_libraries(image_snapshot ${catkin_LIBRARIES})

add_dependencies(image_snapshot ${${PROJECT_NAME}_exported_targets} ${catkin_EXPORTED_TARGETS})

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)