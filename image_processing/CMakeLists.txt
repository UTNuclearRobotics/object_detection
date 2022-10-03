cmake_minimum_required(VERSION 3.0.2)
project(image_processing)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  roscpp
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
  INCLUDE_DIRS include
  # LIBRARIES
  CATKIN_DEPENDS sensor_msgs roscpp image_transport message_generation
  # DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(image_snapshot src/image_snapshot.cpp)
target_link_libraries(image_snapshot ${catkin_LIBRARIES})
add_dependencies(image_snapshot ${${PROJECT_NAME}_exported_targets} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)