cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_processing)

if(NOT "${CMAKE_CXX_STANDARD}")
  set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)

# add_compile_options(-std=c++11)
# # Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  darknet_ros_msgs
  message_filters
  visualization_msgs
  tf2_sensor_msgs
  vision_msgs
  geometry_msgs
  image_transport
  image_processing
  detection_msgs
  cv_bridge
  std_msgs
  rospack
)

# # System dependencies are found with CMake's conventions
# find_package(PCL REQUIRED)

# # Uncomment this if the package has a setup.py. This macro ensures
# # modules and global scripts declared therein get installed
# # See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# ##################################
# # catkin specific configuration ##
# ##################################
catkin_package(
  CATKIN_DEPENDS
  pcl_conversions
  pcl_ros
  roscpp
  message_filters
  darknet_ros_msgs
  tf2_sensor_msgs
  vision_msgs
  geometry_msgs
  image_transport
  image_processing
  detection_msgs
  cv_bridge
  std_msgs
  rospack
)

# ##########
# # Build ##
# ##########

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(object_detection_streamer src/object_detection_streamer.cpp)
add_executable(object_pose_estimation src/object_pose_estimation.cpp)
add_executable(image_buffer src/image_buffer.cpp)

target_link_libraries(object_detection_streamer ${catkin_LIBRARIES})
target_link_libraries(object_pose_estimation ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(image_buffer ${catkin_LIBRARIES})

add_dependencies(object_pose_estimation ${${PROJECT_NAME}_exported_objects} ${catkin_EXPORTED_TARGETS})

# # Add folders to be run by python nosetests
# catkin_add_nosetests(test)