# Object Detection package for ROS

## Overview

This is a ROS package developed object object detection (via RGB camera) fused with object pose estimation (via Lidar).

It is assumed that we have pre-trained Object detection system such as YOLO (Darknet).


### YOLO

To compile this package, you need to build [darknet_ros_msgs](https://github.com/leggedrobotics/darknet_ros/tree/master/darknet_ros_msgs)

Pleae follow instructions from https://github.com/leggedrobotics/darknet_ros

For more information about YOLO, Darknet, available training data and training YOLO see the following link: [YOLO: Real-Time Object Detection](http://pjreddie.com/darknet/yolo/).

### Setup

Ensure all values in object_dection/config/pose_estimation.yaml are appropriate.

`debug_viz` - Publishes all of the associated pointclouds and frames for each detected object while the robot is running for live debugging.  
`camera_optical_fram` - The ***optical*** frame of the camera. Generally this frame is in Z-forward, X-right relative to the base camera link.  
`robot_frame` - The base frame of the robot  
`map_frame` - The map frame (or world frame) which is the static reference frame for robot navigation  
`robot_movement_threshold` - The minimum amount of distace the robot needs to travel before it begins looking for a new object.  It is reset once a object is seen.  
`confidence_threshold` - The minimum probability amount required for a object detection to be observed and its pose estimated  
`distance_between_objects` - If a object is seen but cannot be matched with a previously observed object it will check if the estimated pose of the new detection is within this distance from a previously observed object.  If it is then this new data will be tossed out as an anomaly that it could not be registered.  Generally this would occur if robot localization is not perfect.  
`bbox_pixel_padding` - The number of pixels added to pad the bounding boxes and increase their size.  This is useful if robot localization is not great and objects are not being consistently registered.  
`bbox_edge_x` - The amount (0 to 1) of the camera width of the camera image to ignore on the horizontal edges.  If any of the vertices of the object bounding box is seen close enough to `bbox_edge_x` * `camera_info.width` the object detection will be ignored.  Example 0.1 will ignore any object detection whose bounding box intersects with the outer 10% of the horizontal portion of the camera image.  
`bbox_edge_y` - The amount (0 to 1) of the camera width of the camera image to ignore on the vertical edges.  If any of the vertices of the object bounding box is seen close enough to `bbox_edge_y` * `camera_info.height` the object detection will be ignored.  Example 0.1 will ignore any object detection whose bounding box intersects with the outer 10% of the vertical portion of the camera image.  
`robot_turning_threshold` - The amount (in radians) that the robot's orientation can rotate before looking for a object.  Generally, it is object pose estimate works best if the robot is not rotating while a object is detected.  This allows for the user to tune how much their robot rotation affects object pose estimate.  
`sleep_period` - The time (in seconds) to sleep in between ros spins.  
`pointcloud_stale_time` - The time threshold to use raw lidar pointclouds in pose estimation.  Any lidar data that is older than this time will be ignored when a object is found.  
`save_bag_on_shutdown` - Saves a bag of all in the data in object_detections_ when the node is shutdown. A bag can also be saved by calling the `save_bag` service.  
`object_classes` - A map of the object classes to an associated index to be used when detecting objects.  Each class must have a unique index indetifier for it to be used in pose estimation.  

Ensure the object detection launch file (object_detection/launch/object_detection.launch) is appropriate for your setup. By default it launches a D435i camera and Yolo.

### Test

1) Ensure robot is running (with localization and an appropriate lidar)
2) `roslaunch object_detection object_detection.launch`
3) `roslaunch object_detection pose_estimation.launch`
