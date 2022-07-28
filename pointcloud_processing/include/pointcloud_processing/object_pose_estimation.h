///////////////////////////////////////////////////////////////////////////////
//      Title     : object_pose_estimation.h
//      Platforms : Ubuntu 64-bit
//      Copyright : Copyright© The University of Texas at Austin, 2021. All rights reserved.
//
//          All files within this directory are subject to the following, unless an alternative
//          license is explicitly included within the text of each file.
//
//          This software and documentation constitute an unpublished work
//          and contain valuable trade secrets and proprietary information
//          belonging to the University. None of the foregoing material may be
//          copied or duplicated or disclosed without the express, written
//          permission of the University. THE UNIVERSITY EXPRESSLY DISCLAIMS ANY
//          AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND DOCUMENTATION,
//          INCLUDING ANY WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
//          PARTICULAR PURPOSE, AND WARRANTIES OF PERFORMANCE, AND ANY WARRANTY
//          THAT MIGHT OTHERWISE ARISE FROM COURSE OF DEALING OR USAGE OF TRADE.
//          NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF
//          THE SOFTWARE OR DOCUMENTATION. Under no circumstances shall the
//          University be liable for incidental, special, indirect, direct or
//          consequential damages or loss of profits, interruption of business,
//          or related expenses which may arise from use of software or documentation,
//          including but not limited to those resulting from defects in software
//          and/or documentation, or loss or inaccuracy of data of any kind.
//
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_processing/Snapshot.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <std_srvs/Empty.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <vision_msgs/Detection3DArray.h>

#include <chrono>

// Darknet detection
#include <darknet_ros_msgs/BoundingBoxes.h>

// PCL specific includes
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
// #include <pcl/sample_consensus/model_types.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>

/**
 * Node API
 * 
 * ROS Parameters
 * 
 * ROS Subscribers
 * 
 * ROS Publishers
 * 
*/

namespace object_detection
{
class ObjectPoseEstimation
{
public:
  // basic constructor
  ObjectPoseEstimation();

  // destructor
  ~ObjectPoseEstimation();

  // typedefs
  typedef pcl::PointXYZRGB PointType;
  typedef pcl::PointCloud<PointType> Cloud;
  typedef Cloud::Ptr CloudPtr;

  typedef int ObjectClassID;
  typedef std::string ObjectClassName;
  typedef std::map<ObjectClassName, ObjectClassID> ObjectsMap;

  // ROS Nodehandle
  ros::NodeHandle nh_;

  //map
  ObjectsMap object_classes;

  // convert classes map function (need to access in main)
  ObjectsMap convertClassesMap(std::map<std::string, std::string> input);

  /**
   * @brief Runs all of the top level detection code.  Initiates and manages the object detection demo
   */
  void initiateDetections();

private:
// macros
#define UNKNOWN_OBJECT_ID -1

  typedef struct
  {
    int32_t x;
    int32_t y;
    double z;
  } PixelCoords;

  typedef struct
  {
    std::string object_class;
    sensor_msgs::PointCloud2 cloud;        // cloud should be in map frame
    geometry_msgs::PointStamped position;  // should be in map frame
    geometry_msgs::TransformStamped robot_tf;
    geometry_msgs::TransformStamped camera_tf;
    geometry_msgs::TransformStamped inv_robot_tf;
    geometry_msgs::TransformStamped inv_camera_tf;
    darknet_ros_msgs::BoundingBox bbox;

  } UnassignedDetection;

  typedef struct
  {
    int object_id;
    std::string object_class;
    geometry_msgs::PointStamped position;
    sensor_msgs::PointCloud2 cloud;      // filtered cloud in map frame
    sensor_msgs::PointCloud2 raw_cloud;  // cloud of all raw data in map frame with no filtering
    std::vector<darknet_ros_msgs::BoundingBox> bboxes;
    std::vector<sensor_msgs::PointCloud2>
      fov_clouds;  // these clouds must be saved in the map frame
    std::vector<geometry_msgs::TransformStamped> robot_tfs;
    std::vector<geometry_msgs::TransformStamped> camera_tfs;
    std::vector<geometry_msgs::TransformStamped> inv_robot_tfs;
    std::vector<geometry_msgs::TransformStamped> inv_camera_tfs;
    std::vector<sensor_msgs::Image> images;
    std::vector<sensor_msgs::CompressedImage> cmpr_images;

    ros::Publisher cloud_pub;
    ros::Publisher raw_cloud_pub;
    ros::Publisher obj_position_pub;
    std::vector<ros::Publisher> poses_puber;
    std::vector<ros::Publisher> fov_pc_puber;
    std::vector<ros::Publisher> img_puber;
    std::vector<ros::Publisher> cimg_puber;
  } ObjectDetection;

  // class variables
  bool debug_viz_;
  geometry_msgs::TransformStamped current_robot_tf_, current_camera_tf_, prev_robot_tf_,
    current_inv_cam_tf_, current_inv_rob_tf_, current_lidar_tf_;

  // the optical frame of the RGB camera (not the camera base frame)
  std::string camera_optical_frame_, map_frame_, robot_frame_, lidar_frame_;

  // ROS Nodehandle
  ros::NodeHandle private_nh_;

  // Publishers
  ros::Publisher detected_objects_pub_, lidar_fov_pub_, lidar_bbox_pub_, uobj_pub_, detection_pub_;

  // Subscribers
  ros::Subscriber bbox_sub_, cloud_sub_, camera_info_sub_;

  // Service
  ros::ServiceServer save_server_;
  ros::ServiceClient snapshot_client_;

  // Initialize transform listener
  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // caches for callback data
  darknet_ros_msgs::BoundingBoxes current_boxes_;
  sensor_msgs::CameraInfo camera_info_;

  // object detection variables
  std::vector<ObjectPoseEstimation::UnassignedDetection> unassigned_detections_;
  std::vector<ObjectPoseEstimation::ObjectDetection> object_detections_;

  // debug timers
  std::chrono::high_resolution_clock debug_clock_;

  /**
   * @brief Callback function for bounding boxes detected by Darknet
   * @param msg Bounding boxes
   * @post The message is copied to a local cache
   */
  void bBoxCb(const darknet_ros_msgs::BoundingBoxesConstPtr & msg);

  /**
   * @brief Callback function for the RGB camera info
   * @details Specifically, this node needs the width, height,
   *          and intrinsic matrix of the camera.
   * @param msg Camera info
   * @post The message is copied to a local cache
   */
  void cameraInfoCb(const sensor_msgs::CameraInfoConstPtr msg);

  /**
   * @brief Updates the current_robot_pose_ in the map frame
   * @param map_frame The map frame in string form
   * @param robot_frame The robot base frame in string form
   */
  geometry_msgs::TransformStamped updateTf(const std::string frame1, const std::string frame2);

  /**
   * @brief Checks if the robot position has moved beyond a distance threshold in the map frame
   * @param map_frame The map frame in string form
   * @param robot_frame The robot base frame in string form
   * @param dist_threshold The distance threshold to check against
   * @return True if moved beyond the distance threshold, False if not.
   */
  bool robotHasMoved(const double dist_threshold);

  /**
   * @brief Checks if the robot orientation has varied beyond a certain angle in the past frame
   * @param robot_turning_threshold The turning threshold to check against
   * @return True if rotated beyond the rotational threshold, False if not.
   */
  bool robotIsTurning(const double robot_turning_threshold);

  /**
   * @brief Convert a cartesian point in the camera optical frame to (x,y) pixel coordinates.
   * @details Note: Make sure the point is in the optical camera frame, and not the link frame.
   *          We also provide the depth value of the pixel.
   * @param point The cartesian point to convert
   * @param camera_info The camera information. Specifically we use the
   *                    intrinsic matrix.
   * @return The (x,y) pixel coordinates, plus depth values.
   */
  inline PixelCoords poseToPixel(
    const PointType & point, const sensor_msgs::CameraInfo & camera_info);

  /**
   * @brief Convert a pointcloud into (x,y,depth) pixel coordinate space.
   * @details Note: Make sure the cloud is in the optical camera frame, and not the link frame.
   * @param cloud The cartesian pointcloud to convert
   * @param camera_info The camera information. Specifically we use the
   *                    intrinsic matrix.
   * @return A vector of (x,y,depth) pixel coordinates. Index order is preserved.
   */
  std::vector<PixelCoords> convertCloudToPixelCoords(
    const CloudPtr cloud, const sensor_msgs::CameraInfo & camera_info);

  /**
   * @brief Check a map of known object classes to retreive the class ID for an object class name.
   * @param class_name A known object class name
   * @param map The map of object class names to class IDs
   * @return The class ID. -1 indicates that class_name was not a key in the map.
   */
  ObjectClassID getObjectID(const ObjectClassName class_name, const ObjectsMap & map);

  /**
   * @brief Extract from a pointcloud those points that are within a rectangular bounding box.
   * @param input The input pointcloud
   * @param pixel_coordinates A vector of pixelspace coordinates. These correspond by index
   *                          with the points ins input
   * @param xmin The x-pixel lower bound of the rectangle
   * @param xmax The x-pixel upper bound of the rectangle
   * @param ymin The y-pixel lower bound of the rectangle
   * @param ymax The y-pixel upper bound of the rectangle
   * @return A pointcloud containing only the points within the bounding box.
   */
  CloudPtr filterPointsInBox(
    const CloudPtr input, const std::vector<PixelCoords> & pixel_coordinates, const int xmin,
    const int xmax, const int ymin, const int ymax);

  bool transformPointCloud2(sensor_msgs::PointCloud2 & pointcloud, const std::string object_frame);

  geometry_msgs::TransformStamped invertTransform(const geometry_msgs::TransformStamped & tf_ins);

  /**
   * @brief Callback function for the pointclouds
   * @details This does the core processing to locate objects in the cloud
   * @param input_cloud The pointcloud
   */
  void pointCloudCb(sensor_msgs::PointCloud2 input_cloud);

  /**
   * @brief Determines if the cloud_in can be matched to any object in the object_detections_ vector that is of the same object_class passed in.
   * @param object_class The class of the object ("chair", "fire hydrant", "microwave", ...etc)
   * @param cloud_in The cloud of the unassigned object in the map frame.  This cloud must be reduced down to the bounding box of the object otherwise it isn't that useful.
   * @return Returns object_id of object in the object_detections_ vector if position is within dist threshold.  Returns 0 if not matched.
    */
  int isRegisteredObject(const std::string object_class, sensor_msgs::PointCloud2 cloud_in);

  /**
   * @brief Determines if the position in pos_in is close to a object of type object_class
   * @param object_class The class of the object ("chair", "fire hydrant", "microwave", ...etc)
   * @param pos_in The current position of the object to compare against in the map frame
   * @param dist The distance threshold being checked
   * @return Returns object_id of object in the object_detections_ vector if position is within dist threshold.  Returns 0 if not close enough.
   */
  int isCloseToObject(
    const std::string object_class, const geometry_msgs::PointStamped pos_in, const double dist);

  /**
   * @brief Update the current vector of objects with newly registered object information
   * @param uobj The unassigned detection that has been examined and matches with the object in obj_index
   * @param obj_index The INDEX of the object that uobj has been matched with in the object_detections_ vector.  INDEX not ID!!!
   */
  void updateRegisteredObject(
    const ObjectPoseEstimation::UnassignedDetection uobj, const int obj_index);

  /**
   * @brief Convert the object detections data into a Detection3DArray and publish
   */
  void publishDetectionArray();
  void publishDetection(const int obj_index);

  /**
   * @brief Save the object detections data into bag file
   */
  void saveBag();

  /**
   * @brief Offers a ros service client to trigger a rosbag save of the object detections data
   */
  bool saveBagClient(std_srvs::Empty::Request & req, std_srvs::Empty::Response & res);
};

}  // namespace object_detection