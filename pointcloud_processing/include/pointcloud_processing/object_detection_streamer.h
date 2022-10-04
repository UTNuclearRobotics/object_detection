///////////////////////////////////////////////////////////////////////////////
//      Title     : object_detection_streamer.h
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
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <vision_msgs/Detection3DArray.h>

#include <chrono>

// Darknet detection
#include <darknet_ros_msgs/BoundingBoxes.h>

// PCL specific includes
// #include <pcl/ModelCoefficients.h>
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
#include <pcl/common/io.h>

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
class ObjectDetectionStreamer
{
public:
  // basic constructor
  ObjectDetectionStreamer();

  // destructor
  ~ObjectDetectionStreamer();

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
  std::set<std::string> object_classes;

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

  // class variables
  bool bbox_edge_;
  int bbox_pixels_to_pad_;
  double pcl_stale_time_, detection_confidence_threshold_, bbox_edge_x_, bbox_edge_y_;

  // the optical frame of the RGB camera (not the camera base frame)
  std::string camera_optical_frame_, robot_frame_, lidar_frame_;

  // ROS Nodehandle
  ros::NodeHandle private_nh_;

  // Publishers
  ros::Publisher detections_pub_, lidar_fov_pub_, lidar_det_pub_;

  // Subscribers
  ros::Subscriber bbox_sub_, cloud_sub_, camera_info_sub_;

  // Initialize transform listener
  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // caches for callback data
  darknet_ros_msgs::BoundingBoxes current_boxes_;
  sensor_msgs::CameraInfo camera_info_;

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
 * @brief Extract from a pointcloud those points that are within the FoV of the camera.
 * @param input The input pointcloud
 * @param pixel_coordinates A vector of pixelspace coordinates. These correspond by index
 *                          with the points ins input
 * @param height The pixel height of the camera
 * @param width The pixel width of the camera
 * @return A pointcloud containing only the points within the camera FoV.
 */
  CloudPtr filterPointsInFoV(
    const CloudPtr input, const std::vector<PixelCoords> & pixel_coordinates, const int height,
    const int width);

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
};

}  // namespace object_detection