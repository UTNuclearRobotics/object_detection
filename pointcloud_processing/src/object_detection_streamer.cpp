///////////////////////////////////////////////////////////////////////////////
//      Title     : object_detection_streamer.cpp
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

#include <pointcloud_processing/object_detection_streamer.h>

namespace object_detection
{
ObjectDetectionStreamer::ObjectDetectionStreamer() : nh_(""), private_nh_("~")
{
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
  bbox_sub_ = nh_.subscribe("bounding_boxes", 1, &ObjectDetectionStreamer::bBoxCb, this);
  cloud_sub_ = nh_.subscribe("pointcloud", 1, &ObjectDetectionStreamer::pointCloudCb, this);
  camera_info_sub_ = nh_.subscribe("camera_info", 1, &ObjectDetectionStreamer::cameraInfoCb, this);

  detections_pub_ = private_nh_.advertise<vision_msgs::Detection3DArray>("detections", 1);
  lidar_fov_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>("lidar_fov", 1);
  lidar_det_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>("lidar_det", 1);

  private_nh_.param<std::string>("robot_frame", robot_frame_, "base_link");
  private_nh_.param<std::string>(
    "camera_optical_frame", camera_optical_frame_, "camera_optical_link");

  ROS_DEBUG_STREAM("ROBOT FRAME PARAM: " << robot_frame_);
  ROS_DEBUG_STREAM("CAMERA OPT FRAME PARAM: " << camera_optical_frame_);

  private_nh_.param<int>("bbox_pixel_padding", bbox_pixels_to_pad_, 0);
  private_nh_.param<double>("pointcloud_stale_time", pcl_stale_time_, 0.05);
  private_nh_.param<double>("confidence_threshold", detection_confidence_threshold_, 0.75);

  private_nh_.param<bool>("bbox_edge", bbox_edge_, true);
  private_nh_.param<double>("bbox_edge_x", bbox_edge_x_, 0.1);
  private_nh_.param<double>("bbox_edge_y", bbox_edge_y_, 0.01);
}

// destructor
ObjectDetectionStreamer::~ObjectDetectionStreamer() {}

/**
   * @brief Callback function for bounding boxes detected by Darknet
   * @param msg Bounding boxes
   * @post The message is copied to a local cache
   */
void ObjectDetectionStreamer::bBoxCb(const darknet_ros_msgs::BoundingBoxesConstPtr & msg)
{
  current_boxes_ = *msg;
}

/**
   * @brief Callback function for the RGB camera info
   * @details Specifically, this node needs the width, height,
   *          and intrinsic matrix of the camera.
   * @param msg Camera info
   * @post The message is copied to a local cache
   */
void ObjectDetectionStreamer::cameraInfoCb(const sensor_msgs::CameraInfoConstPtr msg)
{
  camera_info_ = *msg;
}

/**
   * @brief Convert a cartesian point in the camera optical frame to (x,y) pixel coordinates.
   * @details Note: Make sure the point is in the optical camera frame, and not the link frame.
   *          We also provide the depth value of the pixel.
   * @param point The cartesian point to convert
   * @param camera_info The camera information. Specifically we use the
   *                    intrinsic matrix.
   * @return The (x,y) pixel coordinates, plus depth values.
   */
inline ObjectDetectionStreamer::PixelCoords ObjectDetectionStreamer::poseToPixel(
  const ObjectDetectionStreamer::PointType & point, const sensor_msgs::CameraInfo & camera_info)
{
  ObjectDetectionStreamer::PixelCoords result;
  auto t1 = debug_clock_.now();

  result.x = camera_info.K[0] * point.x / point.z + camera_info.K[2];
  result.y = camera_info.K[4] * point.y / point.z + camera_info.K[5];
  result.z = point.z;
  auto t2 = debug_clock_.now();
  if (std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() > 1) {
    ROS_DEBUG_STREAM(
      "TIME POSE TO PIXEL: "
      << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count());
  }
  return result;
}

/**
   * @brief Convert a pointcloud into (x,y,depth) pixel coordinate space.
   * @details Note: Make sure the cloud is in the optical camera frame, and not the link frame.
   * @param cloud The cartesian pointcloud to convert
   * @param camera_info The camera information. Specifically we use the
   *                    intrinsic matrix.
   * @return A vector of (x,y,depth) pixel coordinates. Index order is preserved.
   */
std::vector<ObjectDetectionStreamer::PixelCoords>
ObjectDetectionStreamer::convertCloudToPixelCoords(
  const ObjectDetectionStreamer::CloudPtr cloud, const sensor_msgs::CameraInfo & camera_info)
{
  auto t1 = debug_clock_.now();
  std::vector<ObjectDetectionStreamer::PixelCoords> output;
  output.reserve(cloud->size());

  for (const ObjectDetectionStreamer::PointType & point : cloud->points) {
    output.push_back(poseToPixel(point, camera_info));
  }
  auto t2 = debug_clock_.now();
  // std::chrono::duration_cast<std::chrono::milliseconds> tme = t2 - t1;
  if (std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() > 1) {
    ROS_DEBUG_STREAM(
      "TIME CONVERT CLOUD TO PIXELS: "
      << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count());
  }
  return output;
}

ObjectDetectionStreamer::CloudPtr ObjectDetectionStreamer::filterPointsInFoV(
  const ObjectDetectionStreamer::CloudPtr input, const std::vector<PixelCoords> & pixel_coordinates,
  const int height, const int width)
{
  pcl::PointIndices::Ptr indices_in_fov(new pcl::PointIndices());
  indices_in_fov->indices.reserve(input->size());

  for (int i = 0; i < pixel_coordinates.size(); ++i) {
    if (
      pixel_coordinates[i].z > 0 && pixel_coordinates[i].x >= 0 &&
      pixel_coordinates[i].x <= width && pixel_coordinates[i].y >= 0 &&
      pixel_coordinates[i].y <= height) {
      indices_in_fov->indices.push_back(i);
    }
  }

  CloudPtr cloud_in_fov(new Cloud);
  pcl::ExtractIndices<PointType> camera_fov_filter;

  // Extract the inliers  of the ROI
  camera_fov_filter.setInputCloud(input);
  camera_fov_filter.setIndices(indices_in_fov);
  camera_fov_filter.setNegative(false);
  camera_fov_filter.filter(*cloud_in_fov);

  return cloud_in_fov;
}

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
ObjectDetectionStreamer::CloudPtr ObjectDetectionStreamer::filterPointsInBox(
  const ObjectDetectionStreamer::CloudPtr input,
  const std::vector<ObjectDetectionStreamer::PixelCoords> & pixel_coordinates, const int xmin,
  const int xmax, const int ymin, const int ymax)
{
  auto t1 = debug_clock_.now();
  pcl::PointIndices::Ptr indices_in_bbox(new pcl::PointIndices());
  indices_in_bbox->indices.reserve(input->size());

  int pixels_to_pad;
  private_nh_.param<int>("bbox_pixel_padding", pixels_to_pad, 0);

  for (int i = 0; i < pixel_coordinates.size(); ++i) {
    if (
      pixel_coordinates[i].z > 0 && pixel_coordinates[i].x > (xmin - pixels_to_pad) &&
      pixel_coordinates[i].x < (xmax + pixels_to_pad) &&
      pixel_coordinates[i].y > (ymin - pixels_to_pad) &&
      pixel_coordinates[i].y < (ymax + pixels_to_pad)) {
      indices_in_bbox->indices.push_back(i);
    }
  }

  ObjectDetectionStreamer::CloudPtr cloud_in_bbox(new ObjectDetectionStreamer::Cloud);
  pcl::ExtractIndices<ObjectDetectionStreamer::PointType> bbox_filter;

  // Extract the inliers  of the ROI
  bbox_filter.setInputCloud(input);
  bbox_filter.setIndices(indices_in_bbox);
  bbox_filter.setNegative(false);
  bbox_filter.filter(*cloud_in_bbox);

  auto t2 = debug_clock_.now();
  if (std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() > 1) {
    ROS_DEBUG_STREAM(
      "TIME FILTER POINTS IN BOX: "
      << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count());
  }
  return cloud_in_bbox;
}

bool ObjectDetectionStreamer::transformPointCloud2(
  sensor_msgs::PointCloud2 & pointcloud, const std::string object_frame)
{
  auto t1 = debug_clock_.now();
  geometry_msgs::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform(
      object_frame, tf2::getFrameId(pointcloud), ros::Time(0), ros::Duration(0.1));
  } catch (tf2::TransformException & ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }

  tf2::doTransform(pointcloud, pointcloud, transform);

  auto t2 = debug_clock_.now();
  ROS_DEBUG_STREAM(
    "TIME TF PC: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count());
  return true;
}

/**
   * @brief Callback function for the pointclouds
   * @details This does the core processing to locate objects in the cloud
   * @param input_cloud The pointcloud
   */
void ObjectDetectionStreamer::pointCloudCb(sensor_msgs::PointCloud2 input_cloud)
{
  auto t1 = debug_clock_.now();
  if (lidar_frame_.empty()) {
    lidar_frame_ = input_cloud.header.frame_id;
    return;
  }

  // check if pointcloud is stale
  if (ros::Time::now().toSec() - input_cloud.header.stamp.toSec() > pcl_stale_time_) {
    return;
  }

  // check that we've received bounding boxes
  if (current_boxes_.bounding_boxes.empty()) {
    return;
  }

  // check that we've received camera info
  if (camera_info_.height == 0 || camera_info_.width == 0) {
    return;
  }

  // transform the pointcloud into the RGB optical frame
  if (tf2::getFrameId(input_cloud) != camera_optical_frame_) {
    if (!transformPointCloud2(input_cloud, camera_optical_frame_)) {
      return;
    }
  }

  // Convert sensor_msgs::PointCloud2 to pcl::PointCloud
  ObjectDetectionStreamer::CloudPtr cloud(new ObjectDetectionStreamer::Cloud);
  pcl::fromROSMsg(input_cloud, *cloud);

  // remove NaN points from the cloud
  ObjectDetectionStreamer::CloudPtr cloud_nan_filtered(new ObjectDetectionStreamer::Cloud);
  std::vector<int> rindices;
  pcl::removeNaNFromPointCloud(*cloud, *cloud_nan_filtered, rindices);

  // produce pixel-space coordinates
  const std::vector<ObjectDetectionStreamer::PixelCoords> pixel_coordinates =
    convertCloudToPixelCoords(cloud_nan_filtered, camera_info_);

  // -------------------Extraction of points in the camera FOV------------------------------
  const CloudPtr cloud_fov = filterPointsInFoV(
    cloud_nan_filtered, pixel_coordinates, camera_info_.height, camera_info_.width);

  if (cloud_fov->empty()) {
    ROS_WARN("No pointcloud data found within the camera field of view.");
    return;
  }

  sensor_msgs::PointCloud2 pc2_out;
  pcl::toROSMsg(*cloud_fov, pc2_out);
  lidar_fov_pub_.publish(pc2_out);

  ObjectDetectionStreamer::CloudPtr output_cloud(new ObjectDetectionStreamer::Cloud);
  pcl::toROSMsg(*output_cloud, pc2_out);

  vision_msgs::Detection3DArray detections;
  detections.header.stamp = ros::Time::now();
  detections.header.frame_id = robot_frame_;
  detections.detections.reserve(current_boxes_.bounding_boxes.size());

  // check if any bounding boxes are on or near the edge of the camera image.  If so remove them and return if none left
  /////////////////////////////////////////////////////////////
  for (const darknet_ros_msgs::BoundingBox & box : current_boxes_.bounding_boxes) {
    // do we meet the threshold for a confirmed detection?
    if (box.probability >= detection_confidence_threshold_ && object_classes.count(box.Class)) {
      if (bbox_edge_) {
        // check for bounding boxes being close to edges
        if (box.xmin < camera_info_.width * bbox_edge_x_) {
          ROS_DEBUG("BBOX EDGE LEFT");
          continue;
        }
        if (box.xmax > (camera_info_.width - (camera_info_.width * bbox_edge_x_))) {
          ROS_DEBUG("BBOX EDGE RIGHT");
          continue;
        }

        if (box.ymin < camera_info_.height * bbox_edge_y_) {
          ROS_DEBUG("BBOX EDGE TOP");
          continue;
        }
        if (box.ymax > (camera_info_.height - (camera_info_.height * bbox_edge_y_))) {
          ROS_DEBUG("BBOX EDGE BOTTOM");
          continue;
        }
      }

      // ----------------------Extract points in the bounding box-----------
      const ObjectDetectionStreamer::CloudPtr cloud_in_bbox = filterPointsInBox(
        cloud_nan_filtered, pixel_coordinates, box.xmin, box.xmax, box.ymin, box.ymax);

      // ----------------------Compute centroid-----------------------------
      Eigen::Vector4f centroid_out;
      pcl::compute3DCentroid(*cloud_in_bbox, centroid_out);
      geometry_msgs::Point center_pt;
      center_pt.x = centroid_out[0];
      center_pt.y = centroid_out[1];
      center_pt.z = centroid_out[2];

      // before adding new detection must convert filtered cloud into map frame
      sensor_msgs::PointCloud2 obj_cloud;
      pcl::toROSMsg(*cloud_in_bbox, obj_cloud);
      transformPointCloud2(obj_cloud, robot_frame_);

      // add new detection to unassigned detections vector
      vision_msgs::Detection3D new_detection;
      new_detection.header.stamp = ros::Time::now();
      new_detection.header.frame_id = obj_cloud.header.frame_id;
      new_detection.results[0].id = box.id;
      new_detection.results[0].score = box.probability;
      new_detection.results[0].pose.pose.position = center_pt;
      new_detection.results[0].pose.pose.orientation.w = 1;
      new_detection.bbox.center.position = center_pt;
      new_detection.bbox.center.orientation.w = 1;
      new_detection.source_cloud = obj_cloud;

      PointType min_pt, max_pt;
      pcl::getMinMax3D(*cloud_in_bbox, min_pt, max_pt);
      new_detection.bbox.size.x = max_pt.x - min_pt.x;
      new_detection.bbox.size.y = max_pt.y - min_pt.y;
      new_detection.bbox.size.z = max_pt.z - min_pt.z;

      detections.detections.push_back(new_detection);

      if (current_boxes_.bounding_boxes[0] == box) {
        pc2_out = obj_cloud;
      } else {
        pcl::concatenatePointCloud(obj_cloud, pc2_out, pc2_out);
      }
    }
  }

  lidar_det_pub_.publish(pc2_out);
  detections_pub_.publish(detections);

  auto t2 = debug_clock_.now();
  ROS_DEBUG_STREAM(
    "TIME PCL2 CB: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count());
}  // namespace object_detection

}  // namespace object_detection

int main(int argc, char ** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "object_detection_streamer");

  object_detection::ObjectDetectionStreamer node;

  std::vector<std::string> object_classes;
  while (!node.nh_.hasParam("object_detection_streamer/object_classes")) {
    ROS_ERROR(
      "Waiting for 'object_classes' parameter to begin object detection search and pose "
      "estimation.");
    ros::Duration(3.0).sleep();
  }
  node.nh_.getParam("object_detection_streamer/object_classes", object_classes);

  if (object_classes.empty()) {
    ROS_FATAL(
      "'object_classes' parameter is empty.  This means there are no object types to search for.  "
      "Shutting down node.");
    return 1;
  }

  std::copy(
    object_classes.begin(), object_classes.end(),
    std::inserter(node.object_classes, node.object_classes.end()));

  ros::spin();

  return 0;
}
