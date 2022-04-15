#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <vision_msgs/Detection2DArray.h>

// Darknet detection
#include <darknet_ros_msgs/BoundingBoxes.h>

// PCL specific includes
#include <pcl/ModelCoefficients.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#include <chrono>

// macros
#define UNKNOWN_OBJECT_ID -1

// typedefs
typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef Cloud::Ptr CloudPtr;

typedef int ObjectClassID;
typedef std::string ObjectClassName;
typedef std::map<ObjectClassName, ObjectClassID> ObjectsMap;

// if true, we will publish the FoV and bounding box pointclouds
bool debug_lidar_viz;
std::chrono::high_resolution_clock debug_clock_;

// the optical frame of the RGB camera (not the link frame)
std::string rgb_optical_frame_;

// ROS Nodehandle
ros::NodeHandle * nh;

// Publishers
ros::Publisher detected_objects_pub;
ros::Publisher lidar_fov_pub_;
ros::Publisher lidar_bbox_pub_;

// Initialize transform listener
std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
tf2_ros::Buffer tf_buffer_;

// caches for callback data
darknet_ros_msgs::BoundingBoxes current_boxes_;
sensor_msgs::CameraInfo camera_info_;

// map of our target classes and their class IDs
ObjectsMap object_classes;

// depth-image pixel value
typedef struct
{
  int32_t x;
  int32_t y;
  double z;
} PixelCoords;

/**
 * @brief Callback function for bounding boxes detected by Darknet
 * @param msg Bounding boxes
 * @post The message is copied to a local cache
 */
void bBoxCb(const darknet_ros_msgs::BoundingBoxesConstPtr & msg) { current_boxes_ = *msg; }

/**
 * @brief Callback function for the RGB camera info
 * @details Specifically, this node needs the width, height,
 *          and intrinsic matrix of the camera.
 * @param msg Camera info
 * @post The message is copied to a local cache
 */
void cameraInfoCb(const sensor_msgs::CameraInfoConstPtr msg) { camera_info_ = *msg; }

/**
 * @brief Convert a cartesian point in the camera optical frame to (x,y,depth) pixel coordinates.
 * @details Note: Make sure the point is in the optical camera frame, and not the link frame.
 *          We also provide the depth value of the pixel.
 * @param point The cartesian point to convert
 * @param camera_info The camera information. Specifically we use the
 *                    intrinsic matrix.
 * @return The (x,y,depth) pixel coordinates.
 */
inline PixelCoords poseToPixel(const PointType & point, const sensor_msgs::CameraInfo & camera_info)
{
  PixelCoords result;

  // use the projection equation here: https://en.wikipedia.org/wiki/Camera_resectioning
  result.x = camera_info.K[0] * point.x / point.z + camera_info.K[2];
  result.y = camera_info.K[4] * point.y / point.z + camera_info.K[5];
  result.z = point.z;

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
std::vector<PixelCoords> convertCloudToPixelCoords(
  const CloudPtr cloud, const sensor_msgs::CameraInfo & camera_info)
{
  std::vector<PixelCoords> output;
  output.reserve(cloud->size());

  for (const PointType & point : cloud->points) {
    output.push_back(poseToPixel(point, camera_info));
  }

  return output;
}

/**
 * @brief Check a map of known object classes to retreive the class ID for an object class name.
 * @param class_name A known object class name
 * @param map The map of object class names to class IDs
 * @return The class ID. -1 indicates that class_name was not a key in the map.
 */
ObjectClassID getObjectID(const ObjectClassName class_name, const ObjectsMap & map)
{
  ObjectClassID class_id;

  try {
    class_id = map.at(class_name);
  } catch (const std::out_of_range & e) {
    ROS_ERROR("getObjectID() - No class ID found for name %s", class_name.c_str());
    ROS_ERROR("%s", e.what());
    return ObjectClassID(UNKNOWN_OBJECT_ID);
  }

  return class_id;
}

/**
 * @brief Convert a std::map<std::string, std::string> to
 *        std::map<std::string, int>
 * @param input The map to convert.
 * @return The converted map
 */
ObjectsMap convertClassesMap(std::map<std::string, std::string> input)
{
  ObjectsMap output;

  for (const std::map<std::string, std::string>::value_type & pair : input) {
    output[pair.first] = std::stoi(pair.second);
  }

  return output;
}

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
  const int width)
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
CloudPtr filterPointsInBox(
  const CloudPtr input, const std::vector<PixelCoords> & pixel_coordinates, const int xmin,
  const int xmax, const int ymin, const int ymax)
{
  auto t1 = debug_clock_.now();
  pcl::PointIndices::Ptr indices_in_bbox(new pcl::PointIndices());
  indices_in_bbox->indices.reserve(input->size());

  for (int i = 0; i < pixel_coordinates.size(); ++i) {
    if (
      pixel_coordinates[i].z > 0 && pixel_coordinates[i].x > xmin &&
      pixel_coordinates[i].x < xmax && pixel_coordinates[i].y > ymin &&
      pixel_coordinates[i].y < ymax) {
      indices_in_bbox->indices.push_back(i);
    }
  }

  CloudPtr cloud_in_bbox(new Cloud);
  pcl::ExtractIndices<PointType> bbox_filter;

  // Extract the inliers  of the ROI
  bbox_filter.setInputCloud(input);
  bbox_filter.setIndices(indices_in_bbox);
  bbox_filter.setNegative(false);
  bbox_filter.filter(*cloud_in_bbox);
  auto t2 = debug_clock_.now();
  // if (std::chrono::duration_cast<std::chrono::milliseconds>(t2- t1).count() > 1) {
  ROS_ERROR_STREAM(
    "TIME FILTER POINTS IN BOX: "
    << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count());
  // }
  return cloud_in_bbox;
}

bool transformPointCloud2(sensor_msgs::PointCloud2 & pointcloud, const std::string target_frame)
{
  geometry_msgs::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform(
      target_frame, tf2::getFrameId(pointcloud), ros::Time(0), ros::Duration(0.1));
  } catch (tf2::TransformException & ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }

  tf2::doTransform(pointcloud, pointcloud, transform);

  return true;
}

/**
 * @brief Callback function for the pointclouds
 * @details This does the core processing to locate objects in the cloud
 * @param input_cloud The pointcloud
 */
void pointCloudCb(sensor_msgs::PointCloud2 input_cloud)
{
  auto t1 = debug_clock_.now();
  // check that we've received bounding boxes
  if (current_boxes_.bounding_boxes.empty()) {
    return;
  }

  // check that we've received camera info
  if (camera_info_.height == 0 || camera_info_.width == 0) {
    return;
  }

  const ros::Time now = ros::Time::now();

  double confidence_threshold;
  nh->param("confidence_threshold", confidence_threshold, 0.75);

  // transform the pointcloud into the RGB optical frame
  if (tf2::getFrameId(input_cloud) != rgb_optical_frame_) {
    if (!transformPointCloud2(input_cloud, rgb_optical_frame_)) {
      return;
    }
  }

  // Convert sensor_msgs::PointCloud2 to pcl::PointCloud
  CloudPtr cloud(new Cloud);
  pcl::fromROSMsg(input_cloud, *cloud);

  // remove NaN points from the cloud
  CloudPtr cloud_nan_filtered(new Cloud);
  CloudPtr nanfiltered_cloud(new Cloud);
  std::vector<int> rindices;
  pcl::removeNaNFromPointCloud(*cloud, *cloud_nan_filtered, rindices);

  // produce pixel-space coordinates
  const std::vector<PixelCoords> pixel_coordinates =
    convertCloudToPixelCoords(cloud_nan_filtered, camera_info_);

  // -------------------Extraction of points in the camera FOV------------------------------
  const CloudPtr cloud_fov = filterPointsInFoV(
    cloud_nan_filtered, pixel_coordinates, camera_info_.height, camera_info_.width);

  if (cloud_fov->empty()) {
    ROS_WARN("No pointcloud data found within the camera field of view.");
    return;
  }

  if (debug_lidar_viz) {
    sensor_msgs::PointCloud2 pc2;
    pcl::toROSMsg(*cloud_fov, pc2);
    lidar_fov_pub_.publish(pc2);
  }

  // output
  vision_msgs::Detection2DArray detected_objects;
  detected_objects.header.stamp = now;
  detected_objects.header.frame_id = tf2::getFrameId(input_cloud);
  detected_objects.detections.reserve(current_boxes_.bounding_boxes.size());

  // produce pixel-space coordinates
  const std::vector<PixelCoords> pixel_coordinates_fov =
    convertCloudToPixelCoords(cloud_fov, camera_info_);

  /////////////////////////////////////////////////////////////
  for (const darknet_ros_msgs::BoundingBox & box : current_boxes_.bounding_boxes) {
    const ObjectClassID id = getObjectID(box.Class, object_classes);

    // do we meet the threshold for a confirmed detection?
    if (box.probability >= confidence_threshold && id != UNKNOWN_OBJECT_ID) {
      // ----------------------Extract points in the bounding box-----------
      const CloudPtr cloud_in_bbox =
        filterPointsInBox(cloud_fov, pixel_coordinates_fov, box.xmin, box.xmax, box.ymin, box.ymax);

      if (debug_lidar_viz) {
        sensor_msgs::PointCloud2 pc2;
        pcl::toROSMsg(*cloud_in_bbox, pc2);
        lidar_bbox_pub_.publish(pc2);
      }

      // ----------------------Compute centroid-----------------------------
      Eigen::Vector4f centroid_out;
      pcl::compute3DCentroid(*cloud_in_bbox, centroid_out);

      // add to the output
      vision_msgs::Detection2D object;
      object.bbox.center.x = (box.xmax + box.xmin) / 2;
      object.bbox.center.y = (box.ymax + box.ymin) / 2;
      object.bbox.size_x = box.xmax - box.xmin;
      object.bbox.size_y = box.ymax - box.ymin;

      vision_msgs::ObjectHypothesisWithPose hypothesis;
      hypothesis.id = id;
      hypothesis.score = box.probability;
      hypothesis.pose.pose.position.x = centroid_out[0];
      hypothesis.pose.pose.position.y = centroid_out[1];
      hypothesis.pose.pose.position.z = centroid_out[2];
      hypothesis.pose.pose.orientation.w = 1;

      object.results.push_back(hypothesis);

      detected_objects.detections.push_back(object);
    }
  }

  // publish results
  detected_objects_pub.publish(detected_objects);

  auto t2 = debug_clock_.now();

  ROS_ERROR_STREAM(
    "TIME PC CB: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count());
}

int main(int argc, char ** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "rgb_plus_lidar_processing");
  nh = new ros::NodeHandle("~");

  if (argc != 2) {
    ROS_INFO("usage: rosrun pointcloud_processing rgb_plus_lidar_processing rgb_frame");
    return 1;
  }

  rgb_optical_frame_ = std::string(argv[1]);

  nh->param("debug_lidar_viz", debug_lidar_viz, {true});

  std::map<std::string, std::string> temp_map;
  if (!nh->hasParam("/sep_processing_node/object_classes")) {
    ROS_ERROR("Failed to load dictionary parameter 'object_classes'.");
    return 1;
  }
  nh->getParam("/sep_processing_node/object_classes", temp_map);

  try {
    object_classes = convertClassesMap(temp_map);
  } catch (std::invalid_argument ex) {
    ROS_FATAL("Invalid object_classes parameter.");
    return 1;
  }

  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

  // Initialize subscribers to darknet detection and pointcloud
  ros::Subscriber bbox_sub =
    nh->subscribe<darknet_ros_msgs::BoundingBoxes>("bounding_boxes", 10, bBoxCb);
  ros::Subscriber cloud_sub =
    nh->subscribe<sensor_msgs::PointCloud2>("pointcloud", 10, pointCloudCb);
  ros::Subscriber camera_info_sub =
    nh->subscribe<sensor_msgs::CameraInfo>("camera_info", 100, cameraInfoCb);

  // Create a ROS publisher for the output point cloud
  detected_objects_pub = nh->advertise<vision_msgs::Detection2DArray>("detected_objects", 1);

  if (debug_lidar_viz) {
    lidar_fov_pub_ = nh->advertise<sensor_msgs::PointCloud2>("lidar_fov", 1);
    lidar_bbox_pub_ = nh->advertise<sensor_msgs::PointCloud2>("lidar_bbox", 1);
  }

  ros::spin();

  return 0;
}
