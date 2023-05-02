///////////////////////////////////////////////////////////////////////////////
//      Title     : object_pose_estimation.cpp
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

#include <pointcloud_processing/object_pose_estimation.h>

namespace object_detection
{
ObjectPoseEstimation::ObjectPoseEstimation() : nh_(""), private_nh_("~")
{
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
  bbox_sub_ = nh_.subscribe("bounding_boxes", 1, &ObjectPoseEstimation::bBoxCb, this);
  cloud_sub_ = nh_.subscribe("pointcloud", 1, &ObjectPoseEstimation::pointCloudCb, this);
  camera_info_sub_ = nh_.subscribe("camera_info", 1, &ObjectPoseEstimation::cameraInfoCb, this);
  detected_objects_pub_ =
    private_nh_.advertise<detection_msgs::DetectionArray>("detected_objects", 1);
  detection_pub_ = private_nh_.advertise<detection_msgs::Detection>("detection", 1);

  save_server_ = private_nh_.advertiseService("save_bag", &ObjectPoseEstimation::saveBagCb, this);
  pub_data_server_ =
    private_nh_.advertiseService("publish_data", &ObjectPoseEstimation::pubDataCb, this);

  private_nh_.param<bool>("save_all_detection_data", save_det_data_, true);
  private_nh_.param<bool>("publish_all_detection_data", pub_det_data_, false);
  private_nh_.param<std::string>("map_frame", map_frame_, "map");
  private_nh_.param<std::string>("robot_frame", robot_frame_, "base_link");
  private_nh_.param<std::string>(
    "camera_optical_frame", camera_optical_frame_, "camera_optical_link");

  snapshot_client_ = nh_.serviceClient<image_processing::Snapshot>("image_snapshot/send_snapshot");
}

// destructor
ObjectPoseEstimation::~ObjectPoseEstimation()
{
  bool save;
  private_nh_.param<bool>("save_bag_on_shutdown", save, false);
  if (save) {
    ROS_INFO("Saving object detections bag");
    saveBag();
  }
}

/**
   * @brief Runs all of the top level detection code.  Initiates and manages the object detection demo
   */
void ObjectPoseEstimation::initiateDetections()
{
  double robot_movement_threshold, robot_turning_threshold, sleeper, dist_bet_objs,
    new_obj_dist_limit;
  private_nh_.param<double>("robot_movement_threshold", robot_movement_threshold, 2.0);
  private_nh_.param<double>("robot_turning_threshold", robot_turning_threshold, 0.1);
  private_nh_.param<double>("sleep_period", sleeper, 0.1);
  private_nh_.param<double>("distance_between_objects", dist_bet_objs, 10.0);
  private_nh_.param<double>("new_obj_dist_limit", new_obj_dist_limit, 10.0);

  private_nh_.param<int>("bbox_pixel_padding", bbox_pixels_to_pad_, 0);
  private_nh_.param<double>("pointcloud_stale_time", pcl_stale_time_, 0.05);
  private_nh_.param<double>("confidence_threshold", detection_confidence_threshold_, 0.75);

  private_nh_.param<bool>("bbox_edge", bbox_edge_, true);
  private_nh_.param<double>("bbox_edge_x", bbox_edge_x_, 0.1);
  private_nh_.param<double>("bbox_edge_y", bbox_edge_y_, 0.01);

  // init robot pose to map origin wait for robot and camera transforms for a 2.5 minutes then give up
  ROS_INFO_STREAM("Waiting for transform from " << map_frame_ << " to " << robot_frame_);
  try {
    current_robot_tf_ =
      tf_buffer_.lookupTransform(robot_frame_, map_frame_, ros::Time(0), ros::Duration(600.0));
  } catch (tf2::TransformException & ex) {
    ROS_ERROR_STREAM(
      "Could not get transform from " << map_frame_ << " to " << robot_frame_ << "! Exiting");
    ROS_ERROR("%s", ex.what());
    return;
  }
  ROS_INFO("Initial robot transform acquired");

  ROS_INFO_STREAM("Waiting for transform from " << map_frame_ << " to " << camera_optical_frame_);
  try {
    current_camera_tf_ = tf_buffer_.lookupTransform(
      camera_optical_frame_, map_frame_, ros::Time(0), ros::Duration(600.0));
  } catch (tf2::TransformException & ex) {
    ROS_ERROR_STREAM(
      "Could not get transform from " << map_frame_ << " to " << camera_optical_frame_
                                      << "! Exiting");
    ROS_ERROR("%s", ex.what());
    return;
  }
  ROS_INFO("Initial camera transform acquired");

  prev_robot_tf_ = current_robot_tf_;

  while (ros::ok()) {
    current_robot_tf_ = updateTf(robot_frame_, map_frame_);
    current_camera_tf_ = updateTf(camera_optical_frame_, map_frame_);
    current_inv_rob_tf_ = updateTf(map_frame_, robot_frame_);
    current_inv_cam_tf_ = updateTf(map_frame_, camera_optical_frame_);

    // if robot hasn't moved beyond threshold then do nothing
    if (!robotHasMoved(robot_movement_threshold)) {
      ROS_DEBUG_THROTTLE(15.0, "ROBOT HAS NOT MOVED BEYOND THRESH");
      continue;
    }

    prev_robot_tf_ = current_robot_tf_;

    // robot has now moved beyond threshold. check for objects. break when we have unassigned object detections
    while (ros::ok() && unassigned_detections_.size() == 0) {
      prev_robot_tf_ = current_robot_tf_;

      ros::Duration(sleeper).sleep();

      current_robot_tf_ = updateTf(robot_frame_, map_frame_);
      current_camera_tf_ = updateTf(camera_optical_frame_, map_frame_);
      current_inv_rob_tf_ = updateTf(map_frame_, robot_frame_);
      current_inv_cam_tf_ = updateTf(map_frame_, camera_optical_frame_);
      if (!lidar_frame_.empty()) {
        lidar_to_camera_tf_ = updateTf(camera_optical_frame_, lidar_frame_);
      }

      if (robotIsTurning(robot_turning_threshold)) {
        continue;
      }
      ros::spinOnce();
    }

    // once we have unassigned object detections need to process object comparisons
    if (object_detections_.size() > 0) {
      auto uobj = unassigned_detections_.begin();

      while (uobj != unassigned_detections_.end()) {
        // if the unassigned detection is associated with a known object then must add the data and update new existing object info
        if (const int obj_id = isRegisteredObject(uobj->object_class, uobj->cloud)) {
          ROS_INFO_STREAM(
            "Registered current view to existing object: " << obj_id << " (" << uobj->object_class
                                                           << ")");
          updateRegisteredObject(*uobj, (obj_id - 1));

          // publish results now that a object has been updated
          publishDetectionArray();
          publishDetection(obj_id - 1);

          uobj = unassigned_detections_.erase(uobj);

          // if we find have a detection of the same type that does not overlap with previous views but it really close then let's assume it is the same object and wait for a better view
        } else if (
          const int obj_id = isCloseToObject(uobj->object_class, uobj->position, dist_bet_objs)) {
          ROS_INFO_STREAM(
            "Object " << obj_id << " (" << uobj->object_class << ") "
                      << "is seen but the new FOV does not overlap with at least one of the "
                         "previous FOV's.  Ignoring current view.");
          uobj = unassigned_detections_.erase(uobj);

        } else {
          ++uobj;
        }
      }
    }

    // if we still have unassigned detections then we assign them as new objects
    if (unassigned_detections_.size() > 0) {
      for (const ObjectPoseEstimation::UnassignedDetection & uobj : unassigned_detections_) {
        // check to see if the uassigned object is past the distance limit from the robot to add in as a new object
        double dx, dy;
        dx = uobj.position.point.x - current_inv_rob_tf_.transform.translation.x;
        dy = uobj.position.point.y - current_inv_rob_tf_.transform.translation.y;
        if ((dx * dx + dy * dy) > (new_obj_dist_limit * new_obj_dist_limit)) {
          continue;
        }

        ObjectPoseEstimation::ObjectDetection new_obj;
        new_obj.object_class = uobj.object_class;
        new_obj.object_number = object_detections_.size() + 1;
        new_obj.position = uobj.position;
        new_obj.cloud = uobj.cloud;
        new_obj.raw_cloud = uobj.cloud;
        new_obj.bboxes.push_back(uobj.bbox);
        new_obj.camera_tfs.push_back(current_camera_tf_);
        new_obj.inv_camera_tfs.push_back(current_inv_cam_tf_);

        new_obj.obj_position_pub = nh_.advertise<geometry_msgs::PointStamped>(
          ("obj" + std::to_string(object_detections_.size() + 1) + "_" + new_obj.object_class +
           "_pos"),
          1, true);
        new_obj.cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>(
          ("obj" + std::to_string(object_detections_.size() + 1) + "_" + new_obj.object_class), 1,
          true);
        new_obj.raw_cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>(
          ("obj" + std::to_string(object_detections_.size() + 1) + "_" + new_obj.object_class +
           "_raw"),
          1, true);

        new_obj.obj_position_pub.publish(new_obj.position);
        new_obj.cloud_pub.publish(new_obj.cloud);
        new_obj.raw_cloud_pub.publish(new_obj.cloud);

        ROS_INFO_STREAM(
          "Assigning new object: " << new_obj.object_number << " (" << new_obj.object_class << ")");

        if (save_det_data_) {
          new_obj.robot_tfs.push_back(current_robot_tf_);
          new_obj.inv_robot_tfs.push_back(current_inv_rob_tf_);
          new_obj.fov_clouds.push_back(uobj.cloud);

          image_processing::Snapshot snapshot;
          if (snapshot_client_.call(snapshot)) {
            if (snapshot.response.img_valid) {
              // save image to file

              if (pub_det_data_) {
                new_obj.url_puber.push_back(nh_.advertise<std_msgs::String>(
                  ("obj" + std::to_string(object_detections_.size() + 1) + "_" +
                   new_obj.object_class + "_img_url"),
                  1, true));

                new_obj.url_puber[0].publish(new_obj.img_urls[0]);
              }
            }
          } else {
            ROS_DEBUG("Image Snapshot failed to call");
          }

          if (pub_det_data_) {
            new_obj.poses_puber.push_back(nh_.advertise<geometry_msgs::PoseStamped>(
              ("obj" + std::to_string(object_detections_.size() + 1) + "_" + new_obj.object_class +
               "_pose1"),
              1, true));
            new_obj.fov_pc_puber.push_back(nh_.advertise<sensor_msgs::PointCloud2>(
              ("obj" + std::to_string(object_detections_.size() + 1) + "_" + new_obj.object_class +
               "_fov_pc1"),
              1, true));

            geometry_msgs::PoseStamped temp_pose;
            temp_pose.header.stamp = ros::Time::now();
            temp_pose.header.frame_id = map_frame_;
            temp_pose.pose.position.x =
              new_obj.inv_camera_tfs[new_obj.inv_camera_tfs.size() - 1].transform.translation.x;
            temp_pose.pose.position.y =
              new_obj.inv_camera_tfs[new_obj.inv_camera_tfs.size() - 1].transform.translation.y;
            temp_pose.pose.position.z =
              new_obj.inv_camera_tfs[new_obj.inv_camera_tfs.size() - 1].transform.translation.z;
            temp_pose.pose.orientation.x =
              new_obj.inv_camera_tfs[new_obj.inv_camera_tfs.size() - 1].transform.rotation.x;
            temp_pose.pose.orientation.y =
              new_obj.inv_camera_tfs[new_obj.inv_camera_tfs.size() - 1].transform.rotation.y;
            temp_pose.pose.orientation.z =
              new_obj.inv_camera_tfs[new_obj.inv_camera_tfs.size() - 1].transform.rotation.z;
            temp_pose.pose.orientation.w =
              new_obj.inv_camera_tfs[new_obj.inv_camera_tfs.size() - 1].transform.rotation.w;
            new_obj.poses_puber[0].publish(temp_pose);
            new_obj.fov_pc_puber[0].publish(new_obj.fov_clouds[0]);
          }
        }
        object_detections_.push_back(new_obj);

        // publish results now that a object has been added
        publishDetectionArray();
        publishDetection(object_detections_.size() - 1);
      }

      unassigned_detections_.clear();
    }
  }
}

/**
   * @brief Callback function for bounding boxes detected by Darknet
   * @param msg Bounding boxes
   * @post The message is copied to a local cache
   */
void ObjectPoseEstimation::bBoxCb(const darknet_ros_msgs::BoundingBoxesConstPtr & msg)
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
void ObjectPoseEstimation::cameraInfoCb(const sensor_msgs::CameraInfoConstPtr msg)
{
  camera_info_ = *msg;
}

/**
   * @brief Updates the current_robot_tf_ in the map frame
   * @param frame1 frame1
   * @param frame2 frame2
   */
geometry_msgs::TransformStamped ObjectPoseEstimation::updateTf(
  const std::string frame1, const std::string frame2)
{
  geometry_msgs::TransformStamped temp_tf;

  try {
    temp_tf = tf_buffer_.lookupTransform(frame1, frame2, ros::Time(0), ros::Duration(0.1));
  } catch (tf2::TransformException & ex) {
    ROS_ERROR_STREAM(
      "Could not get transform from "
      << frame2 << " to " << frame1
      << "! Object detection positions may be incorrect because of old transforms!");
    ROS_ERROR("%s", ex.what());
  }

  return temp_tf;
}

/**
   * @brief Checks if the robot position has moved beyond a distance or rotational threshold in the map frame
   * @param robot_movement_threshold The distance threshold to check against
   * @return True if moved beyond the distance or rotational threshold, False if not.
   */
bool ObjectPoseEstimation::robotHasMoved(const double robot_movement_threshold)
{
  if (
    (current_robot_tf_.transform.translation.x - prev_robot_tf_.transform.translation.x) *
        (current_robot_tf_.transform.translation.x - prev_robot_tf_.transform.translation.x) +
      (current_robot_tf_.transform.translation.y - prev_robot_tf_.transform.translation.y) *
        (current_robot_tf_.transform.translation.y - prev_robot_tf_.transform.translation.y) >
    (robot_movement_threshold * robot_movement_threshold)) {
    return true;

  } else {
    return false;
  }
}

/**
   * @brief Checks if the robot orientation has varied beyond a certain angle in the past frame
   * @param robot_turning_threshold The turning threshold to check against
   * @return True if rotated beyond the rotational threshold, False if not.
   */
bool ObjectPoseEstimation::robotIsTurning(const double robot_turning_threshold)
{
  tf2::Quaternion qt1, qt2;
  tf2::fromMsg(current_robot_tf_.transform.rotation, qt1);
  tf2::fromMsg(prev_robot_tf_.transform.rotation, qt2);
  double angle = qt1.angleShortestPath(qt2);
  if (angle > robot_turning_threshold) {
    ROS_DEBUG_STREAM("ROBOT TURNING: " << angle);
    return true;
  } else {
    return false;
  }
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
inline ObjectPoseEstimation::PixelCoords ObjectPoseEstimation::poseToPixel(
  const ObjectPoseEstimation::PointType & point, const sensor_msgs::CameraInfo & camera_info)
{
  ObjectPoseEstimation::PixelCoords result;

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
std::vector<ObjectPoseEstimation::PixelCoords> ObjectPoseEstimation::convertCloudToPixelCoords(
  const ObjectPoseEstimation::CloudPtr cloud, const sensor_msgs::CameraInfo & camera_info)
{
  std::vector<ObjectPoseEstimation::PixelCoords> output;
  output.reserve(cloud->size());

  for (const ObjectPoseEstimation::PointType & point : cloud->points) {
    output.push_back(poseToPixel(point, camera_info));
  }
  return output;
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
ObjectPoseEstimation::CloudPtr ObjectPoseEstimation::filterPointsInBox(
  const ObjectPoseEstimation::CloudPtr input,
  const std::vector<ObjectPoseEstimation::PixelCoords> & pixel_coordinates, const int xmin,
  const int xmax, const int ymin, const int ymax)
{
  pcl::PointIndices::Ptr indices_in_bbox(new pcl::PointIndices());
  indices_in_bbox->indices.reserve(input->size());

  for (int i = 0; i < pixel_coordinates.size(); ++i) {
    if (
      pixel_coordinates[i].z > 0 && pixel_coordinates[i].x > (xmin - bbox_pixels_to_pad_) &&
      pixel_coordinates[i].x < (xmax + bbox_pixels_to_pad_) &&
      pixel_coordinates[i].y > (ymin - bbox_pixels_to_pad_) &&
      pixel_coordinates[i].y < (ymax + bbox_pixels_to_pad_)) {
      indices_in_bbox->indices.push_back(i);
    }
  }

  ObjectPoseEstimation::CloudPtr cloud_in_bbox(new ObjectPoseEstimation::Cloud);
  pcl::ExtractIndices<ObjectPoseEstimation::PointType> bbox_filter;

  // Extract the inliers  of the ROI
  bbox_filter.setInputCloud(input);
  bbox_filter.setIndices(indices_in_bbox);
  bbox_filter.setNegative(false);
  bbox_filter.filter(*cloud_in_bbox);

  return cloud_in_bbox;
}

/**
   * @brief Callback function for the pointclouds
   * @details This does the core processing to locate objects in the cloud
   * @param input_cloud The pointcloud
   */
void ObjectPoseEstimation::pointCloudCb(sensor_msgs::PointCloud2 input_cloud)
{
  if (lidar_frame_.empty()) {
    lidar_frame_ = input_cloud.header.frame_id;
    return;
  }

  if (ros::Time::now().toSec() - input_cloud.header.stamp.toSec() > pcl_stale_time_) {
    ROS_DEBUG_STREAM("POINTCLOUD STALE");
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
  tf2::doTransform(input_cloud, input_cloud, lidar_to_camera_tf_);

  // Convert sensor_msgs::PointCloud2 to pcl::PointCloud
  ObjectPoseEstimation::CloudPtr cloud(new ObjectPoseEstimation::Cloud);
  pcl::fromROSMsg(input_cloud, *cloud);

  // remove NaN points from the cloud
  ObjectPoseEstimation::CloudPtr cloud_nan_filtered(new ObjectPoseEstimation::Cloud);
  std::vector<int> rindices;
  pcl::removeNaNFromPointCloud(*cloud, *cloud_nan_filtered, rindices);

  // produce pixel-space coordinates
  const std::vector<ObjectPoseEstimation::PixelCoords> pixel_coordinates =
    convertCloudToPixelCoords(cloud_nan_filtered, camera_info_);

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
      const ObjectPoseEstimation::CloudPtr cloud_in_bbox = filterPointsInBox(
        cloud_nan_filtered, pixel_coordinates, box.xmin, box.xmax, box.ymin, box.ymax);

      // ----------------------Compute centroid-----------------------------
      Eigen::Vector4f centroid_out;
      pcl::compute3DCentroid(*cloud_in_bbox, centroid_out);
      geometry_msgs::Point temp_pt;
      temp_pt.x = centroid_out[0];
      temp_pt.y = centroid_out[1];
      temp_pt.z = centroid_out[2];

      // before adding new detection must convert filtered cloud into map frame
      sensor_msgs::PointCloud2 obj_cloud;
      pcl::toROSMsg(*cloud_in_bbox, obj_cloud);
      tf2::doTransform(obj_cloud, obj_cloud, current_inv_cam_tf_);

      // add new detection to unassigned detections vector
      ObjectPoseEstimation::UnassignedDetection new_detection;
      new_detection.object_class = box.Class;
      new_detection.cloud = obj_cloud;
      new_detection.position.header.stamp = ros::Time::now();
      new_detection.position.header.frame_id = map_frame_;
      new_detection.camera_tf = current_camera_tf_;
      new_detection.robot_tf = current_robot_tf_;
      new_detection.inv_camera_tf = current_inv_cam_tf_;
      new_detection.inv_robot_tf = current_inv_rob_tf_;
      new_detection.bbox = box;

      // check if the object class has any spaces and replace with underscore
      std::replace(new_detection.object_class.begin(), new_detection.object_class.end(), ' ', '_');

      tf2::doTransform(temp_pt, new_detection.position.point, current_inv_cam_tf_);

      unassigned_detections_.push_back(new_detection);
    }
  }
}

/**
   * @brief Determines if the cloud_in can be matched to any object in the object_detections_ vector that is of the same object_class passed in.
   * @param object_class The class of the object ("chair", "fire hydrant", "microwave", ...etc)
   * @param cloud_in The cloud of the unassigned object in the map frame.  This cloud must be reduced down to the bounding box of the object otherwise it isn't that useful.
   * @return Returns object_number of object in the object_detections_ vector if position is within dist threshold.  Returns 0 if not matched.
    */
int ObjectPoseEstimation::isRegisteredObject(
  const std::string object_class, sensor_msgs::PointCloud2 cloud_in)
{
  for (const ObjectPoseEstimation::ObjectDetection & obj : object_detections_) {
    if (object_class != obj.object_class) {
      continue;
    }

    bool obj_match{true};
    for (int i = 0; i < obj.camera_tfs.size(); ++i) {
      sensor_msgs::PointCloud2 temp_cloud;
      tf2::doTransform(cloud_in, temp_cloud, obj.camera_tfs[i]);

      // Convert sensor_msgs::PointCloud2 to pcl::PointCloud
      ObjectPoseEstimation::CloudPtr cloud(new ObjectPoseEstimation::Cloud);
      pcl::fromROSMsg(temp_cloud, *cloud);

      // produce pixel-space coordinates
      const std::vector<ObjectPoseEstimation::PixelCoords> pixel_coordinates =
        convertCloudToPixelCoords(cloud, camera_info_);
      const ObjectPoseEstimation::CloudPtr cloud_in_bbox = filterPointsInBox(
        cloud, pixel_coordinates, obj.bboxes[i].xmin, obj.bboxes[i].xmax, obj.bboxes[i].ymin,
        obj.bboxes[i].ymax);

      // if there's no overlapping points in the object bounding box then we know it isn't associated with this object
      if (cloud_in_bbox->empty()) {
        obj_match = false;
        break;
      }
    }
    // if we made it through the last loop without breaking then it is a obj match and return the associated object
    if (obj_match) {
      return obj.object_number;
    }
  }
  return 0;
}

/**
   * @brief Determines if the position in pos_in is close to a object of type object_class
   * @param object_class The class of the object ("chair", "fire hydrant", "microwave", ...etc)
   * @param pos_in The current position of the object to compare against in the map frame
   * @param dist The distance threshold being checked
   * @return Returns object_number of object in the object_detections_ vector if position is within dist threshold.  Returns 0 if not close enough.
   */
int ObjectPoseEstimation::isCloseToObject(
  const std::string object_class, const geometry_msgs::PointStamped pos_in, const double dist)
{
  for (const ObjectPoseEstimation::ObjectDetection & obj : object_detections_) {
    if (object_class != obj.object_class) {
      continue;
    }

    if (
      (obj.position.point.x - pos_in.point.x) * (obj.position.point.x - pos_in.point.x) +
        (obj.position.point.y - pos_in.point.y) * (obj.position.point.y - pos_in.point.y) <
      (dist * dist)) {
      return obj.object_number;
    }
  }

  return 0;
}

/**
   * @brief Update the current vector of objects with newly registered object information
   * @param uobj The unassigned detection that has been examined and matches with the object in obj_index
   * @param obj_index The INDEX of the object that uobj has been matched with in the object_detections_ vector.  INDEX not ID!!!
   */
void ObjectPoseEstimation::updateRegisteredObject(
  ObjectPoseEstimation::UnassignedDetection uobj, const int obj_index)
{
  // Before we start messing with the uobj cloud let's save the raw data into the obj detections
  object_detections_[obj_index].fov_clouds.push_back(uobj.cloud);
  pcl::concatenatePointCloud(
    object_detections_[obj_index].raw_cloud, uobj.cloud, object_detections_[obj_index].raw_cloud);

  // Filter existing cloud down by the new bbox
  sensor_msgs::PointCloud2 temp_cloud;
  tf2::doTransform(object_detections_[obj_index].cloud, temp_cloud, uobj.camera_tf);

  // Convert sensor_msgs::PointCloud2 to pcl::PointCloud
  ObjectPoseEstimation::CloudPtr cloud(new ObjectPoseEstimation::Cloud);
  pcl::fromROSMsg(temp_cloud, *cloud);

  // produce pixel-space coordinates
  std::vector<ObjectPoseEstimation::PixelCoords> pixel_coordinates =
    convertCloudToPixelCoords(cloud, camera_info_);
  ObjectPoseEstimation::CloudPtr cloud_in_bbox = filterPointsInBox(
    cloud, pixel_coordinates, uobj.bbox.xmin, uobj.bbox.xmax, uobj.bbox.ymin, uobj.bbox.ymax);

  pcl::toROSMsg(*cloud_in_bbox, temp_cloud);
  tf2::doTransform(temp_cloud, object_detections_[obj_index].cloud, uobj.inv_camera_tf);

  // Filter new cloud down to all other bboxes
  for (int i = 0; i < object_detections_[obj_index].camera_tfs.size(); ++i) {
    tf2::doTransform(uobj.cloud, temp_cloud, object_detections_[obj_index].camera_tfs[i]);
    pcl::fromROSMsg(temp_cloud, *cloud);

    // produce pixel-space coordinates
    pixel_coordinates = convertCloudToPixelCoords(cloud, camera_info_);
    cloud_in_bbox = filterPointsInBox(
      cloud, pixel_coordinates, object_detections_[obj_index].bboxes[i].xmin,
      object_detections_[obj_index].bboxes[i].xmax, object_detections_[obj_index].bboxes[i].ymin,
      object_detections_[obj_index].bboxes[i].ymax);
    pcl::toROSMsg(*cloud_in_bbox, temp_cloud);

    // need to transform back to map frame
    tf2::doTransform(temp_cloud, uobj.cloud, object_detections_[obj_index].inv_camera_tfs[i]);
  }

  // Concatenate clouds and pad new data
  pcl::concatenatePointCloud(
    object_detections_[obj_index].cloud, uobj.cloud, object_detections_[obj_index].cloud);
  pcl::fromROSMsg(object_detections_[obj_index].cloud, *cloud);

  // ----------------------Compute centroid-----------------------------
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud, centroid);

  // Push in new object detection data
  object_detections_[obj_index].position.header.stamp = ros::Time::now();
  object_detections_[obj_index].position.header.frame_id = map_frame_;
  object_detections_[obj_index].position.point.x = centroid[0];
  object_detections_[obj_index].position.point.y = centroid[1];
  object_detections_[obj_index].position.point.z = centroid[2];

  object_detections_[obj_index].bboxes.push_back(uobj.bbox);
  object_detections_[obj_index].camera_tfs.push_back(uobj.camera_tf);
  object_detections_[obj_index].inv_camera_tfs.push_back(uobj.inv_camera_tf);

  object_detections_[obj_index].cloud_pub.publish(object_detections_[obj_index].cloud);
  object_detections_[obj_index].raw_cloud_pub.publish(object_detections_[obj_index].raw_cloud);
  object_detections_[obj_index].obj_position_pub.publish(object_detections_[obj_index].position);

  if (save_det_data_) {
    object_detections_[obj_index].robot_tfs.push_back(uobj.robot_tf);
    object_detections_[obj_index].inv_robot_tfs.push_back(uobj.inv_camera_tf);

    image_processing::Snapshot snapshot;
    if (snapshot_client_.call(snapshot)) {
      // img url stuff
      if (snapshot.response.img_valid) {
        cv_bridge::CvImagePtr cv_ptr;

        std_msgs::String url;

        try {
          cv_ptr = cv_bridge::toCvCopy(snapshot.response.img, sensor_msgs::image_encodings::BGR8);
          cv::imwrite("/home/detection_imgs/OBJECT_DETECTION.jpg", cv_ptr->image);
          url.data = "http://192.168.11.131:4002/home/OBJECT_DETECTION.jpg";

          object_detections_[obj_index].img_urls.push_back(url);

          if (pub_det_data_) {
            object_detections_[obj_index].url_puber.push_back(nh_.advertise<std_msgs::String>(
              ("obj" + std::to_string(obj_index + 1) + "_" +
               object_detections_[obj_index].object_class + "_img_url" +
               std::to_string(object_detections_[obj_index].camera_tfs.size())),
              1, true));

            object_detections_[obj_index]
              .url_puber[object_detections_[obj_index].url_puber.size() - 1]
              .publish(object_detections_[obj_index]
                         .img_urls[object_detections_[obj_index].img_urls.size() - 1]);
          }
        } catch (cv_bridge::Exception & e) {
          ROS_ERROR("Could not save image to file. No detection image url will be sent.");
        }
      }

    } else {
      ROS_DEBUG("Image Snapshot failed to call");
    }

    if (pub_det_data_) {
      object_detections_[obj_index].poses_puber.push_back(nh_.advertise<geometry_msgs::PoseStamped>(
        ("obj" + std::to_string(obj_index + 1) + "_" + object_detections_[obj_index].object_class +
         "_pose" + std::to_string(object_detections_[obj_index].camera_tfs.size())),
        1, true));
      object_detections_[obj_index].fov_pc_puber.push_back(nh_.advertise<sensor_msgs::PointCloud2>(
        ("obj" + std::to_string(obj_index + 1) + "_" + object_detections_[obj_index].object_class +
         "_fov_pc" + std::to_string(object_detections_[obj_index].camera_tfs.size())),
        1, true));

      geometry_msgs::PoseStamped temp_pose;
      temp_pose.header.stamp = ros::Time::now();
      temp_pose.header.frame_id = map_frame_;
      temp_pose.pose.position.x = current_inv_cam_tf_.transform.translation.x;
      temp_pose.pose.position.y = current_inv_cam_tf_.transform.translation.y;
      temp_pose.pose.position.z = current_inv_cam_tf_.transform.translation.z;
      temp_pose.pose.orientation.x = current_inv_cam_tf_.transform.rotation.x;
      temp_pose.pose.orientation.y = current_inv_cam_tf_.transform.rotation.y;
      temp_pose.pose.orientation.z = current_inv_cam_tf_.transform.rotation.z;
      temp_pose.pose.orientation.w = current_inv_cam_tf_.transform.rotation.w;
      object_detections_[obj_index]
        .poses_puber[object_detections_[obj_index].poses_puber.size() - 1]
        .publish(temp_pose);
      object_detections_[obj_index]
        .fov_pc_puber[object_detections_[obj_index].fov_pc_puber.size() - 1]
        .publish(object_detections_[obj_index]
                   .fov_clouds[object_detections_[obj_index].fov_clouds.size() - 1]);
    }
  }
}

/**
   * @brief Convert the object detections data into a Detection3DArray and publish
   */
void ObjectPoseEstimation::publishDetectionArray()
{
  // output
  detection_msgs::DetectionArray detected_objects;
  detected_objects.detections.reserve(object_detections_.size());

  for (const ObjectPoseEstimation::ObjectDetection & obj : object_detections_) {
    // add to the output
    detection_msgs::Detection detection;
    detection.object_class = obj.object_class;
    detection.object_id = obj.object_number;

    detection.pose.header.frame_id = map_frame_;
    detection.pose.header.stamp = ros::Time::now();
    detection.pose.pose.position = obj.position.point;
    detection.pose.pose.orientation.w = 1;
    detection.cloud = obj.cloud;
    detection.url = obj.img_urls.back().data;

    detected_objects.detections.push_back(detection);
  }

  // publish results
  detected_objects_pub_.publish(detected_objects);
}

/**
   * @brief Convert the object detections data into a Detection3DArray and publish
   */
void ObjectPoseEstimation::publishDetection(const int obj_index)
{
  // output
  detection_msgs::Detection detection;
  detection.object_class = object_detections_[obj_index].object_class;
  detection.object_id = object_detections_[obj_index].object_number;

  // add to the output
  detection.pose.header.frame_id = map_frame_;
  detection.pose.header.stamp = ros::Time::now();
  detection.pose.pose.position = object_detections_[obj_index].position.point;
  detection.pose.pose.orientation.w = 1;
  detection.cloud = object_detections_[obj_index].cloud;
  detection.url = object_detections_[obj_index].img_urls.back().data;

  // publish results
  detection_pub_.publish(detection);
}

/**
   * @brief Save the object detections data into bag file 
   */
void ObjectPoseEstimation::saveBag()
{
  rosbag::Bag bag;
  bag.open("object_detections.bag", rosbag::bagmode::Write);
  auto now = ros::Time::now();

  for (const ObjectPoseEstimation::ObjectDetection & obj : object_detections_) {
    bag.write(obj.object_class + std::to_string(obj.object_number) + "_cloud", now, obj.cloud);
    bag.write(
      obj.object_class + std::to_string(obj.object_number) + "_raw_cloud", now, obj.raw_cloud);
    bag.write(
      obj.object_class + std::to_string(obj.object_number) + "_position", now, obj.position);

    for (int i = 0; i < obj.inv_camera_tfs.size(); ++i) {
      bag.write(
        obj.object_class + std::to_string(obj.object_number) + "_camera_tf" + std::to_string(i + 1),
        now, obj.inv_camera_tfs[i]);
    }

    if (!save_det_data_) {
      bag.close();
      return;
    }

    for (int i = 0; i < obj.fov_clouds.size(); ++i) {
      bag.write(
        obj.object_class + std::to_string(obj.object_number) + "_fov" + std::to_string(i + 1) +
          "_cloud",
        now, obj.fov_clouds[i]);
    }

    for (int i = 0; i < obj.inv_robot_tfs.size(); ++i) {
      bag.write(
        obj.object_class + std::to_string(obj.object_number) + "_robot_tf" + std::to_string(i + 1),
        now, obj.robot_tfs[i]);
    }

    for (int i = 0; i < obj.img_urls.size(); ++i) {
      bag.write(
        obj.object_class + std::to_string(obj.object_number) + "_img_urls" + std::to_string(i + 1),
        now, obj.img_urls[i]);
    }
  }
  bag.close();
  return;
}

/**
   * @brief Offers a ros service callback to trigger a republish of latest detection data
   */
bool ObjectPoseEstimation::pubDataCb(
  std_srvs::Empty::Request & req, std_srvs::Empty::Response & res)
{
  if (object_detections_.empty()) {
    return false;
  }

  for (int i = 0; i < object_detections_.size(); i++) {
    publishDetection(i);
  }

  publishDetectionArray();

  return true;
}

/**
   * @brief Offers a ros service callback to trigger a rosbag save of the object detections data
   */
bool ObjectPoseEstimation::saveBagCb(
  std_srvs::Empty::Request & req, std_srvs::Empty::Response & res)
{
  if (object_detections_.empty()) {
    return false;
  }
  saveBag();
  return true;
}
}  // namespace object_detection

int main(int argc, char ** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "object_pose_estimation");

  if (argc != 1) {
    ROS_INFO("usage: rosrun pointcloud_processing object_pose_estimation");
    return 1;
  }

  object_detection::ObjectPoseEstimation node;

  std::vector<std::string> object_classes;
  while (!node.nh_.hasParam("object_pose_estimation/object_classes")) {
    ROS_ERROR(
      "Waiting for 'object_classes' parameter to begin object detection search and pose "
      "estimation.");
    ros::Duration(3.0).sleep();
  }
  node.nh_.getParam("object_pose_estimation/object_classes", object_classes);

  if (object_classes.empty()) {
    ROS_FATAL(
      "'object_classes' parameter is empty.  This means there are no object types to search for.  "
      "Shutting down node.");
    return 1;
  }

  std::copy(
    object_classes.begin(), object_classes.end(),
    std::inserter(node.object_classes, node.object_classes.end()));

  node.initiateDetections();

  return 0;
}
