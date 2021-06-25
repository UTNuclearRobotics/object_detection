///////////////////////////////////////////////////////////////////////////////
//      Title     : target_pose_estimation.cpp
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

#include <pointcloud_processing/target_pose_estimation.h>

namespace target_detection {

  TargetPoseEstimation::TargetPoseEstimation() :
      private_nh_("~")
  {
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
    bbox_sub_ = private_nh_.subscribe("bounding_boxes", 1, &TargetPoseEstimation::bBoxCb, this);
    cloud_sub_ = private_nh_.subscribe("pointcloud", 1, &TargetPoseEstimation::pointCloudCb, this); 
    camera_info_sub_ = private_nh_.subscribe("camera_info", 1, &TargetPoseEstimation::cameraInfoCb, this);
    detected_objects_pub_ = private_nh_.advertise<vision_msgs::Detection3DArray>("detected_objects", 1);

    private_nh_.advertiseService("save_bag", &TargetPoseEstimation::saveBagClient, this);

    private_nh_.param<bool>("debug_viz", debug_viz_, true);
    private_nh_.param<std::string>("map_frame", map_frame_, "map");
    private_nh_.param<std::string>("robot_frame", robot_frame_, "base_link");
    private_nh_.param<std::string>("camera_optical_frame", camera_optical_frame_, "camera_optical_link");

    ROS_DEBUG_STREAM("DEBUG PARAM: " << debug_viz_);
    ROS_DEBUG_STREAM("MAP FRAME PARAM: " << map_frame_);
    ROS_DEBUG_STREAM("ROBOT FRAME PARAM: " << robot_frame_);
    ROS_DEBUG_STREAM("CAMERA OPT FRAME PARAM: " << camera_optical_frame_);

    if (debug_viz_) {
        utgt_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("utgt", 1, true);
    }
  }

  // destructor
  TargetPoseEstimation::~TargetPoseEstimation() {
    bool save;
    private_nh_.param<bool>("save_bag_on_shutdown", save, false);
    if (save) {
      ROS_INFO("SAVING TGT BAG");
      saveBag();
    }
  }


  /**
   * @brief Runs all of the top level detection code.  Initiates and manages the target detection demo
   */
  void TargetPoseEstimation::initiateDetections() {

    ROS_DEBUG("INITIATE DETS");
    double robot_movement_threshold, robot_turning_threshold, sleeper, dist_check;
    private_nh_.param<double>("robot_movement_threshold", robot_movement_threshold, 2.0);
    private_nh_.param<double>("robot_turning_threshold", robot_turning_threshold, 0.1);
    private_nh_.param<double>("sleep_period", sleeper, 0.1);
    private_nh_.param<double>("distance_between_targets", dist_check, 10.0);

    // init robot pose to map origin
    current_robot_tf_.transform.translation.x = 0;
    current_robot_tf_.transform.translation.y = 0;
    current_robot_tf_.transform.translation.z = 0;
    current_robot_tf_.transform.rotation.x = 0;
    current_robot_tf_.transform.rotation.y = 0;
    current_robot_tf_.transform.rotation.z = 0;
    current_robot_tf_.transform.rotation.w = 1;
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

      // robot has now moved beyond threshold. check for targets. break when we have unassigned target detections
      while (ros::ok() && unassigned_detections_.size() == 0) {
        auto t1 = debug_clock_.now();
        prev_robot_tf_ = current_robot_tf_;

        ros::Duration(sleeper).sleep();

        current_robot_tf_ = updateTf(robot_frame_, map_frame_);
        current_camera_tf_ = updateTf(camera_optical_frame_, map_frame_);
        current_inv_rob_tf_ = updateTf(map_frame_, robot_frame_);
        current_inv_cam_tf_ = updateTf(map_frame_, camera_optical_frame_);
        if (!lidar_frame_.empty()) {current_lidar_tf_ = updateTf(camera_optical_frame_, lidar_frame_);}

        if (robotIsTurning(robot_turning_threshold)) {
          continue;
        }
        ros::spinOnce();
        auto t2 = debug_clock_.now();
        // ROS_DEBUG_STREAM("TIME SPIN: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2- t1).count());
      }

      ROS_DEBUG("SHOULD HAVE UNASSIGNED TARGETS NOW");

      // once we have unassigned target detections need to process target comparisons
      if (target_detections_.size() > 0) {
        ROS_DEBUG("CHECKING EXISTING TGTS");
        auto utgt = unassigned_detections_.begin();

        auto t1 = debug_clock_.now();
        while (utgt != unassigned_detections_.end()) {

          // if the unassigned detection is associated with a known target then must add the data and update new existing target info
          if (const int tgt_id = isRegisteredTarget(utgt->target_class, utgt->cloud)) {
            ROS_INFO_STREAM("Registered current view to existing target: " << tgt_id << " (" << utgt->target_class << ")");
            updateRegisteredTarget(*utgt, (tgt_id - 1));

            // publish results now that a target has been updated
            publishDetectionArray();

            utgt = unassigned_detections_.erase(utgt);

          // if we find have a detection of the same type that does not overlap with previous views but it really close then let's assume it is the same target and wait for a better view 
          } else if (const int tgt_id = isCloseToTarget(utgt->target_class, utgt->position, dist_check)) {
            ROS_INFO_STREAM("Target " << tgt_id <<  " (" << utgt->target_class << ") " << "is seen but the new FOV does not overlap with at least one of the previous FOV's.  Ignoring current view.");
            utgt = unassigned_detections_.erase(utgt);

          } else {
            ++utgt;
          }
        }
        auto t2 = debug_clock_.now();
        ROS_DEBUG_STREAM("TIME TO CHECK UTGTS: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2- t1).count());
      } 

      // if we still have unassigned detections then we assign them as new targets
      if (unassigned_detections_.size() > 0) {
        auto t1 = debug_clock_.now();

        for (const TargetPoseEstimation::UnassignedDetection &utgt : unassigned_detections_) {
          TargetPoseEstimation::TargetDetection new_tgt;
          new_tgt.target_class = utgt.target_class;
          new_tgt.target_id = target_detections_.size() + 1;
          new_tgt.position = utgt.position;
          new_tgt.cloud = utgt.cloud;
          new_tgt.raw_cloud = utgt.cloud;
          new_tgt.bboxes.push_back(utgt.bbox); 
          new_tgt.fov_clouds.push_back(utgt.cloud);
          new_tgt.robot_tfs.push_back(current_robot_tf_);
          new_tgt.camera_tfs.push_back(current_camera_tf_);
          new_tgt.inv_robot_tfs.push_back(current_inv_rob_tf_);
          new_tgt.inv_camera_tfs.push_back(current_inv_cam_tf_);

          ROS_INFO_STREAM("Assigning new target: " << new_tgt.target_id << " (" << new_tgt.target_class << ")");

          if (debug_viz_) {
            // debugging
            new_tgt.poses_puber.push_back(nh_.advertise<geometry_msgs::PoseStamped>( ("tgt" + std::to_string(target_detections_.size() + 1) + "_" + new_tgt.target_class + "_pose1"), 1, true));
            new_tgt.fov_pc_puber.push_back(nh_.advertise<sensor_msgs::PointCloud2>( ("tgt" + std::to_string(target_detections_.size() + 1) + "_" + new_tgt.target_class + "_fov_pc1"), 1, true));

            geometry_msgs::PoseStamped temp_pose;
            temp_pose.header.stamp = ros::Time::now();
            temp_pose.header.frame_id = map_frame_;
            temp_pose.pose.position.x = new_tgt.inv_camera_tfs[new_tgt.inv_camera_tfs.size() - 1].transform.translation.x;
            temp_pose.pose.position.y = new_tgt.inv_camera_tfs[new_tgt.inv_camera_tfs.size() - 1].transform.translation.y;
            temp_pose.pose.position.z = new_tgt.inv_camera_tfs[new_tgt.inv_camera_tfs.size() - 1].transform.translation.z;
            temp_pose.pose.orientation.x = new_tgt.inv_camera_tfs[new_tgt.inv_camera_tfs.size() - 1].transform.rotation.x;
            temp_pose.pose.orientation.y = new_tgt.inv_camera_tfs[new_tgt.inv_camera_tfs.size() - 1].transform.rotation.y;
            temp_pose.pose.orientation.z = new_tgt.inv_camera_tfs[new_tgt.inv_camera_tfs.size() - 1].transform.rotation.z;
            temp_pose.pose.orientation.w = new_tgt.inv_camera_tfs[new_tgt.inv_camera_tfs.size() - 1].transform.rotation.w;
            new_tgt.poses_puber[0].publish(temp_pose);
            new_tgt.fov_pc_puber[0].publish(new_tgt.fov_clouds[0]);

            new_tgt.tgt_position_pub = nh_.advertise<geometry_msgs::PointStamped>( ("tgt" + std::to_string(target_detections_.size() + 1) + "_" + new_tgt.target_class + "_pos"), 1, true);
            new_tgt.cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>( ("tgt" + std::to_string(target_detections_.size() + 1) + "_" + new_tgt.target_class), 1, true);
            new_tgt.raw_cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>( ("tgt" + std::to_string(target_detections_.size() + 1) + "_" + new_tgt.target_class + "_raw"), 1, true);

            ROS_DEBUG_STREAM("PUBLISHING NEW TGT");
            ROS_DEBUG_STREAM(new_tgt.cloud.header);
            new_tgt.tgt_position_pub.publish(new_tgt.position);
            new_tgt.cloud_pub.publish(new_tgt.cloud);
            new_tgt.raw_cloud_pub.publish(new_tgt.cloud);
          }
          target_detections_.push_back(new_tgt);
    
          // publish results now that a target has been added
          publishDetectionArray();
        }

        unassigned_detections_.clear();
        auto t2 = debug_clock_.now();
        ROS_DEBUG_STREAM("TIME ASSIGN NEW TGTS: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2- t1).count());
      }
    }
  }


  /**
   * @brief Callback function for bounding boxes detected by Darknet
   * @param msg Bounding boxes
   * @post The message is copied to a local cache
   */
  void TargetPoseEstimation::bBoxCb(const darknet_ros_msgs::BoundingBoxesConstPtr& msg) {
      current_boxes_ = *msg;
  }


  /**
   * @brief Callback function for the RGB camera info
   * @details Specifically, this node needs the width, height,
   *          and intrinsic matrix of the camera.
   * @param msg Camera info
   * @post The message is copied to a local cache
   */
  void TargetPoseEstimation::cameraInfoCb(const sensor_msgs::CameraInfoConstPtr msg) {
      camera_info_ = *msg;
  }


  /**
   * @brief Updates the current_robot_tf_ in the map frame
   * @param frame1 frame1
   * @param frame2 frame2
   */
  geometry_msgs::TransformStamped TargetPoseEstimation::updateTf(const std::string frame1, const std::string frame2) {
    geometry_msgs::TransformStamped temp_tf;
    
    try {
      temp_tf = tf_buffer_.lookupTransform(frame1, frame2, ros::Time(0), ros::Duration(0.1));
    } catch (tf2::TransformException &ex) {
      ROS_ERROR_STREAM("Could not get transform from " << frame1 << " to " << frame2 << "! Target detection positions may be incorrect!");
      ROS_ERROR("%s",ex.what());
    }

    return temp_tf;
  }


  /**
   * @brief Checks if the robot position has moved beyond a distance or rotational threshold in the map frame
   * @param robot_movement_threshold The distance threshold to check against
   * @return True if moved beyond the distance or rotational threshold, False if not.
   */
  bool TargetPoseEstimation::robotHasMoved(const double robot_movement_threshold) {
    if ( (current_robot_tf_.transform.translation.x - prev_robot_tf_.transform.translation.x) * (current_robot_tf_.transform.translation.x - prev_robot_tf_.transform.translation.x)
          + (current_robot_tf_.transform.translation.y - prev_robot_tf_.transform.translation.y) * (current_robot_tf_.transform.translation.y - prev_robot_tf_.transform.translation.y)
          > (robot_movement_threshold * robot_movement_threshold) ) {
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
  bool TargetPoseEstimation::robotIsTurning(const double robot_turning_threshold) {
    
    tf2::Quaternion qt1, qt2;
    tf2::fromMsg(current_robot_tf_.transform.rotation, qt1);
    tf2::fromMsg(prev_robot_tf_.transform.rotation, qt2);
    double angle = qt1.angleShortestPath(qt2);
    if (angle > robot_turning_threshold) {
      ROS_DEBUG_STREAM("ROBOT TURNING: " << angle);
      return true;
    } else {
      // ROS_DEBUG_STREAM("ROBOT NOT TURNING: " << angle);
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
  inline TargetPoseEstimation::PixelCoords TargetPoseEstimation::poseToPixel(const TargetPoseEstimation::PointType &point,
                                const sensor_msgs::CameraInfo &camera_info) {
      TargetPoseEstimation::PixelCoords result;
      auto t1 = debug_clock_.now();

      result.x = camera_info.K[0]*point.x / point.z + camera_info.K[2];
      result.y = camera_info.K[4]*point.y / point.z + camera_info.K[5];
      result.z = point.z;
      auto t2 = debug_clock_.now();
      if (std::chrono::duration_cast<std::chrono::milliseconds>(t2- t1).count() > 1) {
        ROS_DEBUG_STREAM("TIME POSE TO PIXEL: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2- t1).count());
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
  std::vector<TargetPoseEstimation::PixelCoords> TargetPoseEstimation::convertCloudToPixelCoords(const TargetPoseEstimation::CloudPtr cloud,
                                                    const sensor_msgs::CameraInfo &camera_info) {
      auto t1 = debug_clock_.now();
      std::vector<TargetPoseEstimation::PixelCoords> output;
      output.reserve(cloud->size());

      for (const TargetPoseEstimation::PointType &point : cloud->points) {
          output.push_back( poseToPixel(point, camera_info) );
      }
      auto t2 = debug_clock_.now();
      // std::chrono::duration_cast<std::chrono::milliseconds> tme = t2 - t1;
      if (std::chrono::duration_cast<std::chrono::milliseconds>(t2- t1).count() > 1) {
        ROS_DEBUG_STREAM("TIME CONVERT CLOUD TO PIXELS: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2- t1).count());
      }
      return output;
  }


  /**
   * @brief Check a map of known object classes to retreive the class ID for an object class name.
   * @param class_name A known object class name
   * @param map The map of object class names to class IDs
   * @return The class ID. -1 indicates that class_name was not a key in the map.
   */
  TargetPoseEstimation::ObjectClassID TargetPoseEstimation::getObjectID(const TargetPoseEstimation::ObjectClassName class_name, const TargetPoseEstimation::ObjectsMap &map) {
      TargetPoseEstimation::ObjectClassID class_id;

      try {
          class_id = map.at(class_name);
      } 
      catch(const std::out_of_range& e) {
          // ROS_ERROR("getObjectID() - No class ID found for name %s", class_name.c_str());
          // std::cerr << e.what() << '\n';
          return TargetPoseEstimation::ObjectClassID(UNKNOWN_OBJECT_ID);
      }

      return class_id;
  }


  TargetPoseEstimation::ObjectsMap TargetPoseEstimation::convertClassesMap(std::map<std::string, std::string> input) {
      TargetPoseEstimation::ObjectsMap output;

      for (const std::map<std::string, std::string>::value_type &pair : input) {
          output[pair.first] = std::stoi(pair.second);
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
  TargetPoseEstimation::CloudPtr TargetPoseEstimation::filterPointsInBox(const TargetPoseEstimation::CloudPtr input,
                            const std::vector<TargetPoseEstimation::PixelCoords> &pixel_coordinates,
                            const int xmin,
                            const int xmax,
                            const int ymin,
                            const int ymax) {
      auto t1 = debug_clock_.now();
      pcl::PointIndices::Ptr indices_in_bbox(new pcl::PointIndices());
      indices_in_bbox->indices.reserve(input->size());


      int pixels_to_pad;
      private_nh_.param<int>("bbox_pixel_padding", pixels_to_pad, 0);

      for (int i = 0; i < pixel_coordinates.size(); ++i) {
          if (pixel_coordinates[i].z > 0 &&
              pixel_coordinates[i].x > (xmin - pixels_to_pad) &&
              pixel_coordinates[i].x < (xmax + pixels_to_pad) &&
              pixel_coordinates[i].y > (ymin - pixels_to_pad) &&
              pixel_coordinates[i].y < (ymax + pixels_to_pad))
          {
              indices_in_bbox->indices.push_back(i);
          }
      }

      TargetPoseEstimation::CloudPtr cloud_in_bbox(new TargetPoseEstimation::Cloud);
      pcl::ExtractIndices<TargetPoseEstimation::PointType> bbox_filter;

      // Extract the inliers  of the ROI
      bbox_filter.setInputCloud(input);
      bbox_filter.setIndices(indices_in_bbox);
      bbox_filter.setNegative(false);
      bbox_filter.filter(*cloud_in_bbox);

      auto t2 = debug_clock_.now();
      if (std::chrono::duration_cast<std::chrono::milliseconds>(t2- t1).count() > 1) {
        ROS_DEBUG_STREAM("TIME FILTER POINTS IN BOX: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2- t1).count());
      }
      return cloud_in_bbox;
  }


  bool TargetPoseEstimation::transformPointCloud2(sensor_msgs::PointCloud2 &pointcloud,
                            const std::string target_frame) {
      auto t1 = debug_clock_.now();
      geometry_msgs::TransformStamped transform;
      try {
        transform = tf_buffer_.lookupTransform(target_frame, tf2::getFrameId(pointcloud), ros::Time(0), ros::Duration(0.1));
      }
      catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        return false;
      }

      tf2::doTransform(pointcloud, pointcloud, transform);
      
      auto t2 = debug_clock_.now();
      ROS_DEBUG_STREAM("TIME TF PC: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2- t1).count());
      return true;
  }

  geometry_msgs::TransformStamped TargetPoseEstimation::invertTransform(const geometry_msgs::TransformStamped &tf_in) {
    
    tf2::Stamped<tf2::Transform> temp_tf;
    tf2::fromMsg(tf_in, temp_tf);
    geometry_msgs::TransformStamped inv_tf; 
    inv_tf.transform = tf2::toMsg(temp_tf.inverse());
    inv_tf.header.stamp = ros::Time::now();
    inv_tf.header.frame_id = tf_in.child_frame_id;
    inv_tf.child_frame_id = tf_in.header.frame_id;

    return inv_tf;
  }


  /**
   * @brief Callback function for the pointclouds
   * @details This does the core processing to locate objects in the cloud
   * @param input_cloud The pointcloud
   */
  void TargetPoseEstimation::pointCloudCb(sensor_msgs::PointCloud2 input_cloud) {
    ROS_DEBUG("POINTCLOUD CB");
    auto t1 = debug_clock_.now();
    if (lidar_frame_.empty()) {
      ROS_DEBUG("LIDAR FRAME INIT");
      lidar_frame_ = input_cloud.header.frame_id;
      return;
    }
    
    double stale_time;
    private_nh_.param<double>("pointcloud_stale_time", stale_time, 0.05);
    if (ros::Time::now().toSec() - input_cloud.header.stamp.toSec() > stale_time) {
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

    double confidence_threshold;
    private_nh_.param<double>("confidence_threshold", confidence_threshold, 0.75);

    // transform the pointcloud into the RGB optical frame
    tf2::doTransform(input_cloud, input_cloud, current_lidar_tf_);

    // Convert sensor_msgs::PointCloud2 to pcl::PointCloud
    TargetPoseEstimation::CloudPtr cloud(new TargetPoseEstimation::Cloud);
    pcl::fromROSMsg(input_cloud, *cloud);

    // remove NaN points from the cloud
    TargetPoseEstimation::CloudPtr cloud_nan_filtered(new TargetPoseEstimation::Cloud);
    std::vector<int> rindices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud_nan_filtered, rindices);

    // produce pixel-space coordinates
    const std::vector<TargetPoseEstimation::PixelCoords> pixel_coordinates = convertCloudToPixelCoords(cloud_nan_filtered, camera_info_);

    // check if any bounding boxes are on or near the edge of the camera image.  If so remove them and return if none left
    double bbox_edge_x, bbox_edge_y;
    private_nh_.param<double>("bbox_edge_x", bbox_edge_x, 0.1);
    private_nh_.param<double>("bbox_edge_y", bbox_edge_y, 0.01);

    /////////////////////////////////////////////////////////////
    for(const darknet_ros_msgs::BoundingBox &box : current_boxes_.bounding_boxes) {
        const TargetPoseEstimation::ObjectClassID id = getObjectID(box.Class, object_classes);

        // do we meet the threshold for a confirmed detection?
        if (box.probability >= confidence_threshold && id != UNKNOWN_OBJECT_ID) {

          // check for bounding boxes being close to edges
          if (box.xmin < camera_info_.width * bbox_edge_x) {
            ROS_DEBUG("BBOX EDGE LEFT");
            continue;
          }
          if (box.xmax > (camera_info_.width - (camera_info_.width * bbox_edge_x)) ) {
            ROS_DEBUG("BBOX EDGE RIGHT");
            continue;
          }

          if (box.ymin < camera_info_.height * bbox_edge_y) {
            ROS_DEBUG("BBOX EDGE TOP");
            continue;
          }
          if (box.ymax > (camera_info_.height - (camera_info_.height * bbox_edge_y)) ) {
            ROS_DEBUG("BBOX EDGE BOTTOM");
            continue;
          }

            // ----------------------Extract points in the bounding box-----------
            const TargetPoseEstimation::CloudPtr cloud_in_bbox = filterPointsInBox(cloud_nan_filtered,
                                                            pixel_coordinates,
                                                            box.xmin,
                                                            box.xmax,
                                                            box.ymin,
                                                            box.ymax);
            
            // ----------------------Compute centroid-----------------------------
            Eigen::Vector4f centroid_out;
            pcl::compute3DCentroid(*cloud_in_bbox, centroid_out);
            geometry_msgs::Point temp_pt;
            temp_pt.x = centroid_out[0];
            temp_pt.y = centroid_out[1];
            temp_pt.z = centroid_out[2];

            // before adding new detection must convert filtered cloud into map frame
            sensor_msgs::PointCloud2 tgt_cloud;
            pcl::toROSMsg(*cloud_in_bbox, tgt_cloud);
            tf2::doTransform(tgt_cloud, tgt_cloud, current_inv_cam_tf_);

            // add new detection to unassigned detections vector
            TargetPoseEstimation::UnassignedDetection new_detection;
            new_detection.target_class = box.Class;
            new_detection.cloud = tgt_cloud;
            new_detection.position.header.stamp = ros::Time::now();
            new_detection.position.header.frame_id = map_frame_;
            new_detection.camera_tf = current_camera_tf_;
            new_detection.robot_tf = current_robot_tf_;
            new_detection.inv_camera_tf = current_inv_cam_tf_;
            new_detection.inv_robot_tf = current_inv_rob_tf_;
            ROS_DEBUG_STREAM("BOX: " << box);
            new_detection.bbox = box;
            ROS_DEBUG_STREAM("UTGT BOX: " << new_detection.bbox);

            // check if the target class has any spaces and replace with underscore
            std::replace(new_detection.target_class.begin(), new_detection.target_class.end(), ' ', '_');
            
            tf2::doTransform(temp_pt, new_detection.position.point, current_inv_cam_tf_);

            if (debug_viz_) {
              ROS_DEBUG_STREAM("PUBING UTGT");
              ROS_DEBUG_STREAM(new_detection.cloud.header);
              utgt_pub_.publish(new_detection.cloud);
            }
            unassigned_detections_.push_back(new_detection);
        }
    }
    auto t2 = debug_clock_.now();
    ROS_DEBUG_STREAM("TIME PCL2 CB: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2- t1).count());
  }


  /**
   * @brief Determines if the cloud_in can be matched to any target in the target_detections_ vector that is of the same target_class passed in.
   * @param target_class The class of the target ("chair", "fire hydrant", "microwave", ...etc)
   * @param cloud_in The cloud of the unassigned target in the map frame.  This cloud must be reduced down to the bounding box of the target otherwise it isn't that useful.
   * @return Returns target_id of target in the target_detections_ vector if position is within dist threshold.  Returns 0 if not matched.
    */
  int TargetPoseEstimation::isRegisteredTarget(const std::string target_class, sensor_msgs::PointCloud2 cloud_in) {
    auto t1 = debug_clock_.now();

    for (const TargetPoseEstimation::TargetDetection &tgt : target_detections_) {
      if (target_class != tgt.target_class){
        ROS_DEBUG_STREAM("UTGT Does not match target class from TGT " << tgt.target_id);
        ROS_DEBUG_STREAM("UTGT Class: " << target_class << " TGT Class: " << tgt.target_class);
        continue;
      }

      bool tgt_match {true};
      for (int i = 0; i < tgt.camera_tfs.size(); ++i) {
        ROS_DEBUG("COMPARING LIDARS");
        ROS_DEBUG_STREAM(tgt.camera_tfs[i]);
        sensor_msgs::PointCloud2 temp_cloud;
        tf2::doTransform(cloud_in, temp_cloud, tgt.camera_tfs[i]);

        ROS_DEBUG_STREAM("BBOX " << tgt.bboxes[i].xmin << " " << tgt.bboxes[i].xmax << " " << tgt.bboxes[i].ymin << " " << tgt.bboxes[i].ymax);

        // Convert sensor_msgs::PointCloud2 to pcl::PointCloud
        TargetPoseEstimation::CloudPtr cloud(new TargetPoseEstimation::Cloud);
        pcl::fromROSMsg(temp_cloud, *cloud);

        // produce pixel-space coordinates
        const std::vector<TargetPoseEstimation::PixelCoords> pixel_coordinates = convertCloudToPixelCoords(cloud, camera_info_);
        const TargetPoseEstimation::CloudPtr cloud_in_bbox = filterPointsInBox(cloud, pixel_coordinates, tgt.bboxes[i].xmin, tgt.bboxes[i].xmax, tgt.bboxes[i].ymin, tgt.bboxes[i].ymax);

        // if there's no overlapping points in the target bounding box then we know it isn't associated with this target
        if (cloud_in_bbox->empty()) {
            ROS_DEBUG("TGT NOT MATCHED");
            ROS_DEBUG_STREAM("CLOUD IN BOX EMPTY FOR VIEW: " << (i + 1));
            tgt_match = false;
            break;
        }
      }
      // if we made it through the last loop without breaking then it is a tgt match and return the associated target
      if (tgt_match) {
        ROS_DEBUG_STREAM("TGT MATCHED TO: " << tgt.target_id);
        auto t2 = debug_clock_.now();
        ROS_DEBUG_STREAM("TIME IS REGISTERED TGT: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2- t1).count());

        return tgt.target_id;
      }
    }
    auto t2 = debug_clock_.now();
    ROS_DEBUG_STREAM("TIME IS REGISTERED TGT: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2- t1).count());
    return 0;
  }


  /**
   * @brief Determines if the position in pos_in is close to a target of type target_class
   * @param target_class The class of the target ("chair", "fire hydrant", "microwave", ...etc)
   * @param pos_in The current position of the target to compare against in the map frame
   * @param dist The distance threshold being checked
   * @return Returns target_id of target in the target_detections_ vector if position is within dist threshold.  Returns 0 if not close enough.
   */
  int TargetPoseEstimation::isCloseToTarget(const std::string target_class, const geometry_msgs::PointStamped pos_in, const double dist) {
    auto t1 = debug_clock_.now();

    ROS_DEBUG("COMPARING TGT CLOSNESS");

    for (const TargetPoseEstimation::TargetDetection &tgt : target_detections_) {
      if (target_class != tgt.target_class){
        continue;
      }

      if ( (tgt.position.point.x - pos_in.point.x) * (tgt.position.point.x - pos_in.point.x)
            + (tgt.position.point.y - pos_in.point.y) * (tgt.position.point.y - pos_in.point.y)
            < (dist * dist) ) {

        auto t2 = debug_clock_.now();
        ROS_DEBUG_STREAM("TIME IS CLOSE TGT: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2- t1).count());
        return tgt.target_id;
      }
    }
    
    auto t2 = debug_clock_.now();
    ROS_DEBUG_STREAM("TIME IS CLOSE TGT: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2- t1).count());
    return 0;
  }


  /**
   * @brief Update the current vector of targets with newly registered target information
   * @param utgt The unassigned detection that has been examined and matches with the target in tgt_index
   * @param tgt_index The INDEX of the target that utgt has been matched with in the target_detections_ vector.  INDEX not ID!!!
   */
  void TargetPoseEstimation::updateRegisteredTarget(TargetPoseEstimation::UnassignedDetection utgt, const int tgt_index) {
    auto t1 = debug_clock_.now();
    
    // Before we start messing with the utgt cloud let's save the raw data into the tgt detections
    target_detections_[tgt_index].fov_clouds.push_back(utgt.cloud);
    pcl::concatenatePointCloud(target_detections_[tgt_index].raw_cloud, utgt.cloud, target_detections_[tgt_index].raw_cloud);

    // Filter existing cloud down by the new bbox
    sensor_msgs::PointCloud2 temp_cloud;
    tf2::doTransform(target_detections_[tgt_index].cloud, temp_cloud, utgt.camera_tf);

    // Convert sensor_msgs::PointCloud2 to pcl::PointCloud
    TargetPoseEstimation::CloudPtr cloud(new TargetPoseEstimation::Cloud);
    pcl::fromROSMsg(temp_cloud, *cloud);

    // produce pixel-space coordinates
    std::vector<TargetPoseEstimation::PixelCoords> pixel_coordinates = convertCloudToPixelCoords(cloud, camera_info_);
    TargetPoseEstimation::CloudPtr cloud_in_bbox = filterPointsInBox(cloud, pixel_coordinates, utgt.bbox.xmin, utgt.bbox.xmax, utgt.bbox.ymin, utgt.bbox.ymax);

    pcl::toROSMsg(*cloud_in_bbox, temp_cloud);
    tf2::doTransform(temp_cloud, target_detections_[tgt_index].cloud, utgt.inv_camera_tf);

    // Filter new cloud down to all other bboxes
    for (int i = 0; i < target_detections_[tgt_index].camera_tfs.size(); ++i) {
      
      tf2::doTransform(utgt.cloud, temp_cloud, target_detections_[tgt_index].camera_tfs[i]);
      pcl::fromROSMsg(temp_cloud, *cloud);

      // produce pixel-space coordinates
      pixel_coordinates = convertCloudToPixelCoords(cloud, camera_info_);
      cloud_in_bbox = filterPointsInBox(cloud, pixel_coordinates, target_detections_[tgt_index].bboxes[i].xmin, target_detections_[tgt_index].bboxes[i].xmax, target_detections_[tgt_index].bboxes[i].ymin, target_detections_[tgt_index].bboxes[i].ymax);
      pcl::toROSMsg(*cloud_in_bbox, temp_cloud);

      // need to transform back to map frame
      tf2::doTransform(temp_cloud, utgt.cloud, target_detections_[tgt_index].inv_camera_tfs[i]);
    }

    // Concatenate clouds and pad new data
    pcl::concatenatePointCloud(target_detections_[tgt_index].cloud, utgt.cloud, target_detections_[tgt_index].cloud);
    pcl::fromROSMsg(target_detections_[tgt_index].cloud, *cloud);
    
    // ----------------------Compute centroid-----------------------------
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid); 

    // Push in new target detection data
    target_detections_[tgt_index].position.header.stamp = ros::Time::now();
    target_detections_[tgt_index].position.header.frame_id = map_frame_;
    target_detections_[tgt_index].position.point.x = centroid[0];
    target_detections_[tgt_index].position.point.y = centroid[1];
    target_detections_[tgt_index].position.point.z = centroid[2];
    
    target_detections_[tgt_index].bboxes.push_back(utgt.bbox);
    target_detections_[tgt_index].camera_tfs.push_back(utgt.camera_tf);
    target_detections_[tgt_index].robot_tfs.push_back(utgt.robot_tf);
    target_detections_[tgt_index].inv_camera_tfs.push_back(utgt.inv_camera_tf);
    target_detections_[tgt_index].inv_robot_tfs.push_back(utgt.inv_camera_tf);

    // debugging
    if (debug_viz_) {
      target_detections_[tgt_index].tgt_position_pub.publish(target_detections_[tgt_index].position);
      target_detections_[tgt_index].cloud_pub.publish(target_detections_[tgt_index].cloud);
      target_detections_[tgt_index].raw_cloud_pub.publish(target_detections_[tgt_index].raw_cloud);

      target_detections_[tgt_index].poses_puber.push_back(nh_.advertise<geometry_msgs::PoseStamped>( ("tgt" + std::to_string(tgt_index + 1) + "_" + target_detections_[tgt_index].target_class + "_pose" + std::to_string(target_detections_[tgt_index].camera_tfs.size())), 1, true));
      target_detections_[tgt_index].fov_pc_puber.push_back(nh_.advertise<sensor_msgs::PointCloud2>( ("tgt" + std::to_string(tgt_index + 1) + "_" + target_detections_[tgt_index].target_class + "_fov_pc" + std::to_string(target_detections_[tgt_index].camera_tfs.size())), 1, true));

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
      target_detections_[tgt_index].poses_puber[target_detections_[tgt_index].poses_puber.size() - 1].publish(temp_pose);
      target_detections_[tgt_index].fov_pc_puber[target_detections_[tgt_index].fov_pc_puber.size() - 1].publish(target_detections_[tgt_index].fov_clouds[target_detections_[tgt_index].fov_clouds.size() - 1]);
    }

    auto t2 = debug_clock_.now();
    ROS_DEBUG_STREAM("TIME UPDATE TGT PC: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2- t1).count());
  }


  /**
   * @brief Convert the target detections data into a Detection3DArray and publish
   */
  void TargetPoseEstimation::publishDetectionArray() {
    // output
    vision_msgs::Detection3DArray detected_objects;
    detected_objects.header.stamp = ros::Time::now();
    detected_objects.header.frame_id = map_frame_;
    detected_objects.detections.reserve(target_detections_.size());
    
    for (const TargetPoseEstimation::TargetDetection &tgt : target_detections_) {
      // add to the output
      vision_msgs::Detection3D object;
      object.header.frame_id = map_frame_;
      object.header.stamp = ros::Time::now();

      vision_msgs::ObjectHypothesisWithPose hypothesis;
      hypothesis.id = object_classes[tgt.target_class];
      hypothesis.score = 1.0;
      hypothesis.pose.pose.position = tgt.position.point;
      hypothesis.pose.pose.orientation.x = 0;
      hypothesis.pose.pose.orientation.y = 0;
      hypothesis.pose.pose.orientation.z = 0;
      hypothesis.pose.pose.orientation.w = 1;
      // hypothesis.pose.covariance
      object.results.push_back(hypothesis);

      object.bbox.center.position = tgt.position.point;
      object.bbox.center.orientation.x = 0;
      object.bbox.center.orientation.y = 0;
      object.bbox.center.orientation.z = 0;
      object.bbox.center.orientation.w = 1;

      object.source_cloud = tgt.cloud;
      
      TargetPoseEstimation::CloudPtr cloud(new TargetPoseEstimation::Cloud);
      pcl::fromROSMsg(tgt.cloud, *cloud);
      PointType min_pt, max_pt;
      pcl::getMinMax3D(*cloud, min_pt, max_pt);

      object.bbox.size.x = max_pt.x - min_pt.x;
      object.bbox.size.y = max_pt.y - min_pt.y;
      object.bbox.size.z = max_pt.z - min_pt.z;

      ROS_DEBUG_STREAM("TGT " << tgt.target_id << " 3D BBOX SIZE" << object.bbox.size);
      detected_objects.detections.push_back(object);
    }

    // publish results
    detected_objects_pub_.publish(detected_objects);
  }


  /**
   * @brief Save the target detections data into bag file 
   */
  void TargetPoseEstimation::saveBag() {
    rosbag::Bag bag;
    bag.open("target_dections.bag", rosbag::bagmode::Write);
    auto now = ros::Time::now();

    for (const TargetPoseEstimation::TargetDetection &tgt : target_detections_) {
      bag.write(tgt.target_class + std::to_string(tgt.target_id) + "_cloud", now, tgt.cloud);
      bag.write(tgt.target_class + std::to_string(tgt.target_id) + "_raw_cloud", now, tgt.raw_cloud);
      bag.write(tgt.target_class + std::to_string(tgt.target_id) + "_position", now, tgt.position);

      for (int i = 0; i < tgt.fov_clouds.size(); ++i) {
        bag.write(tgt.target_class + std::to_string(tgt.target_id) + "_fov" + std::to_string(i + 1) + "_cloud", now, tgt.fov_clouds[i]);
      }

      for (int i = 0; i < tgt.inv_robot_tfs.size(); ++i) {
        bag.write(tgt.target_class + std::to_string(tgt.target_id) + "_robot_tf" + std::to_string(i + 1), now, tgt.robot_tfs[i]);
      }
      
      for (int i = 0; i < tgt.inv_camera_tfs.size(); ++i) {
        bag.write(tgt.target_class + std::to_string(tgt.target_id) + "_camera_tf" + std::to_string(i + 1), now, tgt.inv_camera_tfs[i]);
      }
    }
    bag.close();
  }


  /**
   * @brief Offers a ros service client to trigger a rosbag save of the target detections data
   */
  bool TargetPoseEstimation::saveBagClient(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    saveBag();
    return true;
  }
}


int main (int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "target_pose_estimation");

    if (argc != 1) {
        ROS_INFO("usage: rosrun pointcloud_processing target_pose_estimation");
        return 1;
    }

    target_detection::TargetPoseEstimation node;

    std::map<std::string, std::string> temp_map;
    if (!node.nh_.hasParam("/target_pose_estimation/object_classes")) {
        ROS_ERROR("Failed to load dictionary parameter 'object_classes'.");
        return 1;
    }
    node.nh_.getParam("/target_pose_estimation/object_classes", temp_map);


    try {
        node.object_classes = node.convertClassesMap(temp_map);
    } catch(std::invalid_argument ex) {
        ROS_FATAL("Invalid object_classes parameter.");
        return 1;
    } 

    node.initiateDetections();

    return 0;
}
