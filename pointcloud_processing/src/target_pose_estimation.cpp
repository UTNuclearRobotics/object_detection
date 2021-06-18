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
    // tf_listener_(tf_buffer_);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);
    bbox_sub_ = private_nh_.subscribe("bounding_boxes", 1, &TargetPoseEstimation::bBoxCb, this);
    cloud_sub_ = private_nh_.subscribe("pointcloud", 1, &TargetPoseEstimation::pointCloudCb, this); 
    camera_info_sub_ = private_nh_.subscribe("camera_info", 1, &TargetPoseEstimation::cameraInfoCb, this);
    detected_objects_pub_ = private_nh_.advertise<vision_msgs::Detection2DArray>("detected_objects", 1);

    private_nh_.param<bool>("debug_lidar_viz", debug_lidar_viz_, true);
    private_nh_.param<std::string>("map_frame", map_frame_, "map");
    private_nh_.param<std::string>("robot_frame", robot_frame_, "base_link");
    private_nh_.param<std::string>("camera_optical_frame", camera_optical_frame_, "camera_optical_link");

    ROS_INFO_STREAM("DEBUG PARAM: " << debug_lidar_viz_);
    ROS_INFO_STREAM("MAP FRAME PARAM: " << map_frame_);
    ROS_INFO_STREAM("ROBOT FRAME PARAM: " << robot_frame_);
    ROS_INFO_STREAM("CAMERA OPT FRAME PARAM: " << camera_optical_frame_);

    if (debug_lidar_viz_) {
        lidar_fov_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("lidar_fov", 1);
        lidar_bbox_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("lidar_bbox", 1);
        utgt_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("utgt", 1, true);
    }
  }

  // destructor
  TargetPoseEstimation::~TargetPoseEstimation() {}


  /**
   * TODO
   */
  void TargetPoseEstimation::initiateDetections() {

    ROS_INFO("INITIATE DETS");
    double robot_movement_threshold;
    private_nh_.param<double>("robot_movement_threshold", robot_movement_threshold, 2.0);
    ROS_INFO_STREAM("MOVEMENT THRESH PARAM: " << robot_movement_threshold);

    // init robot pose to map origin
    current_robot_tf_.transform.translation.x = 0;
    current_robot_tf_.transform.translation.y = 0;
    current_robot_tf_.transform.translation.z = 0;
    current_robot_tf_.transform.rotation.x = 0;
    current_robot_tf_.transform.rotation.y = 0;
    current_robot_tf_.transform.rotation.z = 0;
    current_robot_tf_.transform.rotation.w = 1;
    prev_robot_tf_ = current_robot_tf_;

    while (true) {

      current_robot_tf_ = updateTf(robot_frame_, map_frame_);
      current_camera_tf_ = updateTf(camera_optical_frame_, map_frame_);

      // if robot hasn't moved beyond threshold then do nothing
      if (!robotHasMoved(robot_movement_threshold)) {
        ROS_INFO_THROTTLE(15.0, "ROBOT HAS NOT MOVED BEYOND THRESH");
        continue;
      }

      prev_robot_tf_ = current_robot_tf_;

      // robot has now moved beyond threshold. check for targets. break when we have unassigned target detections
      while (ros::ok() && unassigned_detections_.size() == 0) {
        current_robot_tf_ = updateTf(robot_frame_, map_frame_);
        current_camera_tf_ = updateTf(camera_optical_frame_, map_frame_);
        ROS_INFO("SPINNING");
        ros::spinOnce();
      }

      ROS_INFO("SHOULD HAVE UNASSIGNED TARGETS NOW");

      // once we have unassigned target detections need to process target comparisons
      if (target_detections_.size() > 0) {
        ROS_INFO("CHECKING EXISTING TGTS");
        auto utgt = unassigned_detections_.begin();
        while (utgt != unassigned_detections_.end()) {

          
          // if the unassigned detection is associated with a known target then must add the data and update new existing target info
          if (const int tgt_id = isRegisteredTarget(utgt->cloud)) {
            ROS_INFO("REGISTERED UTGT TO TGT!!!");
            updateRegisteredTarget(*utgt, (tgt_id - 1));
            utgt = unassigned_detections_.erase(utgt);
          } else if (isCloseToTarget(utgt->position)) {
            ROS_INFO("UTGT IS CLOSE TO A TGT BUT NOT OVERLAPPING PC DATA LET'S IGNORE FOR NOW!!!");
            utgt = unassigned_detections_.erase(utgt);
          } else {
            ++utgt;
          }
        }
      } 

      // if we still have unassigned detections then we assign them as new targets
      if (unassigned_detections_.size() > 0) {
        ROS_INFO("ASSIGNING NEW TGTS");
        for (const TargetPoseEstimation::UnassignedDetection &utgt : unassigned_detections_) {
          TargetPoseEstimation::TargetDetection new_tgt;
          new_tgt.target_id = target_detections_.size() + 1;
          new_tgt.position = utgt.position;
          new_tgt.cloud = utgt.cloud;
          new_tgt.bboxes.push_back(utgt.bbox); 
          new_tgt.fov_clouds.push_back(utgt.cloud);
          new_tgt.robot_tfs.push_back(current_robot_tf_);
          new_tgt.camera_tfs.push_back(current_camera_tf_);

          if (debug_lidar_viz_) {
            new_tgt.debug_pub = nh_.advertise<sensor_msgs::PointCloud2>( ("tgt" + std::to_string(target_detections_.size() + 1)), 1, true);

            ROS_INFO_STREAM("PUBLISHING NEW TGT");
            ROS_INFO_STREAM(new_tgt.cloud.header);
            new_tgt.debug_pub.publish(new_tgt.cloud);
          }
          target_detections_.push_back(new_tgt);
        }

        unassigned_detections_.clear();
      }

      // ROS_INFO_STREAM(target_detections_);
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
    geometry_msgs::TransformStamped temp_pose;
    
    try {
      temp_pose = tf_buffer_.lookupTransform(frame1, frame2, ros::Time(0), ros::Duration(0.1));
    } catch (tf2::TransformException &ex) {
      ROS_ERROR_STREAM("Could not get transform from " << frame1 << " to " << frame2 << "! Target detection positions may be incorrect!");
      ROS_ERROR("%s",ex.what());
    }

    return temp_pose;
  }


  /**
   * @brief Checks if the robot position has moved beyond a distance or rotational threshold in the map frame
   * @param robot_movement_threshold The distance threshold to check against
   * @return True if moved beyond the distance or rotational threshold, False if not.
   */
  bool TargetPoseEstimation::robotHasMoved(const double robot_movement_threshold) {
    
    // double xc = current_robot_tf_.transform.translation.x;
    // double yc = current_robot_tf_.transform.translation.y;
    // double xp = prev_robot_tf_.transform.translation.x;
    // double yp = prev_robot_tf_.transform.translation.y;
    // ROS_INFO_STREAM("Xc " << xc << " Xp " << xp << "Yc " << yc << " Yp " << yp << " dist " << ( (xc-xp) * (xc-xp) + (yc-yp) * (yc-yp)));
    if ( (current_robot_tf_.transform.translation.x - prev_robot_tf_.transform.translation.x) * (current_robot_tf_.transform.translation.x - prev_robot_tf_.transform.translation.x)
          + (current_robot_tf_.transform.translation.y - prev_robot_tf_.transform.translation.y) * (current_robot_tf_.transform.translation.y - prev_robot_tf_.transform.translation.y)
          > (robot_movement_threshold * robot_movement_threshold) ) {
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
  inline TargetPoseEstimation::PixelCoords TargetPoseEstimation::poseToPixel(const TargetPoseEstimation::PointType &point,
                                const sensor_msgs::CameraInfo &camera_info) {
      TargetPoseEstimation::PixelCoords result;

      result.x = camera_info.K[0]*point.x / point.z + camera_info.K[2];
      result.y = camera_info.K[4]*point.y / point.z + camera_info.K[5];
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
  std::vector<TargetPoseEstimation::PixelCoords> TargetPoseEstimation::convertCloudToPixelCoords(const TargetPoseEstimation::CloudPtr cloud,
                                                    const sensor_msgs::CameraInfo &camera_info) {
      std::vector<TargetPoseEstimation::PixelCoords> output;
      output.reserve(cloud->size());

      for (const TargetPoseEstimation::PointType &point : cloud->points) {
          output.push_back( poseToPixel(point, camera_info) );
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
          ROS_ERROR("getObjectID() - No class ID found for name %s", class_name.c_str());
          std::cerr << e.what() << '\n';
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
   * @brief Extract from a pointcloud those points that are within the FoV of the camera.
   * @param input The input pointcloud
   * @param pixel_coordinates A vector of pixelspace coordinates. These correspond by index
   *                          with the points ins input
   * @param height The pixel height of the camera
   * @param width The pixel width of the camera
   * @return A pointcloud containing only the points within the camera FoV.
   */
  TargetPoseEstimation::CloudPtr TargetPoseEstimation::filterPointsInFoV(const TargetPoseEstimation::CloudPtr input,
                            const std::vector<TargetPoseEstimation::PixelCoords> &pixel_coordinates,
                            const int height,
                            const int width) {
      pcl::PointIndices::Ptr indices_in_fov(new pcl::PointIndices());
      indices_in_fov->indices.reserve(input->size());

      for (int i = 0; i < pixel_coordinates.size(); ++i) {
          if (pixel_coordinates[i].z > 0 &&
              pixel_coordinates[i].x >= 0 &&
              pixel_coordinates[i].x <= width &&
              pixel_coordinates[i].y >= 0 &&
              pixel_coordinates[i].y <= height)
          {
              indices_in_fov->indices.push_back(i);
          }
      }

      TargetPoseEstimation::CloudPtr cloud_in_fov(new TargetPoseEstimation::Cloud);
      pcl::ExtractIndices<TargetPoseEstimation::PointType> camera_fov_filter;

      // Extract the inliers  of the ROI
      camera_fov_filter.setInputCloud(input);
      camera_fov_filter.setIndices(indices_in_fov);
      camera_fov_filter.setNegative(false);
      camera_fov_filter.filter(*cloud_in_fov);

      if (debug_lidar_viz_) {
          sensor_msgs::PointCloud2 pc2;
          pcl::toROSMsg(*cloud_in_fov, pc2);
          lidar_fov_pub_.publish(pc2);
      }

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
  TargetPoseEstimation::CloudPtr TargetPoseEstimation::filterPointsInBox(const TargetPoseEstimation::CloudPtr input,
                            const std::vector<TargetPoseEstimation::PixelCoords> &pixel_coordinates,
                            const int xmin,
                            const int xmax,
                            const int ymin,
                            const int ymax) {
      pcl::PointIndices::Ptr indices_in_bbox(new pcl::PointIndices());
      indices_in_bbox->indices.reserve(input->size());


      for (int i = 0; i < pixel_coordinates.size(); ++i) {
        int pixels_to_pad;
        private_nh_.param<int>("bbox_pixel_padding", pixels_to_pad, 0);
        // ROS_INFO_STREAM("BBOX PIX PARAM: " << pixels_to_pad);

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

      if (debug_lidar_viz_) {
          sensor_msgs::PointCloud2 pc2;
          pcl::toROSMsg(*cloud_in_bbox, pc2);
          lidar_bbox_pub_.publish(pc2);
      }

      return cloud_in_bbox;
  }


  bool TargetPoseEstimation::transformPointCloud2(sensor_msgs::PointCloud2 &pointcloud,
                            const std::string target_frame) {
      geometry_msgs::TransformStamped transform;
      try {
        transform = tf_buffer_.lookupTransform(target_frame, tf2::getFrameId(pointcloud), ros::Time(0), ros::Duration(0.1));
      }
      catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
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
  void TargetPoseEstimation::pointCloudCb(sensor_msgs::PointCloud2 input_cloud) {
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
      private_nh_.param<double>("confidence_threshold", confidence_threshold, 0.75);
      ROS_INFO_STREAM("CONFIDENCE PARAM: " << confidence_threshold);

      // transform the pointcloud into the RGB optical frame
      if (tf2::getFrameId(input_cloud) != camera_optical_frame_) {
          if (!transformPointCloud2(input_cloud, camera_optical_frame_)) {
              return;
          }  
      }

      // Convert sensor_msgs::PointCloud2 to pcl::PointCloud
      TargetPoseEstimation::CloudPtr cloud(new TargetPoseEstimation::Cloud);
      pcl::fromROSMsg(input_cloud, *cloud);

      // Initialize container for inliers of the ROI
      pcl::PointIndices::Ptr inliers_camera_fov(new pcl::PointIndices());

      // remove NaN points from the cloud
      TargetPoseEstimation::CloudPtr cloud_nan_filtered(new TargetPoseEstimation::Cloud);
      TargetPoseEstimation::CloudPtr nanfiltered_cloud(new TargetPoseEstimation::Cloud);
      std::vector<int> rindices;
      pcl::removeNaNFromPointCloud(*cloud, *cloud_nan_filtered, rindices);

      // produce pixel-space coordinates
      const std::vector<TargetPoseEstimation::PixelCoords> pixel_coordinates = convertCloudToPixelCoords(cloud_nan_filtered, camera_info_);

      // -------------------Extraction of points in the camera FOV------------------------------
      TargetPoseEstimation::CloudPtr cloud_fov = filterPointsInFoV(cloud_nan_filtered, pixel_coordinates, camera_info_.height, camera_info_.width);

      if (cloud_fov->empty()) {
          ROS_WARN("No pointcloud data found within the camera field of view.");
          return;
      }

      // output
      vision_msgs::Detection2DArray detected_objects;
      detected_objects.header.stamp = now;
      detected_objects.header.frame_id = input_cloud.header.frame_id;
      detected_objects.detections.reserve(current_boxes_.bounding_boxes.size());

      // produce pixel-space coordinates
      const std::vector<TargetPoseEstimation::PixelCoords> pixel_coordinates_fov = convertCloudToPixelCoords(cloud_fov, camera_info_);

      /////////////////////////////////////////////////////////////
      for(const darknet_ros_msgs::BoundingBox &box : current_boxes_.bounding_boxes) {
          const TargetPoseEstimation::ObjectClassID id = getObjectID(box.Class, object_classes);

          // do we meet the threshold for a confirmed detection?
          if (box.probability >= confidence_threshold && id != UNKNOWN_OBJECT_ID) {            
              // ----------------------Extract points in the bounding box-----------
              const TargetPoseEstimation::CloudPtr cloud_in_bbox = filterPointsInBox(cloud_fov,
                                                              pixel_coordinates_fov,
                                                              box.xmin,
                                                              box.xmax,
                                                              box.ymin,
                                                              box.ymax);
              
              // ----------------------Compute centroid-----------------------------
              Eigen::Vector4f centroid_out;
              pcl::compute3DCentroid(*cloud_in_bbox, centroid_out); 

              // add to the output
              vision_msgs::Detection2D object;
              object.bbox.center.x = (box.xmax + box.xmin)/2;
              object.bbox.center.y = (box.ymax + box.ymin)/2;
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

              // before adding new detection must convert filtered cloud into map frame
              sensor_msgs::PointCloud2 tgt_cloud;
              pcl::toROSMsg(*cloud_in_bbox, tgt_cloud);
              transformPointCloud2(tgt_cloud, map_frame_);    

              // add new detection to unassigned detections vector
              TargetPoseEstimation::UnassignedDetection new_detection;
              new_detection.cloud = tgt_cloud;
              new_detection.camera_tf = current_camera_tf_;
              new_detection.robot_tf = current_robot_tf_;
              new_detection.bbox = box;
              ROS_INFO("UTGT!!!");

              tf2::Stamped<tf2::Transform> temp_tf;
              tf2::fromMsg(current_camera_tf_, temp_tf);
              geometry_msgs::TransformStamped inv_cam_tf; 
              inv_cam_tf.transform = tf2::toMsg(temp_tf.inverse());
              inv_cam_tf.header.stamp = ros::Time::now();
              inv_cam_tf.header.frame_id = map_frame_;
              inv_cam_tf.child_frame_id = camera_optical_frame_;
              tf2::doTransform(hypothesis.pose.pose.position, new_detection.position.point, inv_cam_tf);

              if (debug_lidar_viz_) {

                ROS_INFO_STREAM("PUBING UTGT");
                ROS_INFO_STREAM(new_detection.cloud.header);
                // utgt_pub_.publish(new_detection.cloud);
              }

              unassigned_detections_.push_back(new_detection);
          }
      }

      // publish results
      detected_objects_pub_.publish(detected_objects);
  }

  /**
   * TODO
   */
  int TargetPoseEstimation::isRegisteredTarget(sensor_msgs::PointCloud2 cloud_in) {
    for (const TargetPoseEstimation::TargetDetection &tgt : target_detections_) {
      bool tgt_match {true};

      for (const geometry_msgs::TransformStamped tgt_cam_tf : tgt.camera_tfs) {
        ROS_INFO_STREAM(tgt_cam_tf);
        sensor_msgs::PointCloud2 temp_cloud;
        tf2::doTransform(cloud_in, temp_cloud, tgt_cam_tf);

      // geometry_msgs::TransformStamped inv_cam_tf; 
      // inv_cam_tf.transform = tf2::toMsg(temp_tf.inverse());
      // inv_cam_tf.header.stamp = ros::Time::now();
      // inv_cam_tf.header.frame_id = camera_optical_frame_; // CHECK THIS WHEN RUNNING MIGHT BE OPPOSITE
      // inv_cam_tf.child_frame_id = map_frame_; // CHECK THIS WHEN RUNNING MIGHT BE OPPOSITE
      // tf2::doTransform(temp_cloud, target_detections_[tgt_index].cloud, inv_cam_tf);

        ROS_WARN("COMPARING LIDARS");
        lidar_bbox_pub_.publish(temp_cloud);

        // Convert sensor_msgs::PointCloud2 to pcl::PointCloud
        TargetPoseEstimation::CloudPtr cloud(new TargetPoseEstimation::Cloud);
        pcl::fromROSMsg(temp_cloud, *cloud);

        if (debug_lidar_viz_) {
          utgt_pub_.publish(temp_cloud);
        }

        // produce pixel-space coordinates
        const std::vector<TargetPoseEstimation::PixelCoords> pixel_coordinates = convertCloudToPixelCoords(cloud, camera_info_);
        const TargetPoseEstimation::CloudPtr cloud_in_bbox = filterPointsInBox(cloud, pixel_coordinates, tgt.bboxes[0].xmin, tgt.bboxes[0].xmax, tgt.bboxes[0].ymin, tgt.bboxes[0].ymax);

        // if there's no overlapping points in the target bounding box then we know it isn't associated with this target
        if (cloud_in_bbox->empty()) {
            ROS_WARN("TGT NOT MATCHED");
            tgt_match = false;
            break;
        }
      }

      // if we made it through the last loop without breaking then it is a tgt match and return the associated target
      if (tgt_match) {
        ROS_INFO("TGT MATCHED!!");
        return tgt.target_id;
      }
    }

    return 0;
  }


  /**
   * TODO
   */
  bool TargetPoseEstimation::isCloseToTarget(const geometry_msgs::PointStamped pos_in) {
    ROS_WARN("COMPARING TGT CLOSNESS");

    for (const TargetPoseEstimation::TargetDetection &tgt : target_detections_) {
      double dist_check;
      private_nh_.param<double>("distance_between_targets", dist_check, 10.0);
      ROS_INFO_STREAM("DISTANCE TGTS PARAM: " << dist_check);

      double xc = tgt.position.point.x;
      double yc = tgt.position.point.y;
      double xp = pos_in.point.x;
      double yp = pos_in.point.y;
      ROS_INFO_STREAM("Xc " << xc << " Xp " << xp << "Yc " << yc << " Yp " << yp << " dist " << ( (xc-xp) * (xc-xp) + (yc-yp) * (yc-yp)));

      if ( (tgt.position.point.x - pos_in.point.x) * (tgt.position.point.x - pos_in.point.x)
            + (tgt.position.point.y - pos_in.point.y) * (tgt.position.point.y - pos_in.point.y)
            < (dist_check * dist_check) ) {
        
        return true;
      }

      return false;
    }
  }


  /**
   * TODO
   */
  int TargetPoseEstimation::updateRegisteredTarget(TargetPoseEstimation::UnassignedDetection utgt, const int tgt_index) {
    
    // Before we start messing with the utgt cloud let's save the raw data into the tgt detections
    target_detections_[tgt_index].fov_clouds.push_back(utgt.cloud);

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

    tf2::Stamped<tf2::Transform> temp_tf;
    tf2::fromMsg(utgt.camera_tf, temp_tf);
    geometry_msgs::TransformStamped inv_cam_tf; 
    inv_cam_tf.transform = tf2::toMsg(temp_tf.inverse());
    inv_cam_tf.header.stamp = ros::Time::now();
    inv_cam_tf.header.frame_id = map_frame_;
    inv_cam_tf.child_frame_id = camera_optical_frame_;
    tf2::doTransform(temp_cloud, target_detections_[tgt_index].cloud, inv_cam_tf);

    // Filter new cloud down to all other bboxes
    for (int i = 0; i < target_detections_[tgt_index].camera_tfs.size(); ++i) {
      
      tf2::doTransform(utgt.cloud, temp_cloud, target_detections_[tgt_index].camera_tfs[i]);
      pcl::fromROSMsg(temp_cloud, *cloud);

      // produce pixel-space coordinates
      pixel_coordinates = convertCloudToPixelCoords(cloud, camera_info_);
      cloud_in_bbox = filterPointsInBox(cloud, pixel_coordinates, target_detections_[tgt_index].bboxes[i].xmin, target_detections_[tgt_index].bboxes[0].xmax, target_detections_[tgt_index].bboxes[0].ymin, target_detections_[tgt_index].bboxes[0].ymax);
      pcl::toROSMsg(*cloud_in_bbox, temp_cloud);

      // need to transform back to map frame
      tf2::fromMsg(target_detections_[tgt_index].camera_tfs[i], temp_tf);
      inv_cam_tf.transform = tf2::toMsg(temp_tf.inverse());
      inv_cam_tf.header.stamp = ros::Time::now();
      inv_cam_tf.header.frame_id = map_frame_;
      inv_cam_tf.child_frame_id = camera_optical_frame_;
      tf2::doTransform(temp_cloud, utgt.cloud, inv_cam_tf);

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
    
    target_detections_[tgt_index].camera_tfs.push_back(utgt.camera_tf);
    target_detections_[tgt_index].robot_tfs.push_back(utgt.robot_tf);

    if (debug_lidar_viz_) {
      target_detections_[tgt_index].debug_pub.publish(target_detections_[tgt_index].cloud);
    }
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
