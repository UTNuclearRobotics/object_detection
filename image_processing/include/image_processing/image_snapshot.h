///////////////////////////////////////////////////////////////////////////////
//      Title     : image_snapshot.h
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

#include <image_processing/Snapshot.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

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

namespace image_snapshot
{
class ImageSnapshot
{
public:
  // basic constructor
  ImageSnapshot();

  // destructor
  ~ImageSnapshot();

  // ROS Nodehandle
  ros::NodeHandle nh_;

private:
  // class variables
  double stale_time_;
  sensor_msgs::Image image_;
  sensor_msgs::CompressedImage compressed_image_;

  // ROS Nodehandle
  ros::NodeHandle private_nh_;

  // Subscribers
  ros::Subscriber camera_info_sub_, compressed_image_sub_;
  image_transport::Subscriber image_sub_;

  // Service
  ros::ServiceServer snapshot_server_;

  // caches for callback data
  sensor_msgs::CameraInfo camera_info_;

  void cameraInfoCb(const sensor_msgs::CameraInfoConstPtr msg);

  void imageCb(const sensor_msgs::ImageConstPtr & msg);

  void compressedImageCb(const sensor_msgs::CompressedImageConstPtr & msg);

  bool sendSnapshot(
    image_processing::Snapshot::Request & req, image_processing::Snapshot::Response & res);
};

}  // namespace image_snapshot