///////////////////////////////////////////////////////////////////////////////
//      Title     : image_snapshot.cpp
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

#include <image_processing/image_snapshot.h>

namespace image_snapshot
{
ImageSnapshot::ImageSnapshot() : private_nh_("~")
{
  image_transport::ImageTransport it(nh_);

  image_sub_ = it.subscribe("image", 1, &ImageSnapshot::imageCb, this);
  compressed_image_sub_ =
    nh_.subscribe("image/compressed", 1, &ImageSnapshot::compressedImageCb, this);
  camera_info_sub_ = nh_.subscribe("camera_info", 1, &ImageSnapshot::cameraInfoCb, this);

  snapshot_server_ =
    private_nh_.advertiseService("send_snapshot", &ImageSnapshot::sendSnapshot, this);

  private_nh_.param<double>("stale_time", stale_time_, 1.0);

  ros::spin();
}

// destructor
ImageSnapshot::~ImageSnapshot() {}

void ImageSnapshot::cameraInfoCb(const sensor_msgs::CameraInfoConstPtr msg) { camera_info_ = *msg; }

void ImageSnapshot::imageCb(const sensor_msgs::ImageConstPtr & msg) { image_ = *msg; }

void ImageSnapshot::compressedImageCb(const sensor_msgs::CompressedImageConstPtr & msg)
{
  compressed_image_ = *msg;
}

bool ImageSnapshot::sendSnapshot(
  image_processing::Snapshot::Request & req, image_processing::Snapshot::Response & res)
{
  res.camera_info = camera_info_;

  if (ros::Time::now().toSec() - image_.header.stamp.toSec() < stale_time_) {
    res.img = image_;
    res.img_valid = true;
  } else {
    res.img_valid = false;
  }

  if (ros::Time::now().toSec() - compressed_image_.header.stamp.toSec() < stale_time_) {
    res.cimg = compressed_image_;
    res.cimg_valid = true;
  } else {
    res.cimg_valid = false;
  }

  return true;
}
}  // namespace image_snapshot

int main(int argc, char ** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "image_snapshot");

  image_snapshot::ImageSnapshot node;

  return 0;
}
