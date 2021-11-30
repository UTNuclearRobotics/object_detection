///////////////////////////////////////////////////////////////////////////////
//      Title     : image_buffer.cpp
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

#include <pointcloud_processing/image_buffer.h>

namespace target_detection {

    ImageBuffer::ImageBuffer() :
        nh_("~")
      , image_sub_nh_(nh_)
    {
        detections_sub_ = nh_.subscribe("detections", 1, &ImageBuffer::detectionsCb, this);

        latest_image_pub_ = nh_.advertise<vision_msgs::Detection3D>("latest_image", 1);

        nh_.setCallbackQueue(&image_callback_queue_);
        image_transport::ImageTransport it(image_sub_nh_);
        image_sub_ = it.subscribe("in_image_base_topic", 1, &ImageBuffer::imageCb, this);
    }


    void ImageBuffer::imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        latest_image_pub_.publish(msg);
    }



    void ImageBuffer::detectionsCb(const vision_msgs::Detection3DConstPtr& msg)
    {
        image_callback_queue_.callAvailable();
    }

} // namespace target_detection


int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "image_buffer");

    target_detection::ImageBuffer node;

    ros::spin();

    return 0;
}