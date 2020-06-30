//
// Created by qiayuan on 6/27/20.
//

#include "galaxy_camera_node.h"

namespace galaxy_camera {

GalaxyCameraNode::GalaxyCameraNode() : nh_("~") {
  image_transport::ImageTransport it(nh_);
  image_pub_ = it.advertiseCamera("image_raw", 1);
  // load the camera info
  nh_.param("camera_frame_id", galaxy_camera::GalaxyCamera::image_.header.frame_id, std::string("pitch_camera"));
  nh_.param("camera_name", camera_name_, std::string("pitch_camera"));
  nh_.param("camera_info_url", camera_info_url_, std::string(""));
  nh_.param("image_width", image_width_, 1280);
  nh_.param("image_height", image_height_, 1024);
  nh_.param("image_offset_x", image_offset_x_, 0);
  nh_.param("image_offset_y", image_offset_y_, 0);
  nh_.param("pixel_format", pixel_format_, std::string("bgr8"));
  info_.reset(new camera_info_manager::CameraInfoManager(nh_, camera_name_, camera_info_url_));
  // check for default camera info
  if (!info_->isCalibrated()) {
    info_->setCameraName(camera_name_);
    sensor_msgs::CameraInfo camera_info;
    camera_info.header.frame_id = galaxy_camera::GalaxyCamera::image_.header.frame_id;
    camera_info.width = image_width_;
    camera_info.height = image_height_;
    info_->setCameraInfo(camera_info);
  }
  ROS_INFO("Starting '%s' at %dx%d", camera_name_.c_str(),
           image_width_, image_height_);
  galaxy_camera_ = new GalaxyCamera(
      nh_, image_pub_, info_->getCameraInfo(),
      image_height_, image_width_, image_width_ * 3,
      image_offset_x_, image_offset_y_, pixel_format_);
}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "galaxy_camera");
  ros::NodeHandle nh("~");
  galaxy_camera::GalaxyCameraNode galaxy_camera_node;
  ros::spin();
}