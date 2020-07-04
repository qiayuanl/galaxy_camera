//
// Created by qiayuan on 6/30/20.
//

#ifndef SRC_GALAXY_CAMERA_INCLUDE_GALAXY_CAMERA_NODE_H_
#define SRC_GALAXY_CAMERA_INCLUDE_GALAXY_CAMERA_NODE_H_
#include "galaxy_camera.h"
#include <camera_info_manager/camera_info_manager.h>

namespace galaxy_camera {
class GalaxyCameraNode {
 public:
  GalaxyCameraNode();
  ~GalaxyCameraNode();
  ros::NodeHandle nh_;
  image_transport::CameraPublisher image_pub_;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> info_;
 private:
  GalaxyCamera *galaxy_camera_;
  std::string camera_name_, camera_info_url_, pixel_format_;
  int image_width_{}, image_height_{}, image_offset_x_{}, image_offset_y_{};

};

}
#endif //SRC_GALAXY_CAMERA_INCLUDE_GALAXY_CAMERA_NODE_H_
