//
// Created by qiayuan on 6/27/20.
//

#ifndef SRC_GALAXY_CAMERA_INCLUDE_GALAXY_CAMERA_H_
#define SRC_GALAXY_CAMERA_INCLUDE_GALAXY_CAMERA_H_
#include <nodelet/nodelet.h>
#include "libgxiapi/GxIAPI.h"
#include "libgxiapi/DxImageProc.h"
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <galaxy_camera/CameraConfig.h>
#include <camera_info_manager/camera_info_manager.h>

namespace galaxy_camera {
class GalaxyCameraNodelet : public nodelet::Nodelet {
 public:
  GalaxyCameraNodelet();
  ~GalaxyCameraNodelet() override;

  void onInit() override;
  image_transport::CameraPublisher image_pub_;
  static sensor_msgs::Image image_;

 private:
  void reconfigCB(CameraConfig &config, uint32_t level);

  ros::NodeHandle nh_;
  GX_DEV_HANDLE dev_handle_{};
  dynamic_reconfigure::Server<CameraConfig> *srv_{};
  int last_channel_ = 0;

  boost::shared_ptr<camera_info_manager::CameraInfoManager> info_manager_;
  std::string camera_name_, camera_info_url_, pixel_format_, frame_id_;
  int image_width_{}, image_height_{}, image_offset_x_{}, image_offset_y_{};
  static char *img_;
  static image_transport::CameraPublisher pub_;
  static sensor_msgs::CameraInfo info_;
  static void GX_STDC onFrameCB(GX_FRAME_CALLBACK_PARAM *pFrame);

};
}

#endif //SRC_GALAXY_CAMERA_INCLUDE_GALAXY_CAMERA_H_
