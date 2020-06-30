//
// Created by qiayuan on 6/27/20.
//

#ifndef SRC_GALAXY_CAMERA_INCLUDE_GALAXY_CAMERA_H_
#define SRC_GALAXY_CAMERA_INCLUDE_GALAXY_CAMERA_H_
#include "libgxiapi/GxIAPI.h"
#include "libgxiapi/DxImageProc.h"
#include <image_transport/image_transport.h>
#include <dynamic_reconfigure/server.h>
#include <galaxy_camera/CameraConfig.h>

namespace galaxy_camera {
class GalaxyCamera {
 public:
  GalaxyCamera(const ros::NodeHandle &nh,
               image_transport::CameraPublisher &pub,
               sensor_msgs::CameraInfo info,
               uint32_t height, uint32_t width, uint32_t step,
               uint32_t offset_x, uint32_t offset_y,
               const std::string &encoding);
  ~GalaxyCamera();

  static sensor_msgs::Image image_;

 private:
  void reconfigCB(CameraConfig &config, uint32_t level);

  GX_DEV_HANDLE dev_handle_{};
  dynamic_reconfigure::Server<CameraConfig> *srv_{};
  int last_channel_ = 0;

  static char *img;
  static image_transport::CameraPublisher pub_;
  static sensor_msgs::CameraInfo info_;
  static void GX_STDC onFrameCB(GX_FRAME_CALLBACK_PARAM *pFrame);

};
}

#endif //SRC_GALAXY_CAMERA_INCLUDE_GALAXY_CAMERA_H_
