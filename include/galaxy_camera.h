//
// Created by qiayuan on 6/27/20.
//

#ifndef SRC_GALAXY_CAMERA_INCLUDE_GALAXY_CAMERA_H_
#define SRC_GALAXY_CAMERA_INCLUDE_GALAXY_CAMERA_H_
#include "libgxiapi/GxIAPI.h"
#include "libgxiapi/DxImageProc.h"
#include <image_transport/image_transport.h>
namespace galaxy_camera {
class GalaxyCamera {
 public:
  GalaxyCamera(image_transport::Publisher
               &pub, uint32_t height, uint32_t width, uint32_t step,
               uint32_t offset_x, uint32_t offset_y,
               const std::string &encoding);
  ~GalaxyCamera();

 private:
  GX_DEV_HANDLE dev_handle_{};

  static char *img;
  static image_transport::Publisher pub_;
  static sensor_msgs::Image image;
  static void GX_STDC onFrameCB(GX_FRAME_CALLBACK_PARAM *pFrame);

};
}

#endif //SRC_GALAXY_CAMERA_INCLUDE_GALAXY_CAMERA_H_
