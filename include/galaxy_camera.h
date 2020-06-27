//
// Created by qiayuan on 6/27/20.
//

#ifndef SRC_GALAXY_CAMERA_INCLUDE_GALAXY_CAMERA_H_
#define SRC_GALAXY_CAMERA_INCLUDE_GALAXY_CAMERA_H_
#include <opencv/cv.hpp>
#include "libgxiapi/GxIAPI.h"
#include"libgxiapi/DxImageProc.h"
#include <image_transport/image_transport.h>

class Camera {
 public:
  static char *img;
  static bool inited_;
  static image_transport::Publisher pub_;
  static sensor_msgs::Image image;
  Camera(image_transport::Publisher &pub);
  static void GX_STDC OnFrameCallbackFun(GX_FRAME_CALLBACK_PARAM *pFrame);
};

#endif //SRC_GALAXY_CAMERA_INCLUDE_GALAXY_CAMERA_H_
