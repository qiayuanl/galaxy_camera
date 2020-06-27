//
// Created by qiayuan on 6/27/20.
//

#include "galaxy_camera.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "galaxy_camera");
  ros::NodeHandle nh("~");
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("image_raw", 1);
  galaxy_camera::GalaxyCamera galaxy_camera(
      pub, 1024, 1280, 1280 * 3, 0, 0, "bgr8");
  ros::spin();
}