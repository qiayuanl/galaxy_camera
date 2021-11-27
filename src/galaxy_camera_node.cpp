//
// Created by luohx on 2020/8/9.
//
#include <galaxy_camera.h>
#include <nodelet/loader.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "galaxy_camera_node");

  nodelet::Loader nodelet;
  const nodelet::M_string &remap(ros::names::getRemappings());
  nodelet::V_string nargv;

  nodelet.load(ros::this_node::getName(),
               "galaxy_camera/GalaxyCameraNodelet",
               remap, nargv);

  ros::spin();
  return 0;
}
