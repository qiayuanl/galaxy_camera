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
#include <dynamic_reconfigure/client.h>
#include <sensor_msgs/TimeReference.h>
//#include <rm_msgs/CameraStatus.h>

namespace galaxy_camera
{
#define FIFO_SIZE 1023

struct TriggerPacket
{
  uint32_t trigger_counter_;
  ros::Time trigger_time_;
};
class GalaxyCameraNodelet : public nodelet::Nodelet
{
public:
  GalaxyCameraNodelet();
  ~GalaxyCameraNodelet() override;

  void onInit() override;
  static sensor_msgs::Image image_;

private:
  void reconfigCB(CameraConfig& config, uint32_t level);
  void triggerCB(const sensor_msgs::TimeReference::ConstPtr& time_ref);

  ros::NodeHandle nh_;
  static GX_DEV_HANDLE dev_handle_;
  dynamic_reconfigure::Server<CameraConfig>* srv_{};
  int last_channel_ = 0;

  boost::shared_ptr<camera_info_manager::CameraInfoManager> info_manager_;
  std::string camera_name_, camera_info_url_, pixel_format_, frame_id_, camera_sn_;
  int image_width_{}, image_height_{}, image_offset_x_{}, image_offset_y_{}, raising_filter_value_{};
  static bool enable_imu_trigger_;
  static char* img_;
  static image_transport::CameraPublisher pub_;
  static sensor_msgs::CameraInfo info_;
  static void GX_STDC onFrameCB(GX_FRAME_CALLBACK_PARAM* pFrame);

  static bool device_open_;
  //  static bool imuCorrespondence(rm_msgs::CameraStatus::Request& req,
  //  rm_msgs::CameraStatus::Response& res);
  ros::ServiceServer imu_correspondence_service_;

  ros::Subscriber trigger_sub_;
  static TriggerPacket fifo_[FIFO_SIZE];
  static uint32_t receive_trigger_counter_;
  static int fifo_front_;
  static int fifo_rear_;
  static void fifoWrite(TriggerPacket pkt);
  static bool fifoRead(TriggerPacket& pkt);
};
}  // namespace galaxy_camera

#endif  // SRC_GALAXY_CAMERA_INCLUDE_GALAXY_CAMERA_H_
