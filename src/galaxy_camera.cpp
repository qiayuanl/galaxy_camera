//
// Created by qiayuan on 6/27/20.
//
#include <pluginlib/class_list_macros.h>
#include <galaxy_camera.h>
#include <utility>
#include <ros/time.h>

namespace galaxy_camera
{
PLUGINLIB_EXPORT_CLASS(galaxy_camera::GalaxyCameraNodelet, nodelet::Nodelet)
GalaxyCameraNodelet::GalaxyCameraNodelet()
{
}

void GalaxyCameraNodelet::onInit()
{
  nh_ = this->getPrivateNodeHandle();
  image_transport::ImageTransport it(nh_);
  pub_ = it.advertiseCamera("image_raw", 1);

  nh_.param("camera_frame_id", image_.header.frame_id, std::string("camera_optical_frame"));
  nh_.param("camera_name", camera_name_, std::string("camera"));
  nh_.param("camera_info_url", camera_info_url_, std::string(""));
  nh_.param("image_width", image_width_, 1280);
  nh_.param("image_height", image_height_, 1024);
  nh_.param("image_offset_x", image_offset_x_, 0);
  nh_.param("image_offset_y", image_offset_y_, 0);
  nh_.param("pixel_format", pixel_format_, std::string("bgr8"));
  nh_.param("frame_id", frame_id_, std::string("camera_optical_frame"));
  nh_.param("camera_sn", camera_sn_, std::string(""));
  nh_.param("enable_imu_trigger", enable_imu_trigger_, true);
  nh_.param("raising_filter_value", raising_filter_value_, 2000);
  info_manager_.reset(new camera_info_manager::CameraInfoManager(nh_, camera_name_, camera_info_url_));

  // check for default camera info
  if (!info_manager_->isCalibrated())
  {
    info_manager_->setCameraName(camera_name_);
    sensor_msgs::CameraInfo camera_info;
    camera_info.header.frame_id = image_.header.frame_id;
    camera_info.width = image_width_;
    camera_info.height = image_height_;
    info_manager_->setCameraInfo(camera_info);
  }
  ROS_INFO("Starting '%s' at %dx%d", camera_name_.c_str(), image_width_, image_height_);
  info_ = std::move(info_manager_->getCameraInfo());
  info_.header.frame_id = frame_id_;
  image_.header.frame_id = frame_id_;
  image_.height = image_height_;
  image_.width = image_width_;
  image_.step = image_width_ * 3;
  image_.data.resize(image_.height * image_.step);
  image_.encoding = pixel_format_;
  img_ = new char[image_.height * image_.step];

  assert(GXInitLib() == GX_STATUS_SUCCESS);  // Initializes the library.
  uint32_t device_num = 0;
  GXUpdateDeviceList(&device_num, 1000);
  assert(device_num > 0);

  GX_OPEN_PARAM open_param;
  // Opens the device.
  if (device_num > 1)
  {
    assert(nh_.hasParam("camera_SN"));
    open_param.accessMode = GX_ACCESS_EXCLUSIVE;
    open_param.openMode = GX_OPEN_SN;
    open_param.pszContent = (char*)&camera_sn_[0];
  }
  else
  {
    open_param.accessMode = GX_ACCESS_EXCLUSIVE;
    open_param.openMode = GX_OPEN_INDEX;
    open_param.pszContent = (char*)"1";
  }

  // Get handle
  assert(GXOpenDevice(&open_param, &dev_handle_) == GX_STATUS_SUCCESS);
  ROS_INFO("Camera Opened");

  int64_t format = 0;
  if (pixel_format_ == "mono8")
    format = GX_PIXEL_FORMAT_MONO8;
  if (pixel_format_ == "mono16")
    format = GX_PIXEL_FORMAT_MONO16;
  if (pixel_format_ == "bgr8")
    format = GX_PIXEL_FORMAT_BAYER_GB8;
  if (pixel_format_ == "rgb8")
    format = GX_PIXEL_FORMAT_BAYER_RG8;
  if (pixel_format_ == "bgra8")
    format = GX_PIXEL_FORMAT_BAYER_BG8;
  if (format == 0)
    static_assert(true, "Illegal format");

  // assert(GXSetEnum(dev_handle_, GX_ENUM_PIXEL_FORMAT, format) == GX_STATUS_SUCCESS);
  assert(GXSetInt(dev_handle_, GX_INT_WIDTH, image_width_) == GX_STATUS_SUCCESS);
  assert(GXSetInt(dev_handle_, GX_INT_HEIGHT, image_height_) == GX_STATUS_SUCCESS);
  assert(GXSetInt(dev_handle_, GX_INT_OFFSET_X, image_offset_x_) == GX_STATUS_SUCCESS);
  assert(GXSetInt(dev_handle_, GX_INT_OFFSET_Y, image_offset_y_) == GX_STATUS_SUCCESS);
  assert(GXSetEnum(dev_handle_, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS) == GX_STATUS_SUCCESS);

  if (enable_imu_trigger_)
  {
    assert(GXSetEnum(dev_handle_, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON) == GX_STATUS_SUCCESS);
    assert(GXSetEnum(dev_handle_, GX_ENUM_TRIGGER_SOURCE, GX_TRIGGER_SOURCE_LINE3) == GX_STATUS_SUCCESS);
    assert(GXSetEnum(dev_handle_, GX_ENUM_TRIGGER_ACTIVATION, GX_TRIGGER_ACTIVATION_RISINGEDGE) == GX_STATUS_SUCCESS);
    assert(GXSetFloat(dev_handle_, GX_FLOAT_TRIGGER_FILTER_RAISING, raising_filter_value_) == GX_STATUS_SUCCESS);
    assert(GXSetEnum(dev_handle_, GX_ENUM_LINE_SELECTOR, GX_ENUM_LINE_SELECTOR_LINE3) == GX_STATUS_SUCCESS);
    assert(GXSetEnum(dev_handle_, GX_ENUM_LINE_MODE, GX_ENUM_LINE_MODE_INPUT) == GX_STATUS_SUCCESS);

    trigger_sub_ = nh_.subscribe("/trigger_time", 50, &galaxy_camera::GalaxyCameraNodelet::triggerCB, this);

    //    imu_correspondence_service_ =
    //        nh_.advertiseService("imu_camera_correspondece", galaxy_camera::GalaxyCameraNodelet::imuCorrespondence);
  }
  else
  {
    assert(GXSetEnum(dev_handle_, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF) == GX_STATUS_SUCCESS);
  }

  GXRegisterCaptureCallback(dev_handle_, nullptr, onFrameCB);

  if (GXStreamOn(dev_handle_) == GX_STATUS_SUCCESS)
  {
    ROS_INFO("Stream On.");
    device_open_ = true;
  }

  ros::NodeHandle p_nh(nh_, "galaxy_camera_dy");
  srv_ = new dynamic_reconfigure::Server<CameraConfig>(p_nh);
  dynamic_reconfigure::Server<CameraConfig>::CallbackType cb =
      boost::bind(&GalaxyCameraNodelet::reconfigCB, this, _1, _2);
  srv_->setCallback(cb);
}

// bool GalaxyCameraNodelet::imuCorrespondence(rm_msgs::CameraStatus::Request&
// req, rm_msgs::CameraStatus::Response& res)
//{
//   res.is_open = device_open_;
//   return true;
// }

void GalaxyCameraNodelet::triggerCB(const sensor_msgs::TimeReference::ConstPtr& time_ref)
{
  galaxy_camera::TriggerPacket pkt;
  pkt.trigger_time_ = time_ref->time_ref;
  pkt.trigger_counter_ = time_ref->header.seq;
  fifoWrite(pkt);
}

void GalaxyCameraNodelet::fifoWrite(TriggerPacket pkt)
{
  if (fifo_front_ == (fifo_rear_ + 1) % FIFO_SIZE)
  {
    ROS_WARN("FIFO overflow!");
    return;
  }
  fifo_[fifo_rear_] = pkt;
  fifo_rear_ = (fifo_rear_ + 1) % FIFO_SIZE;
}

bool GalaxyCameraNodelet::fifoRead(TriggerPacket& pkt)
{
  if (fifo_front_ == fifo_rear_)
    return false;
  pkt = fifo_[fifo_front_];
  fifo_front_ = (fifo_front_ + 1) % FIFO_SIZE;
  return true;
}

void GalaxyCameraNodelet::onFrameCB(GX_FRAME_CALLBACK_PARAM* pFrame)
{
  if (pFrame->status == GX_FRAME_STATUS_SUCCESS)
  {
    if (enable_imu_trigger_)
    {
      TriggerPacket pkt;
      while (!fifoRead(pkt))
      {
        ros::Duration(0.001).sleep();
      }
      if (pkt.trigger_counter_ != receive_trigger_counter_++)
        ROS_WARN("Trigger not in sync!");
      else
      {
        image_.header.stamp = pkt.trigger_time_;
        info_.header.stamp = pkt.trigger_time_;
      }
      ROS_INFO("trigger_counter:%d,receive_trigger_counter:%d,fifo_length:%d,front:%d,rear:%d", pkt.trigger_counter_,
               receive_trigger_counter_, (fifo_rear_ - fifo_front_ + FIFO_SIZE) % FIFO_SIZE, fifo_front_, fifo_rear_);
    }
    else
    {
      ros::Time now = ros::Time::now();
      //      delay_time = (now.sec + now.nsec * 1e-9) - (image_.header.stamp.sec + image_.header.stamp.nsec * 1e-9);
      image_.header.stamp = now;
      info_.header.stamp = now;
    }
    DxRaw8toRGB24((void*)pFrame->pImgBuf, img_, pFrame->nWidth, pFrame->nHeight, RAW2RGB_NEIGHBOUR, BAYERBG, false);
    memcpy((char*)(&image_.data[0]), img_, image_.step * image_.height);
    pub_.publish(image_, info_);
    //    ROS_INFO("%f", delay_time);
  }
}

void GalaxyCameraNodelet::reconfigCB(CameraConfig& config, uint32_t level)
{
  (void)level;
  // Exposure
  if (config.exposure_auto)
  {
    double value;
    GXSetFloat(dev_handle_, GX_FLOAT_AUTO_EXPOSURE_TIME_MAX, config.exposure_max);
    GXSetFloat(dev_handle_, GX_FLOAT_AUTO_EXPOSURE_TIME_MIN, config.exposure_min);
    GXSetEnum(dev_handle_, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_CONTINUOUS);
    GXGetFloat(dev_handle_, GX_FLOAT_EXPOSURE_TIME, &value);
    config.exposure_value = value;
  }
  else
  {
    GXSetEnum(dev_handle_, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF);
    GXSetFloat(dev_handle_, GX_FLOAT_EXPOSURE_TIME, config.exposure_value);
  }

  // Gain
  if (config.gain_auto)
  {
    GXSetFloat(dev_handle_, GX_FLOAT_AUTO_GAIN_MIN, config.gain_min);
    GXSetFloat(dev_handle_, GX_FLOAT_AUTO_GAIN_MAX, config.gain_max);
    GXSetEnum(dev_handle_, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_CONTINUOUS);
    GXGetFloat(dev_handle_, GX_FLOAT_GAIN, &config.gain_value);
  }
  else
  {
    GXSetEnum(dev_handle_, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_OFF);
    GXSetFloat(dev_handle_, GX_FLOAT_GAIN, config.gain_value);
  }

  // Black level
  if (config.black_auto)
  {
    GXSetEnum(dev_handle_, GX_ENUM_BLACKLEVEL_AUTO, GX_BLACKLEVEL_AUTO_CONTINUOUS);
    GXGetFloat(dev_handle_, GX_FLOAT_BLACKLEVEL, &config.black_value);
  }
  else
  {
    GXSetEnum(dev_handle_, GX_ENUM_BLACKLEVEL_AUTO, GX_BLACKLEVEL_AUTO_OFF);
    GXSetFloat(dev_handle_, GX_FLOAT_BLACKLEVEL, config.black_value);
  }
  // Balance White
  switch (config.white_selector)
  {
    case 0:
      GXSetEnum(dev_handle_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_RED);
      break;
    case 1:
      GXSetEnum(dev_handle_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_GREEN);
      break;
    case 2:
      GXSetEnum(dev_handle_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_BLUE);
      break;
  }
  if (last_channel_ != config.white_selector)
  {
    GXGetFloat(dev_handle_, GX_FLOAT_BALANCE_RATIO, &config.white_value);
    last_channel_ = config.white_selector;
  }
  if (config.white_auto)
  {
    GXSetEnum(dev_handle_, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS);
    GXGetFloat(dev_handle_, GX_FLOAT_BALANCE_RATIO, &config.white_value);
  }
  else
  {
    GXSetEnum(dev_handle_, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_OFF);
    GXSetFloat(dev_handle_, GX_FLOAT_BALANCE_RATIO, config.white_value);
  }
}

GalaxyCameraNodelet::~GalaxyCameraNodelet()
{
  GXStreamOff(dev_handle_);
  GXUnregisterCaptureCallback(dev_handle_);
  GXCloseDevice(dev_handle_);
  GXCloseLib();
}

bool GalaxyCameraNodelet::device_open_ = false;
GX_DEV_HANDLE GalaxyCameraNodelet::dev_handle_;
char* GalaxyCameraNodelet::img_;
sensor_msgs::Image GalaxyCameraNodelet::image_;
image_transport::CameraPublisher GalaxyCameraNodelet::pub_;
sensor_msgs::CameraInfo GalaxyCameraNodelet::info_;
bool GalaxyCameraNodelet::enable_imu_trigger_;
struct TriggerPacket GalaxyCameraNodelet::fifo_[FIFO_SIZE];
uint32_t GalaxyCameraNodelet::receive_trigger_counter_ = 0;
int GalaxyCameraNodelet::fifo_front_ = 0;
int GalaxyCameraNodelet::fifo_rear_ = 0;

}  // namespace galaxy_camera
