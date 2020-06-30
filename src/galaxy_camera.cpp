//
// Created by qiayuan on 6/27/20.
//

#include <galaxy_camera.h>

#include <utility>

namespace galaxy_camera {

GalaxyCamera::GalaxyCamera(const ros::NodeHandle &nh,
                           image_transport::CameraPublisher &pub,
                           sensor_msgs::CameraInfo info,
                           uint32_t height, uint32_t width,
                           uint32_t step,
                           uint32_t offset_x, uint32_t offset_y,
                           const std::string &encoding) {
  pub_ = pub;
  info_ = std::move(info);
  image_.height = height;
  image_.width = width;
  image_.step = step;
  image_.data.resize(height * step);
  image_.encoding = encoding;
  img = new char[height * step];
  assert(GXInitLib() == GX_STATUS_SUCCESS); // Initializes the library.
  uint32_t device_num = 0;
  GXUpdateDeviceList(&device_num, 1000);
  assert(device_num == 1); // TODO add multi camera support.
  // Opens the device.
  GX_OPEN_PARAM open_param;
  open_param.accessMode = GX_ACCESS_EXCLUSIVE;
  open_param.openMode = GX_OPEN_INDEX;
  open_param.pszContent = (char *) "1";
  // Get handle
  assert(GXOpenDevice(&open_param, &dev_handle_) == GX_STATUS_SUCCESS);
  ROS_INFO("Camera Opened");

  int64_t format = 0;
  if (encoding == "mono8")
    format = GX_PIXEL_FORMAT_MONO8;
  if (encoding == "mono16")
    format = GX_PIXEL_FORMAT_MONO16;
  if (encoding == "bgr8")
    format = GX_PIXEL_FORMAT_BAYER_GB8;
  if (encoding == "rgb8")
    format = GX_PIXEL_FORMAT_BAYER_RG8;
  if (encoding == "bgra8")
    format = GX_PIXEL_FORMAT_BAYER_BG8;
  if (format == 0)
      static_assert(true, "Illegal format");

  //assert(GXSetEnum(dev_handle_, GX_ENUM_PIXEL_FORMAT, format) == GX_STATUS_SUCCESS);
  assert(GXSetInt(dev_handle_, GX_INT_WIDTH, width) == GX_STATUS_SUCCESS);
  assert(GXSetInt(dev_handle_, GX_INT_HEIGHT, height) == GX_STATUS_SUCCESS);
  assert(GXSetInt(dev_handle_, GX_INT_OFFSET_X, offset_x) == GX_STATUS_SUCCESS);
  assert(GXSetInt(dev_handle_, GX_INT_OFFSET_Y, offset_y) == GX_STATUS_SUCCESS);

  GXRegisterCaptureCallback(dev_handle_,
                            nullptr,
                            GalaxyCamera::onFrameCB);
  GXStreamOn(dev_handle_);
  ROS_INFO("Stream On.");
  GXSetEnum(dev_handle_, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS);

  ros::NodeHandle p_nh(nh, "camera");
  srv_ = new dynamic_reconfigure::Server<CameraConfig>(p_nh);
  dynamic_reconfigure::Server<CameraConfig>::CallbackType
      cb = boost::bind(&GalaxyCamera::reconfigCB, this, _1, _2);
  srv_->setCallback(cb);
}

void GalaxyCamera::onFrameCB(GX_FRAME_CALLBACK_PARAM *pFrame) {
  if (pFrame->status == GX_FRAME_STATUS_SUCCESS) {

    DxRaw8toRGB24((void *) pFrame->pImgBuf, img,
                  pFrame->nWidth, pFrame->nHeight,
                  RAW2RGB_NEIGHBOUR, BAYERBG, false);
    memcpy((char *) (&image_.data[0]), img, image_.step * image_.height);
    ros::Time now = ros::Time().now();
    image_.header.stamp = now;
    info_.header.stamp = now;
    pub_.publish(image_, info_);
  }
}

void GalaxyCamera::reconfigCB(CameraConfig &config, uint32_t level) {
  (void) level;
  // Exposure
  if (config.exposure_auto) {
    double value;
    GXSetFloat(dev_handle_, GX_FLOAT_AUTO_EXPOSURE_TIME_MAX, config.exposure_max);
    GXSetFloat(dev_handle_, GX_FLOAT_AUTO_EXPOSURE_TIME_MIN, config.exposure_min);
    GXSetEnum(dev_handle_, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_CONTINUOUS);
    GXGetFloat(dev_handle_, GX_FLOAT_EXPOSURE_TIME, &value);
    config.exposure_value = value;
  } else {
    GXSetEnum(dev_handle_, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF);
    GXSetFloat(dev_handle_, GX_FLOAT_EXPOSURE_TIME, config.exposure_value);
  }

  // Gain
  if (config.gain_auto) {
    GXSetFloat(dev_handle_, GX_FLOAT_AUTO_GAIN_MIN, config.gain_min);
    GXSetFloat(dev_handle_, GX_FLOAT_AUTO_GAIN_MAX, config.gain_max);
    GXSetEnum(dev_handle_, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_CONTINUOUS);
    GXGetFloat(dev_handle_, GX_FLOAT_GAIN, &config.gain_value);
  } else {
    GXSetEnum(dev_handle_, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_OFF);
    GXSetFloat(dev_handle_, GX_FLOAT_GAIN, config.gain_value);
  }

  // Black level
  if (config.black_auto) {
    GXSetEnum(dev_handle_, GX_ENUM_BLACKLEVEL_AUTO, GX_BLACKLEVEL_AUTO_CONTINUOUS);
    GXGetFloat(dev_handle_, GX_FLOAT_GAIN, &config.black_value);
  } else {
    GXSetEnum(dev_handle_, GX_ENUM_BLACKLEVEL_AUTO, GX_BLACKLEVEL_AUTO_OFF);
    GXGetFloat(dev_handle_, GX_FLOAT_GAIN, &config.black_value);
  }
  // Balance White
  switch (config.white_selector) {
    case 0: GXSetEnum(dev_handle_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_RED);
      break;
    case 1: GXSetEnum(dev_handle_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_GREEN);
      break;
    case 2: GXSetEnum(dev_handle_, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_BLUE);
      break;
  }
  if (last_channel_ != config.white_selector) {
    GXGetFloat(dev_handle_, GX_FLOAT_BALANCE_RATIO, &config.white_value);
    last_channel_ = config.white_selector;
  }
  if (config.white_auto) {
    GXSetEnum(dev_handle_, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS);
    GXGetFloat(dev_handle_, GX_FLOAT_BALANCE_RATIO, &config.white_value);
  } else {
    GXSetEnum(dev_handle_, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_OFF);
    GXSetFloat(dev_handle_, GX_FLOAT_BALANCE_RATIO, config.white_value);
  }
}

GalaxyCamera::~GalaxyCamera() {
  GXStreamOff(dev_handle_);
  GXUnregisterCaptureCallback(dev_handle_);
  GXCloseDevice(dev_handle_);
  GXCloseLib();
}

char *GalaxyCamera::img;
sensor_msgs::Image GalaxyCamera::image_;
image_transport::CameraPublisher GalaxyCamera::pub_;
sensor_msgs::CameraInfo GalaxyCamera::info_;
}
