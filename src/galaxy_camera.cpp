//
// Created by qiayuan on 6/27/20.
//

#include <galaxy_camera.h>

namespace galaxy_camera {

GalaxyCamera::GalaxyCamera(image_transport::Publisher &pub,
                           uint32_t height, uint32_t width,
                           uint32_t step,
                           uint32_t offset_x, uint32_t offset_y,
                           const std::string &encoding) {
  pub_ = pub;
  image.height = height;
  image.width = width;
  image.step = step;
  image.data.resize(height * step);
  image.encoding = encoding;
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
}

void GalaxyCamera::onFrameCB(GX_FRAME_CALLBACK_PARAM *pFrame) {

  if (pFrame->status == GX_FRAME_STATUS_SUCCESS) {

    DxRaw8toRGB24((void *) pFrame->pImgBuf, img,
                  pFrame->nWidth, pFrame->nHeight,
                  RAW2RGB_NEIGHBOUR, BAYERBG, false);
    memcpy((char *) (&image.data[0]), img, image.step * image.height);
    pub_.publish(image);
  }
}

GalaxyCamera::~GalaxyCamera() {
  GXStreamOff(dev_handle_);
  GXUnregisterCaptureCallback(dev_handle_);
  GXCloseDevice(dev_handle_);
  GXCloseLib();
}

char *GalaxyCamera::img;
sensor_msgs::Image GalaxyCamera::image;
image_transport::Publisher GalaxyCamera::pub_;
}
