//
// Created by qiayuan on 6/25/20.
//
#include <zconf.h>
#include <cstdio>

#include <iostream>
#include "galaxy_camera.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "camera");
  ros::NodeHandle nh_;
  image_transport::ImageTransport it(nh_);
  image_transport::Publisher pub = it.advertise("image_raw", 1);
  Camera camera(pub);
  GX_STATUS status = GX_STATUS_SUCCESS;
  GX_DEV_HANDLE hDevice = nullptr;
  GX_OPEN_PARAM stOpenParam;
  uint32_t nDeviceNum = 0;// Initializes the library.
  status = GXInitLib();
  if (status != GX_STATUS_SUCCESS) {
    return 0;
  }// Updates the enumeration list for the devices.
  status = GXUpdateDeviceList(&nDeviceNum, 1000);
  if ((status != GX_STATUS_SUCCESS) || (nDeviceNum <= 0)) {
    return 0;
  }
// Opens the device.
  stOpenParam.
      accessMode = GX_ACCESS_EXCLUSIVE;
  stOpenParam.
      openMode = GX_OPEN_INDEX;
  stOpenParam.
      pszContent = (char *) "1";
  status = GXOpenDevice(&stOpenParam, &hDevice);
  int64_t nPixelFormat = GX_PIXEL_FORMAT_BAYER_RG8;
  status = GXSetEnum(hDevice, GX_ENUM_PIXEL_FORMAT, nPixelFormat);

  if (status == GX_STATUS_SUCCESS) {
    status = GXSetEnum(hDevice, GX_ENUM_AWB_LAMP_HOUSE,
                       GX_AWB_LAMP_HOUSE_ADAPTIVE);
// Sets to continuous automatic white balance mode.
    status = GXSetEnum(hDevice, GX_ENUM_BALANCE_WHITE_AUTO,
                       GX_BALANCE_WHITE_AUTO_CONTINUOUS);

    status = GXSetEnum(hDevice, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_CONTINUOUS);

    GXRegisterCaptureCallback(hDevice,
                              nullptr,
                              Camera::OnFrameCallbackFun);
// Stream On.
    status = GXStreamOn(hDevice);
    printf("%d", status);
    ros::spin();
    status = GXStreamOff(hDevice);
    printf("%d", status);
// Unregisters image processing callback function.
    status = GXUnregisterCaptureCallback(hDevice);
  }
  status = GXCloseDevice(hDevice);
  status = GXCloseLib();
  return 0;
}