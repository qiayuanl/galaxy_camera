//
// Created by qiayuan on 6/27/20.
//

#include <galaxy_camera.h>
#include <cv_bridge/cv_bridge.h>
void Camera::OnFrameCallbackFun(GX_FRAME_CALLBACK_PARAM *pFrame) {
  {
    if (pFrame->status == GX_FRAME_STATUS_SUCCESS) {
      //对图像进行某些操作
      cv::Mat cv_img;
      //cv_img.create(pFrame->nHeight, pFrame->nWidth, CV_8UC3);
      if (!inited_) {
        img = new char[(pFrame->nHeight) * (pFrame->nWidth) * 3];
        inited_ = true;
      }

      DxRaw8toRGB24((void *) pFrame->pImgBuf,
                    img,
                    pFrame->nWidth,
                    pFrame->nHeight,
                    RAW2RGB_NEIGHBOUR,
                    BAYERBG,
                    false);
      //memcpy(cv_img.data, img, (pFrame->nHeight) * (pFrame->nWidth) * 3);
      //sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_img).toImageMsg();
      memcpy((char *) (&image.data[0]), img, 1024 * 1280 * 3);
      pub_.publish(image);
      //cout <<  << endl;
      //cv::imshow("camera1", cv_img);
      //cv::waitKey(1);
    }
  }
}
Camera::Camera(image_transport::Publisher &pub) {
  pub_ = pub;
  image.height = 1024;
  image.width = 1280;
  image.step = 1280 * 3;
  image.data.resize(1024 * 1280 * 3);
  image.encoding = "bgr8";
}

bool Camera::inited_ = false;
char *Camera::img;
sensor_msgs::Image Camera::image;
image_transport::Publisher Camera::pub_;