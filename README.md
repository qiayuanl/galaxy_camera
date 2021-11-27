# galaxy_camera
ROS wrapper for the galaxy camera made by Daheng Imaging.

Dependencies:
- ROS Melodic
- gxiapi

ONLY TESTED ON MER-139!!!

# Getting started
## Install dependencies
- ROS - http://wiki.ros.org/ROS
- gxiapi: - download `Galaxy_Linux-x86_Gige-U3_32bits-64bits_1.2.1911.9122` from
http://gb.daheng-imaging.com/CN/Software and install

## Download and build code
This is a ros_packages,you should put it in your ROS workspace.
1. Get the source:
```
git clone git@github.com:QiayuanLiao/git@github.com:QiayuanLiao/galaxy_camera.git
```
2. Make in your workspace
```
catkin_make
source devel/setup.bash
```
## Test
1. Connect the camera by USB, run:
```
roslaunch galaxy_camera MER-139.launch
```
check the image on rqt_image_view.

2. Adjust the params by rqt_reconfigure:
```
rosrun rqt_reconfigure rqt_reconfigure
```
![image.png](https://i.loli.net/2020/07/01/IKDNkbQY2vJ3Tlx.png)

3. Calibrate:
```
rosrun camera_calibration cameracalibrator.py --size 7x5 --square 0.030 image:=/galaxy_camera/image_raw camera:=/galaxy_camera
```

4. More information:
http://wiki.ros.org/image_pipeline

# TODO
- Multi-camera support
- nodelet support
- test on other device
