Predictive Display: Version 3.0
By: Nazmus Sakib
Email: nazmus.sakib@vt.edu

## `ROS` Packages Used
**vision_opencv:** Standard `ROS` package for interfacing `ROS` with `OpenCV`. The package contains function like `CvBridge()` that converts `ROS` image messages to `OpenCV` `cv::Mat` image matrices used by other `OpenCV` packages and functions. Link: http://wiki.ros.org/vision_opencv. 

**usb_cam:** This package interfaces with standard USB cameras. The `usb_cam.launch` and `usb_cam_double.launch` files have been modified to work with two stereo cameras. For *n* cameras the launch file needs to be modified to include the specifications for each of the *n* cameras. In this work, the `usb_cam` package publishes two different topics. The `/PTZcam/PTZcam/image_raw` publishes topics from the pan-tilt-zoom (PTZ) camera and `/omnicam/omnicam/image_raw` publishes topics from the omnidirectional camera. The maximum frame-rate of the cameras are specified to be at $30~Hz$

Link: http://wiki.ros.org/usb_cam.

**ros-sbgc-driver:** A driver for interfacing with the gimbal. The gimbal uses `SBGC/alexmos` gimbal controller for which no standardized `ROS` package exists. This driver was developed by Haseeb Chowdhury. Contact: haseeb7@vt.edu. Link: https://github.com/haseeb7/ros-sbgc-driver. For other gimbal controllers like the `STorM32 BGC` there exist public `ROS` packages like this: https://github.com/olliw42/storm32bgc.

**freq_sweep:** 
