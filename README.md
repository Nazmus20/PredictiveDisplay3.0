Predictive Display: Version 3.0
By: Nazmus Sakib
Email: nazmus.sakib@vt.edu

The results can be found [here](https://drive.google.com/drive/folders/1f9gV5OK0pZHkXfprwHbA9QsX2AIyfRQ3).

The test file can be found [here](https://drive.google.com/drive/folders/1GoiN19-Ou4Bk_xG9zreU-RFu7i5b3RF3).

This document contains an introduction on different standard and customized `ROS`packages used in the project and a step-by-step method for running the predictive algorithm on collected `rosbag` data. The development of the predictive algorithm can be found in this paper (**ADD LINK TO THE PAPER**).

## `ROS` Packages Used
**vision_opencv:** Standard `ROS` package for interfacing `ROS` with `OpenCV`. The package contains function like `CvBridge()` that converts `ROS` image messages to `OpenCV` `cv::Mat` image matrices used by other `OpenCV` packages and functions. For more details see [this link](http://wiki.ros.org/vision_opencv). 

**usb_cam:** This package interfaces with standard USB cameras. The `usb_cam.launch` and `usb_cam_double.launch` files have been modified to work with two stereo cameras. For *n* cameras the launch file needs to be modified to include the specifications for each of the *n* cameras. For moer information about the `usb_cam` package see [this link](http://wiki.ros.org/usb_cam). In this work, the `usb_cam` package publishes two different topics. The `/PTZcam/PTZcam/image_raw` publishes topics from the pan-tilt-zoom (PTZ) camera and `/omnicam/omnicam/image_raw` publishes topics from the omnidirectional camera. The maximum frame-rate of both the cameras are specified to be at $30~\text{Hz}$. The PTZ camera used is this [Varifocal Lens USB Camera](https://www.amazon.com/Webcamera-usb-Varifocal-1920X1080-Adjustable/dp/B07N1C55CH/ref=sr_1_4?dchild=1&keywords=camera%2Busb%2Blong%2Bfocus&qid=1596408361&sr=8-4&th=1) by Webcamera_usb. The omnidirectional camera is the [Insta360 air](https://www.insta360.com/product/insta360-air/#air_top) by Insta360.

**joystich_drivers:** This package contains `ROS` drivers for generic Linux joysticks. We are currently using the `ps4joy.launch` file under the `joy` sub-package to establish connection between `ROS` and a generic Microsoft XBox controller. More details on its different parameters can be found [here](http://wiki.ros.org/joy). The `ps4joy.launch` file publishes a topic named `/joy_orig` which cointains information on stick and button positions in the range of $[0,~1]$.

**ros-sbgc-driver:** A driver for interfacing with the gimbal. The gimbal uses `SBGC/alexmos` gimbal controller for which no standardized `ROS` package exists. This driver was developed by Haseeb Chowdhury (haseeb7@vt.edu). The `GitHub` link to the driver is [here](https://github.com/haseeb7/ros-sbgc-driver). For other gimbal controllers like the `STorM32 BGC` there exist public `ROS` packages like [this](https://github.com/olliw42/storm32bgc). The gimbal used is the [Infinity MR-S2](https://hdairstudio.com/wp-content/uploads/2020/03/InfinityMR-S2-User-Manual-V.01.pdf) gimbal by HDAir Studio. A customized 3D printed casing that holds both the cameras together is mounted on the gimbal; see Fig. 5a of the paper. All the parameters for the gimbal and the two heterogeneous stereo cameras have been obtained for this specific configuration indicated by Fig 5a. The `ros-sbgc-driver` publishes the IMU angles of the gimbal to the `gimbal_imu_angles` and the encoder angles to the `gimbal_enc_angles` topics. The motion of the gimbal can be controlled by sending reference angular rates or orientations to separate topics. Reference angular rate tracking can be done by publishing to the `/gimbal_target_speed` topic and reference orientation tracking can be done by publishing to the `/gimbal_target_orientation` topic. The `ros_sbgc-driver` package subscribes to these two topics and and converts the commanded values to the necessary PWM signals read by the gimbal.

**freq_sweep:** This `C++` package injects a sinusoidal frequency sweep in the roll, pitch, and yaw gimbal axes to identify its transfer function. The messages are published to the topic called `/gimbal_target_speed` to perform reference speed tracking. The x-, y-, and z-axes of `/gimbal_target_speed` messages denote the roll, pitch, and yaw motions of the gimbal respectively. The commanded messages published to the topics `/gimbal_target_speed` or `/gimbal_target_orientation` and the corresponding measurements obtained from the topics `/gimbal_imu_angles` or `/gimbal_enc_angles` give us the transfer function of the gimbal.

**het_ster_calib:** This package performs heterogeneous stereo calibration of the two cameras. The ouput is a homography matrix that maps pixels from one camera frame to pixels in another camera frame which is required by the predictive algorithm to perform proper image stitching. You need to specify the dierctories where the PTZ and the omni images are saved. The naming convention should be 

## Step-by-step instructions to run the `ROS` packages

### First-time users
**Step 1:** After downloading the `GitHub` repository to your `Linux`-based computer navigate to the downloaded directory named `PredictiveDisplay3.0`. Please `build` the packages using `catkin_make` or `catkin build` and source the `devel/setup.bash` file.

**Step 2:** Type `roslaunch joy2gimbal start.launch` to `launch` the `usb_cam`, `joy`, and `ros-sbgc-driver` together. If there are connectivity errors like `USB0` device is not found then type `sudo chmod 666 /dev/ttyUSB0` to let `ROS` access that port. This launch file publishes camera topics like `/PTZcam/PTZcam/image_raw`, `/omnicam/omnicam/image_raw`, `/gimbal_imu_angles`, and `/gimbal_enc_angles`.

**Step 3:** Then type `roslaunch joy2gimbal joy2gimbal.launch` this launches the `joy2gimbal` package that maps `/joy_orig` topic to `gimbal_target_speed` or `gimbal_target_orientation` based on how it is set up. It also adds transmission delay to the commands being sent to the gimbal and those are recorded in `/delayed_imu_angles` topic. The undelayed commands are published in `/commanded_angles`. Finally, it adds another transmission delay to the incoming `/gimbal_imu_angles` topic. The amount of delays in both these cases are specified in the `launch` file.

**Step 4:** Finally type `roslaunch predictor delay_predictor.launch` which subscribes to the delayed `/gimbal_imu_angles`, `/PTZcam/PTZcam/image_raw`, `/omnicam/omnicam/image_raw`, and /commanded angles` to perform the prediction and stitching as output.

### Demo instructions

We have provided two demo files named `testcase1_diagonal.bag` and `testcase2_horizontal.bag` which contain data collected during normal operation. To utilize this `.bag` file the provided `ROS` package will need to be installed, built, and sourced properly as in Step 1. 

Once that is done, type `roslaunch predictor delay_predictor.launch` to launch the predictive display.

In a separate terminal type `rosbag play filename.bag` to play the rosbag. Users should see the predictive display pop-up. Every time the bag finishes playing, relaunch the launch file and re-play the bag file to continue. 



