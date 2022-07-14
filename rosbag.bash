#!/bin/bash

echo Recording selected topics

#rosbag record /unnamed/control/camera_orientation /PTZcam/PTZcam/image_raw /omnicam/omnicam/image_raw -o test

#rosbag record /gimbal_target_speed /gimbal_imu_angles /joy_orig /PTZcam/PTZcam/image_raw /omnicam/omnicam/image_raw /commanded_angles -o test
rosbag record /gimbal_target_speed -o test



sleep 1
echo Done recording, change bag name appropriately.

exit 0
