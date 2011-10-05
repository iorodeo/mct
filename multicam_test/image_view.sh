#!/bin/bash
export ROS_MASTER_URI=http://felis:11311
rosrun image_view image_view image:=/camera1/camera/image_raw theora &
rosrun image_view image_view image:=/camera2/camera/image_raw theora &
rosrun image_view image_view image:=/camera3/camera/image_raw theora &
rosrun image_view image_view image:=/camera4/camera/image_raw theora &
rosrun image_view image_view image:=/camera5/camera/image_raw theora &
rosrun image_view image_view image:=/camera6/camera/image_raw theora &
rosrun image_view image_view image:=/camera7/camera/image_raw theora &
rosrun image_view image_view image:=/camera8/camera/image_raw theora &
