#!/bin/bash

WSPATH=$(catkin locate)

cameraInfo="/camera/aligned_depth_to_color/camera_info"
cameraAlignedDepthImageRaw="/camera/aligned_depth_to_color/image_raw"
colorCameraInfo="/camera/color/camera_info"
colorCameraRaw="/camera/color/image_raw"
cameraExtrinsics="/camera/extrinsics/depth_to_color"
jointStates="/joint_states"
tf="tf"
tfStatic="/tf_static"

name=$1
output=/home/herb/feeding/images/$name
echo $output

source ${WSPATH}/devel/setup.bash
rosbag record $cameraInfo $cameraAlignedDepthImageRaw $colorCameraInfo $colorCameraRaw $cameraExtrinsics $jointStates $tf $tfStatic -O $output /topic __name:=my_bag &
# rosnode kill /my_bag

# sleep 1
# pkill -f rosbag
