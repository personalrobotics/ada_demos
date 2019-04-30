#!/bin/bash

WSPATH=$(catkin locate)

cameraInfo="/camera/aligned_depth_to_color/camera_info"
cameraAlignedDepthImageRaw="/camera/aligned_depth_to_color/image_raw"
colorCameraInfo="/camera/color/camera_info"
colorCameraRaw="/camera/color/image_raw"
cameraExtrinsics="/camera/extrinsics/depth_to_color"
jointStates="/joint_states"
forque="/forque/forqueSensor"
tf="tf"
tfStatic="/tf_static"
spanetDetection="/food_spanet_detector/detection_image"
spanetImage="/food_spanet_detector/spanet_image"
name=$1"-"$2"-"$3
mkdir $4
mkdir ${4}rosbag/
#output=/home/herb/feeding/rosbag/$name
output=${4}rosbag/$name
echo $output

source ${WSPATH}/devel/setup.bash
rosbag record $cameraInfo $cameraAlignedDepthImageRaw $colorCameraInfo $colorCameraRaw $cameraExtrinsics $jointStates $forque $tf $tfStatic $spanetImage $spanetDetection -O $output /topic __name:=my_bag &


/home/herb/Workspace/spanet_ws/devel/bin/feeding -d collect_spanet --foodName $1 --trial $2 --scenario $3 -af -o $4


echo "Kill rosbag"
rosnode kill /my_bag

sleep 1
pkill -f rosbag
