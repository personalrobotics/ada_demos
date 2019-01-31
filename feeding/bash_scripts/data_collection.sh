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

name=$2"-"$3"-"$4
mkdir $5
mkdir ${5}rosbag/
#output=/home/herb/feeding/rosbag/$name
output=${5}rosbag/$name
echo $output

source ${WSPATH}/devel/setup.bash
rosbag record $cameraInfo $cameraAlignedDepthImageRaw $colorCameraInfo $colorCameraRaw $cameraExtrinsics $jointStates $forque $tf $tfStatic -O $output /topic __name:=my_bag &

/home/herb/Workspace/gilwoo_ws/devel/bin/feeding -d $1 --foodName $2 --direction $3 --trial $4 -af -o $5

echo "Kill rosbag"
rosnode kill /my_bag

sleep 1
pkill -f rosbag
