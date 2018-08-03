#ifndef FEEDING_CHECKERBOARDPERCEPTION_HPP_
#define FEEDING_CHECKERBOARDPERCEPTION_HPP_

#include <Eigen/Dense>
#include <ros/ros.h>
#include <Eigen/Geometry>
#include <ros/topic.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>

namespace feeding {

class CheckerboardPerception {

public:

  CheckerboardPerception(
    ros::NodeHandle nodeHandle,
    std::string imageTopic,
    std::string cameraInfoTopic,
    bool isCompressed,
    int patternSizeWidth,
    int patternSizeHeight,
    float squareSize);

  bool perceiveCheckerboard(Eigen::Isometry3d& checkerboardCornerTransform);

  void receiveCameraInfo();
  void receiveImageMessage(cv_bridge::CvImagePtr cv_ptr);

private:
  ros::NodeHandle mNodeHandle;
  std::string mImageTopic;
  std::string mCameraInfoTopic;
  bool mIsCompressed;
  int mPatternSizeWidth;
  int mPatternSizeHeight;
  float mSquareSize;

  image_geometry::PinholeCameraModel mCameraModel;

};

}

#endif
