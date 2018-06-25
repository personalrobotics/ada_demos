#ifndef PERCEPTION_H
#define PERCEPTION_H

#include <Eigen/Geometry>
#include <ros/ros.h>
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

namespace cameraCalibration {

class Perception
{
public:
  Perception(
      ros::NodeHandle nodeHandle,
      std::string markerTopic,
      std::string cameraInfoTopic,
      bool isCompressed,
      int patternSizeWidth,
      int patternSizeHeight,
      float squareSize);

  bool getTargetTransformInCameraLensFrame(Eigen::Isometry3d& transform);

  void receiveImageMessage(cv_bridge::CvImagePtr cv_ptr);

  void receiveCameraInfo();

private:
  ros::NodeHandle mNodeHandle;
  std::string mMarkerTopic;
  std::string mCameraInfoTopic;
  bool mIsCompressed;
  int mPatternSizeWidth;
  int mPatternSizeHeight;
  float mSquareSize;

  image_geometry::PinholeCameraModel mCameraModel;
};

} // namespace cameraCalibration

#endif  // PERCEPTION_H
