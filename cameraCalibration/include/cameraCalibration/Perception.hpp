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

  /// \param[out] rvec
  /// \param[out] tvec
  Eigen::Isometry3d computeJouleToOptical(
    const Eigen::Isometry3d& targetToWorld,
    const Eigen::Isometry3d& worldToJoule);

  Eigen::Isometry3d computeMeanJouleToOptical(
    const std::vector<Eigen::Isometry3d>& jouleToOpticals);

  // projections should meet the corners on the image
  void visualizeProjection(
    const cv::Mat& rvec,
    const cv::Mat& tvec,
    const std::vector<cv::Point3f>& modelPoints,
    const std::vector<cv::Point2f>& corners,
    const cv::Mat& image);

  void visualizeProjection(
    const Eigen::Isometry3d& targetToWorld,
    const Eigen::Isometry3d& worldToJoule,
    const Eigen::Isometry3d& cameraToJoule);

  void receiveImageMessage(cv_bridge::CvImagePtr cv_ptr);

  void receiveCameraInfo();

  ///\param[out] image
  bool captureFrame(cv::Mat& image);

  Eigen::Isometry3d getCameraToJouleFromJouleToOptical(
    const Eigen::Isometry3d& jouleToOptical,
    const Eigen::Isometry3d& cameraToOptical);

private:

  /// \param[out] modelPoints
  /// \param[out] corners
  /// \param[out] image
  bool recordView(
    const Eigen::Isometry3d& targetToWorld,
    const Eigen::Isometry3d& worldToJoule,
    std::vector<cv::Point3f>& modelPoints,
    std::vector<cv::Point2f>& corners,
    cv::Mat& image
    );

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
