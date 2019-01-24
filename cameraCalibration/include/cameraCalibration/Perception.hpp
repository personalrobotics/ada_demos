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
  /// \param[in] nodeHandle Ros node.
  /// \param[in] imageTopic Image topic.
  /// \param[in] cameraInfoTopic Camera info topic.
  /// \param[in] isCompressed True if compressed image is used.
  /// \param[in] numPatternsInWidth Number of patterns in width.
  /// \param[in] numPatternsInHeight Number of patterns in width.
  /// \param[in] squareSize  Width of each checkerboard in meter.
  Perception(
      ros::NodeHandle nodeHandle,
      std::string imageTopic,
      std::string cameraInfoTopic,
      bool isCompressed,
      int numPatternsInWidth,
      int numPatternsInHeight,
      float squareSize);

  /// Get transform from camera to joule
  /// \param[in] targetToWorld target (center of checkerboard) to world transform
  /// \param[in] worldToJoule world to joule transform
  /// \param[in] cameraToLens camera to lens transform
  /// \param[in] cameraToJouleGuess initial estimate of camera to joule transform
  Eigen::Isometry3d computeCameraToJoule(
    const Eigen::Isometry3d& targetToWorld,
    const Eigen::Isometry3d& worldToJoule,
    const Eigen::Isometry3d& cameraToLens,
    const Eigen::Isometry3d& cameraToJouleGuess);

  /// Get average transform given all estimates
  /// \param[in] cameraToJouleEstimates All estiamtes of camera to joule transform
  Eigen::Isometry3d computeMeanCameraToJouleEstimate(
    const std::vector<Eigen::Isometry3d>& cameraToJouleEstimates);

  /// Visualization of 3D corner points on checkerboard
  /// Projections should meet the detected checkerboard corners.
  /// \param[in] targetToWorld Target (checkerboard center) to world transform.
  /// \param[in] worldToCamera World to camera transform.
  /// \param[in] cameraToLens Camera to Lens transform.
  /// \param[in] cameraToJoule Camera to Joule transform.
  void visualizeProjection(
    const Eigen::Isometry3d& targetToWorld,
    const Eigen::Isometry3d& worldToCamera,
    const Eigen::Isometry3d& cameraToLens,
    const Eigen::Isometry3d& cameraToJoule);

private:

  /// Detect 2D checkerboard corners and update 3D corners
  /// \param[in] targetToWorld Target (checkerboard center) to world transform.
  /// \param[out] modelPoints
  /// \param[out] corners Detected 2D checkerboard corners
  /// \param[out] image Save the captured image.
  bool recordView(
    const Eigen::Isometry3d& targetToWorld,
    std::vector<cv::Point3f>& modelPoints,
    std::vector<cv::Point2f>& corners,
    cv::Mat& image);

  /// Received image message and updates
  /// \param[out] cv_ptr Updates image to cv_ptr.
  void receiveImageMessage(cv_bridge::CvImagePtr& cv_ptr);

  /// Updates camera info
  void receiveCameraInfo();

  /// Visualization of 3D corner points on checkerboard
  /// Projections should meet the detected checkpoint corners
  /// \param[in] rvec Extrinsic rotation matrix
  /// \param[in] tvec Extrinsic translation matrix
  /// \param[in] modelPoints 3D corner points of the checkerboard in
  /// world frame
  /// \param[in] corners 2D corner points of the checkerboard
  /// \param[in] image Image
  /// \param[in] pointsColor Color of 3D corner points
  /// \param[in] cornerColor Color of 2D corner points
  void visualizeProjection(
    const cv::Mat& rvec,
    const cv::Mat& tvec,
    const std::vector<cv::Point3f>& modelPoints,
    const std::vector<cv::Point2f>& corners,
    const cv::Mat& image,
    const cv::Scalar pointsColor = cv::Scalar(50, 255, 70, 255),
    const cv::Scalar cornerColor = cv::Scalar(255, 0, 0, 255));

  /// Captures current image
  /// \param[out] image
  /// \return false if capture fails
  bool captureFrame(cv::Mat& image);

  ros::NodeHandle mNodeHandle;
  std::string mImageTopic;
  std::string mCameraInfoTopic;
  bool mIsCompressed;
  int mPatternSizeWidth;
  int mPatternSizeHeight;
  float mSquareSize;

  image_geometry::PinholeCameraModel mCameraModel;

};

} // namespace cameraCalibration

#endif  // PERCEPTION_H
