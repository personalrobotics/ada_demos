#include "cameraCalibration/Perception.hpp"

namespace cameraCalibration {

//=============================================================================
Perception::Perception(
    ros::NodeHandle nodeHandle,
    std::string markerTopic,
    std::string cameraInfoTopic,
    bool isCompressed,
    int patternSizeWidth,
    int patternSizeHeight,
    float squareSize)
  : mNodeHandle(std::move(nodeHandle))
  , mMarkerTopic(std::move(markerTopic))
  , mCameraInfoTopic(std::move(cameraInfoTopic))
  , mIsCompressed(std::move(isCompressed))
  , mPatternSizeWidth(std::move(patternSizeWidth))
  , mPatternSizeHeight(std::move(patternSizeHeight))
  , mSquareSize(std::move(squareSize))
{
  // Do nothing
}

//=============================================================================
void Perception::receiveCameraInfo()
{
  sensor_msgs::CameraInfoConstPtr info
    = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(
        mCameraInfoTopic, mNodeHandle, ros::Duration(1));
  if (info == nullptr)
  {
    ROS_ERROR("nullptr camera info");
    return;
  }

  mCameraModel.fromCameraInfo(info);
}

//=============================================================================
void Perception::receiveImageMessage(cv_bridge::CvImagePtr cv_ptr)
{
  if (mIsCompressed)
  {
    sensor_msgs::CompressedImageConstPtr msg
      = ros::topic::waitForMessage<sensor_msgs::CompressedImage>(
          mMarkerTopic, mNodeHandle, ros::Duration(1));
    if (msg == nullptr)
    {
      ROS_ERROR("nullptr image message");
      return;
    }

    cv_ptr->header = msg->header;
    try
    {
      cv_ptr->image = cv::imdecode(cv::Mat(msg->data), -1);
      cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;
    }
    catch (cv::Exception &e)
    {
      ROS_ERROR("opencv exception: %s", e.what());
      return;
    }
  }
  else
  {
    sensor_msgs::ImageConstPtr msg
      = ros::topic::waitForMessage<sensor_msgs::Image>(
          mMarkerTopic, mNodeHandle, ros::Duration(1));
    if (msg == nullptr)
    {
      ROS_ERROR("nullptr image message");
      return;
    }

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }
}

//=============================================================================
bool Perception::getTargetTransformInCameraLensFrame(Eigen::Isometry3d& transform)
{
  Eigen::Isometry3d rstMat = Eigen::Isometry3d::Identity();

  receiveCameraInfo();

  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
  receiveImageMessage(cv_ptr);
  if (cv_ptr == nullptr)
  {
    ROS_ERROR("Failed to load image");
    return false;
  }

  cv::Size patternsize(mPatternSizeWidth, mPatternSizeHeight);
  cv::Mat image = cv_ptr->image;
  std::vector<cv::Point2f> corners;

  bool found = cv::findChessboardCorners(
      image, patternsize, corners,
      cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);
  if (!found)
  {
    ROS_ERROR("Could not find chessboard corners");
    return false;
  }

  std::vector<cv::Point3f> cb_p3ds;
  std::vector<int> inliers;
  cv::Mat cb_rvec;
  cv::Mat cb_tvec;
  cv::Mat cb_rmat;

  for (int wi=0; wi<patternsize.width; wi++)
  {
    for (int hi=0; hi<patternsize.height; hi++)
    {
      cb_p3ds.push_back(cv::Point3f(mSquareSize * (wi - (patternsize.width-1.0)/2.0), mSquareSize * (hi - (patternsize.height-1.0)/2.0), 0));
    }
  }

  cv::solvePnPRansac(
      cb_p3ds,
      corners,
      mCameraModel.intrinsicMatrix(),
      mCameraModel.distortionCoeffs(),
      cb_rvec,
      cb_tvec,
      false,
      100,
      8.0,
      0.99,
      inliers);

  std::cout << "inliers: " << inliers.size() << std::endl;

  std::vector<cv::Point2f> imagePoints;
  cv::projectPoints(cb_p3ds, cb_rvec, cb_tvec, mCameraModel.intrinsicMatrix(), mCameraModel.distortionCoeffs(), imagePoints);

  cv::Rodrigues(cb_rvec, cb_rmat);

  for (int ri=0; ri<3; ri++)
  {
    for (int ci=0; ci<3; ci++)
    {
      rstMat(ri, ci) = cb_rmat.at<double>(ri, ci);
    }
    rstMat(ri, 3) = cb_tvec.at<double>(ri);
  }

  //std::cout << rstMat.matrix() << std::endl;

  for (auto point : imagePoints) {
    cv::circle(image, point, 3, cv::Scalar(50, 255, 70, 255), 5);
  }
  cv::drawChessboardCorners(image, patternsize, cv::Mat(corners), found);

  cv::imshow("view", image);
  cv::waitKey(0);

  transform = rstMat;
  return true;
}

} // namespace cameraCalibration
