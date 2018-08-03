
#include "feeding/CheckerboardPerception.hpp"

namespace feeding {

CheckerboardPerception::CheckerboardPerception(
    ros::NodeHandle nodeHandle,
    std::string cameraInfoTopic,
    std::string imageTopic,
    bool isCompressed,
    int patternSizeWidth,
    int patternSizeHeight,
    float squareSize)
  : mNodeHandle(std::move(nodeHandle))
  , mCameraInfoTopic(std::move(cameraInfoTopic))
  , mImageTopic(imageTopic)
  , mIsCompressed(std::move(isCompressed))
  , mPatternSizeWidth(std::move(patternSizeWidth))
  , mPatternSizeHeight(std::move(patternSizeHeight))
  , mSquareSize(std::move(squareSize)) {

}

bool CheckerboardPerception::perceiveCheckerboard(Eigen::Isometry3d& checkerboardCornerTransform) {
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
  std::vector<cv::Point2f> currentCorners;

  bool found = cv::findChessboardCorners(
      image, patternsize, currentCorners,
      cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);
  if (!found)
  {
    ROS_ERROR("Could not find chessboard corners");
    return false;
  }

  std::vector<Eigen::Translation3d> currentModelPoints;

  for (int hi=0; hi<patternsize.height; hi++)
  {
    for (int wi=0; wi<patternsize.width; wi++)
    {
      Eigen::Translation3d point(
        mSquareSize * wi,
        mSquareSize * hi,
        0
      );
      currentModelPoints.push_back(point);
    }
  }

  std::vector<cv::Point3f> cb_p3ds;
  for (auto& point : currentModelPoints) {
    cb_p3ds.push_back(cv::Point3f(point.x(), point.y(), point.z()));
  }
  std::vector<int> inliers;
  cv::Mat cb_rvec;
  cv::Mat cb_tvec;
  cv::Mat cb_rmat;

  cv::solvePnPRansac(
      cb_p3ds,
      currentCorners,
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

  cv::Mat rvec_identity = (cv::Mat_<double>(3, 3) << (1, 0, 0, 0, 1, 0, 0, 0, 1));
  cv::Mat tvec_identity = (cv::Mat_<double>(3, 1) << (0, 0, 0));
  std::vector<cv::Point2f> imagePoints;
  cv::projectPoints(cb_p3ds, rvec_identity, tvec_identity, mCameraModel.intrinsicMatrix(), mCameraModel.distortionCoeffs(), imagePoints);

  cv::Rodrigues(cb_rvec, cb_rmat);

  for (int ri=0; ri<3; ri++)
  {
    for (int ci=0; ci<3; ci++)
    {
      rstMat(ri, ci) = cb_rmat.at<double>(ri, ci);
    }
    rstMat(ri, 3) = cb_tvec.at<double>(ri);
  }

  std::cout << rstMat.matrix() << std::endl;

  for (auto point : imagePoints) {
    cv::circle(image, point, 3, cv::Scalar(50, 255, 70, 255), 5);
  }
  cv::drawChessboardCorners(image, patternsize, cv::Mat(currentCorners), found);

  cv::imshow("view", image);
  cv::waitKey(0);

  checkerboardCornerTransform = rstMat;
  return true;
}

//=============================================================================
void CheckerboardPerception::receiveCameraInfo()
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
void CheckerboardPerception::receiveImageMessage(cv_bridge::CvImagePtr cv_ptr)
{
  if (mIsCompressed)
  {
    sensor_msgs::CompressedImageConstPtr msg
      = ros::topic::waitForMessage<sensor_msgs::CompressedImage>(
          mImageTopic, mNodeHandle, ros::Duration(1));
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
          mImageTopic, mNodeHandle, ros::Duration(1));
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

}
