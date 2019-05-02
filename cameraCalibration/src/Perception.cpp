#include "cameraCalibration/Perception.hpp"
#include <Eigen/Eigenvalues>
#include <opencv2/core/eigen.hpp>
#include <ros/ros.h>

namespace cameraCalibration {

//==============================================================================
void isometryTocv(const Eigen::Isometry3d& transform,
    cv::Mat& rvec,
    cv::Mat& tvec)
{
  cv::Mat rmat;

  // get rvec
  Eigen::MatrixXd rot = transform.linear();
  cv::eigen2cv(rot, rmat);
  cv::Rodrigues(rmat, rvec);

  // get tvec
  Eigen::Vector3d translation(transform.translation());
  cv::eigen2cv(translation, tvec);
  return;
}
//=============================================================================
Perception::Perception(
    ros::NodeHandle nodeHandle,
    std::string imageTopic,
    std::string cameraInfoTopic,
    bool isCompressed,
    int patternSizeWidth,
    int patternSizeHeight,
    float squareSize)
  : mNodeHandle(std::move(nodeHandle))
  , mImageTopic(std::move(imageTopic))
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
void Perception::receiveImageMessage(
  cv_bridge::CvImagePtr& cv_ptr)
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

//=============================================================================
Eigen::Isometry3d Perception::computeCameraToJoule(
  const Eigen::Isometry3d& targetToWorld,
  const Eigen::Isometry3d& worldToJoule,
  const Eigen::Isometry3d& cameraToLens,
  const Eigen::Isometry3d& cameraToJouleGuess)
{
  std::vector<cv::Point3f> pointsToWorld;
  std::vector<cv::Point2f> corners;
  cv::Mat image;

  auto result = recordView(
    targetToWorld,
    pointsToWorld, corners, image);

  //=======================  Solve PnP ==============================
  std::vector<int> inliers;

  ROS_INFO("cameraToJouleGuess");
  ROS_INFO_STREAM("\n" << cameraToJouleGuess.matrix());

  // Initial guess
  // LensToWorld = JouleToWorld * CameraToJoule * LensToCamera;
  auto lensToWorldGuess = worldToJoule.inverse() * cameraToJouleGuess * cameraToLens.inverse();
  auto worldToLensGuess = lensToWorldGuess.inverse();

  cv::Mat cb_rvec, cb_tvec;
  cv::Mat cb_rmat;

  isometryTocv(worldToLensGuess, cb_rvec, cb_tvec);

  cv::solvePnPRansac(
      pointsToWorld,
      corners,
      mCameraModel.intrinsicMatrix(),
      mCameraModel.distortionCoeffs(),
      cb_rvec,
      cb_tvec,
      true,
      100,
      8.0,
      0.99,
      inliers);
  // cv::solvePnPRansac(
  //     pointsToWorld,
  //     corners,
  //     mCameraModel.intrinsicMatrix(),
  //     mCameraModel.distortionCoeffs(),
  //     cb_rvec,
  //     cb_tvec,
  //     true);


  ROS_INFO_STREAM("Inliers: " << inliers.size());

  if (inliers.size() < 9)
    throw std::runtime_error("Too few inliers");

  ROS_INFO("Solved PnP");


  // Turn this on to visualize the mapping
  visualizeProjection(cb_rvec, cb_tvec, pointsToWorld, corners, image);

  cv::Rodrigues(cb_rvec, cb_rmat);

  Eigen::Isometry3d worldToLens = Eigen::Isometry3d::Identity();
  for (int ri=0; ri<3; ri++)
  {
    for (int ci=0; ci<3; ci++)
    {
      worldToLens(ri, ci) = cb_rmat.at<double>(ri, ci);
    }
    worldToLens(ri, 3) = cb_tvec.at<double>(ri);
  }

  auto lensToWorld = worldToLens.inverse();

  // LensToWorld = JouleToWorld * CameraToJoule * LensToCamera;
  // <--> CameraToJoule = JouleToWorld.inv() * LensToWorld * LensToCamera.inverse()
  auto cameraToJoule = worldToJoule * lensToWorld * cameraToLens;

  ROS_INFO_STREAM("Solved CameraToJoule. " <<
    "This should not be too different from the initial guess.");
  ROS_INFO_STREAM("\n" << cameraToJoule.matrix());

  // Turn this on to visualize the mapping
  visualizeProjection(targetToWorld, worldToJoule, cameraToLens, cameraToJoule);

  return cameraToJoule;
}

//=============================================================================
bool Perception::recordView(
  const Eigen::Isometry3d& targetToWorld,
  std::vector<cv::Point3f>& pointsToWorld,
  std::vector<cv::Point2f>& corners,
  cv::Mat& image)
{
  captureFrame(image);

  cv::Size patternsize(mPatternSizeWidth, mPatternSizeHeight);

  bool found = cv::findChessboardCorners(
      image, patternsize, corners,
      cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);
  if (!found)
  {
    ROS_ERROR("Could not find chessboard corners");
    return false;
  }

  for (int hi=0; hi<patternsize.height; hi++)
  {
    for (int wi=0; wi<patternsize.width; wi++)
    {
      Eigen::Translation3d pointToTarget(
        mSquareSize * (wi - (patternsize.width-1.0)/2.0),
        mSquareSize * (hi - (patternsize.height-1.0)/2.0),
        0
      );
      Eigen::Translation3d pointToWorld((targetToWorld * pointToTarget).translation());
      std::cout << "pointsToWorld " << pointToWorld.x() << " " <<  pointToWorld.y() << " " <<  pointToWorld.z() << std::endl;
      pointsToWorld.push_back(cv::Point3f(pointToWorld.x(), pointToWorld.y(), pointToWorld.z()));
    }
  }

  return true;
}

//=============================================================================
Eigen::Isometry3d Perception::computeMeanCameraToJouleEstimate(
  const std::vector<Eigen::Isometry3d>& cameraToJouleEstimates)
{
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> rotations(4, cameraToJouleEstimates.size());
  Eigen::Vector3d sumTranslations(Eigen::Vector3d::Zero());

  for(std::size_t i = 0; i < cameraToJouleEstimates.size(); ++i)
  {
    Eigen::Quaterniond q(cameraToJouleEstimates[i].linear());
    rotations.col(i) = q.coeffs();

    sumTranslations += cameraToJouleEstimates[i].translation();
  }
  auto translation = sumTranslations / cameraToJouleEstimates.size();

  // Get largest Eigen vector
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigensolver(rotations * rotations.transpose());

  if (eigensolver.info() != Eigen::Success)
  {
    throw std::runtime_error("Failed to solve Eigen decomposition.");
  }

  // Find the largest eigen value index
  auto eigenVals = eigensolver.eigenvalues();
  Eigen::VectorXd::Index maxIdx;
  double maxVal = eigenVals.maxCoeff(&maxIdx);

  // The largest eigen vector is the average rotation.
  Eigen::VectorXd rotation = eigensolver.eigenvectors().col(maxIdx);
  rotation.normalize();
  Eigen::Quaterniond q;
  q.coeffs() = rotation;
  auto rotationMatrix = q.toRotationMatrix();

  // To Isometry3d
  Eigen::Isometry3d cameraToJoule(Eigen::Isometry3d::Identity());
  cameraToJoule.linear() = rotationMatrix;
  cameraToJoule.translation() = translation;

  Eigen::Vector3d eulerAngles = cameraToJoule.linear().eulerAngles(2, 1, 0);
  ROS_INFO_STREAM("Average CameraToJoule" << std::endl << cameraToJoule.matrix() << std::endl);
  ROS_INFO_STREAM("Yaw: " << eulerAngles.x() << "\npitch: " << eulerAngles.y() << "\nroll: " << eulerAngles.z() << std::endl);

  // auto jouleToCamera = cameraToJoule.inverse();
  // eulerAngles = jouleToCamera.linear().eulerAngles(2, 1, 0);
  // ROS_INFO_STREAM("Average JouleToCamera" << std::endl << jouleToCamera.matrix() << std::endl);
  // ROS_INFO_STREAM("Yaw: " << eulerAngles.x() << "\npitch: " << eulerAngles.y() << "\nroll: " << eulerAngles.z() << std::endl);

  ROS_INFO_STREAM("Replace st_joule2camera in ada_launch/default.launch with "
   << translation.transpose() << " " << eulerAngles.x() << " " << eulerAngles.y() << " " << eulerAngles.z() << std::endl);
  return cameraToJoule;
}

//=============================================================================
 void Perception::visualizeProjection(
    const cv::Mat& rvec,
    const cv::Mat& tvec,
    const std::vector<cv::Point3f>& pointsToWorld,
    const std::vector<cv::Point2f>& corners,
    const cv::Mat& image,
    const cv::Scalar pointsColor,
    const cv::Scalar cornerColor)
 {

  std::vector<cv::Point2f> imagePoints;
  cv::projectPoints(pointsToWorld, rvec, tvec,
    mCameraModel.intrinsicMatrix(), mCameraModel.distortionCoeffs(), imagePoints);

  for (auto point : imagePoints) {
    cv::circle(image, point, 3, pointsColor, 5);
  }
  for (auto corner : corners) {
    cv::circle(image, corner, 1, cornerColor, 3);
  }

  ROS_INFO( "[Enter] to continue");
  cv::imshow("view", image);
  cv::waitKey(0);
}

//=============================================================================
void Perception::visualizeProjection(
  const Eigen::Isometry3d& targetToWorld,
  const Eigen::Isometry3d& worldToJoule,
  const Eigen::Isometry3d& cameraToLens,
  const Eigen::Isometry3d& cameraToJoule)
{
  // LensToWorld = JouleToWorld * CameraToJoule * LensToCamera;
  auto lensToWorld = worldToJoule.inverse() * cameraToJoule * cameraToLens.inverse();
  auto worldToLens = lensToWorld.inverse();
  // get rvec

  cv::Mat rvec, tvec;
  isometryTocv(worldToLens, rvec, tvec);

  std::vector<cv::Point3f> pointsToWorld;
  std::vector<cv::Point2f> corners;
  cv::Mat image;
  if (!recordView(targetToWorld, pointsToWorld, corners, image))
  {
    ROS_ERROR("Failed to find corners");
    return;
  }

  visualizeProjection(rvec, tvec, pointsToWorld, corners, image,
    cv::Scalar(0, 0, 255, 255),
    cv::Scalar(255, 0, 0, 255));
}

//=============================================================================
bool Perception::captureFrame(cv::Mat& image)
{
  receiveCameraInfo();

  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
  receiveImageMessage(cv_ptr);
  if (cv_ptr == nullptr)
  {
    ROS_ERROR("Failed to load image");
    return false;
  }

  image = cv_ptr->image;
  return true;
}
} // namespace cameraCalibration
