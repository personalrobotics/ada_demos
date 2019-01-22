#include "cameraCalibration/Perception.hpp"
#include <Eigen/Eigenvalues>
#include <opencv2/core/eigen.hpp>

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
Eigen::Isometry3d Perception::computeJouleToOptical(
  const Eigen::Isometry3d& targetToWorld,
  const Eigen::Isometry3d& worldToJoule)
{
  std::vector<cv::Point3f> modelPoints;
  std::vector<cv::Point2f> corners;
  cv::Mat image;

  auto result = recordView(
    targetToWorld, worldToJoule,
    modelPoints, corners, image);

  //=======================  Solve PnP ==============================
  std::vector<int> inliers;

  // Initial guess
  cv::Mat cb_rvec = (cv::Mat_<double>(3,1) << -2.120, 0.206, -2.076);
  cv::Mat cb_tvec = (cv::Mat_<double>(3,1) << -0.021, 0.043, -0.094);
  cv::Mat cb_rmat;

  cv::solvePnP(
      modelPoints,
      corners,
      mCameraModel.intrinsicMatrix(),
      mCameraModel.distortionCoeffs(),
      cb_rvec,
      cb_tvec,
      true, CV_ITERATIVE);

  std::cout << "inliers: " << inliers.size() << std::endl;
  if (inliers.size() < 9)
    throw std::runtime_error("Too few inliers");

  std::cout << "Solved PnP" << std::endl;

  std::cout << "rvec: " << cb_rvec << std::endl;
  std::cout << "tvec: " << cb_tvec << std::endl;

  visualizeProjection(cb_rvec, cb_tvec, modelPoints, corners, image);

  cv::Rodrigues(cb_rvec, cb_rmat);

  Eigen::Isometry3d jouleToOptical = Eigen::Isometry3d::Identity();
  for (int ri=0; ri<3; ri++)
  {
    for (int ci=0; ci<3; ci++)
    {
      jouleToOptical(ri, ci) = cb_rmat.at<double>(ri, ci);
    }
    jouleToOptical(ri, 3) = cb_tvec.at<double>(ri);
  }

  return jouleToOptical;
}

//=============================================================================
bool Perception::recordView(
  const Eigen::Isometry3d& targetToWorld,
  const Eigen::Isometry3d& worldToJoule,
  std::vector<cv::Point3f>& modelPoints,
  std::vector<cv::Point2f>& corners,
  cv::Mat& image)
{
  std::cout << "targetToWorld: " << std::endl << targetToWorld.matrix() << std::endl;
  std::cout << "worldToJoule: " << std::endl << worldToJoule.matrix() << std::endl;

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
      Eigen::Translation3d point(
        mSquareSize * (wi - (patternsize.width-1.0)/2.0),
        mSquareSize * (hi - (patternsize.height-1.0)/2.0),
        0
      );
      Eigen::Translation3d transformedPoint((worldToJoule * targetToWorld * point).translation());
      modelPoints.push_back(cv::Point3f(transformedPoint.x(), transformedPoint.y(), transformedPoint.z()));
    }
  }

  return true;
}

//=============================================================================
Eigen::Isometry3d Perception::computeMeanJouleToOptical(const std::vector<Eigen::Isometry3d>& jouleToOpticals)
{
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> rotations(4, jouleToOpticals.size());
  Eigen::Vector3d sumTranslations(Eigen::Vector3d::Zero());

  for(std::size_t i = 0; i < jouleToOpticals.size(); ++i)
  {
    Eigen::Quaterniond q(jouleToOpticals[i].linear());
    rotations.col(i) = q.coeffs();
    sumTranslations += jouleToOpticals[i].translation();
  }
  auto translation = sumTranslations / jouleToOpticals.size();

  // Get largest Eigen vector
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigensolver(rotations * rotations.transpose());

  if (eigensolver.info() != Eigen::Success)
  {
    throw std::runtime_error("Failed to solve Eigen decomposition.");
  }

  Eigen::VectorXd rotation = eigensolver.eigenvectors().col(0);
  rotation.normalize();

  Eigen::Quaterniond q;
  q.coeffs() = rotation;
  auto rotationMatrix = q.toRotationMatrix();

  // To Isometry3d
  Eigen::Isometry3d mean(Eigen::Isometry3d::Identity());
  mean.linear() = rotationMatrix;
  mean.translation() = translation;

  return mean;
}

//=============================================================================
 void Perception::visualizeProjection(
    const cv::Mat& rvec,
    const cv::Mat& tvec,
    const std::vector<cv::Point3f>& modelPoints,
    const std::vector<cv::Point2f>& corners,
    const cv::Mat& image)
 {

  std::vector<cv::Point2f> imagePoints;
  cv::projectPoints(modelPoints, rvec, tvec,
    mCameraModel.intrinsicMatrix(), mCameraModel.distortionCoeffs(), imagePoints);


  for (auto point : imagePoints) {
    cv::circle(image, point, 3, cv::Scalar(50, 255, 70, 255), 5);
  }
  for (auto corner : corners) {
    cv::circle(image, corner, 1, cv::Scalar(255, 0, 0, 255), 3);
  }
  cv::imshow("view", image);
  cv::waitKey(0);
}

//=============================================================================
void Perception::visualizeProjection(
  const Eigen::Isometry3d& targetToWorld,
  const Eigen::Isometry3d& worldToJoule,
  const Eigen::Isometry3d& cameraToJoule)
{
  // get rvec
  cv::Mat rvec;
  cv::eigen2cv(cameraToJoule.linear().eulerAngles(2, 1, 0), rvec);

  // get tvec
  cv::Mat tvec;
  Eigen::Vector3d translation(cameraToJoule.translation());
  cv::eigen2cv(translation, tvec);

  std::vector<cv::Point3f> modelPoints;
  std::vector<cv::Point2f> corners;
  cv::Mat image;

  recordView(targetToWorld, worldToJoule, modelPoints, corners, image);

  visualizeProjection(rvec, tvec, modelPoints, corners, image);
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
}

//=============================================================================
Eigen::Isometry3d Perception::getCameraToJouleFromJouleToOptical(
  const Eigen::Isometry3d& jouleToOptical,
  const Eigen::Isometry3d& cameraToOptical)
{

  Eigen::Isometry3d cameraToJoule = jouleToOptical.inverse() * cameraToOptical;
  std::cout << "final matrix: " << std::endl << cameraToJoule.matrix() << std::endl;

  Eigen::Vector3d eulerAngles = cameraToJoule.linear().eulerAngles(2, 1, 0);
  std::cout << "yaw: " << eulerAngles.x() << ", pitch: " << eulerAngles.y() << ", roll: " << eulerAngles.z() << std::endl;

  return cameraToJoule;
}


} // namespace cameraCalibration
