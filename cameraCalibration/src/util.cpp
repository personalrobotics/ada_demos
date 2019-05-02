#include "cameraCalibration/util.hpp"
#include <aikido/constraint/TestableIntersection.hpp>
#include <tf_conversions/tf_eigen.h>

namespace po = boost::program_options;

namespace cameraCalibration {

//==============================================================================
void handleArguments(
    int argc, char** argv, bool& adaReal, bool& autoContinueDemo)
{
  // Default options for flags
  po::options_description po_desc("simple_trajectories options");
  po_desc.add_options()("help,h", "Produce help message")(
      "adareal,a", po::bool_switch(&adaReal), "Run ADA in real")(
      "continueAuto,c", po::bool_switch(&autoContinueDemo));

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, po_desc), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    std::cout << po_desc << std::endl;
    exit(0);
  }
}

//==============================================================================
void waitForUser(const std::string& msg)
{
  ROS_INFO((msg + " Press [ENTER]").c_str());
  char input = ' ';
  std::cin.get(input);
  if (input == 'n')
  {
    exit(0);
  }
}

//==============================================================================
Eigen::Isometry3d createIsometry(
    double x, double y, double z, double roll, double pitch, double yaw)
{
  Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
  isometry.translation() = Eigen::Vector3d(x, y, z);
  Eigen::Matrix3d rotation;
  rotation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
             * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
             * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
  isometry.linear() = rotation;

  return isometry;
}

//==============================================================================
Eigen::Isometry3d createIsometry(std::vector<double> vec)
{
  if (vec.size() < 6)
  {
    throw std::runtime_error("Vector size to small: " + vec.size());
  }
  return createIsometry(vec[0], vec[1], vec[2], vec[3], vec[4], vec[5]);
}

//==============================================================================
Eigen::MatrixXd createBwMatrixForTSR(
    double horizontalTolerance,
    double verticalTolerance,
    double yawMin,
    double yawMax)
{
  Eigen::MatrixXd bw = Eigen::Matrix<double, 6, 2>::Zero();
  bw(0, 0) = -horizontalTolerance;
  bw(0, 1) = horizontalTolerance;
  bw(1, 0) = -horizontalTolerance;
  bw(1, 1) = horizontalTolerance;
  bw(2, 0) = -verticalTolerance;
  bw(2, 1) = verticalTolerance;
  bw(5, 0) = yawMin;
  bw(5, 1) = yawMax;
  return bw;
}

//==============================================================================
aikido::constraint::dart::TSR getCalibrationTSR(const Eigen::Isometry3d& transform)
{
  auto tsr = pr_tsr::getDefaultPlateTSR();
  tsr.mT0_w = transform;
  tsr.mTw_e.translation() = Eigen::Vector3d{0, 0, 0};

  tsr.mBw = createBwMatrixForTSR(
      0.001, 0.001, 0, 0);
  return tsr;
}

//==============================================================================
void printPose(const Eigen::Isometry3d& pose)
{
  double x1 = pose.translation().x();
  Eigen::Quaterniond quat(pose.linear());
  ROS_INFO_STREAM("(" << pose.translation().x() << ", " << pose.translation().y() << ", " << pose.translation().z() << "), (" <<
    quat.w() << ", " << quat.x() << ", " << quat.y() << ", " << quat.z() << ")"
  );
}

//==============================================================================
Eigen::Isometry3d getWorldToJoule(tf::TransformListener& tfListener) {
  return getRelativeTransform(tfListener,
    "/j2n6s200_joule", "/map");
}

//==============================================================================
Eigen::Isometry3d getCameraToLens(tf::TransformListener& tfListener) {
  return getRelativeTransform(tfListener,
    "/camera_link", "/camera_color_optical_frame");
}

//==============================================================================
Eigen::Isometry3d getCameraToJoule(tf::TransformListener& tfListener)
{
  return getRelativeTransform(tfListener,
    "/camera_link", "/j2n6s200_joule");
}

//==============================================================================
Eigen::Isometry3d getRelativeTransform(tf::TransformListener& tfListener,
  const std::string& from, const std::string& to)
{
  tf::StampedTransform tfStampedTransform;
  try{
    tfListener.lookupTransform(to, from,
                            ros::Time(0), tfStampedTransform);
  }
  catch (tf::TransformException ex){
    throw std::runtime_error("Failed to get TF Transform: " + std::string(ex.what()));
  }

  Eigen::Isometry3d transform;
  tf::transformTFToEigen(tfStampedTransform, transform);

  return transform;
}
}
