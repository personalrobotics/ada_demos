#include "feeding/util.hpp"
#include <aikido/common/Spline.hpp>
#include <dart/common/StlHelpers.hpp>
#include "external/ParabolicRamp.h"

namespace po = boost::program_options;

namespace feeding {

inline int sgn(double x)
{
  return (x < 0) ? -1 : (x > 0);
}

//==============================================================================
void handleArguments(
    int argc,
    char** argv,
    bool& adaReal,
    bool& autoContinueDemo,
    bool& useFTSensing)
{
  // Default options for flags
  po::options_description po_desc("simple_trajectories options");
  po_desc.add_options()("help,h", "Produce help message")(
      "adareal,a", po::bool_switch(&adaReal), "Run ADA in real")(
      "continueAuto,c",
      po::bool_switch(&autoContinueDemo),
      "Continue Demo automatically")(
      "ftSensing,f",
      po::bool_switch(&useFTSensing),
      "Use Force/Torque sensing");

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
bool waitForUser(const std::string& msg)
{
  ROS_INFO((msg + " Press [ENTER]").c_str());
  char input = ' ';
  std::cin.get(input);
  return input != 'n';
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
int quadraticRootFinder(
    double a, double b, double c, double& r1, double& r2, double epsilon)
{
  if (a == 0)
  {
    // 1st order
    if (b == 0)
    {
      return -1;
    }
    else
    {
      r1 = -c / b;
      return 1;
    }
  }

  if (c == 0)
  {
    r1 = 0;
    r2 = -b / a;
    return 2;
  }

  double det = b * b - 4.0 * a * c;
  if (det < 0.0)
  {
    return -1;
  }
  else if (det == 0.0)
  {
    r1 = -b / (2.0 * a);
    return 1;
  }
  else
  {
    double sqrt_det = sqrt(det);
    if (abs(-b - sqrt_det) < abs(a))
    {
      r1 = 0.5 * (-b + sqrt_det) / a;
    }
    else
    {
      r1 = 2.0 * c / (-b - sqrt_det);
    }

    if (abs(-b + sqrt_det) < abs(a))
    {
      r1 = 0.5 * (-b - sqrt_det) / a;
    }
    else
    {
      r2 = 2.0 * c / (-b + sqrt_det);
    }

    if (r1 < 0 && r1 > -epsilon)
      r1 = 0;
    if (r2 < 0 && r2 > -epsilon)
      r2 = 0;
    return 2;
  }

  return -1;
}

//==============================================================================
double calcSwitchTime(
    double x0, double x1, double dx0, double dx1, double accel)
{
  int res = -1;
  double t1 = 0.0, t2 = 0.0;
  double epsilon = 1e-6;

  double a = 0.0, b = 0.0, c = 0.0;
  if (abs(accel) > 1.0)
  {
    a = accel;
    b = 2.0 * dx0;
    c = 0.5 * (sqrt(dx0) - sqrt(dx1)) / a + x0 - x1;
  }
  else
  {
    a = accel * accel;
    b = 2.0 * accel * dx0;
    c = 0.5 * (sqrt(dx0) - sqrt(dx1)) + (x0 - x1) * accel;
  }

  res = quadraticRootFinder(a, b, c, t1, t2, epsilon);

  if (res == 1)
  {
    if (t1 < 0)
      return 0.0;
    return t1;
  }
  else if (res == 2)
  {
    if (t1 < 0 || t1 * abs(accel) < (dx1 - dx0) * sgn(accel))
    {
      if (t2 < 0 || t2 * abs(accel) < (dx1 - dx0) * sgn(accel))
        return 0.0;

      t1 = t2;
      return t1;
    }

    return std::min(t1, t2);
  }
  return 0.0;
}

void from_vector(Eigen::VectorXd e_vector, std::vector<double>& s_vector)
{
  s_vector = std::vector<double>(
      e_vector.data(), e_vector.data() + e_vector.rows() * e_vector.cols());
}

//==============================================================================
double calcMinTime(
    Eigen::VectorXd& startPosition,
    Eigen::VectorXd& endPosition,
    Eigen::VectorXd& startVelocity,
    Eigen::VectorXd& endVelocity,
    Eigen::VectorXd& maxVelocity,
    Eigen::VectorXd& maxAcceleration)
{
  ROS_INFO_STREAM("startPos: " << startPosition.matrix());

  ParabolicRamp::Vector amax, vmax;
  ParabolicRamp::ParabolicRampND ramp;
  from_vector(startPosition, ramp.x0);
  from_vector(endPosition, ramp.x1);
  from_vector(startVelocity, ramp.dx0);
  from_vector(endVelocity, ramp.dx1);
  from_vector(maxVelocity, vmax);
  from_vector(maxAcceleration, amax);

  if (ramp.IsValid())
  {
    std::cout << "VALID SEGMENT" << std::endl;
    for (int i =0; i<amax.size(); i++) {
      ROS_INFO_STREAM(amax[i] << ",  " << vmax[i] << ",  " << ramp.x0[i] << ",  " << ramp.x1[i] << ramp.dx0[i] << ",  " << ramp.dx1[i]);
    }
    if (ramp.SolveMinTime(amax, vmax))
    {
      ParabolicRamp::Vector betweenVelocityRamp;
      ramp.Derivative((ramp.endTime) / 2, betweenVelocityRamp);
      ROS_INFO("Between Velocity Ramp");
      for (int i =0; i<betweenVelocityRamp.size(); i++) {
        ROS_INFO_STREAM(betweenVelocityRamp[i]);
      }
      std::cout << "TIME SEGMENT SUCCEED, ramp.endTime: " << ramp.endTime << std::endl;
      return ramp.endTime;
    }
    throw std::runtime_error("calcMinTime: SolveMinTime failed");
  }

  throw std::runtime_error("calcMinTime: ramp not valid");
}

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline> createTimedSplineTrajectory(
    Eigen::VectorXd& startPosition,
    Eigen::VectorXd& endPosition,
    Eigen::VectorXd& startVelocity,
    Eigen::VectorXd& endVelocity,
    Eigen::VectorXd& maxVelocity,
    Eigen::VectorXd& maxAcceleration,
    aikido::statespace::ConstStateSpacePtr stateSpace,
    double startTime)
{
  using dart::common::make_unique;
  using CubicSplineProblem
      = aikido::common::SplineProblem<double, int, 4, Eigen::Dynamic, 2>;

  std::size_t dimension = stateSpace->getDimension();

  // create an empty spline
  auto outputTrajectory
      = make_unique<aikido::trajectory::Spline>(stateSpace, startTime);

  // calculate min time
  double trajTime = calcMinTime(
      startPosition,
      endPosition,
      startVelocity,
      endVelocity,
      maxVelocity,
      maxAcceleration);

  // add waypoint
  CubicSplineProblem problem(Eigen::Vector2d(0, trajTime), 4, dimension);
  problem.addConstantConstraint(0, 0, startPosition);
  problem.addConstantConstraint(0, 1, startVelocity);
  problem.addConstantConstraint(1, 0, endPosition);
  problem.addConstantConstraint(1, 1, endVelocity);
  const auto spline = problem.fit();

  auto startState = stateSpace->createState();
  stateSpace->expMap(startPosition, startState);

  // Add the ramp to the output trajectory.
  assert(spline.getCoefficients().size() == 1);
  const auto& coefficients = spline.getCoefficients().front();
  outputTrajectory->addSegment(coefficients, trajTime, startState);


  ROS_INFO_STREAM("ouput duration: " << outputTrajectory->getDuration());
  Eigen::VectorXd startVelocityOutput(stateSpace->getDimension());
  Eigen::VectorXd betweenVelocityOutput(stateSpace->getDimension());
  Eigen::VectorXd endVelocityOutput(stateSpace->getDimension());
  outputTrajectory->evaluateDerivative(outputTrajectory->getStartTime(), 1, startVelocityOutput);
  outputTrajectory->evaluateDerivative((outputTrajectory->getStartTime() + outputTrajectory->getEndTime()) / 2, 1, betweenVelocityOutput);
  outputTrajectory->evaluateDerivative(outputTrajectory->getEndTime(), 1, endVelocityOutput);
  ROS_INFO_STREAM("start velocity: " << startVelocityOutput.matrix());
  ROS_INFO_STREAM("between velocity: " << betweenVelocityOutput.matrix());
  ROS_INFO_STREAM("end velocity: " << endVelocityOutput.matrix());

  return outputTrajectory;
}

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline> createTimedSplineTrajectory(
    const aikido::trajectory::Interpolated& interpolated,
    Eigen::VectorXd& startVelocity,
    Eigen::VectorXd& endVelocity,
    Eigen::VectorXd& maxVelocity,
    Eigen::VectorXd& maxAcceleration)
{
  auto stateSpace = interpolated.getStateSpace();
  auto startTime = interpolated.getStartTime();
  Eigen::VectorXd startConfig(stateSpace->getDimension());
  Eigen::VectorXd endConfig(stateSpace->getDimension());
  auto startState = stateSpace->createState();
  auto endState = stateSpace->createState();
  interpolated.evaluate(interpolated.getStartTime(), startState);
  interpolated.evaluate(interpolated.getEndTime(), endState);
  interpolated.getStateSpace()->logMap(startState, startConfig);
  interpolated.getStateSpace()->logMap(endState, endConfig);
  return createTimedSplineTrajectory(startConfig, endConfig,
                                     startVelocity, endVelocity,
                                     maxVelocity, maxAcceleration,
                                     stateSpace, startTime);
}

}
