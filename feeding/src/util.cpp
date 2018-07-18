#include "feeding/util.hpp"
#include <algorithm>
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

void from_vector(const Eigen::VectorXd& e_vector, std::vector<double>& s_vector)
{
  s_vector = std::vector<double>(
      e_vector.data(), e_vector.data() + e_vector.rows() * e_vector.cols());
}

void to_vector(const std::vector<double>& s_vector, Eigen::VectorXd& e_vector)
{
  for(std::size_t i=0; i<s_vector.size(); i++)
  {
    e_vector[i] = s_vector[i];
  }
}

//==============================================================================
double calcMinTime(
    const Eigen::VectorXd& startPosition,
    const Eigen::VectorXd& endPosition,
    const Eigen::VectorXd& startVelocity,
    const Eigen::VectorXd& endVelocity,
    const Eigen::VectorXd& maxVelocity,
    const Eigen::VectorXd& maxAcceleration,
    std::vector<double>& timeSeq,
    std::vector<Eigen::VectorXd>& posSeq,
    std::vector<Eigen::VectorXd>& velSeq)
{
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
    if (ramp.SolveMinTime(amax, vmax))
    {
      timeSeq.clear();
      posSeq.clear();
      velSeq.clear();

      std::set<double> timeSet;
      timeSet.insert(0.0);
      for (const auto& ramp1d : ramp.ramps)
      {
        timeSet.insert(ramp1d.tswitch1);
        timeSet.insert(ramp1d.tswitch2);
      }
      timeSet.insert(ramp.endTime);

      timeSeq = std::vector<double>(timeSet.begin(), timeSet.end());

      ParabolicRamp::Vector currPos, currVel;
      Eigen::VectorXd currPosVec(startPosition.size());
      Eigen::VectorXd currVelVec(startPosition.size());
      for(std::vector<double>::iterator it=timeSeq.begin();
          it!=timeSeq.end(); ++it)
      {
        double t = (*it);
        ramp.Evaluate(t, currPos);
        ramp.Derivative(t, currVel);
        to_vector(currPos, currPosVec);
        to_vector(currVel, currVelVec);

        //std::cout << "AT TIME " << t << " POS " << currPosVec.matrix().transpose();
        //std::cout << " VEL " << currVelVec.matrix().transpose() << std::endl;

        posSeq.push_back(currPosVec);
        velSeq.push_back(currVelVec);
      }

      return ramp.endTime;
    }
    throw std::runtime_error("calcMinTime: SolveMinTime failed");
  }

  throw std::runtime_error("calcMinTime: ramp not valid");
}

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline> createTimedSplineTrajectory(
    const Eigen::VectorXd& startPosition,
    const Eigen::VectorXd& endPosition,
    const Eigen::VectorXd& startVelocity,
    const Eigen::VectorXd& endVelocity,
    const Eigen::VectorXd& maxVelocity,
    const Eigen::VectorXd& maxAcceleration,
    aikido::statespace::ConstStateSpacePtr stateSpace,
    double startTime)
{
  using dart::common::make_unique;
  using CubicSplineProblem
      = aikido::common::SplineProblem<double, int, 4, Eigen::Dynamic, 2>;

  std::size_t dimension = stateSpace->getDimension();

  std::cout << "START TIME " << startTime << std::endl;
  // create an empty spline
  auto outputTrajectory
      = make_unique<aikido::trajectory::Spline>(stateSpace, startTime);

  // calculate min time
  std::vector<double> timeSeq;
  std::vector<Eigen::VectorXd> posSeq;
  std::vector<Eigen::VectorXd> velSeq;
  double trajTime = calcMinTime(
      startPosition,
      endPosition,
      startVelocity,
      endVelocity,
      maxVelocity,
      maxAcceleration,
      timeSeq,
      posSeq,
      velSeq);


  std::cout << "TIME SEQ: ";
  for(std::size_t i=0; i<timeSeq.size()-1; i++)
  {
    double segTime = timeSeq[i+1] - timeSeq[i];
    // add waypoint
    if (segTime > 0)
    {
      /*
      std::cout << "AT TIME " << timeSeq[i] << " DURATION " << segTime;
      std::cout << " CUR_POS " << posSeq[i].matrix().transpose();
      std::cout << " CUR_VEL " << velSeq[i].matrix().transpose();
      std::cout << " NEX_POS " << posSeq[i+1].matrix().transpose();
      std::cout << " NEX_VEL " << velSeq[i+1].matrix().transpose() << std::endl;*/
      CubicSplineProblem problem(Eigen::Vector2d(0, segTime), 4, dimension);
      problem.addConstantConstraint(0, 0, Eigen::VectorXd::Zero(dimension));
      problem.addConstantConstraint(0, 1, velSeq[i]);
      problem.addConstantConstraint(1, 0, posSeq[i+1]-posSeq[i]);
      problem.addConstantConstraint(1, 1, velSeq[i+1]);
      const auto spline = problem.fit();

      auto currState = stateSpace->createState();
      stateSpace->expMap(posSeq[i], currState);
      // Add the ramp to the output trajectory.
      assert(spline.getCoefficients().size() == 1);
      const auto& coefficients = spline.getCoefficients().front();
      outputTrajectory->addSegment(coefficients, segTime, currState);
    }
  }

  ROS_INFO_STREAM("ouput duration: " << outputTrajectory->getDuration());
  Eigen::VectorXd startVelocityOutput(stateSpace->getDimension());
  Eigen::VectorXd betweenVelocityOutput(stateSpace->getDimension());
  Eigen::VectorXd endVelocityOutput(stateSpace->getDimension());
  Eigen::VectorXd startPositionOutput(stateSpace->getDimension());
  Eigen::VectorXd betweenPositionOutput(stateSpace->getDimension());
  Eigen::VectorXd endPositionOutput(stateSpace->getDimension());
  auto tmpState = stateSpace->createState();
  outputTrajectory->evaluate(outputTrajectory->getStartTime(), tmpState);
  stateSpace->logMap(tmpState, startPositionOutput);
  outputTrajectory->evaluateDerivative(outputTrajectory->getStartTime(), 1, startVelocityOutput);
  outputTrajectory->evaluate((outputTrajectory->getStartTime() + outputTrajectory->getEndTime()) / 2, tmpState);
  stateSpace->logMap(tmpState, betweenPositionOutput);
  outputTrajectory->evaluateDerivative((outputTrajectory->getStartTime() + outputTrajectory->getEndTime()) / 2, 1, betweenVelocityOutput);
  outputTrajectory->evaluate(outputTrajectory->getEndTime(), tmpState);
  stateSpace->logMap(tmpState, endPositionOutput);
  outputTrajectory->evaluateDerivative(outputTrajectory->getEndTime(), 1, endVelocityOutput);
  ROS_INFO_STREAM("start position: " << startPositionOutput.matrix().transpose() << " velocity: " << startVelocityOutput.matrix().transpose());
  ROS_INFO_STREAM("between position: " << betweenPositionOutput.matrix().transpose() << " velocity: " << betweenVelocityOutput.matrix().transpose());
  ROS_INFO_STREAM("end position: " << endPositionOutput.matrix().transpose() << " velocity: " << endVelocityOutput.matrix().transpose());

  return outputTrajectory;
}

//==============================================================================
std::unique_ptr<aikido::trajectory::Spline> createTimedSplineTrajectory(
    const aikido::trajectory::Interpolated& interpolated,
    const Eigen::VectorXd& startVelocity,
    const Eigen::VectorXd& endVelocity,
    const Eigen::VectorXd& maxVelocity,
    const Eigen::VectorXd& maxAcceleration)
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

void printStateWithTime(
    double t,
    std::size_t dimension,
    Eigen::VectorXd& stateVec,
    Eigen::VectorXd& velocityVec,
    std::ofstream& cout)
{
  cout << t << ",";
  for (std::size_t i = 0; i < dimension; i++)
  {
    cout << stateVec[i] << "," << velocityVec[i];
    if(i<dimension-1)
    {
      cout << ",";
    }
  }
  cout << std::endl;
  return;
}

void dumpSplinePhasePlot(
    const aikido::trajectory::Spline& spline,
    const std::string& filename,
    double timeStep)
{
  std::ofstream phasePlotFile;
  phasePlotFile.open(filename);
  auto stateSpace = spline.getStateSpace();
  std::size_t dim = stateSpace->getDimension();

  auto state = stateSpace->createState();
  Eigen::VectorXd stateVec(dim);
  Eigen::VectorXd velocityVec(dim);
  double t = spline.getStartTime();
  while (t + timeStep < spline.getEndTime())
  {
    spline.evaluate(t, state);
    spline.evaluateDerivative(t, 1, velocityVec);
    stateSpace->logMap(state, stateVec);
    printStateWithTime(t, dim, stateVec, velocityVec, phasePlotFile);
    t += timeStep;
  }
  spline.evaluate(spline.getEndTime(), state);
  spline.evaluateDerivative(spline.getEndTime(), 1, velocityVec);
  stateSpace->logMap(state, stateVec);
  printStateWithTime(t, dim, stateVec, velocityVec, phasePlotFile);

  phasePlotFile.close();
  return;
}


}
