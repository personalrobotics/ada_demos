#ifndef FEEDING_UTIL_HPP_
#define FEEDING_UTIL_HPP_

#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <aikido/statespace/StateSpace.hpp>
#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/trajectory/Spline.hpp>
#include <boost/program_options.hpp>
#include <ros/ros.h>

namespace feeding {

/// Deals with the arguments supplied to the executable.
/// \param[in] argc and argv are the typical parameters of main(..)
/// \param[out] adaReal is true when the robot is used and not the simulation
/// \param[out] autoContinueDemo is true when the demo continues to the next
/// step without asking for confirmation
/// \param[out] useFTSensing turns the FTSensor and the MoveUntilTouchController
/// on and off
void handleArguments(
    int argc,
    char** argv,
    bool& adaReal,
    bool& autoContinueDemo,
    bool& useFTSensing);

/// Displays a message and waits for the user to press the enter key
/// \param[in] The message to display.
/// \return False if the user entered 'n'.
bool waitForUser(const std::string& msg);

/// Loads and returns a ros parameter.
/// Throws a runtime_error if the parameter is not set.
/// \param[in] paramName The name of the parameter.
/// \param[in] nodeHandle Handle of the ros node.
/// \return The value of the ros parameter.
template <class T>
T getRosParam(const std::string& paramName, const ros::NodeHandle& nh)
{
  T value;
  if (!nh.getParam(paramName, value))
  {
    throw std::runtime_error("Failed to load ros parameter " + paramName);
  }
  return value;
}

/// Convenience function to create an Eigen Isometry3D based on position and
/// rotation.
/// \param[in] x,y,z Position
/// \param[in] roll,pitch,yaw Rotation
/// \return The transform.
Eigen::Isometry3d createIsometry(
    double x,
    double y,
    double z,
    double roll = 0,
    double pitch = 0,
    double yaw = 0);

/// Convenience function to create an Eigen Isometry3D based on position and
/// rotation.
/// \param[in] vec Positi and rotation in a vector like this:
/// [x,y,z,roll,pitch,yaw]
/// \return The transform.
Eigen::Isometry3d createIsometry(std::vector<double> vec);

/// Convenience function to create the Bw Matrix that is needed for TSRs more
/// easily.
Eigen::MatrixXd createBwMatrixForTSR(
    double horizontalTolerance,
    double verticalTolerance,
    double yawMin,
    double yawMax);

/// Create a timed spline with only two waypoints with start/end velocities
std::unique_ptr<aikido::trajectory::Spline> createTimedSplineTrajectory(
    const Eigen::VectorXd& startPosition,
    const Eigen::VectorXd& endPosition,
    const Eigen::VectorXd& startVelocity,
    const Eigen::VectorXd& endVelocity,
    const Eigen::VectorXd& maxVelocity,
    const Eigen::VectorXd& maxAcceleration,
    aikido::statespace::ConstStateSpacePtr stateSpace,
    double startTime = 0.);

std::unique_ptr<aikido::trajectory::Spline> createTimedSplineTrajectory(
    const aikido::trajectory::Interpolated& interpolated,
    const Eigen::VectorXd& startVelocity,
    const Eigen::VectorXd& endVelocity,
    const Eigen::VectorXd& maxVelocity,
    const Eigen::VectorXd& maxAcceleration);

double findClosetStateOnTrajectory(const aikido::trajectory::Trajectory* traj,
                                   const Eigen::VectorXd& config,
                                   double timeStep=0.01);

std::unique_ptr<aikido::trajectory::Spline> concatenate(const aikido::trajectory::Spline& traj1,
                                                        const aikido::trajectory::Spline& traj2);

void printStateWithTime(
    double t,
    std::size_t dimension,
    Eigen::VectorXd& stateVec,
    Eigen::VectorXd& velocityVec,
    std::ofstream& cout);

void dumpSplinePhasePlot(
    const aikido::trajectory::Spline& spline,
    const std::string& filename,
    double timeStep);

} // feeding

#endif
