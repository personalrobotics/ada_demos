#ifndef FEEDING_UTIL_HPP_
#define FEEDING_UTIL_HPP_

#include <iostream>
#include <Eigen/Dense>
#include <boost/program_options.hpp>
#include <ros/ros.h>

namespace feeding {

/// Deals with the arguments supplied to the executable.
/// \param[in] argc and argv are the typical parameters of main(..)
/// \param[out] adaReal is true when the robot is used and not the simulation
/// \param[out] autoContinueDemo is true when the demo continues to the next
/// step without asking for confirmation
void handleArguments(
    int argc, char** argv, bool& adaReal, bool& autoContinueDemo);

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
}

#endif
