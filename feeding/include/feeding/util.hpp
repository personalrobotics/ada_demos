#ifndef UTIL_H
#define UTIL_H

#include <iostream>
#include <Eigen/Dense>
#include <boost/program_options.hpp>
#include <ros/ros.h>

namespace feeding {

/// deals with the arguments supplied to the executable.
/// argc and argv are the typical parameters of main(..)
/// adaReal is true when the robot is used and not the simulation
/// autoContinueDemo is true when the demo continues to the next step without
/// asking for confirmation
void handleArguments(
    int argc, char** argv, bool& adaReal, bool& autoContinueDemo);

/// Displays a message and waits for the user to press the enter key
/// If the user enters "n", this method will exit the program.
void waitForUser(const std::string& msg);

/// Loads and returns a ros parameter.
/// Throws a runtime_error if the parameter is not set.
template <class T>
T getRosParam(const std::string& paramName, ros::NodeHandle nh)
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
Eigen::Isometry3d createIsometry(
    double x,
    double y,
    double z,
    double roll = 0,
    double pitch = 0,
    double yaw = 0);

/// Convenience function to create an Eigen Isometry3D based on position and
/// rotation.
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
