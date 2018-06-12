#ifndef UTIL_H
#define UTIL_H

#include <boost/program_options.hpp>
#include <ros/ros.h>
#include <iostream>
#include <Eigen/Dense>

namespace feeding {



void handleArguments(int argc, char** argv, bool& adaReal, bool& autoContinueDemo);

void waitForUser(const std::string& msg);

template<class T>
T getRosParam(const std::string& paramName, const ros::NodeHandle& nh) {
  T value;
  if (!nh.getParam(paramName, value)) {
    throw std::runtime_error("Failed to load ros parameter " + paramName);
  }
  return value;
}

Eigen::Isometry3d createIsometry(
    double x,
    double y,
    double z,
    double roll = 0,
    double pitch = 0,
    double yaw = 0);

Eigen::Isometry3d createIsometry(std::vector<double> vec);

Eigen::MatrixXd createBwMatrixForTSR(
    double horizontalTolerance,
    double verticalTolerance,
    double yawMin,
    double yawMax);

}

#endif