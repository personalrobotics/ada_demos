#ifndef UTIL_H
#define UTIL_H

#include <boost/program_options.hpp>
#include <ros/ros.h>
#include <iostream>
#include <Eigen/Dense>
#include <pr_tsr/plate.hpp>
#include <tf/transform_listener.h>
#include <libada/Ada.hpp>

namespace cameraCalibration {

/// deals with the arguments supplied to the executable.
void handleArguments(int argc, char** argv, bool& adaReal, bool& autoContinueDemo);

/// Displays a message and waits for the user to press the enter key
/// If the user enters "n", this method will exit the program.
void waitForUser(const std::string& msg);

/// Loads and returns a ros parameter.
/// Throws a runtime_error if the parameter is not set.
template<class T>
T getRosParam(const std::string& paramName, const ros::NodeHandle& nh) {
  T value;
  if (!nh.getParam(paramName, value)) {
    throw std::runtime_error("Failed to load ros parameter " + paramName);
  }
  return value;
}

/// Convenience function to create an Eigen Isometry3D based on position and rotation.
Eigen::Isometry3d createIsometry(
    double x,
    double y,
    double z,
    double roll = 0,
    double pitch = 0,
    double yaw = 0);

/// Convenience function to create an Eigen Isometry3D based on position and rotation.
Eigen::Isometry3d createIsometry(std::vector<double> vec);

/// Convenience function to create the Bw Matrix that is needed for TSRs more easily.
Eigen::MatrixXd createBwMatrixForTSR(
    double horizontalTolerance,
    double verticalTolerance,
    double yawMin,
    double yawMax);

aikido::constraint::dart::TSR getCalibrationTSR(
    const Eigen::Isometry3d& transform);

bool moveArmToTSR(
    aikido::constraint::dart::TSR& goalTSR,
    ada::Ada& ada,
    aikido::constraint::dart::CollisionFreePtr collisionFreeConstraint,
    aikido::statespace::dart::MetaSkeletonStateSpacePtr armSpace);

bool moveArmOnTrajectory(
    aikido::trajectory::TrajectoryPtr trajectory,
    ada::Ada& ada,
    aikido::constraint::dart::CollisionFreePtr collisionFreeConstraint,
    aikido::statespace::dart::MetaSkeletonStateSpacePtr armSpace);

void printPose(const Eigen::Isometry3d& pose);

/// Returns the transform from map to j2n6s200_joule.
/// \param[in] tfListner Transform listener.
Eigen::Isometry3d getWorldToJoule(tf::TransformListener& tfListener);

/// Returns the transform from camera_link to camera_color_optical_frame.
/// \param[in] tfListner Transform listener.
Eigen::Isometry3d getCameraToLens(tf::TransformListener& tfListener);

/// Returns the transform from camera_link to j2n6s200_joule.
/// This is only an estimate published by a static transform node.
/// \param[in] tfListner Transform listener.
Eigen::Isometry3d getCameraToJoule(tf::TransformListener& tfListener);

/// Returns the transform between two links.
/// \param[in] from Link from which the transform is computed.
/// \param[in] to Link to which the transform is computed.
Eigen::Isometry3d getRelativeTransform(tf::TransformListener& tfListener,
  const std::string& from, const std::string& to);

}
#endif
