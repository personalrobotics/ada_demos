#ifndef FEEDING_UTIL_HPP_
#define FEEDING_UTIL_HPP_

#include <fstream>
#include <iostream>
#include <Eigen/Dense>
#include <aikido/statespace/StateSpace.hpp>
#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/trajectory/Spline.hpp>
#include <boost/program_options.hpp>
#include <ros/ros.h>

namespace feeding {

  static const std::vector<std::string> FOOD_NAMES
      = {"strawberry", "melon", "cantaloupe", "celery", "carrot"};

  static const std::vector<std::string> ACTIONS
      = {"calibrate", "pickupfork", "putdownfork"};

  static const std::vector<std::string> DEMO_TYPES
      = {"nips", "collect_push_skewer", "collect_skewer"};

  /// Deals with the arguments supplied to the executable.
  /// \param[in] description Description for this program
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
      bool& useFTSensing,
      std::string& demoType,
      const std::string& description = "Ada Feeding Demo");

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

  /// Gets user selection of food and actions
  /// param[in] food_only If true, only food choices are valid
  /// param[in]] nodeHandle Ros Node to set food name for detection.
  std::string getUserInput(
    bool food_only, ros::NodeHandle& nodeHandle);

} // feeding

#endif
