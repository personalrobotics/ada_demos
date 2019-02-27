#ifndef FEEDING_UTIL_HPP_
#define FEEDING_UTIL_HPP_

#include <fstream>
#include <iostream>
#include <utility>
#include <Eigen/Dense>
#include <aikido/statespace/StateSpace.hpp>
#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/trajectory/Spline.hpp>
#include <boost/optional.hpp>
#include <boost/program_options.hpp>
#include <dart/dart.hpp>
#include <libada/Ada.hpp>

#include <ros/ros.h>
#include <tf/transform_listener.h>

namespace feeding {

static const std::vector<std::string> FOOD_NAMES
    = {"strawberry", "melon", "cantaloupe", "celery", "carrot"};

static const std::vector<std::string> ACTIONS
    = {"calibrate", "pickupfork", "putdownfork"};

/// Deals with the arguments supplied to the executable.
/// \param[in] description Description for this program
/// \param[in] argc and argv are the typical parameters of main(..)
/// \param[out] adaReal is true when the robot is used and not the simulation
/// \param[out] autoContinueDemo is true when the demo continues to the next
/// step without asking for confirmation
/// \param[out] useFTSensing turns the FTSensor and the MoveUntilTouchController
/// \param[out] foodName Name fo food for data collection
/// \param[out] directionIndex Current direction (angle) for data collection
/// \param[out] trialIndex Current trial index for data collection
/// \param[out] dataCollectorPath Directory to store data collection
/// on and off
void handleArguments(
    int argc,
    char** argv,
    bool& adaReal,
    bool& autoContinueDemo,
    bool& useFTSensing,
    std::string& demoType,
    std::string& foodName,
    std::size_t& directionIndex,
    std::size_t& trialIndex,
    std::string& dataCollectorPath,
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

/// Gets user selection of food and actions using the Amazon Alexa
/// param[in]] nodeHandle Ros Node to set food name for detection.
std::string getUserInputFromAlexa(ros::NodeHandle& nodeHandle);

/// Gets user selection of food and actions
/// param[in] food_only If true, only food choices are valid
/// param[in]] nodeHandle Ros Node to set food name for detection.
std::string getUserInput(bool food_only, ros::NodeHandle& nodeHandle);

int getUserInputWithOptions(
    const std::vector<std::string>& optionPrompts, const std::string& prompt);

/// Sets position limits of a metaskeleton.
/// \param[in] metaSkeleton Metaskeleton to modify.
/// \param[in] lowerLimits Lowerlimits of the joints.
/// \param[in] upperLimits Upperlimits of the joints.
/// \param[in] indices Indices of the joints to modify.
/// returns Pair of current lower and upper limits of the chosen joints.
std::pair<Eigen::VectorXd, Eigen::VectorXd> setPositionLimits(
    const ::dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const Eigen::VectorXd& lowerLimits = Eigen::VectorXd::Ones(4) * -6.28,
    const Eigen::VectorXd& upperLimits = Eigen::VectorXd::Ones(4) * 6.28,
    const std::vector<std::size_t>& indices
    = std::vector<std::size_t>{0, 3, 4, 5});

/// Get relative transform between two transforms.
/// \param[in] tfListner TFListener
/// \param[in] from Transform from which the relative transform is computed.
/// \param[in] to Transform to which the relative transform is computed.
/// returns Relative transform from from to to.
Eigen::Isometry3d getRelativeTransform(
    tf::TransformListener& tfListener,
    const std::string& from,
    const std::string& to);

Eigen::Isometry3d removeRotation(const Eigen::Isometry3d& transform);

void printRobotConfiguration(const std::shared_ptr<ada::Ada>& ada);

bool isCollisionFree(
    const std::shared_ptr<ada::Ada>& ada,
    const aikido::constraint::dart::CollisionFreePtr& collisionFree);

double getDistance(
    const Eigen::Isometry3d& item1, const Eigen::Isometry3d& item2);

Eigen::Isometry3d getForqueTransform(tf::TransformListener& tfListener);

aikido::distance::ConfigurationRankerPtr getConfigurationRanker(
    const std::shared_ptr<::ada::Ada>& ada);

} // feeding

#endif
