#ifndef FEEDING_FEEDINGDEMO_HPP_
#define FEEDING_FEEDINGDEMO_HPP_

#include <aikido/planner/World.hpp>
#include <aikido/rviz/TSRMarker.hpp>
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include <ros/ros.h>
#include <libada/Ada.hpp>

#include "feeding/FTThresholdHelper.hpp"
#include "feeding/Perception.hpp"
#include "feeding/PerceptionPreProcess.hpp"
#include "feeding/PerceptionServoClient.hpp"
#include "feeding/Workspace.hpp"

namespace feeding {

static const std::vector<std::string> FOOD_NAMES
    = {"strawberry", "melon", "cantaloupe", "celery", "carrot"};

static const std::vector<std::string> ACTIONS
    = {"calibrate", "pickupfork", "putdownfork"};

/// The FeedingDemo class is responsible for
/// - The robot (loading + control)
/// - The workspace
///
/// It contains not only generalized functions, for example moveArmToTSR(...),
/// but also some functions that are very specialized for the feeding demo,
/// like moveInFrontOfPerson(). It uses the robot, workspace info and ros
/// parameters
/// to accomplish these tasks.
class FeedingDemo
{

public:
  /// Constructor for the Feeding Demo.
  /// Takes care of setting up the robot and the workspace
  /// \param[in] adaReal True if the real robot is used, false it's running in
  /// simulation.
  /// \param[in] useFTSensing turns the FTSensor and the
  /// MoveUntilTouchController on and off
  /// \param[in] nodeHandle Handle of the ros node.
  FeedingDemo(bool adaReal, bool useFTSensing, ros::NodeHandle nodeHandle);

  /// Destructor for the Feeding Demo.
  /// Also shuts down the trajectory controllers.
  ~FeedingDemo();

  /// Gets the aikido world
  aikido::planner::WorldPtr getWorld();

  /// Gets the workspace
  Workspace& getWorkspace();

  /// Gets Ada
  ada::Ada& getAda();

  /// Gets the transform of the default food object (defined in Workspace)
  Eigen::Isometry3d getDefaultFoodTransform();

  /// Checks robot and workspace collisions.
  /// \param[out] result Contains reason for collision.
  /// \return True if no collision was detected.
  bool isCollisionFree(std::string& result);

  /// Prints the configuration of the robot joints.
  void printRobotConfiguration();

  /// Attach food to forque
  void grabFoodWithForque();

  /// Detach food from forque and remove it from the aikido world.
  void ungrabAndDeleteFood();

  /// Moves the robot to the start configuration as defined in the ros
  /// parameter.
  void moveToStartConfiguration();

  void moveAboveForque();
  void moveIntoForque();
  void moveOutOfForque();

  /// Moves the forque above the plate.
  bool moveAbovePlate();
  void moveAbovePlateAnywhere();

  /// Moves the forque above the food item using the values in the ros
  /// parameters.
  /// \param[in] foodTransform the transform of the food which the robot should
  /// move over.

  bool moveAboveFood(
      const Eigen::Isometry3d& foodTransform,
      int pickupAngleMode,
      float rotAngle = 0.0,
      float angle = 0.0,
      bool useAngledTranslation = true);

  void rotateForque(
      const Eigen::Isometry3d& foodTransform,
      float angle,
      int pickupAngleMode,
      bool useAngledTranslation = true);

  void moveNextToFood(
      const Eigen::Isometry3d& foodTransform,
      float angle,
      bool useAngledTranslation = true);

  void moveNextToFood(
      Perception* perception, float angle, Eigen::Isometry3d forqueTransform);

  void pushFood(float angle, bool useAngledTranslation = true);

  void pushFood(
      float angle,
      Eigen::Isometry3d forqueTransform,
      bool useAngledTranslation = true);

  void pushFood(float angle, double pushDist, bool useAngledTranslation = true);

  void pushFood(
      float angle,
      double pushDist,
      Eigen::Isometry3d forqueTransform,
      bool useAngledTranslation = true);

  // void scoopFood();

  void moveOutOfPlate();

  /// Moves the forque downwards into the food.
  /// This function does not throw an exception if the trajectory is aborted,
  /// because we expect that.
  bool moveIntoFood();

  bool moveIntoFood(Perception* perception);

  /// Moves the forque upwards above the food.
  void moveOutOfFood();
  void moveOutOfFood(float dist);

  /// Moves the forque to a position ready to approach the person.
  bool moveInFrontOfPerson();

  bool tiltUpInFrontOfPerson();

  void tiltDownInFrontOfPerson();

  /// Moves the forque towards the person.
  /// This function does not throw an exception if the trajectory is aborted,
  /// because we expect that.
  bool moveTowardsPerson();

  bool moveTowardsPerson(Perception* perception);

  void moveDirectlyToPerson(bool tilted);

  /// Moves the forque away from the person.
  void moveAwayFromPerson();

  void visualizeTrajectory(aikido::trajectory::TrajectoryPtr trajectory);

  aikido::rviz::WorldInteractiveMarkerViewerPtr getViewer();

  void pickUpFork();
  void putDownFork();

  /// Gets user selection of food and actions
  /// param[in] food_only If true, only food choices are valid
  /// param[in]] nodeHandle Ros Node to set food name for detection.
  std::string getUserInput(bool food_only, ros::NodeHandle& nodeHandle);

  ///
  /// param[in] foodName if empty, takes user input.
  void skewer(
      std::string foodName,
      FTThresholdHelper& ftThresholdHelper,
      Perception& perception,
      ros::NodeHandle nodeHandle,
      bool autoContinueDemo,
      bool adaReal,
      int max_trial_per_item = 3);

  void feedFoodToPerson(
      Perception& perception,
      ros::NodeHandle nodeHandle,
      bool autoContinueDemo,
      bool tilted = true);

private:
  bool mIsFTSensingEnabled = false;
  bool mAdaReal;
  ros::NodeHandle mNodeHandle;
  aikido::planner::WorldPtr mWorld;

  std::shared_ptr<ada::Ada> mAda;
  aikido::statespace::dart::MetaSkeletonStateSpacePtr mArmSpace;
  std::unique_ptr<Workspace> mWorkspace;
  aikido::constraint::dart::CollisionFreePtr mCollisionFreeConstraint;

  std::unique_ptr<PerceptionServoClient> mServoClient;

  std::vector<aikido::rviz::TSRMarkerPtr> tsrMarkers;

  aikido::rviz::WorldInteractiveMarkerViewerPtr mViewer;
  aikido::rviz::FrameMarkerPtr frameMarker;
  aikido::rviz::TrajectoryMarkerPtr trajectoryMarkerPtr;
};
}

#endif
