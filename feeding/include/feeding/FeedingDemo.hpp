#ifndef FEEDING_FEEDINGDEMO_HPP_
#define FEEDING_FEEDINGDEMO_HPP_

#include <aikido/planner/World.hpp>
#include <aikido/rviz/TSRMarker.hpp>
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include <ros/ros.h>
#include <libada/Ada.hpp>

#include "feeding/FTThresholdHelper.hpp"
#include "feeding/perception/Perception.hpp"
#include "feeding/perception/PerceptionPreProcess.hpp"
#include "feeding/perception/PerceptionServoClient.hpp"
#include "feeding/Workspace.hpp"

namespace feeding {


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
  FeedingDemo(
    bool adaReal,
    ros::NodeHandle nodeHandle,
    bool useFTSensingToStopTrajectories,
    std::shared_ptr<FTThresholdHelper> ftThresholdHelper = nullptr,
    bool autoContinueDemo = false);

  /// Destructor for the Feeding Demo.
  /// Also shuts down the trajectory controllers.
  ~FeedingDemo();

  void setPerception(std::shared_ptr<Perception> perception);

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

  /// param[in] foodName if empty, takes user input.
  void skewer(
      std::string foodName,
      ros::NodeHandle& nodeHandle,
      int max_trial_per_item = 3);

  void feedFoodToPerson(
      ros::NodeHandle& nodeHandle,
      bool tilted = true);

  boost::optional<Eigen::Isometry3d> detectFood(
      const std::string& foodName,
      bool waitTillDetected = true);

  void setFTThreshold(FTThreshold threshold);

  void waitForUser(const std::string& prompt);

  Eigen::Isometry3d detectAndMoveAboveFood(
    const std::string& foodName,
    int pickupAngleMode,
    float rotAngle,
    float angle,
    bool useAngledTranslation);

  void pushAndSkewer(
    const std::string& foodName,
    int pickupAngleMode,
    float rotAngle,
    float tiltAngle);

  void rotateAndSkewer(
    const std::string& foodName,
    float rotateForqueAngle);

private:
  bool mIsFTSensingEnabled;
  bool mAdaReal;
  bool mAutoContinueDemo;
  ros::NodeHandle mNodeHandle;
  std::shared_ptr<Perception> mPerception;
  std::shared_ptr<FTThresholdHelper> mFTThresholdHelper;

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

  std::vector<std::string> mFoodNames;
  std::vector<double> mSkeweringForces;
  std::unordered_map<std::string, double> mFoodSkeweringForces;


};
}

#endif
