#ifndef FEEDING_FEEDINGDEMO_HPP_
#define FEEDING_FEEDINGDEMO_HPP_

#include <aikido/planner/World.hpp>
#include <aikido/rviz/TSRMarker.hpp>
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include <aikido/distance/ConfigurationRanker.hpp>

#include <ros/ros.h>
#include <libada/Ada.hpp>

#include "feeding/FTThresholdHelper.hpp"
#include "feeding/Workspace.hpp"
#include "feeding/perception/Perception.hpp"
#include "feeding/perception/PerceptionPreProcess.hpp"
#include "feeding/perception/PerceptionServoClient.hpp"


namespace feeding {

enum TargetItem
{
  FOOD,
  PLATE,
  FORQUE,
  PERSON
};

enum TiltStyle
{
  VERTICAL,
  ANGLED,
  NONE
};

static const std::map<TargetItem, const std::string> TargetToString{
    {FOOD, "food"}, {PLATE, "plate"}, {FORQUE, "forque"}, {PERSON, "person"}};

/// The FeedingDemo class is responsible for
/// - The robot (loading + control)
/// - The workspace
///
/// It contains functions that are very specialized for the feeding demo,
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
  /// \param[in] useVisualServo If true, perception servo is used.
  /// \param[in] allowFreeRotation, If true, items specified as rotationFree
  /// get rotational freedom.
  /// \param[in] nodeHandle Handle of the ros node.
  FeedingDemo(
      bool adaReal,
      ros::NodeHandle nodeHandle,
      bool useFTSensingToStopTrajectories,
      bool useVisualServo,
      bool allowFreeRotation,
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
  std::shared_ptr<ada::Ada> getAda();

  /// Gets the transform of the default food object (defined in Workspace)
  /// Valid only for simulation mode
  Eigen::Isometry3d getDefaultFoodTransform();

  aikido::rviz::WorldInteractiveMarkerViewerPtr getViewer();

  void setFTThreshold(FTThreshold threshold);

  void waitForUser(const std::string& prompt);

  void visualizeTrajectory(aikido::trajectory::TrajectoryPtr trajectory);

  aikido::constraint::dart::CollisionFreePtr getCollisionConstraint();

  /// Moves the robot to the start configuration as defined in the ros
  /// parameter.
  void moveToStartConfiguration();

  /// Move out of target item.
  /// \param[in] item Item currently skewered by the robot
  /// \param[in] ignoreCollision If true, collision constraint is ignored in planning.
  void moveOutOf(TargetItem item, bool ignoreCollision = false);

  /// This function does not throw an exception if the trajectory is aborted,
  /// because it is an expected behavior when FT sensor is activated.
  /// \param[in] item
  /// \param[in] tiltStyle
  /// \param[in] endEffectorDirection Workspace direction for end effector to move along.
  bool moveInto(TargetItem item,
    TiltStyle tiltStyle = TiltStyle::VERTICAL,
    const Eigen::Vector3d& endEffectorDirection = Eigen::Vector3d(0, 0, -1));

  bool moveAbove(
      const Eigen::Isometry3d& targetTransform,
      const Eigen::Isometry3d& endEffectorTransform,
      double horizontalTolerance,
      double verticalTolerance,
      double rotationTolerance,
      double tiltTolerance);

  void moveAboveForque();

  /// Moves the forque above the plate.
  bool moveAbovePlate();

  /// Moves the forque above the food item using the values in the ros
  /// parameters.
  /// \param[in] foodTransform the transform of the food which the robot should
  /// move over.
  bool moveAboveFood(
      std::string foodName,
      const Eigen::Isometry3d& foodTransform,
      float rotAngle,
      TiltStyle tiltStyle);

  /// Moves the forque to a position ready to approach the person.
  bool moveInFrontOfPerson();

  /// Moves the forque towards the person.
  /// This function does not throw an exception if the trajectory is aborted,
  /// because we expect that.
  bool moveTowardsPerson();

  void moveDirectlyToPerson(bool tilted);

  void pickUpFork();
  void putDownFork();

  /// param[in] foodName if empty, takes user input.
  void skewer(
      std::string foodName,
      ros::NodeHandle& nodeHandle,
      int max_trial_per_item = 3,
      bool ignoreCollisionWhenMovingOut = true);

  void feedFoodToPerson(ros::NodeHandle& nodeHandle, bool tilted = true);

  boost::optional<Eigen::Isometry3d> detectFood(
      const std::string& foodName, bool waitTillDetected = true);

  Eigen::Isometry3d detectAndMoveAboveFood(
      const std::string& foodName,
      float rotAngle = 0.0,
      TiltStyle tiltStyle = TiltStyle::VERTICAL);

  void scoop();

  /// Resets the environmnet.
  void reset();

  bool moveWithEndEffectorTwist(
    const Eigen::Vector6d& twists,
    double durations = 1.0,
    bool respectCollision = true);

private:
  /// Attach food to forque
  void grabFoodWithForque();

  /// Detach food from forque and remove it from the aikido world.
  void ungrabAndDeleteFood();

  /// Returns configuration ranker to be used for planners;
  /// \param[in] configuration Nominal configuration for ranker.
  aikido::distance::ConfigurationRankerPtr getRanker(
    const Eigen::VectorXd& configuration = Eigen::VectorXd(0));

  bool mIsFTSensingEnabled;
  bool mAdaReal;
  bool mAutoContinueDemo;
  bool mVisualServo;
  bool mAllowRotationFree;
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
  std::vector<std::string> mRotationFreeFoodNames;
  std::vector<double> mSkeweringForces;
  std::unordered_map<std::string, double> mFoodSkeweringForces;
  std::unordered_map<std::string, int> mPickUpAngleModes;

  std::unordered_map<std::string, double> mPlateTSRParameters;
  std::unordered_map<std::string, double> mFoodTSRParameters;

  double mPlanningTimeout;
  int mMaxNumTrials;
  double mEndEffectorOffsetPositionTolerance;
  double mEndEffectorOffsetAngularTolerance;

};
}

#endif
