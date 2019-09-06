#include "feeding/FeedingDemo.hpp"
#include <aikido/rviz/TrajectoryMarker.hpp>
#include <boost/optional.hpp>

#include <libada/util.hpp>

#include <fstream>
#include <iostream>
#include <string>

#include "feeding/FoodItem.hpp"
#include "feeding/util.hpp"

using ada::util::createIsometry;
using ada::util::getRosParam;
using aikido::constraint::dart::CollisionFreePtr;

const bool TERMINATE_AT_USER_PROMPT = true;

static const std::size_t MAX_NUM_TRIALS = 3;
static const double inf = std::numeric_limits<double>::infinity();

namespace feeding {

//==============================================================================
FeedingDemo::FeedingDemo(
    bool adaReal,
    ros::NodeHandle nodeHandle,
    bool useFTSensingToStopTrajectories,
    bool useVisualServo,
    bool allowFreeRotation,
    std::shared_ptr<FTThresholdHelper> ftThresholdHelper,
    bool autoContinueDemo)
  : mAdaReal(adaReal)
  , mNodeHandle(nodeHandle)
  , mFTThresholdHelper(ftThresholdHelper)
  , mVisualServo(useVisualServo)
  , mAllowRotationFree(allowFreeRotation)
  , mAutoContinueDemo(autoContinueDemo)
  , mIsFTSensingEnabled(useFTSensingToStopTrajectories)
{
  mWorld = std::make_shared<aikido::planner::World>("feeding");

  std::string armTrajectoryExecutor = mIsFTSensingEnabled
                                          ? "move_until_touch_topic_controller"
                                          : "rewd_trajectory_controller";

  mAda = std::make_shared<ada::Ada>(
      mWorld,
      !mAdaReal,
      getRosParam<std::string>("/ada/urdfUri", mNodeHandle),
      getRosParam<std::string>("/ada/srdfUri", mNodeHandle),
      getRosParam<std::string>("/ada/endEffectorName", mNodeHandle),
      armTrajectoryExecutor);
  mArmSpace = mAda->getArm()->getStateSpace();

  Eigen::Isometry3d robotPose = createIsometry(
      getRosParam<std::vector<double>>("/ada/baseFramePose", mNodeHandle));

  mWorkspace
      = std::make_shared<Workspace>(mWorld, robotPose, mAdaReal, mNodeHandle);

  // Setting up collisions
  dart::collision::CollisionDetectorPtr collisionDetector
      = dart::collision::FCLCollisionDetector::create();
  std::shared_ptr<dart::collision::CollisionGroup> armCollisionGroup
      = collisionDetector->createCollisionGroup(
          mAda->getMetaSkeleton().get(),
          mAda->getHand()->getEndEffectorBodyNode());
  std::shared_ptr<dart::collision::CollisionGroup> envCollisionGroup
      = collisionDetector->createCollisionGroup(
          mWorkspace->getTable().get(),
          mWorkspace->getWorkspaceEnvironment().get(),
          mWorkspace->getWheelchair().get());

  mCollisionFreeConstraint
      = std::make_shared<aikido::constraint::dart::CollisionFree>(
          mArmSpace, mAda->getArm()->getMetaSkeleton(), collisionDetector);
  mCollisionFreeConstraint->addPairwiseCheck(
      armCollisionGroup, envCollisionGroup);

  dart::collision::CollisionDetectorPtr relaxedCollisionDetector
      = dart::collision::FCLCollisionDetector::create();
  std::shared_ptr<dart::collision::CollisionGroup> relaxedArmCollisionGroup
      = relaxedCollisionDetector->createCollisionGroup(
          mAda->getMetaSkeleton().get(),
          mAda->getHand()->getEndEffectorBodyNode());
  std::shared_ptr<dart::collision::CollisionGroup> relaxedEnvCollisionGroup
      = relaxedCollisionDetector->createCollisionGroup(
          mWorkspace->getTable().get(),
          mWorkspace->getWorkspaceEnvironmentWithWallFurtherBack().get(),
          mWorkspace->getWheelchair().get());

  mCollisionFreeConstraintWithWallFurtherBack
      = std::make_shared<aikido::constraint::dart::CollisionFree>(
          mArmSpace,
          mAda->getArm()->getMetaSkeleton(),
          relaxedCollisionDetector);
  mCollisionFreeConstraintWithWallFurtherBack->addPairwiseCheck(
      relaxedArmCollisionGroup, relaxedEnvCollisionGroup);

  // visualization
  mViewer = std::make_shared<aikido::rviz::WorldInteractiveMarkerViewer>(
      mWorld,
      getRosParam<std::string>("/visualization/topicName", mNodeHandle),
      getRosParam<std::string>("/visualization/baseFrameName", mNodeHandle));
  mViewer->setAutoUpdate(true);

  if (mAdaReal)
  {
    mAda->startTrajectoryExecutor();
  }

  mFoodNames
      = getRosParam<std::vector<std::string>>("/foodItems/names", mNodeHandle);
  mSkeweringForces
      = getRosParam<std::vector<double>>("/foodItems/forces", mNodeHandle);
  mRotationFreeFoodNames = getRosParam<std::vector<std::string>>(
      "/rotationFree/names", mNodeHandle);
  auto pickUpAngleModes = getRosParam<std::vector<int>>(
      "/foodItems/pickUpAngleModes", mNodeHandle);

  for (int i = 0; i < mFoodNames.size(); i++)
  {
    mFoodSkeweringForces[mFoodNames[i]] = mSkeweringForces[i];
    mPickUpAngleModes[mFoodNames[i]] = pickUpAngleModes[i];
  }

  mPlateTSRParameters["height"]
      = getRosParam<double>("/feedingDemo/heightAbovePlate", mNodeHandle);
  mPlateTSRParameters["horizontalTolerance"] = getRosParam<double>(
      "/planning/tsr/horizontalToleranceAbovePlate", mNodeHandle);
  mPlateTSRParameters["verticalTolerance"] = getRosParam<double>(
      "/planning/tsr/verticalToleranceAbovePlate", mNodeHandle);
  mPlateTSRParameters["rotationTolerance"] = getRosParam<double>(
      "/planning/tsr/rotationToleranceAbovePlate", mNodeHandle);

  mFoodTSRParameters["height"]
      = getRosParam<double>("/feedingDemo/heightAboveFood", mNodeHandle);
  mFoodTSRParameters["horizontalTolerance"] = getRosParam<double>(
      "/planning/tsr/horizontalToleranceNearFood", mNodeHandle);
  mFoodTSRParameters["verticalTolerance"] = getRosParam<double>(
      "/planning/tsr/verticalToleranceNearFood", mNodeHandle);
  mFoodTSRParameters["rotationTolerance"] = getRosParam<double>(
      "/planning/tsr/rotationToleranceNearFood", mNodeHandle);
  mFoodTSRParameters["tiltTolerance"]
      = getRosParam<double>("/planning/tsr/tiltToleranceNearFood", mNodeHandle);
  mMoveOufOfFoodLength
      = getRosParam<double>("/feedingDemo/moveOutofFood", mNodeHandle);

  mPlanningTimeout
      = getRosParam<double>("/planning/timeoutSeconds", mNodeHandle);
  mMaxNumTrials = getRosParam<int>("/planning/maxNumberOfTrials", mNodeHandle);

  mEndEffectorOffsetPositionTolerance = getRosParam<double>(
      "/planning/endEffectorOffset/positionTolerance", mNodeHandle),
  mEndEffectorOffsetAngularTolerance = getRosParam<double>(
      "/planning/endEffectorOffset/angularTolerance", mNodeHandle);

  mWaitTimeForFood = std::chrono::milliseconds(
      getRosParam<int>("/feedingDemo/waitMillisecsAtFood", mNodeHandle));
  mWaitTimeForPerson = std::chrono::milliseconds(
      getRosParam<int>("/feedingDemo/waitMillisecsAtPerson", mNodeHandle));

  mPersonTSRParameters["distance"]
      = getRosParam<double>("/feedingDemo/distanceToPerson", mNodeHandle);
  mPersonTSRParameters["horizontalTolerance"] = getRosParam<double>(
      "/planning/tsr/horizontalToleranceNearPerson", mNodeHandle);
  mPersonTSRParameters["verticalTolerance"] = getRosParam<double>(
      "/planning/tsr/verticalToleranceNearPerson", mNodeHandle);

  mForkHolderAngle = getRosParam<double>("/study/forkHolderAngle", mNodeHandle);
  mForkHolderTranslation = getRosParam<std::vector<double>>(
      "/study/forkHolderTranslation", mNodeHandle);

  std::vector<double> tiltOffsetVector
      = getRosParam<std::vector<double>>("/study/tiltOffset", mNodeHandle);
  mTiltOffset = Eigen::Vector3d(
      tiltOffsetVector[0], tiltOffsetVector[1], tiltOffsetVector[2]);

  mVelocityLimits = getRosParam<std::vector<double>>("/study/velocityLimits", mNodeHandle);
}

//==============================================================================
FeedingDemo::~FeedingDemo()
{
  if (mAdaReal)
  {
    // wait for a bit so controller actually stops moving
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    mAda->stopTrajectoryExecutor();
  }
}

//==============================================================================
void FeedingDemo::setPerception(std::shared_ptr<Perception> perception)
{
  mPerception = perception;
}

//==============================================================================
ros::NodeHandle FeedingDemo::getNodeHandle()
{
  return mNodeHandle;
}

//==============================================================================
aikido::planner::WorldPtr FeedingDemo::getWorld()
{
  return mWorld;
}

//==============================================================================
std::shared_ptr<Workspace> FeedingDemo::getWorkspace()
{
  return mWorkspace;
}

//==============================================================================
std::shared_ptr<ada::Ada> FeedingDemo::getAda()
{
  return mAda;
}

//==============================================================================
bool FeedingDemo::isAdaReal()
{
  return mAdaReal;
}

//==============================================================================
CollisionFreePtr FeedingDemo::getCollisionConstraint()
{
  return mCollisionFreeConstraint;
}

//==============================================================================
CollisionFreePtr FeedingDemo::getCollisionConstraintWithWallFurtherBack()
{
  return mCollisionFreeConstraintWithWallFurtherBack;
}

//==============================================================================
Eigen::Isometry3d FeedingDemo::getDefaultFoodTransform()
{
  return mWorkspace->getDefaultFoodItem()
      ->getRootBodyNode()
      ->getWorldTransform();
}

//==============================================================================
aikido::rviz::WorldInteractiveMarkerViewerPtr FeedingDemo::getViewer()
{
  return mViewer;
}

//==============================================================================
void FeedingDemo::reset()
{
  mWorkspace->reset();
}

//==============================================================================
std::shared_ptr<FTThresholdHelper> FeedingDemo::getFTThresholdHelper()
{
  return mFTThresholdHelper;
}

//==============================================================================
Eigen::Isometry3d FeedingDemo::getPlateEndEffectorTransform() const
{
  Eigen::Isometry3d eeTransform
      = mAda->getHand()->getEndEffectorTransform("plate").get();
  eeTransform.linear() = eeTransform.linear()
                         * Eigen::Matrix3d(Eigen::AngleAxisd(
                               M_PI * 0.5, Eigen::Vector3d::UnitZ()));
  eeTransform.translation()
      = Eigen::Vector3d(0, 0, mPlateTSRParameters.at("height"));

  return eeTransform;
}
} // namespace feeding
