#include "feeding/FeedingDemo.hpp"
#include <aikido/rviz/TrajectoryMarker.hpp>
#include <aikido/distance/NominalConfigurationRanker.hpp>
#include <boost/optional.hpp>

#include <pr_tsr/plate.hpp>
#include <libada/util.hpp>

#include <fstream>
#include <iostream>
#include <string>
#include "feeding/util.hpp"

using ada::util::getRosParam;
using ada::util::createBwMatrixForTSR;
using ada::util::createIsometry;
using aikido::distance::NominalConfigurationRanker;
using aikido::constraint::dart::TSR;
using aikido::constraint::dart::CollisionFreePtr;

const bool TERMINATE_AT_USER_PROMPT = true;

static const std::vector<double> weights = {1, 1, 10, 0.01, 0.01, 0.01};
static const std::size_t MAX_NUM_TRIALS = 3;
static const double inf = std::numeric_limits<double>::infinity();
static const std::vector<double> velocityLimits{0.2, 0.2, 0.2, 0.2, 0.2, 0.4};
static const std::vector<std::string> optionPrompts{"(1) success", "(2) fail"};

namespace feeding {

//==============================================================================
FeedingDemo::FeedingDemo(
    bool adaReal,
    ros::NodeHandle nodeHandle,
    bool useFTSensingToStopTrajectories,
    bool useVisualServo,
    bool allowRotationFree,
    std::shared_ptr<FTThresholdHelper> ftThresholdHelper,
    bool autoContinueDemo)
  : mAdaReal(adaReal)
  , mNodeHandle(nodeHandle)
  , mFTThresholdHelper(ftThresholdHelper)
  , mVisualServo(useVisualServo)
  , mAllowRotationFree(allowRotationFree)
  , mAutoContinueDemo(autoContinueDemo)
  , mIsFTSensingEnabled(useFTSensingToStopTrajectories)
{
  mWorld = std::make_shared<aikido::planner::World>("feeding");

  std::string armTrajectoryExecutor = mIsFTSensingEnabled
                                          ? "move_until_touch_topic_controller"
                                          : "trajectory_controller";

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

  mWorkspace = std::unique_ptr<Workspace>(
      new Workspace(mWorld, robotPose, mAdaReal, mNodeHandle));

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
          mWorkspace->getWheelchair().get()
          );
  mCollisionFreeConstraint
      = std::make_shared<aikido::constraint::dart::CollisionFree>(
          mArmSpace, mAda->getArm()->getMetaSkeleton(), collisionDetector);
  mCollisionFreeConstraint->addPairwiseCheck(
      armCollisionGroup, envCollisionGroup);

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
  mRotationFreeFoodNames
      = getRosParam<std::vector<std::string>>("/rotationFree/names", mNodeHandle);
  auto pickUpAngleModes
      = getRosParam<std::vector<int>>("/foodItems/pickUpAngleModes", mNodeHandle);

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
  mFoodTSRParameters["horizontal"]
      = getRosParam<double>("/planning/tsr/horizontalToleranceNearFood", mNodeHandle);
  mFoodTSRParameters["verticalTolerance"]
      = getRosParam<double>("/planning/tsr/verticalToleranceNearFood", mNodeHandle);
  mFoodTSRParameters["rotationTolerance"]
      = getRosParam<double>("/planning/tsr/rotationToleranceNearFood", mNodeHandle);
  mFoodTSRParameters["tiltTolerance"]
      = getRosParam<double>("/planning/tsr/tiltToleranceNearFood", mNodeHandle);

  mPlanningTimeout = getRosParam<double>("/planning/timeoutSeconds", mNodeHandle);
  mMaxNumTrials = getRosParam<int>("/planning/maxNumberOfTrials", mNodeHandle);

  mEndEffectorOffsetPositionTolerance
      = getRosParam<double>("/planning/endEffectorOffset/positionTolerance", mNodeHandle),
  mEndEffectorOffsetAngularTolerance
      = getRosParam<double>("/planning/endEffectorOffset/angularTolerance", mNodeHandle);

  mWaitTimeForFood
      = std::chrono::milliseconds(
        getRosParam<int>("/feedingDemo/waitMillisecsAtFood", mNodeHandle));
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
aikido::planner::WorldPtr FeedingDemo::getWorld()
{
  return mWorld;
}

//==============================================================================
Workspace& FeedingDemo::getWorkspace()
{
  return *mWorkspace;
}

//==============================================================================
std::shared_ptr<ada::Ada> FeedingDemo::getAda()
{
  return mAda;
}

//==============================================================================
CollisionFreePtr FeedingDemo::getCollisionConstraint()
{
  return mCollisionFreeConstraint;
}

//==============================================================================
Eigen::Isometry3d FeedingDemo::getDefaultFoodTransform()
{
  return mWorkspace->getDefaultFoodItem()
      ->getRootBodyNode()
      ->getWorldTransform();
}

//==============================================================================
void FeedingDemo::grabFoodWithForque()
{
  if (!mWorkspace->getDefaultFoodItem())
  {
    mWorkspace->addDefaultFoodItemAtPose(
        mAda->getHand()->getEndEffectorBodyNode()->getTransform());
  }
  mAda->getHand()->grab(mWorkspace->getDefaultFoodItem());
}

//==============================================================================
void FeedingDemo::ungrabAndDeleteFood()
{
  mAda->getHand()->ungrab();
  mWorkspace->deleteFood();
}

//==============================================================================
void FeedingDemo::moveAboveForque()
{
  double forkHolderAngle
      = getRosParam<double>("/study/forkHolderAngle", mNodeHandle);
  std::vector<double> forkHolderTranslation = getRosParam<std::vector<double>>(
      "/study/forkHolderTranslation", mNodeHandle);

  auto aboveForqueTSR = pr_tsr::getDefaultPlateTSR();
  Eigen::Isometry3d forquePose = Eigen::Isometry3d::Identity();
  // y positive is closer to wheelchair
  // z
  // forquePose.translation() = Eigen::Vector3d{0.57, -0.019, 0.012};
  // forquePose.linear() = Eigen::Matrix3d(Eigen::AngleAxisd(0.15,
  // Eigen::Vector3d::UnitX()));
  forquePose.translation() = Eigen::Vector3d{forkHolderTranslation[0],
                                             forkHolderTranslation[1],
                                             forkHolderTranslation[2]};
  ROS_INFO_STREAM("fork holder angle: " << forkHolderAngle);
  forquePose.linear() = Eigen::Matrix3d(
      Eigen::AngleAxisd(forkHolderAngle, Eigen::Vector3d::UnitX()));
  aboveForqueTSR.mT0_w = forquePose;

  aboveForqueTSR.mBw = createBwMatrixForTSR(0.0001, 0.0001, 0);
  aboveForqueTSR.mTw_e.matrix()
      *= mAda->getHand()->getEndEffectorTransform("plate")->matrix();

  if (!mAda->moveArmToTSR(aboveForqueTSR, mCollisionFreeConstraint,
    mPlanningTimeout,
    mMaxNumTrials))
  {
    ROS_ERROR("Trajectory execution failed");
  }
}

//==============================================================================
bool FeedingDemo::moveAbovePlate()
{
  waitForUser("Move above plate");

  Eigen::Isometry3d target
      = mWorkspace->getPlate()->getRootBodyNode()->getWorldTransform();
  Eigen::Isometry3d eeTransform
      = *mAda->getHand()->getEndEffectorTransform("plate");
  eeTransform.linear()
      = eeTransform.linear()
        * Eigen::Matrix3d(
              Eigen::AngleAxisd(M_PI * 0.5, Eigen::Vector3d::UnitZ()));
  eeTransform.translation() = Eigen::Vector3d(0, 0, mPlateTSRParameters["height"]);

  return moveAbove(target, eeTransform,
    mPlateTSRParameters["horizontalTolerance"],
    mPlateTSRParameters["verticalTolerance"],
    mPlateTSRParameters["rotationTolerance"],
    0.0);
}

//==============================================================================
bool FeedingDemo::moveAbove(
      const Eigen::Isometry3d& targetTransform,
      const Eigen::Isometry3d& endEffectorTransform,
      double horizontalTolerance,
      double verticalTolerance,
      double rotationTolerance,
      double tiltTolerance)
{
  TSR target;

  target.mT0_w = targetTransform;
  target.mBw = createBwMatrixForTSR(
    horizontalTolerance,
    verticalTolerance,
    rotationTolerance,
    tiltTolerance);

  target.mTw_e.matrix() = endEffectorTransform.matrix();

  // auto tsr = mViewer->addTSRMarker(target);
  // waitForUser("Check TSR");
  try
  {
    auto trajectoryCompleted
        = mAda->moveArmToTSR(target, mCollisionFreeConstraint,
          mPlanningTimeout,
          mMaxNumTrials,
          getConfigurationRanker(),
          velocityLimits,
          ada::TrajectoryPostprocessType::KUNZ);
    return trajectoryCompleted;
  }
  catch (...)
  {
    ROS_WARN("Error in trajectory completion!");
    return false;
  }
}

//==============================================================================
bool FeedingDemo::moveAboveFood(
  std::string foodName,
  const Eigen::Isometry3d& foodTransform,
  float rotateAngle, TiltStyle tiltStyle)
{
  waitForUser("Rotate forque to angle " + std::to_string(rotateAngle));

  Eigen::Isometry3d target;
  Eigen::Isometry3d eeTransform
      = *mAda->getHand()->getEndEffectorTransform("food");
  Eigen::AngleAxisd rotation
      = Eigen::AngleAxisd(-rotateAngle, Eigen::Vector3d::UnitZ());
  const double height = mFoodTSRParameters["height"];

  if (tiltStyle == TiltStyle::NONE)
  {
    target = foodTransform;
    eeTransform.linear() = eeTransform.linear() * rotation;
    eeTransform.translation()[2] = -height;
  }
  else if (tiltStyle == TiltStyle::VERTICAL)
  {
    target = removeRotation(foodTransform);
    eeTransform.linear() = eeTransform.linear()
      * rotation
      * Eigen::AngleAxisd( -M_PI * 0.5, Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd( M_PI + 0.5, Eigen::Vector3d::UnitX());
    eeTransform.translation()[2] = height;
  }
  else // angled
  {
    target = removeRotation(foodTransform);
    eeTransform.linear() = eeTransform.linear()
      * rotation
      * Eigen::AngleAxisd( M_PI * 0.5, Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd( M_PI * 5.0 / 6.0, Eigen::Vector3d::UnitX());
    eeTransform.translation() = Eigen::Vector3d{
      -sin(M_PI*0.25) * height * 0.5,
      0,
      cos(M_PI*0.25) * height * 0.5};
  }

  double rotationTolerance = mFoodTSRParameters["rotationTolerance"];
  if (mAllowRotationFree
    && std::find(mRotationFreeFoodNames.begin(),
      mRotationFreeFoodNames.end(), foodName) != mRotationFreeFoodNames.end())
  {
    rotationTolerance = M_PI;
  }

  return moveAbove(target, eeTransform,
      mFoodTSRParameters["horizontalTolerance"],
      mFoodTSRParameters["verticalTolerance"],
      rotationTolerance,
      0.0);
}

//==============================================================================
void FeedingDemo::scoop()
{
  /* temporarily disabling
  std::vector<std::string> twists{
    "/scoop/twist1", "/scoop/twist2", "/scoop/twist3"};

  for (const auto & param : twists)
  {
    auto success =
      moveWithEndEffectorTwist(
        Eigen::Vector6d(getRosParam<std::vector<double>>(param, mNodeHandle).data()));
    if (!success)
    {
      ROS_ERROR_STREAM("Failed to execute " << param << std::endl);
      throw std::runtime_error("Failed to execute scoop");
    }
  }
  */
}

//==============================================================================
void FeedingDemo::moveOutOf(TargetItem item, bool ignoreCollision)
{
  if (item != FOOD && item != PLATE && item != FORQUE)
    throw std::invalid_argument(
        "MoveOutOf[" + TargetToString.at(item) + "] not supported");

  waitForUser("Move Out of " + TargetToString.at(item));
  setFTThreshold(AFTER_GRAB_FOOD_FT_THRESHOLD);

  double length;
  Eigen::Vector3d direction(0, 0, 1);

  if (item == PLATE)
    length = getRosParam<double>("/feedingDemo/heightOutOfPlate", mNodeHandle);
  else if (item == FOOD)
    length = getRosParam<double>("/feedingDemo/moveOutofFood", mNodeHandle);
  else
  {
    length = 0.04;
    direction = Eigen::Vector3d(0, -1, 0);
  }

  for(int i = 0; i < 3; ++i)
  {
    if (ignoreCollision)
    {
      bool trajectoryCompleted = mAda->moveArmToEndEffectorOffset(
        direction, length, nullptr, //mCollisionFreeConstraint,
        mPlanningTimeout,
        mEndEffectorOffsetPositionTolerance,
        mEndEffectorOffsetAngularTolerance);
    }
    else
    {
      bool trajectoryCompleted = mAda->moveArmToEndEffectorOffset(
        direction, length, mCollisionFreeConstraint,
        mPlanningTimeout,
        mEndEffectorOffsetPositionTolerance,
        mEndEffectorOffsetAngularTolerance);
    }
  }
  setFTThreshold(STANDARD_FT_THRESHOLD);
  // trajectoryCompleted might be false because the forque hit the food
  // along the way and the trajectory was aborted
}

//==============================================================================
bool FeedingDemo::moveInto(TargetItem item,
  TiltStyle tiltStyle,
  const Eigen::Vector3d& endEffectorDirection)
{
  using aikido::control::ros::RosTrajectoryExecutor;

  waitForUser("Move into " + TargetToString.at(item));

  setFTThreshold(GRAB_FOOD_FT_THRESHOLD);

  if (item != FOOD && item != FORQUE)
    throw std::invalid_argument(
        "MoveInto[" + TargetToString.at(item) + "] not supported");

  if (item == TargetItem::FORQUE)
    return mAda->moveArmToEndEffectorOffset(
        Eigen::Vector3d(0, 1, 0), 0.01, mCollisionFreeConstraint,
    mPlanningTimeout,
    mEndEffectorOffsetPositionTolerance,
    mEndEffectorOffsetAngularTolerance);

  if (mAdaReal && mPerception && mVisualServo)
  {
    ROS_INFO("Servoing into food");
    auto rosExecutor
        = std::dynamic_pointer_cast<RosTrajectoryExecutor>(
          mAda->getTrajectoryExecutor());

    if (rosExecutor == nullptr)
    {
      throw std::runtime_error("no ros executor");
    }

    int numDofs = mAda->getArm()->getMetaSkeleton()->getNumDofs();
    Eigen::VectorXd velocityLimits = Eigen::VectorXd::Ones(numDofs) * 0.2;

    PerceptionServoClient servoClient(
        mNodeHandle,
        boost::bind(&Perception::getTrackedFoodItemPose, mPerception.get()),
        mArmSpace,
        mAda,
        mAda->getArm()->getMetaSkeleton(),
        mAda->getHand()->getEndEffectorBodyNode(),
        rosExecutor,
        mCollisionFreeConstraint,
        0.1,
        velocityLimits,
        0.1,
        0.002);
    servoClient.start();

    return servoClient.wait(10000.0);
  }

  double length = 0.025;

  for(int i = 0; i < 2; ++i)
  {
    // Collision constraint is not set because f/t sensor stops execution.
    auto result = mAda->moveArmToEndEffectorOffset(
        endEffectorDirection, length, nullptr,
        mPlanningTimeout,
        mEndEffectorOffsetPositionTolerance,
        mEndEffectorOffsetAngularTolerance);
    ROS_INFO_STREAM(" Execution result: " << result);
  }

  return true;

}

//==============================================================================
bool FeedingDemo::moveInFrontOfPerson()
{
  ROS_INFO_STREAM("move in front of person");

  double distanceToPerson
      = getRosParam<double>("/feedingDemo/distanceToPerson", mNodeHandle);
  double horizontalToleranceNearPerson = getRosParam<double>(
      "/planning/tsr/horizontalToleranceNearPerson", mNodeHandle);
  double verticalToleranceNearPerson = getRosParam<double>(
      "/planning/tsr/verticalToleranceNearPerson", mNodeHandle);

  aikido::constraint::dart::TSR personTSR;
  Eigen::Isometry3d personPose = Eigen::Isometry3d::Identity();
  personPose.translation() = mWorkspace->getPersonPose()
                                 .translation(); // + Eigen::Vector3d(0,0,0.2);
  personPose.linear()
      = Eigen::Matrix3d(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
  personTSR.mT0_w = personPose;
  personTSR.mTw_e.translation() = Eigen::Vector3d{0, distanceToPerson, 0};

  personTSR.mBw = createBwMatrixForTSR(
      horizontalToleranceNearPerson, verticalToleranceNearPerson, 0, 0);
  personTSR.mTw_e.matrix()
      *= mAda->getHand()->getEndEffectorTransform("person")->matrix();

  return mAda->moveArmToTSR(
    personTSR,
    mCollisionFreeConstraint,
    mPlanningTimeout,
    mMaxNumTrials,
    getConfigurationRanker(),
    velocityLimits,
    ada::TrajectoryPostprocessType::KUNZ);
}

//==============================================================================
void FeedingDemo::moveDirectlyToPerson(bool tilted)
{
  double distanceToPerson = 0.02;
  double horizontalToleranceNearPerson = getRosParam<double>(
      "/planning/tsr/horizontalToleranceNearPerson", mNodeHandle);
  double verticalToleranceNearPerson = getRosParam<double>(
      "/planning/tsr/verticalToleranceNearPerson", mNodeHandle);

  Eigen::Isometry3d personPose = createIsometry(
      getRosParam<std::vector<double>>("/study/personPose", mNodeHandle));
  if (tilted)
  {
    std::vector<double> tiltOffsetVector
        = getRosParam<std::vector<double>>("/study/tiltOffset", mNodeHandle);
    Eigen::Vector3d tiltOffset{
        tiltOffsetVector[0], tiltOffsetVector[1], tiltOffsetVector[2]};
    personPose.translation() += tiltOffset;
  }

  aikido::constraint::dart::TSR personTSR;
  personTSR.mT0_w = personPose;
  personTSR.mTw_e.translation() = Eigen::Vector3d{0, distanceToPerson, 0};

  if (tilted)
  {
    personTSR.mBw = createBwMatrixForTSR(
        horizontalToleranceNearPerson,
        verticalToleranceNearPerson,
        -M_PI / 4,
        M_PI / 4);
    Eigen::Isometry3d eeTransform
        = *mAda->getHand()->getEndEffectorTransform("person");
    eeTransform.linear()
        = eeTransform.linear()
          * Eigen::Matrix3d(
                Eigen::AngleAxisd(M_PI * -0.25, Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(M_PI * 0.25, Eigen::Vector3d::UnitX()));
    personTSR.mTw_e.matrix() *= eeTransform.matrix();
  }
  else
  {
    personTSR.mBw = createBwMatrixForTSR(
        horizontalToleranceNearPerson, verticalToleranceNearPerson, 0, 0);
    personTSR.mTw_e.matrix()
        *= mAda->getHand()->getEndEffectorTransform("person")->matrix();
  }

  if (!mAda->moveArmToTSR(
      personTSR,
      mCollisionFreeConstraint,
      mPlanningTimeout,
      mMaxNumTrials,
      getConfigurationRanker(),
      velocityLimits,
      ada::TrajectoryPostprocessType::KUNZ))
  {
    ROS_WARN_STREAM("Execution failed");
  }
}

//==============================================================================
bool FeedingDemo::moveTowardsPerson()
{
  waitForUser("Move towards person");

  if (!mAdaReal)
  {
    return mAda->moveArmToEndEffectorOffset(
        Eigen::Vector3d(0, 1, 0),
        getRosParam<double>("/feedingDemo/distanceToPerson", mNodeHandle) * 0.4,
        mCollisionFreeConstraint,
        mPlanningTimeout,
        mEndEffectorOffsetPositionTolerance,
        mEndEffectorOffsetAngularTolerance);
  }

  std::shared_ptr<aikido::control::TrajectoryExecutor> executor
      = mAda->getTrajectoryExecutor();
  std::shared_ptr<aikido::control::ros::RosTrajectoryExecutor> rosExecutor
      = std::dynamic_pointer_cast<aikido::control::ros::RosTrajectoryExecutor>(
          executor);

  if (rosExecutor == nullptr)
  {
    throw std::runtime_error("no ros executor");
  }

  int numDofs = mAda->getArm()->getMetaSkeleton()->getNumDofs();
  Eigen::VectorXd velocityLimits = Eigen::VectorXd::Zero(numDofs);
  for (int i = 0; i < numDofs; i++)
    velocityLimits[i] = 0.2;

  feeding::PerceptionServoClient servoClient(
      mNodeHandle,
      boost::bind(&Perception::perceiveFace, mPerception.get()),
      mArmSpace,
      mAda,
      mAda->getArm()->getMetaSkeleton(),
      mAda->getHand()->getEndEffectorBodyNode(),
      rosExecutor,
      mCollisionFreeConstraint,
      0.2,
      velocityLimits,
      0,
      0.06);
  servoClient.start();
  return servoClient.wait(30);
}

//==============================================================================
aikido::rviz::WorldInteractiveMarkerViewerPtr FeedingDemo::getViewer()
{
  return mViewer;
}

//==============================================================================
void FeedingDemo::pickUpFork()
{
  mAda->openHand();
  moveAboveForque();
  moveInto(TargetItem::FORQUE);

  std::vector<std::string> optionPrompts{"(1) close", "(2) leave-as-is"};
  auto input = getUserInputWithOptions(optionPrompts, "Close Hand?");

  if (input == 1)
  {
    mAda->closeHand();
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }

  moveOutOf(TargetItem::FORQUE);
  moveAbovePlate();
}

//==============================================================================
void FeedingDemo::putDownFork()
{
  mAda->closeHand();
  moveAboveForque();
  moveInto(TargetItem::FORQUE);

  mAda->openHand();
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  moveOutOf(TargetItem::FORQUE);
  moveAbovePlate();
}

//==============================================================================
void FeedingDemo::skewer(std::string foodName)
{
  moveAbovePlate();
  TargetFoodItem item = detectAndMoveAboveFood(foodName);

  for(std::size_t i = 0; i < MAX_NUM_TRIALS; ++i)
  {
    auto tiltStyle = item.getAction().tiltStyle;
    ROS_INFO_STREAM(
        "Getting " << foodName << "with " << mFoodSkeweringForces[foodName]
                               << "N with angle mode " << tiltStyle);

    // ===== INTO FOOD =====
    waitForUser("Move forque into food");
    moveInto(TargetItem::FOOD);
    grabFoodWithForque();
    std::this_thread::sleep_for(mWaitTimeForFood);

    // ===== OUT OF FOOD =====
    moveOutOf(TargetItem::FOOD, true);
    std::this_thread::sleep_for(mWaitTimeForFood);
    if (!mAdaReal || (mAdaReal &&
        getUserInputWithOptions(optionPrompts, "Did I succeed?") == 1))
    {
      ROS_INFO_STREAM("Successful");
      return;
    }
    ungrabAndDeleteFood();
    ROS_INFO_STREAM("Try again.");
  }

  moveAbovePlate();
}

//==============================================================================
void FeedingDemo::feedFoodToPerson(ros::NodeHandle& nodeHandle, bool tilted)
{

  bool moveSuccess = false;

  moveInFrontOfPerson();
  /*
  for (std::size_t i = 0; i < 2; ++i)
  {
    moveInFrontOfPerson();
    nodeHandle.setParam("/feeding/facePerceptionOn", true);

    waitForUser("Move towards person");

    moveSuccess = moveTowardsPerson();
    nodeHandle.setParam("/feeding/facePerceptionOn", false);

    if (moveSuccess)
      break;
    ROS_INFO_STREAM("Moved failed, backing up and retrying");
  }

  if (!moveSuccess) {
    ROS_INFO_STREAM("Servoing failed. Falling back to direct movement...");
    moveInFrontOfPerson();
    moveDirectlyToPerson(tilted);
  }*/

  // ===== EATING =====
  ROS_WARN("Human is eating");
  std::this_thread::sleep_for(
      std::chrono::milliseconds(
          getRosParam<int>("/feedingDemo/waitMillisecsAtPerson", nodeHandle)));
  ungrabAndDeleteFood();

  waitForUser("Move away from person");

  // ===== BACK TO PLATE =====
  waitForUser("Move back to plate");

  moveAbovePlate();
}

//==============================================================================
std::vector<TargetFoodItem> FeedingDemo::detectFoodItems(
    const std::string& foodName)
{
  if (!mAdaReal)
  {
    return std::vector<TargetFoodItem>{
      TargetFoodItem("item", nullptr, getDefaultFoodTransform(), 1.0)};
  }
  return mPerception->perceiveFood(foodName);
}

//==============================================================================
void FeedingDemo::setFTThreshold(FTThreshold threshold)
{
  if (mAdaReal && mIsFTSensingEnabled)
    if (!mFTThresholdHelper->setThresholds(AFTER_GRAB_FOOD_FT_THRESHOLD))
      exit(1);
}

//==============================================================================
void FeedingDemo::waitForUser(const std::string& prompt)
{
  if (!mAutoContinueDemo)
    ada::util::waitForUser(prompt, mAda);
}

//==============================================================================
TargetFoodItem FeedingDemo::detectAndMoveAboveFood(
      const std::string& foodName)
{
  auto candidateItems = detectFoodItems(foodName);

  TargetFoodItem targetItem;

  if (candidateItems.size() == 0)
    throw std::runtime_error("Failed to detect any food.");

  bool moveAboveSuccessful = false;
  for(const auto& item, candidateItems)
  {
    auto action = item.getAction();

    if (!moveAboveFood(item.getName(), item.getPose(),
        item.getAction(), action.rotationAngle, action.tiltAngle))
    {
      ROS_INFO_STREAM("Failed to move above " << item.getName());
      continue;
    }
    moveAboveSuccessful = true;
    targetItem = item;
    break;
  }

  if (!moveAboveSuccessful)
  {
    ROS_ERROR("Failed to move above any food.");
    throw std::runtime_error("Failed to move above any food.");
  }
  mPerception->setFoodItemToTrack(item);
  return item;
}

//==============================================================================
void FeedingDemo::reset()
{
  mWorkspace->reset();
}

//==============================================================================
bool FeedingDemo::moveWithEndEffectorTwist(
    const Eigen::Vector6d& twists,
    double duration,
    bool respectCollision)
{
  /* temporarily disabling
  return mAda->moveArmWithEndEffectorTwist(
    Eigen::Vector6d(getRosParam<std::vector<double>>("/scoop/twist1", mNodeHandle).data()),
    respectCollision ? mCollisionFreeConstraint : nullptr,
    duration,
    getRosParam<double>("/planning/timeoutSeconds", mNodeHandle),
    getRosParam<double>(
          "/planning/endEffectorTwist/positionTolerance", mNodeHandle),
    getRosParam<double>(
          "/planning/endEffectorTwist/angularTolerance", mNodeHandle));
          */
}


} // namespace feeding
