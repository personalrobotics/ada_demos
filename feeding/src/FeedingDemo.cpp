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

const bool TERMINATE_AT_USER_PROMPT = true;

static const std::vector<double> weights = {1, 1, 10, 0.01, 0.01, 0.01};
static const std::size_t MAX_NUM_TRIALS = 3;
static const double inf = std::numeric_limits<double>::infinity();
static const std::vector<double> velocityLimits{0.2, 0.2, 0.2, 0.2, 0.2, 0.4};

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
Eigen::Isometry3d FeedingDemo::getDefaultFoodTransform()
{
  return mWorkspace->getDefaultFoodItem()
      ->getRootBodyNode()
      ->getWorldTransform();
}

//==============================================================================
bool FeedingDemo::isCollisionFree()
{
  std::string result;
  auto robotState = mAda->getStateSpace()->getScopedStateFromMetaSkeleton(
      mAda->getMetaSkeleton().get());
  aikido::constraint::dart::CollisionFreeOutcome collisionCheckOutcome;
  if (!mCollisionFreeConstraint->isSatisfied(
          robotState, &collisionCheckOutcome))
  {
    result = "Robot is in collison: " + collisionCheckOutcome.toString();
    return false;
  }
  result = "Robot is not in collision";
  ROS_INFO_STREAM(result);
  return true;
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
void FeedingDemo::moveToStartConfiguration()
{
  auto home
      = getRosParam<std::vector<double>>("/ada/homeConfiguration", mNodeHandle);
  if (mAdaReal)
  {
    ROS_INFO_STREAM("Real mode, not moving.");
  }
  else
  {
    ROS_INFO_STREAM("Set to home pose.");
    mAda->getArm()->getMetaSkeleton()->setPositions(
        Eigen::Vector6d(home.data()));
  }
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
  aboveForqueTSR.mTw_e.translation() = Eigen::Vector3d{0, 0, 0};

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
bool FeedingDemo::moveAbovePlate(bool liftUpBeforeAction)
{
  if (liftUpBeforeAction)
  {
    waitForUser("Lift forque up (without collision checking)");
    double length = 0.03;
    auto result = mAda->moveArmToEndEffectorOffset(
      Eigen::Vector3d(0, 0, 1), length, nullptr,
      mPlanningTimeout,
      mEndEffectorOffsetPositionTolerance,
      mEndEffectorOffsetAngularTolerance);

    if (!result)
    {
      waitForUser("Lift up failed. Returning.");
      return false;
    }
  }
  waitForUser("Move above plate");

  auto target = mWorkspace->getPlate()->getRootBodyNode()->getWorldTransform();
  Eigen::Isometry3d eeTransform
      = *mAda->getHand()->getEndEffectorTransform("plate");
  eeTransform.linear()
      = eeTransform.linear()
        * Eigen::Matrix3d(
              Eigen::AngleAxisd(M_PI * 0.5, Eigen::Vector3d::UnitZ()));

  return moveAbove(TargetItem::PLATE, target, eeTransform, 0.0, TiltStyle::NONE,
    mPlateTSRParameters["height"],
    mPlateTSRParameters["horizontalTolerance"],
    mPlateTSRParameters["verticalTolerance"],
    mPlateTSRParameters["rotationTolerance"],
    0.0);
}

//==============================================================================
void FeedingDemo::visualizeTrajectory(
    aikido::trajectory::TrajectoryPtr trajectory)
{
  dart::dynamics::BodyNodePtr endEffector
      = mAda->getMetaSkeleton()->getBodyNode("j2n6s200_forque_end_effector");

  if (!mViewer)
  {
    ROS_WARN("Visualize trajectory: viewer is nullptr");
  }
  else if (!endEffector)
  {
    ROS_WARN("Visualize trajectory: endEffector is nullptr");
  }
  else if (!trajectory)
  {
    ROS_WARN("Visualize trajectory: trajectory is nullptr");
  }
  else
  {
    mViewer->removeTrajectoryMarker(trajectoryMarkerPtr);
    trajectoryMarkerPtr = mViewer->addTrajectoryMarker(
        trajectory,
        mAda->getArm()->getMetaSkeleton(),
        *endEffector,
        Eigen::Vector4d{1, 1, 1, 1},
        0.01,
        16u);
  }
}

//==============================================================================
bool FeedingDemo::moveAbove(
      TargetItem item,
      const Eigen::Isometry3d& targetTransform,
      const Eigen::Isometry3d& endEffectorTransform,
      float rotAngle,
      TiltStyle tiltStyle,
      double height,
      double horizontalTolerance,
      double verticalTolerance,
      double rotationTolerance,
      double tiltTolerance)
{
  TSR target;

  // TODO: does this work for rotation-specified items?
  if (item == TargetItem::FOOD)
    target.mT0_w = removeRotation(targetTransform);
  else
    target.mT0_w = targetTransform;

  target.mBw = createBwMatrixForTSR(
    horizontalTolerance,
    verticalTolerance,
    rotationTolerance,
    tiltTolerance);

  target.mTw_e.matrix() *= endEffectorTransform.matrix();
  target.mTw_e.translation() = Eigen::Vector3d(0, 0, height);

  try
  {
    auto trajectoryCompleted
        = mAda->moveArmToTSR(target, mCollisionFreeConstraint,
          mPlanningTimeout,
          mMaxNumTrials,
          getRanker(),
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
    float rotAngle,
    TiltStyle tiltStyle)
{
  waitForUser("Move above food");

  Eigen::Isometry3d eeTransform
      = *mAda->getHand()->getEndEffectorTransform(TargetToString.at(TargetItem::FOOD));

  eeTransform.linear()
    = eeTransform.linear()
      * Eigen::Matrix3d(
            Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));

  double rotationTolerance = mFoodTSRParameters["rotationTolerance"];
  if (mAllowRotationFree
    && std::find(mRotationFreeFoodNames.begin(),
      mRotationFreeFoodNames.end(), foodName) != mRotationFreeFoodNames.end())
  {
    rotationTolerance = M_PI;
  }

  return moveAbove(TargetItem::FOOD,
    foodTransform,
    eeTransform,
    rotAngle,
    tiltStyle,
    mFoodTSRParameters["height"],
    mFoodTSRParameters["horizontalTolerance"],
    mFoodTSRParameters["verticalTolerance"],
    rotationTolerance,
    mFoodTSRParameters["tiltTolerance"]);
}

//==============================================================================
bool FeedingDemo::moveAboveFood(
    const Eigen::Isometry3d& foodTransform,
    int pickupAngleMode)
{
  ROS_INFO_STREAM("Move above food");

  double heightAboveFood
      = getRosParam<double>("/feedingDemo/heightAboveFood", mNodeHandle);

  // If the robot is not simulated, we want to plan the trajectory to move a
  // little further downwards,
  // so that the MoveUntilTouchController can take care of stopping the
  // trajectory.
  double horizontalToleranceNearFood = getRosParam<double>(
      "/planning/tsr/horizontalToleranceNearFood", mNodeHandle);
  double verticalToleranceNearFood = getRosParam<double>(
      "/planning/tsr/verticalToleranceNearFood", mNodeHandle);

  aikido::constraint::dart::TSR aboveFoodTSR;
  aboveFoodTSR.mT0_w = removeRotation(foodTransform);

  Eigen::Isometry3d eeTransform
      = *mAda->getHand()->getEndEffectorTransform("food");

  if (pickupAngleMode != 0)
  {
    // celery-style
    // eeTransform.linear() = eeTransform.linear() *
    // Eigen::Matrix3d(Eigen::AngleAxisd( M_PI * 0.5, Eigen::Vector3d::UnitZ())
    // * Eigen::AngleAxisd( M_PI - angle + 0.5, Eigen::Vector3d::UnitX()));

    if (pickupAngleMode == 1)
    {
      // strawberry-style
      eeTransform.linear()
          = eeTransform.linear()
            * Eigen::Matrix3d(
                  Eigen::AngleAxisd(-M_PI * 0.5, Eigen::Vector3d::UnitZ())
                  * Eigen::AngleAxisd(M_PI + 0.5, Eigen::Vector3d::UnitX()));
    }
    else
    {
      // banana-style
      eeTransform.linear()
          = eeTransform.linear()
            * Eigen::Matrix3d(
                  Eigen::AngleAxisd(M_PI * 0.5, Eigen::Vector3d::UnitZ())
                  * Eigen::AngleAxisd(M_PI + 0.5, Eigen::Vector3d::UnitX()));
    }
  }
  else
  {
    eeTransform.linear()
      = eeTransform.linear()
        * Eigen::Matrix3d(
              Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
  }
  ROS_INFO_STREAM("Move above food \n" << eeTransform.linear());

  // Allow rotational freedom
  aboveFoodTSR.mBw = createBwMatrixForTSR(
    horizontalToleranceNearFood, verticalToleranceNearFood, -M_PI, M_PI);

  aboveFoodTSR.mTw_e.matrix() *= eeTransform.matrix();

  ROS_INFO_STREAM("Move above food \n" << eeTransform.linear());

  float distance = heightAboveFood * 0.85;

  if (pickupAngleMode == 0)
  {
    // vertical
    aboveFoodTSR.mTw_e.translation() = Eigen::Vector3d{0, 0, distance};
  }
  else if (pickupAngleMode == 1)
  {
    // strawberry style angled
    aboveFoodTSR.mTw_e.translation() = Eigen::Vector3d{0, 0, distance};
  }
  else
  {
    // banana style angled
    aboveFoodTSR.mTw_e.translation() = Eigen::Vector3d{
        -sin(M_PI * 0.25) * distance, 0, cos(M_PI * 0.25) * distance};
  }

  bool trajectoryCompleted = false;
  try
  {
    trajectoryCompleted
        = mAda->moveArmToTSR(aboveFoodTSR, mCollisionFreeConstraint,
          mPlanningTimeout,
          mMaxNumTrials,
          getRanker(),
          velocityLimits,
          ada::TrajectoryPostprocessType::KUNZ);

    std::cout << "Trajectory completed: " << trajectoryCompleted << std::endl;
  }
  catch (...)
  {
    ROS_WARN("Error in trajectory completion!");
  }

  return trajectoryCompleted;
}

//==============================================================================
bool FeedingDemo::rotateForque(float rotateAngle, TiltStyle tiltStyle)
{
  waitForUser("Rotate forque to angle " + std::to_string(rotateAngle));

  aikido::constraint::dart::TSR aboveFoodTSR;
  Eigen::Isometry3d eeTransform
      = *mAda->getHand()->getEndEffectorTransform("food");

  aboveFoodTSR.mT0_w = getDefaultFoodTransform();

  Eigen::AngleAxisd rotation = Eigen::AngleAxisd(-rotateAngle, Eigen::Vector3d::UnitZ());
  if (tiltStyle == TiltStyle::NONE)
  {
    eeTransform.linear() = eeTransform.linear() * Eigen::Matrix3d(rotation);
  }
  else if (tiltStyle == TiltStyle::VERTICAL)
  {
    eeTransform.linear()
        = eeTransform.linear()
          * Eigen::Matrix3d(
                rotation
                * Eigen::AngleAxisd(
                      M_PI + 0.5, rotation * Eigen::Vector3d::UnitX()));
  }
  else if (tiltStyle == TiltStyle::ANGLED)
  {
    eeTransform.linear()
        = eeTransform.linear()
          * Eigen::Matrix3d(
                rotation
                * Eigen::AngleAxisd(
                      M_PI - 0.5,
                      rotation
                          * Eigen::Vector3d::
                                UnitX()));
  }
  else
  {
    ROS_ERROR("Rotate forque params not recognized");
    return false;
  }

  aboveFoodTSR.mBw = createBwMatrixForTSR(
      mFoodTSRParameters["horizontalTolerance"],
      mFoodTSRParameters["verticalTolerance"],
      mFoodTSRParameters["rotationTolerance"]);
  aboveFoodTSR.mTw_e.matrix() *= eeTransform.matrix();

  aboveFoodTSR.mTw_e.translation()
        = Eigen::Vector3d{0, 0, -mFoodTSRParameters["height"] * 0.85};

  if (!mAda->moveArmToTSR(aboveFoodTSR, mCollisionFreeConstraint,
        mPlanningTimeout,
        mMaxNumTrials))
  {
    ROS_ERROR("Trajectory execution failed");
    return false;
  }

  return true;
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
    std::cout << "Pull out " << i << std::endl;
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
bool FeedingDemo::moveInto(TargetItem item)
{
  waitForUser("Move into " + TargetToString.at(item));

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
    std::shared_ptr<aikido::control::TrajectoryExecutor> executor
        = mAda->getTrajectoryExecutor();

    std::shared_ptr<aikido::control::ros::RosTrajectoryExecutor> rosExecutor
        = std::
            dynamic_pointer_cast<aikido::control::ros::RosTrajectoryExecutor>(
                executor);

    if (rosExecutor == nullptr)
    {
      throw std::runtime_error("no ros executor");
    }

    int numDofs = mAda->getArm()->getMetaSkeleton()->getNumDofs();
    Eigen::VectorXd velocityLimits = Eigen::VectorXd::Ones(numDofs) * 0.2;

    PerceptionServoClient servoClient(
        mNodeHandle,
        boost::bind(&Perception::perceiveFood, mPerception.get()),
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

  double length = 0.05;
  if (!mAdaReal)
  {
    auto length
      = getRosParam<double>("/feedingDemo/heightAboveFood", mNodeHandle)
        + getRosParam<double>("/feedingDemo/heightIntoFood", mNodeHandle);
  }

  for(int i = 0; i < 2; ++i)
  {
    // Collision constraint is not set because f/t sensor stops execution.
    auto result = mAda->moveArmToEndEffectorOffset(
        Eigen::Vector3d(0, 0, -1), length, nullptr,
        mPlanningTimeout,
        mEndEffectorOffsetPositionTolerance,
        mEndEffectorOffsetAngularTolerance);
    std::cout <<" Execution " << result << std::endl;
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
    getRanker(),
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
      getRanker(),
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

  waitForUser("Forque?");
  moveAboveForque();

  waitForUser("In?");
  moveInto(TargetItem::FORQUE);

  if (ada::util::waitForUser("Close?"))
  {
    mAda->closeHand();
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }

  waitForUser("Out?");
  moveOutOf(TargetItem::FORQUE);

  waitForUser("Above Plate?");
  moveAbovePlate();
}

//==============================================================================
void FeedingDemo::putDownFork()
{
  mAda->closeHand();

  waitForUser("Forque?");
  moveAboveForque();

  waitForUser("In?");
  moveInto(TargetItem::FORQUE);

  mAda->openHand();
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  waitForUser("Out?");
  moveOutOf(TargetItem::FORQUE);

  waitForUser("Above Plate?");
  moveAbovePlate();
}

//==============================================================================
void FeedingDemo::skewer(
    std::string foodName, ros::NodeHandle& nodeHandle, int max_trial_per_item,
    bool ignoreCollisionWhenMovingOut)
{
  bool foodPickedUp = false;
  int num_tries = 0;

  if (foodName == "")
  {
    mPerception->reset();
    foodName = getUserInput(true, nodeHandle);
  }

  while (!foodPickedUp)
  {
    auto pickupAngleMode = mPickUpAngleModes[foodName];

    ROS_INFO_STREAM(
        "Getting " << foodName << "with "
                               << mFoodSkeweringForces[foodName]
                               << "N with angle mode "
                               << pickupAngleMode
                               << std::endl);

    // ===== Detect and Move Above Food =====
    if (!mPerception->setFoodName(foodName))
      throw std::runtime_error("Unknown food");

    try{
      detectAndMoveAboveFood(foodName, pickupAngleMode);
    }
    catch(std::runtime_error e)
    {
     // ===== ABOVE PLATE =====
      moveAbovePlate();
      waitForUser("Move forque above plate complete");
      continue;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(800));

    // ===== INTO FOOD =====
    waitForUser("Move forque into food");

    double zForceBeforeSkewering = 0;
    if (mFTThresholdHelper->startDataCollection(20))
    {
      Eigen::Vector3d currentForce, currentTorque;
      while (!mFTThresholdHelper->isDataCollectionFinished(
          currentForce, currentTorque))
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      zForceBeforeSkewering = currentForce.x();
    }
    double torqueThreshold = 2;

    if (!mFTThresholdHelper->setThresholds(
            mFoodSkeweringForces[foodName], torqueThreshold))
    {
      ROS_ERROR("Failed to set F/T threshold");
      exit(1);
    }

    moveInto(TargetItem::FOOD);

    std::this_thread::sleep_for(
        std::chrono::milliseconds(
            getRosParam<int>("/feedingDemo/waitMillisecsAtFood", nodeHandle)));
    grabFoodWithForque();

    // ===== OUT OF FOOD =====
    moveOutOf(TargetItem::FOOD, ignoreCollisionWhenMovingOut);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    double forceDifference = 0;
    double zForceAfter = 0;
    if (mAdaReal && mFTThresholdHelper->startDataCollection(20))
    {
      Eigen::Vector3d currentForce, currentTorque;
      while (!mFTThresholdHelper->isDataCollectionFinished(
          currentForce, currentTorque))
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      zForceAfter = currentForce.x();
      forceDifference = zForceBeforeSkewering - currentForce.x();
    }
    if (mAdaReal)
      ROS_WARN_STREAM(
          "force difference: " << (zForceBeforeSkewering - zForceAfter));

    std::vector<std::string> optionPrompts{"(1) success", "(2) fail"};
    // (mAdaReal && forceDifference > 0.01)
    if (!mAdaReal ||
      (mAdaReal && getUserInputWithOptions(optionPrompts, "Did I succeed?") == 1))
    {
      ROS_INFO_STREAM("Successful");
      return;
    }
    else
    {
      ungrabAndDeleteFood();
      if (num_tries >= max_trial_per_item)
      {
        ROS_WARN_STREAM(
            "Ooops! I think I didn't manage to pick up the "
            << foodName
            << ".  Maybe we should try a different food item."
            << std::endl);
        mPerception->reset();
        foodName = getUserInput(true, nodeHandle);
        num_tries = 0;
      }
      else
      {
        ROS_WARN_STREAM(
            "Ooops! I think I didn't manage to pick up the "
            << foodName
            << ". Let me try one more time."
            << std::endl);

        num_tries++;
      }
    }
    // ===== ABOVE PLATE =====
    moveAbovePlate(true);
    waitForUser("Move forque above plate complete");

  }
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
boost::optional<Eigen::Isometry3d> FeedingDemo::detectFood(
    const std::string& foodName, bool waitTillDetected)
{
  if (!mAdaReal)
    return getDefaultFoodTransform();

  for(std::size_t i = 0; i < MAX_NUM_TRIALS; ++i)
  {
    auto detected = mPerception->perceiveFood();
    if (!detected)
    {
      ROS_WARN_STREAM("I can't see the " << foodName << std::endl);
      continue;
    }
    return detected.get();
  }
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
    ada::util::waitForUser(prompt, TERMINATE_AT_USER_PROMPT);
}

//==============================================================================
Eigen::Isometry3d FeedingDemo::detectAndMoveAboveFood(
    const std::string& foodName, int pickupAngleMode)
{
  for(std::size_t i = 0; i < MAX_NUM_TRIALS; ++i)
  {
    auto foodTransform = detectFood(foodName, true);
    if (!foodTransform)
    {
      ROS_WARN("Failed to detect food");
      continue;
    }

    if (!moveAboveFood(foodTransform.get(), pickupAngleMode))
    {
      mPerception->reset();
      waitForUser("Trajectory failed! Reposition food and try again!");
      continue;
    }
    return foodTransform.get();
  }

  throw std::runtime_error("Failed to detect or move above food");
}

//==============================================================================
Eigen::Isometry3d FeedingDemo::detectAndMoveAboveFood(
      const std::string& foodName,
      float rotAngle,
      TiltStyle tiltStyle)
{
  for(std::size_t i = 0; i < MAX_NUM_TRIALS; ++i)
  {
    auto foodTransform = detectFood(foodName, true);
    if (!foodTransform)
    {
      ROS_WARN("Failed to detect food");
      continue;
    }

    if (!moveAboveFood(foodName, foodTransform.get(), rotAngle, tiltStyle))
    {
      mPerception->reset();
      waitForUser("Trajectory failed! Reposition food and try again!");
      continue;
    }
    return foodTransform.get();
  }

  throw std::runtime_error("Failed to detect or move above food");
}

//==============================================================================
void FeedingDemo::reset()
{
  mWorkspace->reset();
}

//==============================================================================
aikido::distance::ConfigurationRankerPtr FeedingDemo::getRanker(
  const Eigen::VectorXd& configuration)
{
  auto metaSkeleton = mAda->getArm()->getMetaSkeleton();
  auto nominalState = mArmSpace->createState();

  if (configuration.size() > 0)
    mArmSpace->convertPositionsToState(configuration, nominalState);
  else
    nominalState = mArmSpace->getScopedStateFromMetaSkeleton(metaSkeleton.get());

  return std::make_shared<NominalConfigurationRanker>(
    mArmSpace,
    metaSkeleton,
    weights,
    nominalState);
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
