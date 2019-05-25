
#include <aikido/io/CatkinResourceRetriever.hpp>
#include <aikido/io/util.hpp>
#include <aikido/planner/World.hpp>
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include <dart/dynamics/Frame.hpp>
#include <pr_tsr/plate.hpp>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <libada/Ada.hpp>
#include "cameraCalibration/Perception.hpp"
#include "cameraCalibration/util.hpp"

using namespace cameraCalibration;

// Robot To World
static const Eigen::Isometry3d robotPose = createIsometry(0.0, 0, 0, 0, 0, 3.1415);

// TargetToWorld = RobotToWorld * TargetToRobot
// static const Eigen::Isometry3d targetToWorld = robotPose.inverse() * createIsometry(.425, 0.15, -0.005, 3.1415, 0, 0);
static const Eigen::Isometry3d targetToWorld = robotPose.inverse() * createIsometry(0.0, 0.4, 0, 3.1415, 0, 0);
std::vector<Eigen::Isometry3d> cameraToJouleEstimates;

static const double planningTimeout = 1.0;
static const int maxNumTrials = 50;

bool tryPerceivePoint(
        std::string frameName,
        Perception& perception,
        tf::TransformListener& tfListener,
        aikido::rviz::WorldInteractiveMarkerViewer& jouleViewer,
        aikido::rviz::WorldInteractiveMarkerViewer& targetPointViewer,
        std::vector<Eigen::Isometry3d>& targetPointsInCameraLensFrame,
        std::vector<Eigen::Isometry3d>& cameraLensPointsInWorldFrame,
        std::vector<dart::dynamics::SimpleFramePtr>& frames,
        std::vector<aikido::rviz::FrameMarkerPtr>& frameMarkers) {

  Eigen::Isometry3d worldToJoule = getWorldToJoule(tfListener);
  Eigen::Isometry3d cameraToJoule = getCameraToJoule(tfListener);

  try{

    cameraToJouleEstimates.emplace_back(
      perception.computeCameraToJoule(targetToWorld, worldToJoule,
        getCameraToLens(tfListener),
        cameraToJoule));

    Eigen::Isometry3d joule = getWorldToJoule(tfListener).inverse();
    dart::dynamics::SimpleFramePtr jouleFrame = std::make_shared<dart::dynamics::SimpleFrame>(dart::dynamics::Frame::World(), "joule_" + frameName, joule);
    frames.push_back(jouleFrame);
    frameMarkers.push_back(jouleViewer.addFrame(jouleFrame.get(), 0.07, 0.007));
    return true;
  }
  catch (...)
  {
    ROS_ERROR("Failed to compute transform");
  }

  return false;
}


int main(int argc, char** argv)
{

  // ===== STARTUP =====

  // Is the real robot used or simulation?
  bool adaReal = false;

  // Should the demo continue without asking for human input at each step?
  bool autoContinueDemo = false;

  handleArguments(argc, argv, adaReal, autoContinueDemo);
  ROS_INFO_STREAM("Simulation Mode: " << !adaReal);

  // start node
  ros::init(argc, argv, "feeding");
  ros::NodeHandle nodeHandle("~");

  // start demo
  std::shared_ptr<aikido::planner::World> world
      = std::make_shared<aikido::planner::World>("feeding");
  ada::Ada ada(
      world,
      !adaReal,
      "package://ada_description/robots_urdf/ada_with_camera.urdf",
      "package://ada_description/robots_urdf/ada_with_camera.srdf",
      "j2n6s200_hand_tip");
  auto armSpace = ada.getArm()->getStateSpace();

  std::cout << ada.getArm()->getMetaSkeleton()->getPositions().transpose() << std::endl;

  // Setting up workspace
  const auto resourceRetriever
      = std::make_shared<aikido::io::CatkinResourceRetriever>();
  Eigen::Isometry3d tablePose
      = robotPose.inverse() * createIsometry(-0.35, 0.38, -0.745, 0, 0, 0);
  auto table = loadSkeletonFromURDF(
      resourceRetriever,
      "package://pr_assets/data/furniture/table_feeding.urdf",
      tablePose);
  world->addSkeleton(table);
  Eigen::Isometry3d environmentPose
      = robotPose.inverse() * createIsometry(0, 0, 0, 0, 0, 0);
  auto environment = loadSkeletonFromURDF(
      resourceRetriever,
      "package://pr_assets/data/furniture/workspace_feeding_demo.urdf",
      environmentPose);
  world->addSkeleton(environment);

  // Setting up collisions
  dart::collision::CollisionDetectorPtr collisionDetector
      = dart::collision::FCLCollisionDetector::create();
  std::shared_ptr<dart::collision::CollisionGroup> armCollisionGroup
      = collisionDetector->createCollisionGroup(
          ada.getMetaSkeleton().get(), ada.getHand()->getEndEffectorBodyNode());
  std::shared_ptr<dart::collision::CollisionGroup> envCollisionGroup
      = collisionDetector->createCollisionGroup(
          table.get(), environment.get());
  auto collisionFreeConstraint
      = std::make_shared<aikido::constraint::dart::CollisionFree>(
          armSpace, ada.getArm()->getMetaSkeleton(), collisionDetector);
  collisionFreeConstraint->addPairwiseCheck(
      armCollisionGroup, envCollisionGroup);

  if (adaReal)
    ada.startTrajectoryExecutor();

  Perception perception(
      nodeHandle,
      "/camera/color/image_raw/compressed",
      "/camera/color/camera_info",
      true,
      5, // number of center points in width
      4, // number of center points in height
      0.012 // size of checkerboard in meter
      );
  tf::TransformListener tfListener;
  std::vector<Eigen::Isometry3d> targetPointsInCameraLensFrame;
  std::vector<Eigen::Isometry3d> cameraLensPointsInWorldFrame;

  // visualization
  aikido::rviz::WorldInteractiveMarkerViewer viewer(
      world, "dart_markers/cameraCalibration", "map");
  viewer.setAutoUpdate(true);
  auto frame1 = viewer.addFrame(
      ada.getMetaSkeleton()->getBodyNode("j2n6s200_end_effector"), 0.02, 0.002);
  auto frame2 = viewer.addFrame(
      ada.getMetaSkeleton()->getBodyNode("j2n6s200_hand_tip"), 0.02, 0.002);

  aikido::rviz::WorldInteractiveMarkerViewer jouleViewer(
      world, "dart_markers/cameraCalibration/cameraLens", "map");
  jouleViewer.setAutoUpdate(true);
  aikido::rviz::WorldInteractiveMarkerViewer targetPointViewer(
      world, "dart_markers/cameraCalibration/targetPoint", "map");
  targetPointViewer.setAutoUpdate(true);

  waitForUser("Startup complete.");

  ada.getHand()->executePreshape("closed").wait();

  waitForUser("Move to initial position");

  // ===== CALIBRATION PROCEDURE =====
  auto targetTSR = getCalibrationTSR(targetToWorld);
  dart::dynamics::SimpleFramePtr targetFrame = std::make_shared<dart::dynamics::SimpleFrame>(dart::dynamics::Frame::World(), "targetFrame", targetToWorld);
  auto targetFrameMarker = viewer.addFrame(targetFrame.get(), 0.07, 0.007);

  if (!ada.moveArmToTSR(targetTSR, collisionFreeConstraint, planningTimeout, maxNumTrials))
  {
    throw std::runtime_error("Trajectory execution failed");
  }
  waitForUser("Step 1 complete.");

  std::vector<dart::dynamics::SimpleFramePtr> frames;
  std::vector<aikido::rviz::FrameMarkerPtr> frameMarkers;

  /*
    // 20 - 56
  for (int i= 20; i<=56; i+=2) {
    double angle = 0.1745*i;
    auto tsr = getCalibrationTSR(robotPose.inverse() * createIsometry(
      0.0 + sin(angle)*0.1 + cos(angle)*-0.03,
      0.4 - cos(angle)*0.1 + sin(angle)*-0.03,
      0.05, 3.58, 0, angle));
    if (!ada.moveArmToTSR(tsr, collisionFreeConstraint,
        planningTimeout, maxNumTrials))

    {
      ROS_INFO_STREAM("Fail: Step " << i);
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      if (tryPerceivePoint("circle1_step" + std::to_string(i),
            perception, tfListener, jouleViewer, targetPointViewer,
            targetPointsInCameraLensFrame, cameraLensPointsInWorldFrame,
            frames, frameMarkers)) {
        ROS_INFO_STREAM("Success: Step " << i);
      } else {
        ROS_INFO_STREAM("Perception fail: Step " << i);
      }
    }
  }
  */


    // 20 - 56
  for (int i= 20; i<=56; i+=2) {
    double angle = 0.1745*i;
    auto tsr = getCalibrationTSR(robotPose.inverse() * createIsometry(
      0.0 + sin(angle)*0.1 + cos(angle)*-0.03,
      0.4 - cos(angle)*0.1 + sin(angle)*-0.03,
      0.1, 3.58, 0, angle));
    if (!ada.moveArmToTSR(tsr, collisionFreeConstraint,
        planningTimeout, maxNumTrials))

    {
      ROS_INFO_STREAM("Fail: Step " << i);
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      if (tryPerceivePoint("circle1_step" + std::to_string(i),
            perception, tfListener, jouleViewer, targetPointViewer,
            targetPointsInCameraLensFrame, cameraLensPointsInWorldFrame,
            frames, frameMarkers)) {
        ROS_INFO_STREAM("Success: Step " << i);
      } else {
        ROS_INFO_STREAM("Perception fail: Step " << i);
      }
    }
  }

  // ===== CALCULATE CALIBRATION =====
  ROS_INFO_STREAM("Got " << cameraToJouleEstimates.size() << " estimates.");
  ROS_INFO_STREAM("Solving for Mean CameraToJoule ");

  auto cameraToJoule = perception.computeMeanCameraToJouleEstimate(cameraToJouleEstimates);
  Eigen::Isometry3d worldToJoule = getWorldToJoule(tfListener);

  perception.visualizeProjection(targetToWorld, worldToJoule,
    getCameraToLens(tfListener),
    cameraToJoule);

  ROS_INFO_STREAM("Visualize final projection");

  for (int i= 20; i<=56; i+=15) {
    double angle = 0.1745*i;
    auto tsr = getCalibrationTSR(robotPose.inverse() * createIsometry(
      0.0 + sin(angle)*0.1 + cos(angle)*-0.03,
      0.4 - cos(angle)*0.1 + sin(angle)*-0.03,
      0.05, 3.58, 0, angle));

    if (!ada.moveArmToTSR(tsr, collisionFreeConstraint,
      planningTimeout, maxNumTrials))
    {
      ROS_INFO_STREAM("Fail: Step " << i);
    } else
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      Eigen::Isometry3d worldToJoule = getWorldToJoule(tfListener);
      perception.visualizeProjection(targetToWorld, worldToJoule,
        getCameraToLens(tfListener), cameraToJoule);
    }
  }

  waitForUser("Move back to center");

  if (!ada.moveArmToTSR(targetTSR, collisionFreeConstraint,
      planningTimeout, maxNumTrials))
  {
    throw std::runtime_error("Trajectory execution failed");
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(3000));

  // ===== DONE =====
  if (adaReal)
    ada.stopTrajectoryExecutor();

  waitForUser("Calibration finished.");
  ros::shutdown();
  return 0;
}
