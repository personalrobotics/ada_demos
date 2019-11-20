
#include <aikido/io/CatkinResourceRetriever.hpp>
#include <aikido/io/util.hpp>
#include <aikido/planner/World.hpp>
#include <aikido/rviz/InteractiveMarkerViewer.hpp>
#include <dart/dynamics/Frame.hpp>
#include <pr_tsr/plate.hpp>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <libada/Ada.hpp>
#include "cameraCalibration/Perception.hpp"
#include "cameraCalibration/util.hpp"

using namespace cameraCalibration;

// Robot To World
static const Eigen::Isometry3d robotPose = createIsometry(
  0.7, -0.1, -0.25, 0, 0, 3.1415);

// TargetToWorld = RobotToWorld * TargetToRobot
static const Eigen::Isometry3d targetToWorld = robotPose.inverse() * createIsometry(
  .425, 0.15, -0.005, 3.1415, 0, 0);
std::vector<Eigen::Isometry3d> cameraToJouleEstimates;

bool tryPerceivePoint(
        std::string frameName,
        Perception& perception,
        tf::TransformListener& tfListener,
        aikido::rviz::InteractiveMarkerViewer& jouleViewer,
        aikido::rviz::InteractiveMarkerViewer& targetPointViewer,
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
  auto armSpace
      = std::make_shared<aikido::statespace::dart::MetaSkeletonStateSpace>(
          ada.getArm()->getMetaSkeleton().get());

  // Setting up workspace
  const auto resourceRetriever
      = std::make_shared<aikido::io::CatkinResourceRetriever>();
  Eigen::Isometry3d tablePose
      = robotPose.inverse() * createIsometry(0.76, 0.38, -0.745, 0, 0, 0);
  auto table = loadSkeletonFromURDF(
      resourceRetriever,
      "package://pr_assets/data/furniture/table_feeding.urdf",
      tablePose);
  world->addSkeleton(table);
  Eigen::Isometry3d wheelchairPose = createIsometry(0, 0, 0, 0, 0, 0);
  auto wheelchair = loadSkeletonFromURDF(
      resourceRetriever,
      "package://pr_assets/data/furniture/wheelchair.urdf",
      wheelchairPose);
  world->addSkeleton(wheelchair);
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
          table.get(), wheelchair.get(), environment.get());
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
      5,
      4,
      0.0215);
  tf::TransformListener tfListener;
  std::vector<Eigen::Isometry3d> targetPointsInCameraLensFrame;
  std::vector<Eigen::Isometry3d> cameraLensPointsInWorldFrame;

  // visualization
  aikido::rviz::InteractiveMarkerViewer viewer(
      "dart_markers/cameraCalibration", "map", world);
  viewer.setAutoUpdate(true);
  auto frame1 = viewer.addFrame(
      ada.getMetaSkeleton()->getBodyNode("j2n6s200_end_effector"), 0.02, 0.002);
  auto frame2 = viewer.addFrame(
      ada.getMetaSkeleton()->getBodyNode("j2n6s200_hand_tip"), 0.02, 0.002);

  aikido::rviz::InteractiveMarkerViewer jouleViewer(
      "dart_markers/cameraCalibration/cameraLens", "map", world);
  jouleViewer.setAutoUpdate(true);
  aikido::rviz::InteractiveMarkerViewer targetPointViewer(
      "dart_markers/cameraCalibration/targetPoint", "map", world);
  targetPointViewer.setAutoUpdate(true);

  waitForUser("Startup complete.");

  ada.getHand()->executePreshape("closed").wait();

  waitForUser("Move to initial position");

  // ===== CALIBRATION PROCEDURE =====
  Eigen::Isometry3d targetPointPose
      = robotPose.inverse() * createIsometry(.425, 0.15, -0.005, 3.1415, 0, 0);
  auto targetTSR = getCalibrationTSR(targetPointPose);
  dart::dynamics::SimpleFramePtr targetFrame = std::make_shared<dart::dynamics::SimpleFrame>(dart::dynamics::Frame::World(), "targetFrame", targetPointPose);
  auto targetFrameMarker = viewer.addFrame(targetFrame.get(), 0.07, 0.007);

  if (!moveArmToTSR(targetTSR, ada, collisionFreeConstraint, armSpace))
  {
    throw std::runtime_error("Trajectory execution failed");
  }
  waitForUser("Step 1 complete.");

  std::vector<dart::dynamics::SimpleFramePtr> frames;
  std::vector<aikido::rviz::FrameMarkerPtr> frameMarkers;

    // 20 - 56
  for (int i= 20; i<=56; i+=5) {
    double angle = 0.1745*i;
    auto tsr = getCalibrationTSR(robotPose.inverse() * createIsometry(
      0.425 + sin(angle)*0.1 + cos(angle)*-0.03,
      0.15 - cos(angle)*0.1 + sin(angle)*-0.03,
      0.05, 3.58, 0, angle));
    if
    (!moveArmToTSR(tsr, ada, collisionFreeConstraint, armSpace))
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

  for (int i = 20; i <= 56; i+=5)
  {
    double angle = 0.1745 * i;
    auto tsr = getCalibrationTSR(
        robotPose.inverse()
        * createIsometry(
              .425 + sin(angle) * 0.2,
              0.15 - cos(angle) * 0.2,
              0.1,
              3.98,
              0,
              angle));
    if (!moveArmToTSR(tsr, ada, collisionFreeConstraint, armSpace))
    {
      ROS_INFO_STREAM("Fail: Step " << i);
    }
    else
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      if (tryPerceivePoint("circle2_step" + std::to_string(i),
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
  auto cameraToJoule = perception.computeMeanCameraToJouleEstimate(cameraToJouleEstimates);
  Eigen::Isometry3d worldToJoule = getWorldToJoule(tfListener);
  perception.visualizeProjection(targetToWorld, worldToJoule,
    getCameraToLens(tfListener),
    cameraToJoule);

  ROS_INFO_STREAM("Visualize final projection");

  for (int i= 20; i<=56; i+=15) {
    double angle = 0.1745*i;
    auto tsr = getCalibrationTSR(robotPose.inverse() * createIsometry(
      0.425 + sin(angle)*0.1 + cos(angle)*-0.03,
      0.15 - cos(angle)*0.1 + sin(angle)*-0.03,
      0.05, 3.58, 0, angle));

    if (!moveArmToTSR(tsr, ada, collisionFreeConstraint, armSpace))
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

  if (!moveArmToTSR(targetTSR, ada, collisionFreeConstraint, armSpace))
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
