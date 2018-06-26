
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

bool tryPerceivePoint(
        std::string frameName,
        Perception& perception,
        tf::TransformListener& tfListener,
        aikido::rviz::WorldInteractiveMarkerViewer& cameraLensViewer,
        aikido::rviz::WorldInteractiveMarkerViewer& targetPointViewer,
        std::vector<Eigen::Isometry3d>& targetPointsInCameraLensFrame,
        std::vector<Eigen::Isometry3d>& cameraLensPointsInWorldFrame,
        std::vector<dart::dynamics::SimpleFramePtr>& frames,
        std::vector<aikido::rviz::FrameMarkerPtr>& frameMarkers) {

  Eigen::Isometry3d perceivedTargetPoint;
  if (perception.getTargetTransformInCameraLensFrame(perceivedTargetPoint))
  {
    ROS_INFO_STREAM("perceived distance: " << perceivedTargetPoint.translation().norm());

    targetPointsInCameraLensFrame.push_back(perceivedTargetPoint);
    cameraLensPointsInWorldFrame.push_back(
        getCameraLensInWorldFrame(tfListener));

    Eigen::Isometry3d cameraLensTransform = getCameraLensInWorldFrame(tfListener);
    dart::dynamics::SimpleFramePtr cameraFrame = std::make_shared<dart::dynamics::SimpleFrame>(dart::dynamics::Frame::World(), "camLens_" + frameName, cameraLensTransform);
    frames.push_back(cameraFrame);
    frameMarkers.push_back(cameraLensViewer.addFrame(cameraFrame.get(), 0.07, 0.007));

    Eigen::Isometry3d targetPointTransform = getCameraLensInWorldFrame(tfListener) * perceivedTargetPoint;
    dart::dynamics::SimpleFramePtr targetFrame = std::make_shared<dart::dynamics::SimpleFrame>(dart::dynamics::Frame::World(), "perceivedTarget_" + frameName, targetPointTransform);
    frames.push_back(targetFrame);
    frameMarkers.push_back(targetPointViewer.addFrame(targetFrame.get(), 0.07, 0.007));

    // Eigen::Isometry3d cameraLensPerceivedTransform = perceivedTargetPoint.inverse();
    // dart::dynamics::SimpleFramePtr cameraFramePerceived = std::make_shared<dart::dynamics::SimpleFrame>(dart::dynamics::Frame::World(), "cameralensperceived_cirlce2_" + std::to_string(i), cameraLensPerceivedTransform);
    // frames.push_back(cameraFramePerceived);
    // frameMarkers.push_back(viewer.addFrame(cameraFramePerceived.get(), 0.07, 0.007));

    //std::cout << cameraLensTransform.matrix() << std::endl;
    return true;
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
  Eigen::Isometry3d robotPose = createIsometry(0.7, -0.1, -0.28, 0, 0, 3.1415);

  // Setting up workspace
  const auto resourceRetriever
      = std::make_shared<aikido::io::CatkinResourceRetriever>();
  Eigen::Isometry3d tablePose
      = robotPose.inverse() * createIsometry(0.76, 0.38, -0.745, 0, 0, 0);
  auto table = loadSkeletonFromURDF(
      resourceRetriever,
      "package://pr_ordata/data/furniture/table_feeding.urdf",
      tablePose);
  world->addSkeleton(table);
  Eigen::Isometry3d wheelchairPose = createIsometry(0, 0, 0, 0, 0, 0);
  auto wheelchair = loadSkeletonFromURDF(
      resourceRetriever,
      "package://pr_ordata/data/furniture/wheelchair.urdf",
      wheelchairPose);
  world->addSkeleton(wheelchair);
  Eigen::Isometry3d environmentPose
      = robotPose.inverse() * createIsometry(0, 0, 0, 0, 0, 0);
  auto environment = loadSkeletonFromURDF(
      resourceRetriever,
      "package://pr_ordata/data/furniture/workspace_feeding_demo.urdf",
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
  {
    ada.startTrajectoryExecutor();
  }

  Perception perception(
      nodeHandle,
      "/camera/color/image_raw/compressed",
      "/camera/color/camera_info",
      true,
      9,
      7,
      0.012111);
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

  aikido::rviz::WorldInteractiveMarkerViewer cameraLensViewer(
      world, "dart_markers/cameraCalibration/cameraLens", "map");
  cameraLensViewer.setAutoUpdate(true);
  aikido::rviz::WorldInteractiveMarkerViewer targetPointViewer(
      world, "dart_markers/cameraCalibration/targetPoint", "map");
  targetPointViewer.setAutoUpdate(true);

  waitForUser("Startup complete.");

  ada.getHand()->executePreshape("closed").wait();

  waitForUser("Move to initial position");

  // ===== CALIBRATION PROCEDURE =====
  Eigen::Isometry3d targetPointPose
      = robotPose.inverse() * createIsometry(.425, 0.15, -0.005, 3.1415, 0, 0);
  auto firstTSR = getCalibrationTSR(targetPointPose);
  // auto frame2 = viewer.addTSRMarker(firstTSR, 20);

  if (!moveArmToTSR(firstTSR, ada, collisionFreeConstraint, armSpace))
  {
    throw std::runtime_error("Trajectory execution failed");
  }
  waitForUser("Step 1 complete.");

  std::vector<dart::dynamics::SimpleFramePtr> frames;
  std::vector<aikido::rviz::FrameMarkerPtr> frameMarkers;


  for (int i= 20; i<=56; i++) {
    double angle = 0.1745*i;
    auto tsr = getCalibrationTSR(robotPose.inverse() * createIsometry(
      0.425 + sin(angle)*0.1 + cos(angle)*-0.05,
      0.15 - cos(angle)*0.1 + sin(angle)*-0.05,
      0.05, 3.58, 0, angle)); if
    (!moveArmToTSR(tsr, ada, collisionFreeConstraint, armSpace))
    {
      ROS_INFO_STREAM("Fail: Step " << i);
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1500));
      if (tryPerceivePoint("circle1_step" + std::to_string(i),
            perception, tfListener, cameraLensViewer, targetPointViewer,
            targetPointsInCameraLensFrame, cameraLensPointsInWorldFrame,
            frames, frameMarkers)) {
        ROS_INFO_STREAM("Success: Step " << i);
      } else {
        ROS_INFO_STREAM("Perception fail: Step " << i);
      }
    }
  }

  for (int i = 20; i <= 56; i++)
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
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      if (tryPerceivePoint("circle2_step" + std::to_string(i),
            perception, tfListener, cameraLensViewer, targetPointViewer,
            targetPointsInCameraLensFrame, cameraLensPointsInWorldFrame,
            frames, frameMarkers)) {
        ROS_INFO_STREAM("Success: Step " << i);
      } else {
        ROS_INFO_STREAM("Perception fail: Step " << i);
      }
    }
  }

  // ===== CALCULATE CALIBRATION =====
  std::vector<Eigen::Isometry3d> differences;
  for (int i = 0; i < targetPointsInCameraLensFrame.size(); i++)
  {
    Eigen::Isometry3d cameraLensPointInWorldFrame2
        = targetPointsInCameraLensFrame[i].inverse() * targetPointPose;
    Eigen::Isometry3d difference = cameraLensPointsInWorldFrame[i]
                                   * cameraLensPointInWorldFrame2.inverse();
    differences.push_back(difference);
    printPose(difference);
  }


  // ===== DONE =====
  if (adaReal)
  {
    ada.stopTrajectoryExecutor();
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(3000));
  waitForUser("Calibration finished.");
  ros::shutdown();
  return 0;
}
