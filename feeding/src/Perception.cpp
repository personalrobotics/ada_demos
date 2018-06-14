#include "feeding/Perception.hpp"
#include <aikido/perception/ObjectDatabase.hpp>
#include "feeding/util.hpp"

namespace feeding {

Perception::Perception(
    aikido::planner::WorldPtr world, ada::Ada& ada, ros::NodeHandle& nodeHandle)
  : world(world), nodeHandle(nodeHandle)
{
  std::string detectorDataURI
      = getRosParam<std::string>("/perception/detectorDataUri", nodeHandle);
  std::string referenceFrameName
      = getRosParam<std::string>("/perception/referenceFrameName", nodeHandle);
  std::string detectorTopicName
      = getRosParam<std::string>("/perception/detectorTopicName", nodeHandle);

  const auto resourceRetriever
      = std::make_shared<aikido::io::CatkinResourceRetriever>();

  objDetector = std::unique_ptr<aikido::perception::PoseEstimatorModule>(
      new aikido::perception::PoseEstimatorModule(
          nodeHandle,
          detectorTopicName,
          std::make_shared<aikido::perception::ObjectDatabase>(
              resourceRetriever, detectorDataURI),
          resourceRetriever,
          referenceFrameName,
          aikido::robot::util::getBodyNodeOrThrow(
              *ada.getMetaSkeleton(), referenceFrameName)));
}

bool Perception::perceiveFoodClosest(Eigen::Isometry3d& foodTransform, Eigen::Isometry3d forqueTip)
{
  objDetector->detectObjects(
      world,
      ros::Duration(
          getRosParam<double>("/perception/timeoutSeconds", nodeHandle)));

  // choose apricot closest to forque end effector
  std::vector<dart::dynamics::MetaSkeletonPtr> skeletons;
  for (int i=0; i<world->getNumSkeletons(); i++) {
    dart::dynamics::MetaSkeletonPtr skeleton = world->getSkeleton(i);
    if (skeleton->getName().find("apricot") == 0) {
      skeletons.push_back(skeleton);
    }
  }

  if (skeletons.empty()) {
      return false;
  }

  dart::dynamics::MetaSkeletonPtr closestSkeleton = nullptr;
  double closestDistance;
  for (auto& skeleton : skeletons) {
    if (skeleton->getNumBodyNodes() == 0) {continue;}
    double distance = (skeleton->getBodyNode(0)->getWorldTransform().translation() - forqueTip.translation()).norm();
    if (distance < closestDistance) {
      closestDistance = distance;
      closestSkeleton = skeleton;
    }
  }

  if (!closestSkeleton) {
      return false;
  }

  foodTransform = closestSkeleton->getBodyNode(0)->getWorldTransform();
  return true;
}

}
