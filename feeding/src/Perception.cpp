#include "feeding/Perception.hpp"
#include "feeding/util.hpp"
#include <aikido/perception/ObjectDatabase.hpp>

namespace feeding {

Perception::Perception(aikido::planner::WorldPtr world, ada::Ada& ada, ros::NodeHandle& nodeHandle) : world(world), nodeHandle(nodeHandle) {
  std::string detectorDataURI = getRosParam<std::string>("/perception/detectorDataUri", nodeHandle);
  std::string referenceFrameName = getRosParam<std::string>("/perception/referenceFrameName", nodeHandle);
  std::string detectorTopicName = getRosParam<std::string>("/perception/detectorTopicName", nodeHandle);

  const auto resourceRetriever
      = std::make_shared<aikido::io::CatkinResourceRetriever>();

  objDetector = std::unique_ptr<aikido::perception::PoseEstimatorModule>(new aikido::perception::PoseEstimatorModule(
      nodeHandle,
      detectorTopicName,
      std::make_shared<aikido::perception::ObjectDatabase>(
          resourceRetriever, detectorDataURI),
      resourceRetriever,
      referenceFrameName,
      aikido::robot::util::getBodyNodeOrThrow(
          *ada.getMetaSkeleton(), referenceFrameName)));
}

bool Perception::perceiveFood(Eigen::Isometry3d& foodTransform) {
  objDetector->detectObjects(world, ros::Duration(getRosParam<double>("/perception/timeoutSeconds", nodeHandle)));

  std::string perceivedFoodName = "apricot_1";
    auto perceivedFood = world->getSkeleton(perceivedFoodName);
    if (perceivedFood != nullptr)
    {
      foodTransform = Eigen::Isometry3d::Identity();
      foodTransform.translation() = perceivedFood->getJoint(0)
                                   ->getChildBodyNode()
                                   ->getTransform()
                                   .translation();
      return true; 
    } else {
      return false;
    }
}


}
