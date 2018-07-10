#include "feeding/Perception.hpp"
#include <aikido/perception/ObjectDatabase.hpp>
#include "feeding/util.hpp"

namespace feeding {

//==============================================================================
Perception::Perception(
    aikido::planner::WorldPtr world,
    dart::dynamics::MetaSkeletonPtr adasMetaSkeleton,
    ros::NodeHandle nodeHandle)
  : mWorld(world), mNodeHandle(nodeHandle)
{
  std::string detectorDataURI
      = getRosParam<std::string>("/perception/detectorDataUri", mNodeHandle);
  std::string referenceFrameName
      = getRosParam<std::string>("/perception/referenceFrameName", mNodeHandle);
  std::string detectorTopicName
      = getRosParam<std::string>("/perception/detectorTopicName", mNodeHandle);

  const auto resourceRetriever
      = std::make_shared<aikido::io::CatkinResourceRetriever>();

  mObjDetector = std::unique_ptr<aikido::perception::PoseEstimatorModule>(
      new aikido::perception::PoseEstimatorModule(
          mNodeHandle,
          detectorTopicName,
          std::make_shared<aikido::perception::ObjectDatabase>(
              resourceRetriever, detectorDataURI),
          resourceRetriever,
          referenceFrameName,
          aikido::robot::util::getBodyNodeOrThrow(
              *adasMetaSkeleton, referenceFrameName)));
}

//==============================================================================
bool Perception::perceiveFood(Eigen::Isometry3d& foodTransform)
{

  double ms = (std::chrono::duration_cast< std::chrono::milliseconds >(
    std::chrono::system_clock::now().time_since_epoch()).count() % 10000);
  double xdiff = ms / 5000.0;
  double ydiff = xdiff + 0.5;
  if (ydiff > 2) {ydiff -= 2;}
  if (xdiff > 1) {xdiff = 2-xdiff;}
  if (ydiff > 1) {ydiff = 2-ydiff;}
  xdiff *= 0.3;
  ydiff *= 0.08;
//   ROS_INFO_STREAM("xdiff: " << xdiff << ",  ydiff: " << ydiff);
  foodTransform = createIsometry(0.1 + xdiff, -0.25 + ydiff , 0.3, 0, 0, 0);
//   ROS_INFO_STREAM("transform: " << foodTransform.matrix());

  return true;
    /*
  mObjDetector->detectObjects(
      mWorld,
      ros::Duration(
          getRosParam<double>("/perception/timeoutSeconds", mNodeHandle)));

  // just choose one for now
  std::string perceivedFoodName
      = getRosParam<std::string>("/perception/foodName", mNodeHandle);
  auto perceivedFood = mWorld->getSkeleton(perceivedFoodName);
  if (perceivedFood != nullptr)
  {
    foodTransform = Eigen::Isometry3d::Identity();
    foodTransform.translation()
        = perceivedFood->getBodyNode(0)->getWorldTransform().translation();
    return true;
  }
  else
  {
    return false;
  }
  */
}
}
