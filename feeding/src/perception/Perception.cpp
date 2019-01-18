#include "feeding/perception/Perception.hpp"
#include "feeding/util.hpp"

#include <algorithm>
#include <aikido/perception/ObjectDatabase.hpp>
#include <tf_conversions/tf_eigen.h>
#include <libada/util.hpp>

using ada::util::getRosParam;

namespace feeding {

//==============================================================================
Perception::Perception(
    aikido::planner::WorldPtr world,
    dart::dynamics::MetaSkeletonPtr adasMetaSkeleton,
    ros::NodeHandle nodeHandle)
  : mWorld(world)
  , mNodeHandle(nodeHandle)
  , mLastPerceivedFoodTransform(Eigen::Isometry3d::Identity())
  , mFaceZOffset(0.0)
{
  std::string detectorDataURI
      = getRosParam<std::string>("/perception/detectorDataUri", mNodeHandle);
  std::string referenceFrameName
      = getRosParam<std::string>("/perception/referenceFrameName", mNodeHandle);
  std::string foodDetectorTopicName = getRosParam<std::string>(
      "/perception/foodDetectorTopicName", mNodeHandle);
  std::string faceDetectorTopicName = getRosParam<std::string>(
      "/perception/faceDetectorTopicName", mNodeHandle);

  const auto resourceRetriever
      = std::make_shared<aikido::io::CatkinResourceRetriever>();

  mObjectDatabase = std::make_shared<aikido::perception::ObjectDatabase>(
      resourceRetriever, detectorDataURI);

  mFoodDetector = std::unique_ptr<aikido::perception::PoseEstimatorModule>(
      new aikido::perception::PoseEstimatorModule(
          mNodeHandle,
          foodDetectorTopicName,
          mObjectDatabase,
          resourceRetriever,
          referenceFrameName,
          aikido::robot::util::getBodyNodeOrThrow(
              *adasMetaSkeleton, referenceFrameName)));

  mFaceDetector = std::unique_ptr<aikido::perception::PoseEstimatorModule>(
      new aikido::perception::PoseEstimatorModule(
          mNodeHandle,
          faceDetectorTopicName,
          mObjectDatabase,
          resourceRetriever,
          referenceFrameName,
          aikido::robot::util::getBodyNodeOrThrow(
              *adasMetaSkeleton, referenceFrameName)));

  mPerceptionTimeout
      = getRosParam<double>("/perception/timeoutSeconds", mNodeHandle);
  mFoodNameToPerceive
      = getRosParam<std::string>("/perception/foodName", mNodeHandle);
  mPerceivedFaceName
      = getRosParam<std::string>("/perception/faceName", mNodeHandle);

  mLastPerceivedFoodTransform.translation().z() = 0.28;

  mFoodNames
      = getRosParam<std::vector<std::string>>("/foodItems/names", nodeHandle);
}

//==============================================================================
boost::optional<Eigen::Isometry3d> Perception::perceiveAllFood()
{
  double shortestDistance = 1.0; // 1m. 
  Eigen::Isometry3d foodTransform;
  bool perceiveDepthPlane = false;

  for (std::string foodName : mFoodNames)
  {
    Eigen::Isometry3d transform;
    auto detected = perceiveFood(foodName, perceiveDepthPlane);
    if (detected)
    {
      transform = detected.get();
      auto distance = getDistanceFromForque(detected.get());
      if (distance < shortestDistance)
      {
        shortestDistance = distance;
        foodTransform = transform;
      }
    }
  }

  if (shortestDistance >= 1)
    return boost::optional<Eigen::Isometry3d>{};
  else
    return foodTransform;
}


//==============================================================================
boost::optional<Eigen::Isometry3d> Perception::perceiveFood(
    const std::string& foodNameToFind,
    bool perceiveDepthPlane)
{
  mFoodDetector->detectObjects(mWorld, ros::Duration(mPerceptionTimeout));
  Eigen::Isometry3d cameraToWorldTransform = getCameraToWorldTransform();

  dart::dynamics::SkeletonPtr perceivedFood;
  Eigen::Isometry3d foodTransform;

  ROS_INFO("Looking for food items");

  double distFromForque = 10.0; // 1m
  std::string chosenFoodName("");
    
  for (int skeletonFrameIdx = 0; skeletonFrameIdx < 5; skeletonFrameIdx++)
  {
    for (std::size_t index = 0; ; ++index)
    {
      std::string currentFoodName = foodNameToFind + "_" + std::to_string(index)
                                    + "_" + std::to_string(skeletonFrameIdx);
      auto currentPerceivedFood = mWorld->getSkeleton(currentFoodName);
      if (!currentPerceivedFood)
      {
        // ROS_INFO_STREAM(currentFoodName << " not in world");
        if (index > 10)
          break;
        else
          continue;
      }

      ROS_INFO_STREAM(currentFoodName << " detected ");

      Eigen::Isometry3d currentFoodTransform
          = currentPerceivedFood->getBodyNode(0)->getWorldTransform();

      if (!perceiveDepthPlane)
      {
        Eigen::Vector3d start(currentFoodTransform.translation());
        Eigen::Vector3d end(getOpticalToWorld().translation());
        Eigen::ParametrizedLine<double, 3> line(
            start, (end - start).normalized());
        Eigen::Vector3d intersection = line.intersectionPoint(mDepthPlane);
        currentFoodTransform.translation() = intersection;
      }
      else
      {
        if (currentFoodTransform.translation().z() < 0.11)
        {
          ROS_INFO_STREAM(
              "discarding " << currentFoodName << " because of depth");
          continue;
        }
      }

      auto currentDistFromForque = getDistanceFromForque(currentFoodTransform);

      ROS_INFO_STREAM("Distance " << currentDistFromForque);
      if (currentDistFromForque < distFromForque)
      {
        foodTransform = currentFoodTransform;
        chosenFoodName = currentFoodName;
        distFromForque = currentDistFromForque;
      }
    }
  }

  if (distFromForque >= 1)
  {    
    ROS_WARN("food perception failed");
    return boost::optional<Eigen::Isometry3d>{};
  }

  foodTransform.linear()
      = cameraToWorldTransform.linear() * foodTransform.linear();
  foodTransform.linear() = getForqueTransform().linear() * foodTransform.linear();

  ROS_WARN_STREAM("Found " << chosenFoodName);
  // magic offsets
  if ((foodNameToFind.find("strawberry") == 0)
      || (foodNameToFind.find("cantaloupe") == 0))
  {
    foodTransform.translation() += Eigen::Vector3d(0.015, 0, 0);
  }

  ROS_INFO_STREAM("food perception successful for " << chosenFoodName);
  return foodTransform;
}

//==============================================================================
boost::optional<Eigen::Isometry3d> Perception::perceiveFace()
{

  bool detected
      = mFaceDetector->detectObjects(mWorld, ros::Duration(mPerceptionTimeout));
  if (!detected)
  {
    ROS_WARN("face perception failed");
    return boost::optional<Eigen::Isometry3d>{};
  }

  // just choose one for now
  for (int skeletonFrameIdx = 0; skeletonFrameIdx < 5; skeletonFrameIdx++)
  {
    auto perceivedFace = mWorld->getSkeleton(
        mPerceivedFaceName + "_" + std::to_string(skeletonFrameIdx));
    if (perceivedFace != nullptr)
    {
      auto faceTransform = perceivedFace->getBodyNode(0)->getWorldTransform();

      // fixed distance:
      double fixedFaceY
          = getRosParam<double>("/feedingDemo/fixedFaceY", mNodeHandle);
      if (fixedFaceY > 0)
      {
        faceTransform.translation().y() = fixedFaceY;
      }

      faceTransform.translation().z() += 0.00;
      // faceTransform.translation().z() += 0.0;
      faceTransform.translation().z()
          = faceTransform.translation().z() + mFaceZOffset;
      ROS_INFO_STREAM(
          "perceived Face: "
          << faceTransform.translation().matrix().transpose());
      return faceTransform;
    }
  }
  ROS_WARN("face perception failed");
  return boost::optional<Eigen::Isometry3d>{};
}

//==============================================================================
bool Perception::isMouthOpen()
{
  // return mObjectDatabase->mObjData["faceStatus"].as<bool>();
  ROS_WARN("Always returning true for isMouthOpen");
  return true;
}

//==============================================================================
Eigen::Isometry3d Perception::getOpticalToWorld()
{
  return getRelativeTransform(
        mTFListener,
        "/world",
        "/camera_color_optical_frame");
}

//==============================================================================
Eigen::Isometry3d Perception::getForqueTransform()
{
  return getRelativeTransform(
    mTFListener,
    "/map",
    "/j2n6s200_forque_end_effector");
}

//==============================================================================
Eigen::Isometry3d Perception::getCameraToWorldTransform()
{
  return getRelativeTransform(
    mTFListener,
    "/camera_color_optical_frame",
    "/map");
}

//==============================================================================
double Perception::getDistanceFromForque(const Eigen::Isometry3d& item)
{
  Eigen::Isometry3d forqueTransform = getForqueTransform();
  std::cout << "Forque position " << forqueTransform.translation().transpose() << std::endl;

  Eigen::Vector3d cameraDirection
      = getCameraToWorldTransform().linear() * Eigen::Vector3d(0, 0, 1);
  mDepthPlane = Eigen::Hyperplane<double, 3>(
      cameraDirection, item.translation());


  Eigen::Vector3d start(forqueTransform.translation());
  Eigen::Vector3d end(start + 
      forqueTransform.linear() * Eigen::Vector3d(0, 0, 1));
  Eigen::ParametrizedLine<double, 3> line(
      start, (end - start).normalized());
  Eigen::Vector3d intersection = line.intersectionPoint(mDepthPlane);
  
  return (item.translation() - intersection).norm();
}

} // namespace feeding
