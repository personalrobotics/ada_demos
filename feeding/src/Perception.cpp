#include "feeding/Perception.hpp"
#include "feeding/util.hpp"

#include <algorithm>
#include <aikido/perception/ObjectDatabase.hpp>
#include <tf_conversions/tf_eigen.h>
#include <libada/util.hpp>

using ada::util::getRosParam;

namespace {

//==============================================================================
Eigen::Isometry3d getOpticalToWorld(const tf::TransformListener& tfListener)
{
  tf::StampedTransform tfStampedTransform;
  try
  {
    tfListener.lookupTransform(
        "/world",
        "/camera_color_optical_frame",
        ros::Time(0),
        tfStampedTransform);
  }
  catch (tf::TransformException ex)
  {
    throw std::runtime_error(
        "Failed to get TF Transform: " + std::string(ex.what()));
  }
  Eigen::Isometry3d cameraLensPointInWorldFrame;
  tf::transformTFToEigen(tfStampedTransform, cameraLensPointInWorldFrame);
  return cameraLensPointInWorldFrame;
}

//==============================================================================
Eigen::Isometry3d getForqueTransform(const tf::TransformListener& tfListener)
{
  tf::StampedTransform tfStampedTransform;
  try
  {
    tfListener.lookupTransform(
        "/map",
        "/j2n6s200_forque_end_effector",
        ros::Time(0),
        tfStampedTransform);
  }
  catch (tf::TransformException ex)
  {
    throw std::runtime_error(
        "Failed to get TF Transform: " + std::string(ex.what()));
  }
  Eigen::Isometry3d forqueTransformInWorldFrame;
  tf::transformTFToEigen(tfStampedTransform, forqueTransformInWorldFrame);
  return forqueTransformInWorldFrame;
}

//==============================================================================
Eigen::Isometry3d getCameraToWorldTransform(
    const tf::TransformListener& tfListener)
{
  tf::StampedTransform tfStampedTransform;
  try
  {
    tfListener.lookupTransform(
        "/camera_color_optical_frame",
        "/map",
        ros::Time(0),
        tfStampedTransform);
  }
  catch (tf::TransformException ex)
  {
    throw std::runtime_error(
        "Failed to get TF Transform: " + std::string(ex.what()));
  }
  Eigen::Isometry3d forqueTransformInWorldFrame;
  tf::transformTFToEigen(tfStampedTransform, forqueTransformInWorldFrame);
  return forqueTransformInWorldFrame;
}
}

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
bool Perception::perceiveFood(Eigen::Isometry3d& foodTransform)
{
  return perceiveFood(foodTransform, false, nullptr);
}

//==============================================================================
bool Perception::perceiveFood(
    Eigen::Isometry3d& foodTransform,
    bool perceiveDepthPlane,
    aikido::rviz::WorldInteractiveMarkerViewerPtr viewer,
    const std::string& foundFoodName,
    bool perceiveAnyFood)
{
  mFoodDetector->detectObjects(mWorld, ros::Duration(mPerceptionTimeout));
  Eigen::Isometry3d forqueTransform = getForqueTransform(mTFListener);
  Eigen::Isometry3d cameraToWorldTransform
      = getCameraToWorldTransform(mTFListener);

  dart::dynamics::SkeletonPtr perceivedFood;

  ROS_INFO("Looking for food items");

  double distFromForque = -1;
  std::string fullFoodName;

  std::vector<std::string> fullFoodNames;
  std::vector<Eigen::Isometry3d> foodTransforms;

  int chosenIndex = -1;
  for (std::string foodName : mFoodNames)
  {
    if (!perceiveAnyFood && mFoodNameToPerceive != foodName)
    {
      ROS_INFO_STREAM(
          mFoodNameToPerceive << "  " << foodName << "  " << perceiveAnyFood);
      continue;
    }

    for (int skeletonFrameIdx = 0; skeletonFrameIdx < 5; skeletonFrameIdx++)
    {
      int index = 0;
      while (true)
      {
        index++;
        std::string currentFoodName = foodName + "_" + std::to_string(index)
                                      + "_" + std::to_string(skeletonFrameIdx);
        auto currentPerceivedFood = mWorld->getSkeleton(currentFoodName);

        if (!currentPerceivedFood)
        {
          ROS_INFO_STREAM(currentFoodName << " not in aikido world");
          if (index > 10)
          {
            break;
          }
          else
          {
            continue;
          }
        }
        Eigen::Isometry3d currentFoodTransform
            = currentPerceivedFood->getBodyNode(0)->getWorldTransform();

        if (!perceiveDepthPlane)
        {
          // ROS_WARN_STREAM("Food transform before: " <<
          // currentFoodTransform.translation().matrix().transpose());
          Eigen::Vector3d start(currentFoodTransform.translation());
          Eigen::Vector3d end(getOpticalToWorld(mTFListener).translation());
          Eigen::ParametrizedLine<double, 3> line(
              start, (end - start).normalized());
          Eigen::Vector3d intersection = line.intersectionPoint(depthPlane);
          currentFoodTransform.translation() = intersection;
          // ROS_WARN_STREAM("Food transform after: " <<
          // currentFoodTransform.translation().matrix().transpose());
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

        double currentDistFromForque = 0;
        {
          Eigen::Vector3d start(forqueTransform.translation());
          Eigen::Vector3d end(
              forqueTransform.translation()
              + forqueTransform.linear() * Eigen::Vector3d(0, 0, 1));
          Eigen::ParametrizedLine<double, 3> line(
              start, (end - start).normalized());
          Eigen::Vector3d intersection = line.intersectionPoint(depthPlane);
          currentDistFromForque
              = (currentFoodTransform.translation() - intersection).norm();
        }

        fullFoodNames.push_back(currentFoodName);
        foodTransforms.push_back(currentFoodTransform);

        ROS_INFO_STREAM(
            "dist from forque: " << currentDistFromForque << ", "
                                 << currentFoodName);
        if (chosenIndex < 0 || currentDistFromForque < distFromForque)
        {
          distFromForque = currentDistFromForque;
          chosenIndex = fullFoodNames.size() - 1;
        }
      }
    }
  }

  // static std::default_random_engine generator(time(0));
  // static std::uniform_real_distribution<double> distribution(0,1);
  // int chosenIndex = (int) std::min((distribution(generator) *
  // fullFoodNames.size()), (double) fullFoodNames.size()-1);
  // ROS_INFO_STREAM("random index: " << chosenIndex << ", size: " <<
  // fullFoodNames.size());

  if (fullFoodNames.size() > 0)
  {
    foodTransform = foodTransforms[chosenIndex];
    fullFoodName = fullFoodNames[chosenIndex];

    double distFromLastTransform = (mLastPerceivedFoodTransform.translation()
                                    - foodTransform.translation())
                                       .norm();
    if (!perceiveDepthPlane && distFromLastTransform > 0.05)
    {
      ROS_WARN("food transform too far from last one!");
      return false;
    }

    foodTransform.linear()
        = cameraToWorldTransform.linear() * foodTransform.linear();
    // dart::dynamics::SimpleFramePtr foodFrame =
    // std::make_shared<dart::dynamics::SimpleFrame>(dart::dynamics::Frame::World(),
    // "foodFrame", foodTransform);
    // frames.push_back(foodFrame);
    // frameMarkers.push_back(viewer->addFrame(foodFrame.get(), 0.07, 0.007));
    foodTransform.linear() = forqueTransform.linear() * foodTransform.linear();
    // ROS_WARN_STREAM("Food transform: " <<
    // foodTransform.translation().matrix());

    if (perceiveDepthPlane)
    {
      Eigen::Vector3d cameraDirection
          = cameraToWorldTransform.linear() * Eigen::Vector3d(0, 0, 1);
      depthPlane = Eigen::Hyperplane<double, 3>(
          cameraDirection, foodTransform.translation());
    }

    // magic offset to fix bananas
    // -0.008
    if ((fullFoodName.find("strawberry") == 0)
        || (fullFoodName.find("cantaloupe") == 0))
    {
      ROS_WARN("Strawberry Found!");
      foodTransform.translation() += Eigen::Vector3d(0.015, 0, 0);
    }
    else
    {
      ROS_WARN("Food Name:");
      ROS_WARN_STREAM(fullFoodName);
    }

    mLastPerceivedFoodTransform = foodTransform;
    // Eigen::Vector3d foodTranslation = foodTransform.translation();
    // foodTranslation += Eigen::Vector3d(0,0,-0.01);
    // foodTransform.translation() = foodTranslation;
    ROS_INFO_STREAM("food perception successful for " << fullFoodName);
    return true;
  }
  else
  {
    ROS_WARN("food perception failed");
    return false;
  }
}

//==============================================================================
bool Perception::perceiveFace(Eigen::Isometry3d& faceTransform)
{
  bool detected
      = mFaceDetector->detectObjects(mWorld, ros::Duration(mPerceptionTimeout));
  if (!detected)
  {
    ROS_WARN("face perception failed");
    return false;
  }

  // just choose one for now
  for (int skeletonFrameIdx = 0; skeletonFrameIdx < 5; skeletonFrameIdx++)
  {
    auto perceivedFace = mWorld->getSkeleton(
        mPerceivedFaceName + "_" + std::to_string(skeletonFrameIdx));
    if (perceivedFace != nullptr)
    {
      faceTransform = perceivedFace->getBodyNode(0)->getWorldTransform();

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
      return true;
    }
  }
  ROS_WARN("face perception failed");
  return false;
}

//==============================================================================
bool Perception::setFoodName(std::string foodName)
{
  std::vector<std::string> foodNames
      = getRosParam<std::vector<std::string>>("/foodItems/names", mNodeHandle);
  if (std::find(foodNames.begin(), foodNames.end(), foodName)
      != foodNames.end())
  {
    mFoodNameToPerceive = foodName;
    return true;
  }
  return false;
}

//==============================================================================
bool Perception::isMouthOpen()
{
  // return mObjectDatabase->mObjData["faceStatus"].as<bool>();
  ROS_WARN("Always returning true for isMouthOpen");
  return true;
}

} // namespace feeding
