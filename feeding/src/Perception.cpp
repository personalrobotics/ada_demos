#include "feeding/Perception.hpp"
#include <aikido/perception/ObjectDatabase.hpp>
#include <tf_conversions/tf_eigen.h>
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
}

Eigen::Isometry3d Perception::getOpticalToWorld()
{
  tf::StampedTransform tfStampedTransform;
  try
  {
    mTFListener.lookupTransform(
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

Eigen::Isometry3d Perception::getForqueTransform()
{
  tf::StampedTransform tfStampedTransform;
  try
  {
    mTFListener.lookupTransform(
        "/world",
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

bool Perception::setFoodName(std::string foodName) {
  std::vector<std::string> foodNames = getRosParam<std::vector<std::string>>("/foodItems/names", mNodeHandle);
    if (std::find(foodNames.begin(), foodNames.end(), foodName) != foodNames.end()) {
        mFoodNameToPerceive = foodName;
        return true;
    }
    return false;
}

//==============================================================================
bool Perception::perceiveFood(Eigen::Isometry3d& foodTransform)
{
  return perceiveFood(foodTransform, true);
}

//==============================================================================
bool Perception::perceiveFood(
    Eigen::Isometry3d& foodTransform, bool onlyPerceiveFoodRightBelow)
{

  //   double ms = (std::chrono::duration_cast< std::chrono::milliseconds >(
  //     std::chrono::system_clock::now().time_since_epoch()).count() % 10000);
  //   double xdiff = ms / 5000.0;
  //   double ydiff = xdiff + 0.5;
  //   if (ydiff > 2) {ydiff -= 2;}
  //   if (xdiff > 1) {xdiff = 2-xdiff;}
  //   if (ydiff > 1) {ydiff = 2-ydiff;}
  //   xdiff *= 0.3;
  //   ydiff *= 0.08;
  // //   ROS_INFO_STREAM("xdiff: " << xdiff << ",  ydiff: " << ydiff);
  //   foodTransform = createIsometry(0.1 + xdiff, -0.25 + ydiff , 0.3, 0, 0,
  //   0);
  // //   ROS_INFO_STREAM("transform: " << foodTransform.matrix());
  //   return true;

  mFoodDetector->detectObjects(mWorld, ros::Duration(mPerceptionTimeout));
  Eigen::Isometry3d forqueTransform = getForqueTransform();

  dart::dynamics::SkeletonPtr perceivedFood;

  ROS_INFO("Looking for food items");

  double distFromForque = -1;
  double diffNorm;
//   for (std::string perceivedFoodName : foodNames)
//   {
    int index = 1;
    while (true)
    {
      std::string currentFoodName
          = mFoodNameToPerceive + "_" + std::to_string(index);
      auto currentPerceivedFood = mWorld->getSkeleton(currentFoodName);

      if (!currentPerceivedFood)
      {
        break;
      }
      Eigen::Isometry3d currentFoodTransform;
      currentFoodTransform.setIdentity();
      currentFoodTransform.translation() = currentPerceivedFood->getBodyNode(0)
                                               ->getWorldTransform()
                                               .translation();

      Eigen::Vector3d diffVector = currentFoodTransform.translation()
                                   - forqueTransform.translation()
                                   - Eigen::Vector3d(0, 0, 0.07);
      diffVector *= 1.0 / diffVector.z();

      double currentDistFromForque
          = (currentFoodTransform.translation() - forqueTransform.translation())
                .norm();

      ROS_INFO_STREAM(
          currentFoodName << " current dist from forque: "
                          << currentDistFromForque
                          << "    diff: ("
                          << diffVector.x()
                          << ", "
                          << diffVector.y()
                          << ", "
                          << diffVector.z()
                          << +") norm: "
                          << diffVector.norm());

      if (distFromForque < 0 || currentDistFromForque < distFromForque)
      {
        distFromForque = currentDistFromForque;
        foodTransform = currentFoodTransform;
        perceivedFood = currentPerceivedFood;
        diffNorm = diffVector.norm();
      }
      index++;
    }
//   }

  if (perceivedFood != nullptr)
  {
    if (onlyPerceiveFoodRightBelow && diffNorm > 1.05 && false)
    {
      ROS_WARN_STREAM(
          "discarding perceived food because diffNorm " << diffNorm
                                                        << " too big");
      return false;
    }

    foodTransform.setIdentity();
    foodTransform.translation()
        = perceivedFood->getBodyNode(0)->getWorldTransform().translation();
    //ROS_WARN_STREAM("Food transform: " << foodTransform.matrix());

    if (foodTransform.translation().z() < 0.26 || true)
    {
      Eigen::Vector3d start(foodTransform.translation());
      Eigen::Vector3d end(getOpticalToWorld().translation());
      Eigen::ParametrizedLine<double, 3> line(
          start, (end - start).normalized());
      // TODO(daniel): Rotation needs to be adjusted if camera doesn't point
      // straight downwards
      Eigen::Hyperplane<double, 3> plane(
          /*mLastPerceivedFoodTransform.linear() */ Eigen::Vector3d(0, 0, 1),
          // Eigen::Vector3d(mLastPerceivedFoodTransform.translation()));
        //   Eigen::Vector3d(0, 0, 0.25));
             Eigen::Vector3d(0, 0, 0.268));
      Eigen::Vector3d intersection = line.intersectionPoint(plane);
      foodTransform.translation() = intersection;
      //   ROS_INFO_STREAM("start: " << start.matrix());
      //   ROS_INFO_STREAM("end: " << end.matrix());
      //   ROS_INFO_STREAM("normal: " << (mLastPerceivedFoodTransform.linear() *
      //   Eigen::Vector3d(0,0,1)).matrix());
      //   ROS_INFO_STREAM("planePoint: " <<
      //   Eigen::Vector3d(mLastPerceivedFoodTransform.translation().matrix()));
      //   ROS_INFO_STREAM("intersectionPoint: " << intersection.matrix());
    }

    // mLastPerceivedFoodTransform = foodTransform;
    // Eigen::Vector3d foodTranslation(foodTransform.translation().x(),
    // foodTransform.translation().y(), foodTransform.translation().z() - 0.02);
    // foodTransform.translation() = foodTranslation;
    return true;
  }
  else
  {
    return false;
  }
}

bool Perception::perceiveFace(Eigen::Isometry3d& faceTransform)
{
  mFaceDetector->detectObjects(mWorld, ros::Duration(mPerceptionTimeout));

  // just choose one for now
  auto perceivedFace = mWorld->getSkeleton(mPerceivedFaceName);
  if (perceivedFace != nullptr)
  {
    faceTransform = perceivedFace->getBodyNode(0)->getWorldTransform();
    ROS_INFO_STREAM("perceived Face: " << faceTransform.matrix());
    return true;
  }
  else
  {
    return false;
  }
}

bool Perception::isMouthOpen()
{
  // return mObjectDatabase->mObjData["faceStatus"].as<bool>();
  return true;
}
}
