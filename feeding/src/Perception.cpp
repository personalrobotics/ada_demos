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
  std::string detectorTopicName
      = getRosParam<std::string>("/perception/detectorTopicName", mNodeHandle);

  const auto resourceRetriever
      = std::make_shared<aikido::io::CatkinResourceRetriever>();

  mObjectDatabase = std::make_shared<aikido::perception::ObjectDatabase>(resourceRetriever, detectorDataURI);

  mObjDetector = std::unique_ptr<aikido::perception::PoseEstimatorModule>(
      new aikido::perception::PoseEstimatorModule(
          mNodeHandle,
          detectorTopicName,
          mObjectDatabase,
          resourceRetriever,
          referenceFrameName,
          aikido::robot::util::getBodyNodeOrThrow(
              *adasMetaSkeleton, referenceFrameName)));

  mPerceptionTimeout = getRosParam<double>("/perception/timeoutSeconds", mNodeHandle);
  mPerceivedFoodName = getRosParam<std::string>("/perception/foodName", mNodeHandle);
  mPerceivedFaceName = getRosParam<std::string>("/perception/faceName", mNodeHandle);
}


Eigen::Isometry3d getOpticalToWorld(tf::TransformListener& tfListener) {
  tf::StampedTransform tfStampedTransform;
  try{
    tfListener.lookupTransform("/world", "/camera_color_optical_frame",
                            ros::Time(0), tfStampedTransform);
  }
  catch (tf::TransformException ex){
    throw std::runtime_error("Failed to get TF Transform: " + std::string(ex.what()));
  }
  Eigen::Isometry3d cameraLensPointInWorldFrame;
  tf::transformTFToEigen(tfStampedTransform, cameraLensPointInWorldFrame);
  return cameraLensPointInWorldFrame;
}

//==============================================================================
bool Perception::perceiveFood(Eigen::Isometry3d& foodTransform)
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
//   foodTransform = createIsometry(0.1 + xdiff, -0.25 + ydiff , 0.3, 0, 0, 0);
// //   ROS_INFO_STREAM("transform: " << foodTransform.matrix());
//   return true;


  tf::TransformListener tfListener;

  mObjDetector->detectObjects(
      mWorld,
      ros::Duration(mPerceptionTimeout));


  // just choose one for now
  auto perceivedFood = mWorld->getSkeleton(mPerceivedFoodName);
  if (perceivedFood != nullptr)
  {
    foodTransform = Eigen::Isometry3d::Identity();
    foodTransform.translation() = perceivedFood->getBodyNode(0)->getWorldTransform().translation();

    if (foodTransform.translation().z() < 0.26) {
      ROS_WARN("Food below table!");
      Eigen::Vector3d start(foodTransform.translation());
      Eigen::Vector3d end(getOpticalToWorld(tfListener).translation());
      Eigen::ParametrizedLine<double,3> line(start, (end-start).normalized());
      // TODO(daniel): Rotation needs to be adjusted if camera doesn't point straight downwards
      Eigen::Hyperplane<double,3> plane(mLastPerceivedFoodTransform.linear() * Eigen::Vector3d(0,0,1), Eigen::Vector3d(mLastPerceivedFoodTransform.translation()));
      Eigen::Vector3d intersection = line.intersectionPoint(plane);
      foodTransform.translation() = intersection;
    //   ROS_INFO_STREAM("start: " << start.matrix());
    //   ROS_INFO_STREAM("end: " << end.matrix());
    //   ROS_INFO_STREAM("normal: " << (mLastPerceivedFoodTransform.linear() * Eigen::Vector3d(0,0,1)).matrix());
    //   ROS_INFO_STREAM("planePoint: " << Eigen::Vector3d(mLastPerceivedFoodTransform.translation().matrix()));
    //   ROS_INFO_STREAM("intersectionPoint: " << intersection.matrix());
    }

    mLastPerceivedFoodTransform = foodTransform;
    return true;
  }
  else
  {
    return false;
  }
  
}

bool Perception::perceiveFace(Eigen::Isometry3d& faceTransform)
{
  mObjDetector->detectObjects(
      mWorld,
      ros::Duration(mPerceptionTimeout));

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

bool Perception::isMouthOpen() {
  //return mObjectDatabase->mObjData["faceStatus"].as<bool>();
  return true;
}

}
