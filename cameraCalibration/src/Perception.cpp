#include "cameraCalibration/Perception.hpp"

namespace cameraCalibration {


Perception::Perception(
    ros::NodeHandle nodeHandle,
    std::string markerTopic)
  : mNodeHandle(std::move(nodeHandle))
  , mMarkerTopic(std::move(markertopic))
{
  // Do nothing
}

Eigen::Isometry3d Perception::getTargetTransformInCameraLensFrame() const
{
  return Eigen::Isometry3d::Identity();
}

}
