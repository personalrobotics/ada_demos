#include "cameraCalibration/Perception.hpp"

namespace cameraCalibration {
  

Perception::Perception(ros::NodeHandle nodeHandle) {

}

Eigen::Isometry3d Perception::getTargetTransformInCameraLensFrame() const {
  return Eigen::Isometry3d::Identity();
}

}
