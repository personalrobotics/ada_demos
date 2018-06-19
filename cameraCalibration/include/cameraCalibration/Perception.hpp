#ifndef PERCEPTION_H
#define PERCEPTION_H

#include <Eigen/Dense>
#include <aikido/perception/PoseEstimatorModule.hpp>
#include <ros/ros.h>

namespace cameraCalibration {
  

class Perception {

public:

  Perception(ros::NodeHandle nodeHandle);

  Eigen::Isometry3d getTargetTransformInCameraLensFrame() const;

private:

};


}

#endif
