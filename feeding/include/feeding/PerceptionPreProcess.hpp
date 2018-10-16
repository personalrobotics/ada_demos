#ifndef FEEDING_PERCEPTIONPREPROCESS_HPP_
#define FEEDING_PERCEPTIONPREPROCESS_HPP_

#include <mutex>
#include <aikido/control/ros/RosTrajectoryExecutor.hpp>
#include <aikido/control/ros/RosTrajectoryExecutor.hpp>
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/trajectory/Spline.hpp>
#include <aikido/trajectory/Spline.hpp>
#include <dart/dynamics/BodyNode.hpp>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include "feeding/AdaMover.hpp"
#include "feeding/Perception.hpp"

namespace feeding {

class PerceptionPreProcess
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PerceptionPreProcess(
      boost::function<bool(Eigen::Isometry3d&)> getTransform, float angle);

  bool applyOffset(Eigen::Isometry3d& foodTransform);

protected:
  boost::function<bool(Eigen::Isometry3d&)> mGetTransform;
  float mAngle;
};
}

#endif // FEEDING_PERCEPTIONPREPROCESS_HPP_
