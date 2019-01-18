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
#include "feeding/perception/Perception.hpp"

namespace feeding {

class PerceptionPreProcess
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PerceptionPreProcess(
      boost::function<boost::optional<Eigen::Isometry3d>(void)> getTransform,
      float angle,
      float prePushOffset,
      Eigen::Isometry3d forqueTransform);

  boost::optional<Eigen::Isometry3d> applyOffset();

protected:
  boost::function<boost::optional<Eigen::Isometry3d>(void)> mGetTransform;
  float mAngle;
  float mPrePushOffset;
  Eigen::Isometry3d mForqueTransform;
};
}

#endif // FEEDING_PERCEPTIONPREPROCESS_HPP_
