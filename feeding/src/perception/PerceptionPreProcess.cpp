#include "feeding/perception/PerceptionPreProcess.hpp"
#include <chrono>
#include <aikido/constraint/Satisfied.hpp>
#include <aikido/planner/ConfigurationToConfiguration.hpp>
#include <aikido/planner/SnapConfigurationToConfigurationPlanner.hpp>
#include <aikido/planner/parabolic/ParabolicTimer.hpp>
#include <aikido/planner/vectorfield/VectorFieldPlanner.hpp>
#include <aikido/planner/vectorfield/VectorFieldUtil.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSaver.hpp>
#include "feeding/util.hpp"

const static std::string JOINT_STATE_TOPIC_NAME = "/joint_states";

namespace feeding {

//==============================================================================
PerceptionPreProcess::PerceptionPreProcess(
    boost::function<boost::optional<Eigen::Isometry3d>(void)> getTransform,
    float angle,
    float prePushOffset,
    Eigen::Isometry3d forqueTransform)
{
  mGetTransform = getTransform;
  mAngle = angle;
  mPrePushOffset = prePushOffset;
  mForqueTransform = forqueTransform;
}

//==============================================================================
boost::optional<Eigen::Isometry3d> PerceptionPreProcess::applyOffset()
{
  auto transform = mGetTransform();
  if (!transform)
    return boost::optional<Eigen::Isometry3d>{};

  ROS_INFO_STREAM(transform.get().matrix());
  Eigen::Vector3d diff(0, mPrePushOffset, 0);
  transform.get().translation() += mForqueTransform.inverse().linear() * diff;

  ROS_INFO_STREAM(transform.get().matrix());
  return transform.get();
}

} // namespace feeding
