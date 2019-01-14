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

static std::string JOINT_STATE_TOPIC_NAME = "/joint_states";

namespace feeding {

namespace {

Eigen::VectorXd getSymmetricLimits(
    const Eigen::VectorXd& lowerLimits, const Eigen::VectorXd& upperLimits)
{
  assert(
      static_cast<std::size_t>(lowerLimits.size())
      == static_cast<std::size_t>(upperLimits.size()));

  std::size_t limitSize = static_cast<std::size_t>(lowerLimits.size());
  Eigen::VectorXd symmetricLimits(limitSize);
  for (std::size_t i = 0; i < limitSize; ++i)
  {
    symmetricLimits[i] = std::min(-lowerLimits[i], upperLimits[i]);
  }
  return symmetricLimits;
}

} // namespace

//==============================================================================
PerceptionPreProcess::PerceptionPreProcess(
    boost::function<bool(Eigen::Isometry3d&)> getTransform,
    float angle,
    float prePushOffset,
    Eigen::Isometry3d forqueTransform)
{
  mGetTransform = getTransform;
  mAngle = angle;
  mPrePushOffset = prePushOffset;
  mForqueTransform = forqueTransform;
}

bool PerceptionPreProcess::applyOffset(Eigen::Isometry3d& foodTransform)
{
  if (mGetTransform(foodTransform))
  {
    ROS_INFO_STREAM(foodTransform.matrix());
    Eigen::Vector3d diff(0, mPrePushOffset, 0);
    foodTransform.translation() += mForqueTransform.inverse().linear() * diff;
    // float xOff = cos(mAngle) * mDistBeforePush;
    // float yOff = sin(mAngle) * mDistBeforePush;
    // foodTransform.translation() += Eigen::Vector3d(-xOff, yOff, 0);
    //        foodTransform.linear() *= Eigen::Matrix3d(Eigen::AngleAxisd(
    //        (-M_PI * 0.5) - mAngle, Eigen::Vector3d::UnitZ()));
    ROS_INFO_STREAM(foodTransform.matrix());
    return true;
  }
  ROS_INFO("Error");
  return false;
}

} // namespace feeding
