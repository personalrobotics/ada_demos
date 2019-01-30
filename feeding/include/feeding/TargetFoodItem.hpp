#ifndef FEEDING_TARGETFOODITEM_HPP_
#define FEEDING_TARGETFOODITEM_HPP_

#include <dart/dart.hpp>
#include <aikido/perception/detectedItem.hpp>

namespace feeding {

class TargetFoodItem
{
public:
  // Score is between [0, 1].
  explicit TargetFoodItem(
      const std::string& name,
      dart::dynamics::MetaSkeletonPtr skeleton,
      const Eigen::Isometry3d& pose,
      double score,
      const AcquisitionAction action);

  explicit TargetFoodItem(
      aikido::perception::DetectedItem item);

  Eigen::Isometry3d getPose() const;

  std::string getName() const;

private:
  const std::string mName;
  const std::string mUID; // unique id necessary for tracking
  dart::dynamics::MetaSkeletonPtr mObject;
  const Eigen::Isometry3d mPose;
  const AcquisitionAction mBestActionToTake;
  const double mScore;
};

} // namespace feeding

#endif