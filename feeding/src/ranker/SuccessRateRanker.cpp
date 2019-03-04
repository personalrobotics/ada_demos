#include "feeding/ranker/SuccessRateRanker.hpp"

#include <dart/common/StlHelpers.hpp>
#include "feeding/AcquisitionAction.hpp"

namespace feeding {

//==============================================================================
void SuccessRateRanker::sort(
    std::vector<std::unique_ptr<FoodItem>>& items) const
{
  // Descending since score is the succes rate.
  TargetFoodRanker::sort(items, SORT_ORDER::DESCENDING);
}

//==============================================================================
std::unique_ptr<FoodItem> SuccessRateRanker::createFoodItem(
    const aikido::perception::DetectedObject& item,
    const Eigen::Isometry3d& forqueTransform) const
{
  double successRate = item.getInfoByKey<double>("score");
  std::string itemAction = item.getInfoByKey<std::string>("action");
  if (StringToTiltStyle.find(itemAction) == StringToTiltStyle.end())
  {
    std::stringstream ss;
    ss << "Action [" << itemAction << "] not recognized." << std::endl;
    throw std::invalid_argument(ss.str());
  }
  TiltStyle tiltStyle = StringToTiltStyle.at(itemAction);
  double rotation = item.getInfoByKey<double>("rotation") * M_PI / 180.0;

  // TODO: Make AcquisitionAction deterministic on tiltStyle?

  AcquisitionAction action(tiltStyle, rotation, 0.0, Eigen::Vector3d(0, 0, -1));

  auto itemPose = item.getMetaSkeleton()->getBodyNode(0)->getWorldTransform();

  std::size_t idx = item.getName().find("+");
  std::string name = item.getName();
  if (idx != std::string::npos)
  {
    name = name.substr(0, idx);
    std::cout << "name " << name << std::endl;
  }

  std::cout << "SuccessRateRanker " << successRate << ", " << item.getInfoByKey<double>("rotation") * M_PI / 180.0  << " " << item.getName() << std::endl;
  std::cout << "tiltStyle " << itemAction << std::endl;

  return dart::common::make_unique<FoodItem>(
      name,
      item.getUid(),
      item.getMetaSkeleton(),
      action,
      successRate);
}

} // namespace feeding
