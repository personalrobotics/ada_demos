#include "feeding/FTThresholdHelper.hpp"
#include <thread>
#include "feeding/util.hpp"

namespace feeding {

//==============================================================================
FTThresholdHelper::FTThresholdHelper(
    bool useThresholdControl, ros::NodeHandle nodeHandle)
  : mUseThresholdControl(useThresholdControl), mNodeHandle(nodeHandle)
{
  if (!mUseThresholdControl)
    return;

  mFTThresholdClient = std::unique_ptr<rewd_controllers::FTThresholdClient>(
      new rewd_controllers::FTThresholdClient(
          getRosParam<std::string>(
              "/ftSensor/controllerFTThresholdTopic", mNodeHandle)));
}

//==============================================================================
void FTThresholdHelper::init()
{
  if (!mUseThresholdControl)
    return;

  auto thresholdPair = getThresholdValues(STANDARD_FT_THRESHOLD);
  mFTThresholdClient->trySetThresholdRepeatedly(
      thresholdPair.first, thresholdPair.second);
}

//==============================================================================
bool FTThresholdHelper::setThreshold(FTThreshold threshold)
{
  if (!mUseThresholdControl)
    return true;

  auto thresholdPair = getThresholdValues(STANDARD_FT_THRESHOLD);
  return mFTThresholdClient->setThreshold(
      thresholdPair.first, thresholdPair.second);
}

//==============================================================================
std::pair<double, double> FTThresholdHelper::getThresholdValues(
    FTThreshold threshold)
{
  double forceThreshold = 0;
  double torqueThreshold = 0;
  switch (threshold)
  {
    case STANDARD_FT_THRESHOLD:
      forceThreshold = getRosParam<double>(
          "/ftSensor/thresholds/standard/force", mNodeHandle);
      torqueThreshold = getRosParam<double>(
          "/ftSensor/thresholds/standard/torque", mNodeHandle);
      break;
    case GRAB_FOOD_FT_THRESHOLD:
      forceThreshold = getRosParam<double>(
          "/ftSensor/thresholds/grabFood/force", mNodeHandle);
      torqueThreshold = getRosParam<double>(
          "/ftSensor/thresholds/grabFood/torque", mNodeHandle);
      break;
    case AFTER_GRAB_FOOD_FT_THRESHOLD:
      forceThreshold = getRosParam<double>(
          "/ftSensor/thresholds/afterGrabFood/force", mNodeHandle);
      torqueThreshold = getRosParam<double>(
          "/ftSensor/thresholds/afterGrabFood/torque", mNodeHandle);
      break;
    default:
      throw std::runtime_error(
          "Unknown F/T Threshold type: " + std::to_string(threshold));
  }
  return std::pair<double, double>(forceThreshold, torqueThreshold);
}
}
