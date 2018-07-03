#include "feeding/FTThresholdHelper.hpp"
#include <thread>
#include "feeding/util.hpp"

namespace feeding {

//==============================================================================
FTThresholdHelper::FTThresholdHelper(
    bool useThresholdControl, ros::NodeHandle nodeHandle)
  : useThresholdControl(useThresholdControl), nodeHandle(nodeHandle)
{
  if (!useThresholdControl)
    return;

  ftThresholdClient = std::unique_ptr<rewd_controllers::FTThresholdClient>(
      new rewd_controllers::FTThresholdClient(
          getRosParam<std::string>(
              "/ftSensor/controllerFTThresholdTopic", nodeHandle),
          nodeHandle));
}

//==============================================================================
void FTThresholdHelper::init()
{
  if (!useThresholdControl)
    return;

  auto thresholdPair = getThresholdValues(STANDARD_FT_THRESHOLD);
  ftThresholdClient->trySetThresholdRepeatedly(
      thresholdPair.first, thresholdPair.second);
}

//==============================================================================
bool FTThresholdHelper::trySetThreshold(FTThreshold threshold)
{
  if (!useThresholdControl)
    return true;

  auto thresholdPair = getThresholdValues(STANDARD_FT_THRESHOLD);
  return ftThresholdClient->trySetThreshold(
      thresholdPair.first, thresholdPair.second);
}

//==============================================================================
void FTThresholdHelper::setThreshold(FTThreshold threshold)
{
  if (!useThresholdControl)
    return;

  auto thresholdPair = getThresholdValues(STANDARD_FT_THRESHOLD);
  return ftThresholdClient->setThreshold(
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
          "/ftSensor/thresholds/standard/force", nodeHandle);
      torqueThreshold = getRosParam<double>(
          "/ftSensor/thresholds/standard/torque", nodeHandle);
      break;
    case GRAB_FOOD_FT_THRESHOLD:
      forceThreshold = getRosParam<double>(
          "/ftSensor/thresholds/grabFood/force", nodeHandle);
      torqueThreshold = getRosParam<double>(
          "/ftSensor/thresholds/grabFood/torque", nodeHandle);
      break;
    case AFTER_GRAB_FOOD_FT_THRESHOLD:
      forceThreshold = getRosParam<double>(
          "/ftSensor/thresholds/afterGrabFood/force", nodeHandle);
      torqueThreshold = getRosParam<double>(
          "/ftSensor/thresholds/afterGrabFood/torque", nodeHandle);
      break;
    default:
      throw std::runtime_error(
          "Unknown F/T Threshold type: " + std::to_string(threshold));
  }
  return std::pair<double, double>(forceThreshold, torqueThreshold);
}
}
