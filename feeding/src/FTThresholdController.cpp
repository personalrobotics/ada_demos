#include "feeding/FTThresholdController.hpp"
#include <thread>

using SetFTThresholdAction = pr_control_msgs::SetForceTorqueThresholdAction;
using FTThresholdActionClient
    = actionlib::SimpleActionClient<SetFTThresholdAction>;

namespace feeding {

FTThresholdController::FTThresholdController(bool useThresholdControl, const ros::NodeHandle& nodeHandle) : useThresholdControl(useThresholdControl), nodeHandle(nodeHandle) {
  if (!useThresholdControl) return;

  std::string controllerThresholdTopic = getRosParam<std::string>("/ftSensor/controllerFTThresholdTopic", nodeHandle);
  ftThresholdActionClient = std::unique_ptr<FTThresholdActionClient>(
    new FTThresholdActionClient(controllerThresholdTopic));
  ROS_INFO("Waiting for FT Threshold Action Server to start...");
  ftThresholdActionClient->waitForServer();
  ROS_INFO("FT Threshold Action Server started.");
}

void FTThresholdController::init() {
  if (!useThresholdControl) return;

  bool setFTSuccessful = false;
  while (!setFTSuccessful)
  {
    setFTSuccessful = setThreshold(STANDARD_FT_THRESHOLD);
    if (setFTSuccessful) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    if (!ros::ok())
    {
      exit(0);
    }
  }
}

void FTThresholdController::setThreshold(FTThreshold threshold) {
  if (!useThresholdControl) return true;
  double forceThreshold = 0;
  double torqueThreshold = 0;

  switch(threshold) {
    case STANDARD_FT_THRESHOLD:
      forceThreshold = getRosParam<double>("/ftSensor/thresholds/standard/force", nodeHandle);
      torqueThreshold = getRosParam<double>("/ftSensor/thresholds/standard/torque", nodeHandle);
      break;
    case GRAB_FOOD_FT_THRESHOLD:
      forceThreshold = getRosParam<double>("/ftSensor/thresholds/grabFood/force", nodeHandle);
      torqueThreshold = getRosParam<double>("/ftSensor/thresholds/grabFood/torque", nodeHandle);
      break;
    case AFTER_GRAB_FOOD_FT_THRESHOLD:
      forceThreshold = getRosParam<double>("/ftSensor/thresholds/afterGrabFood/force", nodeHandle);
      torqueThreshold = getRosParam<double>("/ftSensor/thresholds/afterGrabFood/torque", nodeHandle);
      break;
    case TOWARDS_PERSON_FT_THRESHOLD:
      forceThreshold = getRosParam<double>("/ftSensor/thresholds/towardsPerson/force", nodeHandle);
      torqueThreshold = getRosParam<double>("/ftSensor/thresholds/towardsPerson/torque", nodeHandle);
      break;
    default:
      throw std::runtime_error("Unknown F/T Threshold type: " + std::to_string(threshold));
  }

  pr_control_msgs::SetForceTorqueThresholdGoal goal;
  goal.force_threshold = forceThreshold;
  goal.torque_threshold = torqueThreshold;
  ftThresholdActionClient->sendGoal(goal);
  bool finished_before_timeout
      = ftThresholdActionClient->waitForResult(ros::Duration(5.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ftThresholdActionClient->getState();
    if (state != actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED)
    {
      throw std::runtime_error(
          "F/T Thresholds could not be set: %s %s",
          state.toString().c_str(),
          ftThresholdActionClient->getResult()->message.c_str());
    }
  }
  else
  {
    throw std::runtime_error("F/T Thresholds could not be set: Timeout");
  }
}


}