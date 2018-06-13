#ifndef FTTHRESHOLDCONTROLLER_H
#define FTTHRESHOLDCONTROLLER_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <pr_control_msgs/SetForceTorqueThresholdAction.h>
#include "feeding/util.hpp"

namespace feeding {

enum FTThreshold {
	STANDARD_FT_THRESHOLD,
	GRAB_FOOD_FT_THRESHOLD,
	AFTER_GRAB_FOOD_FT_THRESHOLD,
	TOWARDS_PERSON_FT_THRESHOLD
};

class FTThresholdController {

  bool adaReal;
	bool useThresholdControl;
  const ros::NodeHandle& nodeHandle;
  std::unique_ptr<actionlib::SimpleActionClient<pr_control_msgs::SetForceTorqueThresholdAction>> ftThresholdActionClient;

public:

  FTThresholdController(bool adaReal, bool useThresholdControl, const ros::NodeHandle& nodeHandle);

	void init();

	bool setThreshold(FTThreshold);

};

}

#endif