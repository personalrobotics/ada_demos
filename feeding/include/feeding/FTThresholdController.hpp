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

/// The FTThresholdController configures the MoveUntilTouchController's thresholds.
/// When those thresholds are exceeded, the controller stops the movement.
class FTThresholdController {

	bool useThresholdControl;
  const ros::NodeHandle& nodeHandle;
  std::unique_ptr<actionlib::SimpleActionClient<pr_control_msgs::SetForceTorqueThresholdAction>> ftThresholdActionClient;

public:

	/// Constructor.
	/// With useThresholdControl you can turn this whole objects on and off.
	/// Useful if you don't use the MoveUntilTouchController and don't need to set these thresholds
  FTThresholdController(bool useThresholdControl, const ros::NodeHandle& nodeHandle);

	/// Needs to be called before setting the first thresholds.
	/// Blocks until the threshold could be set successfully.
	/// Can be aborted with Ctrl-C.
	void init();

	/// Sets the MoveUntilTouchControllers Thresholds accordingly.
	/// Throws a runtime_error if we useThresholdControl and we are unable to set the thresholds.
	void setThreshold(FTThreshold);

};

}

#endif