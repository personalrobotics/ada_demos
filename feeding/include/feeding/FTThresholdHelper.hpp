#ifndef FTTHRESHOLDHELPER_H
#define FTTHRESHOLDHELPER_H

#include <ros/ros.h>
#include <rewd_controllers/FTThresholdClient.hpp>

namespace feeding {

enum FTThreshold
{
  STANDARD_FT_THRESHOLD,
  GRAB_FOOD_FT_THRESHOLD,
  AFTER_GRAB_FOOD_FT_THRESHOLD
};

/// The FTThresholdHelper configures the MoveUntilTouchController's
/// thresholds.
/// When those thresholds are exceeded, the controller stops the movement.
class FTThresholdHelper
{

public:
  /// Constructor.
  /// With useThresholdControl you can turn this whole object on and off.
  /// Useful if you don't use the MoveUntilTouchController and don't need to set
  /// these thresholds
  FTThresholdHelper(bool useThresholdControl, ros::NodeHandle nodeHandle);

  /// Needs to be called before setting the first thresholds.
  /// Blocks until the threshold could be set successfully.
  /// Can be aborted with Ctrl-C.
  void init();

  /// Sets the MoveUntilTouchControllers Thresholds accordingly.
  /// Throws a runtime_error if we useThresholdControl and we are unable to set
  /// the thresholds.
  void setThreshold(FTThreshold);

  /// Sets the MoveUntilTouchControllers Thresholds accordingly.
  /// Returns whether the thresholds were set successfully.
  bool trySetThreshold(FTThreshold);

private:
  bool useThresholdControl;
  ros::NodeHandle nodeHandle;

  std::unique_ptr<rewd_controllers::FTThresholdClient> ftThresholdClient;

  std::pair<double, double> getThresholdValues(FTThreshold threshold);
};
}

#endif
