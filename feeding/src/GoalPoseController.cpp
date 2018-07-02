#include <pluginlib/class_list_macros.h>
#include <feeding/GoalPoseController.hpp>

namespace feeding
{
//=============================================================================
GoalPoseController::GoalPoseController()
    : MultiInterfaceController{true}  // allow_optional_interfaces
    , GoalPoseControllerBase{}
{
}

//=============================================================================
GoalPoseController::~GoalPoseController() {}

//=============================================================================
bool GoalPoseController::init(hardware_interface::RobotHW* robot,
                                     ros::NodeHandle& n)
{
  return initController(robot, n);
}

//=============================================================================
void GoalPoseController::starting(const ros::Time& time)
{
  std::cout << "GoalPoseController::starting" << std::endl;
  startController(time);
}

//=============================================================================
void GoalPoseController::stopping(const ros::Time& time)
{
  stopController(time);
}

//=============================================================================
void GoalPoseController::update(const ros::Time& time,
                                       const ros::Duration& period)
{
  std::cout << "GoalPoseController::update" << std::endl;
  updateStep(time, period);
}

//=============================================================================
bool GoalPoseController::shouldAcceptRequests() 
{ 
  return isRunning(); 
}

}  // namespace feeding

//=============================================================================
PLUGINLIB_EXPORT_CLASS(feeding::GoalPoseController,
                       controller_interface::ControllerBase)
