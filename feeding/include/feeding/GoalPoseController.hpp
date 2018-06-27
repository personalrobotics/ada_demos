#ifndef FEEDING_GOALPOSECONTROLLER_HPP_
#define FEEDING_GOALPOSECONTROLLER_HPP_

#include <hardware_interface/joint_command_interface.h>
#include <rewd_controllers/MultiInterfaceController.hpp>
#include "GoalPoseControllerBase.hpp"

namespace feeding
{
class GoalPoseController final
    : public rewd_controllers::MultiInterfaceController<hardware_interface::
                                          PositionJointInterface,
                                      hardware_interface::
                                          VelocityJointInterface,
                                      hardware_interface::EffortJointInterface,
                                      hardware_interface::JointStateInterface>,
      public GoalPoseControllerBase
{
public:
  GoalPoseController();
  ~GoalPoseController();

  // inherit documentation
  bool init(hardware_interface::RobotHW* robot, ros::NodeHandle& n) override;

  /** \brief This is called from within the realtime thread just before the
   * first call to \ref update
   *
   * \param time The current time
   */
  void starting(const ros::Time& time) override;

  // inherit documentation
  void stopping(const ros::Time& time) override;

  // inherit documentation
  void update(const ros::Time& time, const ros::Duration& period) override;

protected:
  /** \brief The JointTrajectoryControllerBase should accept new trajectories
   * and cancel requests when this controller is started.
   *
   * \returns true when isRunning() is true;
   */
  bool shouldAcceptRequests() override;
};

}  // namespace feeding

#endif  // FEEDING_GOALPOSECONTROLLER_HPP_
