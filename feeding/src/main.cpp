
#include "feeding/FeedingDemo.hpp"
#include "feeding/util.hpp"
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include <ros/ros.h>

using namespace feeding;

int main(int argc, char** argv) {

	// ===== STARTUP =====

  // Is the real robot used or simulation?
  bool adaReal = false;

  // Should the demo continue without asking for human input at each step?
  bool autoContinueDemo = false;

  handleArguments(argc, argv, adaReal, autoContinueDemo);
  ROS_INFO_STREAM("Simulation Mode: " << !adaReal);

	// start node
  ros::init(argc, argv, "feeding");
  ros::NodeHandle nodeHandle("~");

  // start demo
	FeedingDemo feedingDemo(adaReal, nodeHandle);

	// visualization
	aikido::rviz::WorldInteractiveMarkerViewer viewer(
      feedingDemo.getWorld(),
      getRosParam<std::string>("/visualizationName", nodeHandle),
      getRosParam<std::string>("/baseFrameName", nodeHandle));
	viewer.setAutoUpdate(true);

  std::string collisionCheckResult;
  if (!feedingDemo.isCollisionFree(collisionCheckResult)) {
    throw std::runtime_error(collisionCheckResult);
  }
	feedingDemo.closeHand();

	if (!autoContinueDemo)
		waitForUser("Startup complete.");


	// ===== ABOVE PLATE =====
	if (!autoContinueDemo)
		waitForUser("Move forque above plate"),
	feedingDemo.moveAbovePlate();

	// ===== DONE =====
  	waitForUser("Demo finished.");
  ros::shutdown();
  return 0;
}