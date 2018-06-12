
#include "feeding/FeedingDemo.hpp"
#include "feeding/util.hpp"
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include <ros/ros.h>
#include <pr_tsr/plate.hpp>

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


	// ===== ABOVE FOOD =====
	if (!autoContinueDemo)
		waitForUser("Perceive Food");
	Eigen::Isometry3d foodTransform;
	if (adaReal) {
    // TODO
		//foodTransform = perception.perceiveFood();
	} else {
		foodTransform = feedingDemo.getDefaultFoodTransform();
	}
	if (!autoContinueDemo)
		waitForUser("Move forque above food"),
	feedingDemo.moveAboveFood(foodTransform);


	// ===== INTO FOOD =====
	if (!autoContinueDemo)
		waitForUser("Move forque into food"),
	//ftSensor.setThreshold(GRAB_FOOD_FT_THRESHOLD);
	feedingDemo.moveIntoFood();
  std::this_thread::sleep_for(std::chrono::milliseconds(getRosParam<int>("/waitMillisecsAtFood", nodeHandle)));
	feedingDemo.grabFoodWithForque();


	// ===== OUT OF FOOD =====
	if (!autoContinueDemo)
		waitForUser("Move forque out of food"),
	//ftSensor.setThreshold(AFTER_GRAB_FOOD_FT_THRESHOLD);
	feedingDemo.moveOutOfFood();
	//ftSensor.setThreshold(STANDARD_FT_THRESHOLD);

	// ===== IN FRONT OF PERSON =====
	if (!autoContinueDemo)
		waitForUser("Move forque in front of person");
	feedingDemo.moveInFrontOfPerson();


	// ===== TOWARDS PERSON =====
	if (!autoContinueDemo)
		waitForUser("Move towards person");
	//ftSensor.setThreshold(TOWARDS_PERSON_FT_THRESHOLD);
	feedingDemo.moveTowardsPerson();
  std::this_thread::sleep_for(std::chrono::milliseconds(getRosParam<int>("/waitMillisecsAtPerson", nodeHandle)));
	feedingDemo.ungrabAndDeleteFood();
	//ftSensor.setThreshold(STANDARD_FT_THRESHOLD);


	// ===== AWAY FROM PERSON =====
	feedingDemo.moveAwayFromPerson();


	// ===== BACK TO PLATE =====
	if (!autoContinueDemo)
		waitForUser("Move back to plate");
	feedingDemo.moveAbovePlate();


	// ===== DONE =====
  	waitForUser("Demo finished.");
  ros::shutdown();
  return 0;
}