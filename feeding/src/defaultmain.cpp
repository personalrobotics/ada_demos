
#include "feeding/FTThresholdHelper.hpp"
#include "feeding/FeedingDemo.hpp"
#include "feeding/Perception.hpp"
#include "feeding/util.hpp"
#include <ros/ros.h>
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>

namespace feeding {

int defaultmain(FeedingDemo& feedingDemo,
                FTThresholdHelper& ftThresholdHelper,
                Perception& perception,
                aikido::rviz::WorldInteractiveMarkerViewerPtr viewer,
                ros::NodeHandle nodeHandle,
                bool autoContinueDemo,
                bool adaReal) {

  // ===== ABOVE PLATE =====
  if (!autoContinueDemo)
  {
    if (!waitForUser("Move forque above plate"))
    {
      return 0;
    }
  }
  // feedingDemo.moveAbovePlate();

  // ===== ABOVE FOOD =====
  std::vector<std::string> foodNames = getRosParam<std::vector<std::string>>("/foodItems/names", nodeHandle);
  std::vector<double> skeweringForces = getRosParam<std::vector<double>>("/foodItems/forces", nodeHandle);
  std::unordered_map<std::string, double> foodSkeweringForces;
  for (int i=0; i<foodNames.size(); i++) {
    foodSkeweringForces[foodNames[i]] = skeweringForces[i];
  }

  std::string foodName = "celery";
  Eigen::Isometry3d foodTransform = feedingDemo.getDefaultFoodTransform();
  bool foodFound = true;
  std::cout << "\033[1;32mAlright! Let's get the " << foodName << "!\033[0;32m  (Gonna skewer with " << foodSkeweringForces[foodName] << "N)\033[0m" << std::endl << std::endl;

  bool foodPickedUp = false;
  while (!foodPickedUp) {
    
    if (!autoContinueDemo)
    {
      if (!waitForUser("Move forque above food"))
      {
        return 0;
      }
    }
    feedingDemo.moveAboveFood(foodTransform, 0, viewer);
    // bool perceptionSuccessful = perception.perceiveFood(foodTransform, false, viewer);
    // if (!perceptionSuccessful) {
    //   std::cout << "\033[1;33mI can't see the " << foodName << " anymore...\033[0m" << std::endl;
    // } else {
    //   feedingDemo.moveAboveFood(foodTransform, 0, viewer);
    // }

    // ===== INTO FOOD =====
    if (!autoContinueDemo)
    {
      if (!waitForUser("Move forque into food"))
      {
        return 0;
      }
    }

    double zForceBeforeSkewering = 0;
    if (ftThresholdHelper.startDataCollection(50)) {
      Eigen::Vector3d currentForce, currentTorque;
      while (!ftThresholdHelper.isDataCollectionFinished(currentForce, currentTorque)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      zForceBeforeSkewering = currentForce.x();
    }
    double torqueThreshold = 2;
    if (!ftThresholdHelper.setThresholds(foodSkeweringForces[foodName], torqueThreshold))
    {
      return 1;
    }
    if (adaReal && false) {
      feedingDemo.moveIntoFood(&perception, viewer);
    } else {
      feedingDemo.moveIntoFood();
    }
    std::this_thread::sleep_for(
        std::chrono::milliseconds(
            getRosParam<int>("/feedingDemo/waitMillisecsAtFood", nodeHandle)));
    feedingDemo.grabFoodWithForque();

    // ===== OUT OF FOOD =====
    if (!autoContinueDemo)
    {
      if (!waitForUser("Move forque out of food"))
      {
        return 0;
      }
    }
    if (!ftThresholdHelper.setThresholds(AFTER_GRAB_FOOD_FT_THRESHOLD))
    {
      return 1;
    }
    feedingDemo.moveOutOfFood();
    if (!ftThresholdHelper.setThresholds(STANDARD_FT_THRESHOLD))
    {
      return 1;
    }

      double forceDifference = 100;
      double zForceAfter = 0;
      if (ftThresholdHelper.startDataCollection(50)) {
        Eigen::Vector3d currentForce, currentTorque;
        while (!ftThresholdHelper.isDataCollectionFinished(currentForce, currentTorque)) {
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        zForceAfter = currentForce.x();
        forceDifference = std::fabs(zForceBeforeSkewering - currentForce.x());
      }
      ROS_WARN_STREAM("force difference: " << (zForceBeforeSkewering -zForceAfter));

      if (forceDifference > 3.022) {
        foodPickedUp = true;
        ROS_INFO_STREAM("FOOD PICKED UP!");
      } else {
        std::cout << "\033[1;32mOoops! I think I didn't manage to pick up the " << foodName << ". Let me try again!\033[0;32m" << std::endl;
        feedingDemo.moveAbovePlate();
      }
  }

  // ===== IN FRONT OF PERSON =====
  if (!autoContinueDemo)
  {
    if (!waitForUser("Move forque in front of person"))
    {
      return 0;
    }
  }
  feedingDemo.moveInFrontOfPerson();
  nodeHandle.setParam("/feeding/facePerceptionOn", true);

  // while (true) {
  //     Eigen::Isometry3d faceTransform;
  //   bool perceptionSuccessful = perception.perceiveFace(faceTransform);
  // }
  // ===== TOWARDS PERSON =====
  if (!autoContinueDemo)
  {
    if (!waitForUser("Move towards person"))
    {
      return 0;
    }
  }
  feedingDemo.moveTowardsPerson(&perception, viewer);
  nodeHandle.setParam("/feeding/facePerceptionOn", false);
  ROS_WARN("Human is eating");
  std::this_thread::sleep_for(
      std::chrono::milliseconds(
          getRosParam<int>("/feedingDemo/waitMillisecsAtPerson", nodeHandle)));
  feedingDemo.ungrabAndDeleteFood();

  // ===== AWAY FROM PERSON =====
  feedingDemo.moveAwayFromPerson();

  // ===== BACK TO PLATE =====
  if (!autoContinueDemo)
  {
    if (!waitForUser("Move back to plate"))
    {
      return 0;
    }
  }
  feedingDemo.moveAbovePlate();

  // ===== DONE =====
  waitForUser("Demo finished.");
}

};
