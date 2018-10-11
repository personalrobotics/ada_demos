
#include "feeding/FTThresholdHelper.hpp"
#include "feeding/FeedingDemo.hpp"
#include "feeding/Perception.hpp"
#include "feeding/util.hpp"
#include <ros/ros.h>
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>

namespace feeding {

int pushingmain(FeedingDemo& feedingDemo,
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
  feedingDemo.moveAbovePlate();

  // ===== ABOVE FOOD =====
  std::vector<std::string> foodNames = getRosParam<std::vector<std::string>>("/foodItems/names", nodeHandle);
  std::vector<double> skeweringForces = getRosParam<std::vector<double>>("/foodItems/forces", nodeHandle);
  std::unordered_map<std::string, double> foodSkeweringForces;
  for (int i=0; i<foodNames.size(); i++) {
    foodSkeweringForces[foodNames[i]] = skeweringForces[i];
  }

  Eigen::Isometry3d foodTransform;
  bool foodFound = false;
  std::string foodName;
  while (!foodFound) {
    std::cout << std::endl << "\033[1;32mWhich food item do you want?\033[0m     > ";
    foodName = "";
    std::cin >> foodName;
    if (!ros::ok()) {return 0;}
    if (!perception.setFoodName(foodName)) {
      std::cout << "\033[1;33mI don't know about any food that's called '" << foodName << ". Wanna get something else?\033[0m" << std::endl;
      continue;
    }

    if (adaReal)
    {
      bool perceptionSuccessful = perception.perceiveFood(foodTransform, false, viewer);
      if (!perceptionSuccessful) {
        std::cout << "\033[1;33mI can't see the " << foodName << "... Wanna get something else?\033[0m" << std::endl;
        continue;
      } else {
        foodFound = true;
      }
    }
    else
    {
      foodTransform = feedingDemo.getDefaultFoodTransform();
      foodFound = true;
    }
  }
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
    if (adaReal) {
      bool perceptionSuccessful = perception.perceiveFood(foodTransform, false, viewer);
      if (!perceptionSuccessful) {
        std::cout << "\033[1;33mI can't see the " << foodName << " anymore...\033[0m" << std::endl;
      } else {
        feedingDemo.moveAboveFood(foodTransform, 0, viewer);
      }
    }

    double zForceBeforeSkewering = 0;
    if (adaReal && ftThresholdHelper.startDataCollection(20)) {
      Eigen::Vector3d currentForce, currentTorque;
      while (!ftThresholdHelper.isDataCollectionFinished(currentForce, currentTorque)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      zForceBeforeSkewering = currentForce.z();
    }

    // ===== NEXT TO FOOD ====
    if (!autoContinueDemo)
    {
      if (!waitForUser("Move forque next to food"))
      {
        return 0;
      }
    }
    feedingDemo.moveNextToFood(foodTransform, 0, viewer);

    // ===== PUSH FOOD ====
    if (!autoContinueDemo)
    {
      if (!waitForUser("Push food"))
      {
        return 0;
      }
    }
    if (!ftThresholdHelper.setThresholds(foodSkeweringForces[foodName], torqueThreshold))
    {
      return 1;
    }
    feedingDemo.grabFoodWithForque();
    feedingDemo.pushFood(foodTransform, 0, viewer);
    break;
  }
  // ===== DONE =====
  waitForUser("Demo finished.");
}

};
