
#include "feeding/FTThresholdHelper.hpp"
#include "feeding/FeedingDemo.hpp"
#include "feeding/Perception.hpp"
#include "feeding/util.hpp"
#include <ros/ros.h>
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>

namespace feeding {

int acquisitionmain(FeedingDemo& feedingDemo,
                FTThresholdHelper& ftThresholdHelper,
                Perception& perception,
                aikido::rviz::WorldInteractiveMarkerViewerPtr viewer,
                ros::NodeHandle nodeHandle,
                bool autoContinueDemo,
                bool adaReal) {


  for (int trial=0; trial<10; trial++) {
    std::cout << "\033[1;33mSTARTING TRIAL " << trial << "\033[0m" << std::endl;

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
    if (!autoContinueDemo)
    {
      if (!waitForUser("Move forque above food"))
      {
        return 0;
      }
    }
    std::vector<std::string> foodNames = getRosParam<std::vector<std::string>>("/foodItems/names", nodeHandle);
    std::vector<double> skeweringForces = getRosParam<std::vector<double>>("/foodItems/forces", nodeHandle);
    std::unordered_map<std::string, double> foodSkeweringForces;
    for (int i=0; i<foodNames.size(); i++) {
      foodSkeweringForces[foodNames[i]] = skeweringForces[i];
    }

    Eigen::Isometry3d foodTransform;
    std::string foodName;

    if (adaReal)
    {
      bool perceptionSuccessful = perception.perceiveFood(foodTransform, true, viewer, foodName, true);
      if (!perceptionSuccessful) {
        std::cout << "\033[1;33mI can't see any food! Exiting.\033[0m" << std::endl;
        return 0;
      }
    }
    else
    {
      foodTransform = feedingDemo.getDefaultFoodTransform();
      foodName = "apricot";
    }
    perception.setFoodName(foodName);
    std::cout << "\033[1;32mI'm gonna get the " << foodName << "!\033[0;32m  (Gonna skewer with " << foodSkeweringForces[foodName] << "N)\033[0m" << std::endl << std::endl;

    bool angledSkewering = (foodName == "banana");

    if (angledSkewering) {
      feedingDemo.moveAboveFood(foodTransform, 0.25*M_PI, viewer);
      bool perceptionSuccessful = perception.perceiveFood(foodTransform, true, viewer);
      if (!perceptionSuccessful) {
        std::cout << "\033[1;33mI can't see the " << foodName << " anymore...\033[0m" << std::endl;
        continue;
      } else {
        feedingDemo.moveAboveFood(foodTransform, 0.25*M_PI, viewer);
      }
    } else {
      feedingDemo.moveAboveFood(foodTransform, 0, viewer);
      // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      // bool perceptionSuccessful = perception.perceiveFood(foodTransform, true, viewer);
      // if (!perceptionSuccessful) {
      //   std::cout << "\033[1;33mI can't see the " << foodName << " anymore...\033[0m" << std::endl;
      //   continue;
      // } else {
      //   feedingDemo.moveAboveFood(foodTransform, 0, viewer);
      // }
    }

    // ===== INTO FOOD =====
    if (!autoContinueDemo)
    {
      if (!waitForUser("Move forque into food"))
      {
        return 0;
      }
    }
    double torqueThreshold = 2;
    if (!ftThresholdHelper.setThresholds(foodSkeweringForces[foodName], torqueThreshold))
    {
      return 1;
    }
    if (adaReal) {
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
  }

  // ===== DONE =====
  waitForUser("Demo finished.");
}

};
