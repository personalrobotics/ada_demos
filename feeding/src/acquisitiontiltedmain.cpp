
#include "feeding/FTThresholdHelper.hpp"
#include "feeding/FeedingDemo.hpp"
#include "feeding/Perception.hpp"
#include "feeding/util.hpp"
#include <ros/ros.h>
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>

namespace feeding {

int acquisitiontiltedmain(FeedingDemo& feedingDemo,
                FTThresholdHelper& ftThresholdHelper,
                Perception& perception,
                aikido::rviz::WorldInteractiveMarkerViewerPtr viewer,
                ros::NodeHandle nodeHandle,
                bool autoContinueDemo,
                bool adaReal) {


  std::vector<std::string> foodNames = getRosParam<std::vector<std::string>>("/foodItems/names", nodeHandle);
  std::vector<double> skeweringForces = getRosParam<std::vector<double>>("/foodItems/forces", nodeHandle);
  std::unordered_map<std::string, double> foodSkeweringForces;
  for (int i=0; i<foodNames.size(); i++) {
    foodSkeweringForces[foodNames[i]] = skeweringForces[i];
  }

    if (!autoContinueDemo)
    {
      if (!waitForUser("Ready to start."))
      {
        return 0;
      }
    }

  for (int trial=0; trial<10; trial++) {
    std::cout << "\033[1;33mSTARTING TRIAL " << trial << "\033[0m" << std::endl;

    // ===== ABOVE PLATE =====
    bool stepSuccessful = false;
    while (!stepSuccessful) {
      try {
        // feedingDemo.moveAbovePlate();
        stepSuccessful = true;
      } catch (std::runtime_error) {
        if (!waitForUser("Trajectory execution failed. Try again?")) {continue;}
      }
    }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // ===== SELECT FOOD =====
    if (!autoContinueDemo)
    {
      if (!waitForUser("Move forque above food"))
      {
        return 0;
      }
    }

    // Eigen::Isometry3d foodTransform;
    // std::string foodName;

    // if (adaReal)
    // {
    //   bool perceptionSuccessful = perception.perceiveFood(foodTransform, true, viewer, foodName, true);
    //   if (!perceptionSuccessful) {
    //     std::cout << "\033[1;33mI can't see any food! Next trial.\033[0m" << std::endl;
    //     continue;
    //   }
    // }
    // else
    // {
    //   foodTransform = feedingDemo.getDefaultFoodTransform();
    //   foodName = "apricot";
    // }
    // perception.setFoodName(foodName);
    // std::cout << "\033[1;32mI'm gonna get the " << foodName << "!\033[0;32m  (Gonna skewer with " << foodSkeweringForces[foodName] << "N)\033[0m" << std::endl << std::endl;


  Eigen::Isometry3d foodTransform = feedingDemo.getDefaultFoodTransform();
  bool foodFound = true;
  std::string foodName = "celery";
  // while (!foodFound) {
  //   std::cout << std::endl << "\033[1;32mWhich food item do you want?\033[0m     > ";
  //   foodName = "";
  //   std::cin >> foodName;
  //   if (!ros::ok()) {return 0;}
  //   nodeHandle.setParam("/deep_pose/publish_spnet", true);
  //   nodeHandle.setParam("/deep_pose/spnet_food_name", foodName);
  //   std::this_thread::sleep_for(std::chrono::milliseconds(400));
  //   if (!perception.setFoodName(foodName)) {
  //     std::cout << "\033[1;33mI don't know about any food that's called '" << foodName << ". Wanna get something else?\033[0m" << std::endl;
  //     continue;
  //   }

  //   if (adaReal)
  //   {
  //     bool perceptionSuccessful = perception.perceiveFood(foodTransform, true, viewer);
  //     ROS_INFO_STREAM("perceptionSuccessfulBool: " << perceptionSuccessful);
  //     if (!perceptionSuccessful) {
  //       std::cout << "\033[1;33mI can't see the " << foodName << "... Wanna get something else?\033[0m" << std::endl;
  //       continue;
  //     } else {
  //       foodFound = true;
  //     }
  //   }
  //   else
  //   {
  //     foodTransform = feedingDemo.getDefaultFoodTransform();
  //     foodFound = true;
  //   }
  // }
  // std::cout << "\033[1;32mAlright! Let's get the " << foodName << "!\033[0;32m  (Gonna skewer with " << foodSkeweringForces[foodName] << "N)\033[0m" << std::endl << std::endl;

  nodeHandle.setParam("/deep_pose/publish_spnet", true);
  nodeHandle.setParam("/deep_pose/spnet_food_name", foodName);
  std::this_thread::sleep_for(std::chrono::milliseconds(700));

  bool bananaStyleTilt = false; (foodName == "banana" || foodName == "celery");


  // ===== ABOVE FOOD =====
  stepSuccessful = false;
  bool continueWithNextTrial = false;
  while (!stepSuccessful && !continueWithNextTrial) {
    try {
      if (bananaStyleTilt) {
        feedingDemo.moveAboveFood(foodTransform, 0, viewer);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        // bool perceptionSuccessful = perception.perceiveFood(foodTransform, true, viewer);
        // if (!perceptionSuccessful) {
        //   std::cout << "\033[1;33mI can't see the " << foodName << " anymore...\033[0m" << std::endl;
        //   continueWithNextTrial = true;
        // } else {
        //   feedingDemo.moveAboveFood(foodTransform, 0.25*M_PI, viewer);
        // }
      } else {

        // Grape style skewering angle: -0.05*M_PI, viewer, false




        feedingDemo.moveAboveFood(foodTransform, 0, viewer, false);
        // feedingDemo.moveAboveFood(foodTransform, 0, viewer, false);





        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        
        stepSuccessful = true;
        continue;

        // bool perceptionSuccessful = perception.perceiveFood(foodTransform, true, viewer);
        // if (!perceptionSuccessful) {
        //   std::cout << "\033[1;33mI can't see the " << foodName << " anymore...\033[0m" << std::endl;
        //   continue;
        // } else {
        //   feedingDemo.moveAboveFood(foodTransform, -0.05*M_PI, viewer, false);
        // }
      }
      stepSuccessful = true;
    } catch (std::runtime_error) {
      if (!waitForUser("Trajectory execution failed. Try again?")) {continue;}
    }
  }
  if (continueWithNextTrial) {continue;}


    // ===== INTO FOOD =====
    if (!autoContinueDemo)
    {
      if (!waitForUser("Move forque into food"))
      {
        return 0;
      }
    }
    double torqueThreshold = 2;
    double forceThreshold = 3;
    // if (!ftThresholdHelper.setThresholds(foodSkeweringForces[foodName], torqueThreshold))
    if (!ftThresholdHelper.setThresholds(forceThreshold, torqueThreshold))
    {
      return 1;
    }
    if (adaReal) {
      feedingDemo.moveIntoFood();
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
    try {
      feedingDemo.moveOutOfFood();
    } catch (std::runtime_error) {
      waitForUser("Unable to move out of food. Starting next trial.");
      continue;
    }
    if (!ftThresholdHelper.setThresholds(STANDARD_FT_THRESHOLD))
    {
      return 1;
    }
  }

  // ===== DONE =====
  waitForUser("Demo finished.");
}

};
