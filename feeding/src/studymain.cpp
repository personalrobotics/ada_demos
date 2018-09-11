
#include "feeding/FTThresholdHelper.hpp"
#include "feeding/FeedingDemo.hpp"
#include "feeding/Perception.hpp"
#include "feeding/util.hpp"
#include <ros/ros.h>
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>

namespace feeding {

int studymain(FeedingDemo& feedingDemo,
                FTThresholdHelper& ftThresholdHelper,
                Perception& perception,
                aikido::rviz::WorldInteractiveMarkerViewerPtr viewer,
                ros::NodeHandle nodeHandle,
                bool autoContinueDemo,
                bool adaReal) {

  std::cout << std::endl << "\033[1;32m      ***** BITE TRANSFER STUDY MODE *****\033[0m" << std::endl;
  std::cout << std::endl << "\033[1;32mWhich food item do you want?\033[0m" << std::endl;
  std::cout << "\033[0;32m1) Strawberry\033[0m" << std::endl;
  std::cout << "\033[0;32m2) Cantaloupe\033[0m" << std::endl;
  std::cout << "\033[0;32m3) Celery\033[0m" << std::endl;
  std::cout << "\033[0;32m4) Carrot\033[0m" << std::endl;

  std::string foodName = "";
  while (foodName == "") { 
    std::cout << "> ";
    std::string idString;
    std::cin  >> idString;
    try {
      int id = std::stoi(idString);
      if (id < 1 || id > 4) {
        throw std::invalid_argument("");
      }
      switch (id) {
        case 1: foodName = "strawberry";break;
        case 2: foodName = "cantaloupe";break;
        case 3: foodName = "celery";break;
        case 4: foodName = "carrot";break;
      }
    } catch (const std::invalid_argument& ia) {
      std::cout << "\033[1;31mInvalid argument. Try again.\033[0m" << std::endl;
    }
  }

  std::cout << std::endl << "\033[1;32mWhat step should I proceed with (1-6)?\033[0m" << std::endl;

  int stepIdx = -1;
  while (stepIdx < 0) {
    std::cout << "> ";
    std::string stepIdxString;
    std::cin  >> stepIdxString;
    try {
      int idx = std::stoi(stepIdxString);
      if (idx < 1 || idx > 6) {
        throw std::invalid_argument("");
      }
      stepIdx = idx;
    } catch (const std::invalid_argument& ia) {
      std::cout << "\033[1;31mInvalid argument. Try again.\033[0m" << std::endl;
    }
  }

  nodeHandle.setParam("/deep_pose/forceFood", true);
  nodeHandle.setParam("/deep_pose/forceFoodName", foodName);
  nodeHandle.setParam("/deep_pose/publish_spnet", stepIdx == 1 || stepIdx == 3 || stepIdx == 5);
  nodeHandle.setParam("/deep_pose/spnet_food_name", foodName);
  nodeHandle.setParam("/deep_pose/invertSPNetDirection", stepIdx == 4 || stepIdx == 5);
  std::this_thread::sleep_for(std::chrono::milliseconds(400));

  std::cout << std::endl << "\033[1;32mRunning bite transfer study for " << foodName << " beginning on step " << stepIdx << ".\033[0m" << std::endl;


  bool skipSkewering = getRosParam<bool>("/study/skipSkewering", nodeHandle);

  if (!skipSkewering) {
  
    // ===== ABOVE PLATE =====
    if (!autoContinueDemo)
    {
      if (!waitForUser("Move forque above plate"))
      {
        return 0;
      }
    }
    feedingDemo.moveAbovePlate();

    // ===== PERCEPTION =====
    std::vector<std::string> foodNames = getRosParam<std::vector<std::string>>("/foodItems/names", nodeHandle);
    std::vector<double> skeweringForces = getRosParam<std::vector<double>>("/foodItems/forces", nodeHandle);
    std::unordered_map<std::string, double> foodSkeweringForces;
    for (int i=0; i<foodNames.size(); i++) {
      foodSkeweringForces[foodNames[i]] = skeweringForces[i];
    }

    Eigen::Isometry3d foodTransform;
    if (adaReal)
    {
      bool foodFound = false;
      while (!foodFound) {
        foodFound = perception.perceiveFood(foodTransform, false, viewer);
        if (!foodFound) {
          std::cout << "\033[1;33mI can't see the " << foodName << "\033[0m" << std::endl;
          if (!waitForUser("Try perception again?")) {return 0;}
        }
      }
    }
    else
    {
      foodTransform = feedingDemo.getDefaultFoodTransform();
    }
    std::cout << "\033[1;32mAlright! Let's get the " << foodName << "!\033[0;32m  (Gonna skewer with " << foodSkeweringForces[foodName] << "N)\033[0m" << std::endl << std::endl;

    // ===== ABOVE FOOD =====
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
      bool perceptionSuccessful = perception.perceiveFood(foodTransform, false, viewer);
      if (!perceptionSuccessful) {
        std::cout << "\033[1;33mI can't see the " << foodName << " anymore...\033[0m" << std::endl;
      } else {
        feedingDemo.moveAboveFood(foodTransform, 0, viewer);
      }

      double zForceBeforeSkewering = 0;
      if (ftThresholdHelper.startDataCollection(20)) {
        Eigen::Vector3d currentForce, currentTorque;
        while (!ftThresholdHelper.isDataCollectionFinished(currentForce, currentTorque)) {
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        zForceBeforeSkewering = currentForce.z();
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

      double forceDifference = 100;
      if (ftThresholdHelper.startDataCollection(20)) {
        Eigen::Vector3d currentForce, currentTorque;
        while (!ftThresholdHelper.isDataCollectionFinished(currentForce, currentTorque)) {
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        forceDifference = std::fabs(zForceBeforeSkewering - currentForce.z());
      }
      ROS_INFO_STREAM("force difference: " << forceDifference);

      if (forceDifference > 0.022) {
        foodPickedUp = true;
      } else {
        std::cout << "\033[1;32mOoops! I think I didn't manage to pick up the " << foodName << "\033[0;32m" << std::endl;
        bool shouldTryAgain = waitForUser("\033[1;32mDo you want me to try again?\033[0;32m");
        if (shouldTryAgain) {
          feedingDemo.moveAbovePlate();
          bool perceptionSuccessful = perception.perceiveFood(foodTransform, false, viewer);
          if (!perceptionSuccessful) {
            std::cout << "\033[1;32mOoops! I can't find the " << foodName << " anymore! I think I lost it :(\033[0;32m" << std::endl;
            return 0;
          }
        } else {
          foodPickedUp = true;
        }
      }
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

  if (stepIdx == 2 || stepIdx == 3) {
    feedingDemo.tiltUpInFrontOfPerson(viewer);
  } else if (stepIdx == 4 || stepIdx == 5) {
    feedingDemo.tiltDownInFrontOfPerson(viewer);
  }

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
