
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include <ros/ros.h>
#include "feeding/FTThresholdHelper.hpp"
#include "feeding/FeedingDemo.hpp"
#include "feeding/Perception.hpp"
#include "feeding/util.hpp"

namespace feeding {

int demomain(
    FeedingDemo& feedingDemo,
    FTThresholdHelper& ftThresholdHelper,
    Perception& perception,
    ros::NodeHandle nodeHandle,
    bool autoContinueDemo,
    bool adaReal)
{
  ftThresholdHelper.init();

  aikido::rviz::WorldInteractiveMarkerViewerPtr viewer
      = feedingDemo.getViewer();

  std::cout << std::endl
            << "\033[1;32m      ***** DEMO MODE *****\033[0m" << std::endl;

  while (waitForUser("next step?"))
  {

    std::cout << std::endl
              << "\033[1;32mWhich food item do you want?\033[0m" << std::endl;
    std::cout << "\033[0;32m1) Strawberry\033[0m" << std::endl;
    std::cout << "\033[0;32m2) Melon\033[0m" << std::endl;
    std::cout << "\033[0;32m3) Cantaloupe\033[0m" << std::endl;
    std::cout << "\033[0;32m4) Celery\033[0m" << std::endl;
    std::cout << "\033[0;32m5) Carrot\033[0m" << std::endl;
    std::cout << "\033[0;32m6) [Calibrate to person]\033[0m" << std::endl;
    std::cout << "\033[0;32m7) [Pick up fork]\033[0m" << std::endl;
    std::cout << "\033[0;32m8) [Put down fork]\033[0m" << std::endl;

    std::string foodName = "";
    while (foodName == "")
    {
      std::cout << "> ";
      std::string idString;
      std::cin >> idString;
      try
      {
        int id = std::stoi(idString);
        if (id < 1 || id > 8)
        {
          throw std::invalid_argument("");
        }
        switch (id)
        {
          case 1:
            foodName = "strawberry";
            break;
          case 2:
            foodName = "melon";
            break;
          case 3:
            foodName = "cantaloupe";
            break;
          case 4:
            foodName = "celery";
            break;
          case 5:
            foodName = "carrot";
            break;
          case 6:
            foodName = "calibrate";
            break;
          case 7:
            foodName = "pickupfork";
            break;
          case 8:
            foodName = "putdownfork";
            break;
        }
      }
      catch (const std::invalid_argument& ia)
      {
        std::cout << "\033[1;31mInvalid argument. Try again.\033[0m"
                  << std::endl;
      }
    }

    nodeHandle.setParam("/deep_pose/forceFood", false);
    nodeHandle.setParam("/deep_pose/forceFoodName", foodName);
    nodeHandle.setParam("/deep_pose/publish_spnet", (true));
    nodeHandle.setParam("/deep_pose/spnet_food_name", foodName);
    nodeHandle.setParam("/deep_pose/invertSPNetDirection", false);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    std::cout << std::endl
              << "\033[1;32mRunning bite transfer study for " << foodName
              << ".\033[0m" << std::endl;

    // ===== FORQUE PICKUP =====

    if (foodName == "pickupfork")
    {
      // feedingDemo.setFTSensingEnabled(false);
      feedingDemo.openHand();

      // waitForUser("Above Plate?");
      // feedingDemo.moveAbovePlate(viewer);

      waitForUser("Forque?");
      feedingDemo.moveAboveForque();

      waitForUser("In?");
      feedingDemo.moveIntoForque();

      if (waitForUser("Close?"))
      {
        feedingDemo.closeHand();
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      }

      waitForUser("Out?");
      feedingDemo.moveOutOfForque();

      waitForUser("Above Plate?");
      feedingDemo.moveAbovePlate(viewer);
    }
    else if (foodName == "putdownfork")
    {
      // feedingDemo.setFTSensingEnabled(false);
      feedingDemo.closeHand();

      // waitForUser("Above Plate?");
      // feedingDemo.moveAbovePlate(viewer);

      waitForUser("Forque?");
      feedingDemo.moveAboveForque();

      waitForUser("In?");
      feedingDemo.moveIntoForque();

      feedingDemo.openHand();
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));

      waitForUser("Out?");
      feedingDemo.moveOutOfForque();

      waitForUser("Above Plate?");
      feedingDemo.moveAbovePlate(viewer);
    }
    else
    {
      // feedingDemo.setFTSensingEnabled(true);
      // std::this_thread::sleep_for(std::chrono::milliseconds(5000));
      // ftThresholdHelper.init();
      // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

      feedingDemo.moveAbovePlate(viewer);

      bool skipSkewering
          = getRosParam<bool>("/study/skipSkewering", nodeHandle);

      if (!skipSkewering)
      {
        bool foodPickedUp = false;

        while (!foodPickedUp)
        {

          // ===== ABOVE PLATE =====
          if (!autoContinueDemo)
          {
            if (!waitForUser("Move forque above plate"))
            {
              return 0;
            }
          }
          feedingDemo.moveAbovePlate(viewer);

          // ===== PERCEPTION =====
          std::vector<std::string> foodNames
              = getRosParam<std::vector<std::string>>(
                  "/foodItems/names", nodeHandle);
          std::vector<double> skeweringForces
              = getRosParam<std::vector<double>>(
                  "/foodItems/forces", nodeHandle);
          std::unordered_map<std::string, double> foodSkeweringForces;
          for (int i = 0; i < foodNames.size(); i++)
          {
            foodSkeweringForces[foodNames[i]] = skeweringForces[i];
          }

          perception.setFoodName(foodName);
          Eigen::Isometry3d foodTransform;
          bool perceptionSuccessful
              = perception.perceiveFood(foodTransform, true, viewer);
          if (!perceptionSuccessful)
          {
            std::cout << "\033[1;33mI can't see the " << foodName
                      << "...\033[0m" << std::endl;
            continue;
          }

          // foodTransform = feedingDemo.getDefaultFoodTransform();

          // ===== ABOVE FOOD =====
          // 0 vertical
          // 1 strawberry-style
          // 2 banana-style
          int pickupAngleMode
              = getRosParam<int>("/study/pickupAngleMode", nodeHandle);
          bool angledSkewering;

          if (!autoContinueDemo)
          {
            if (!waitForUser("Move forque above food"))
            {
              return 0;
            }
          }
          if (pickupAngleMode == 1)
          {
            feedingDemo.moveAboveFood(
                foodTransform, pickupAngleMode, viewer, true);
          }
          else
          {
            feedingDemo.moveAboveFood(
                foodTransform, pickupAngleMode, viewer, true);
          }

          std::this_thread::sleep_for(std::chrono::milliseconds(800));
          perceptionSuccessful
              = perception.perceiveFood(foodTransform, true, viewer);
          if (!perceptionSuccessful)
          {
            std::cout << "\033[1;33mI can't see the " << foodName
                      << " anymore...\033[0m" << std::endl;
            continue;
          }
          else
          {
            if (pickupAngleMode == 1)
            {
              feedingDemo.moveAboveFood(
                  foodTransform, pickupAngleMode, viewer, true);
            }
            else
            {
              feedingDemo.moveAboveFood(
                  foodTransform, pickupAngleMode, viewer, true);
            }
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(200));

          // ===== INTO FOOD =====
          if (!autoContinueDemo)
          {
            if (!waitForUser("Move forque into food"))
            {
              return 0;
            }
          }
          double zForceBeforeSkewering = 0;
          if (ftThresholdHelper.startDataCollection(50))
          {
            Eigen::Vector3d currentForce, currentTorque;
            while (!ftThresholdHelper.isDataCollectionFinished(
                currentForce, currentTorque))
            {
              std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            zForceBeforeSkewering = currentForce.x();
          }
          double torqueThreshold = 2;
          if (!ftThresholdHelper.setThresholds(
                  foodSkeweringForces[foodName], torqueThreshold))
          {
            return 1;
          }
          if (adaReal)
          {
            feedingDemo.moveIntoFood(&perception, viewer);
          }
          else
          {
            feedingDemo.moveIntoFood();
          }
          std::this_thread::sleep_for(
              std::chrono::milliseconds(
                  getRosParam<int>(
                      "/feedingDemo/waitMillisecsAtFood", nodeHandle)));
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
          if (ftThresholdHelper.startDataCollection(50))
          {
            Eigen::Vector3d currentForce, currentTorque;
            while (!ftThresholdHelper.isDataCollectionFinished(
                currentForce, currentTorque))
            {
              std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            zForceAfter = currentForce.x();
            forceDifference = zForceBeforeSkewering - currentForce.x();
          }
          ROS_WARN_STREAM(
              "force difference: " << (zForceBeforeSkewering - zForceAfter));

          if (forceDifference < 0 || true)
          {
            foodPickedUp = true;
            ROS_INFO_STREAM("FOOD PICKED UP!");
          }
          else
          {
            std::cout
                << "\033[1;32mOoops! I think I didn't manage to pick up the "
                << foodName << ". Let me try again!\033[0;32m" << std::endl;
            feedingDemo.moveAbovePlate(viewer);
          }

          // bool shouldContinue = waitForUser("\033[1;32mDo you want me to
          // continue?\033[0;32m");
          // if (shouldContinue) {
          //   foodPickedUp = true;
          // }
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
      nodeHandle.setParam("/feeding/facePerceptionOn", true);
      feedingDemo.moveInFrontOfPerson();

      bool tilted = true;
      feedingDemo.moveTowardsPerson(&perception, viewer);

      if (tilted)
      {
        feedingDemo.tiltUpInFrontOfPerson(viewer);
      }

      // ===== EATING =====
      ROS_WARN("Human is eating");
      std::this_thread::sleep_for(
          std::chrono::milliseconds(
              getRosParam<int>(
                  "/feedingDemo/waitMillisecsAtPerson", nodeHandle)));
      feedingDemo.ungrabAndDeleteFood();

      if (!autoContinueDemo)
      {
        if (!waitForUser("Move away from person 1"))
        {
          return 0;
        }
        if (!waitForUser("Move away from person 2"))
        {
          return 0;
        }
      }

      if (!tilted)
      {
        feedingDemo.moveAwayFromPerson();
      }

      // ===== BACK TO PLATE =====
      if (!autoContinueDemo)
      {
        if (!waitForUser("Move back to plate"))
        {
          return 0;
        }
      }
      feedingDemo.moveAbovePlate(viewer);
    }
  }

  // ===== DONE =====
  ROS_INFO("Demo finished.");
}
};
