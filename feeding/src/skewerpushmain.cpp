
#include "feeding/FTThresholdHelper.hpp"
#include "feeding/FeedingDemo.hpp"
#include "feeding/Perception.hpp"
#include "feeding/util.hpp"
#include <ros/ros.h>
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>

namespace feeding {

int skewerpushmain(FeedingDemo& feedingDemo,
                FTThresholdHelper& ftThresholdHelper,
                Perception& perception,
                aikido::rviz::WorldInteractiveMarkerViewerPtr viewer,
                ros::NodeHandle nodeHandle,
                bool autoContinueDemo,
                bool adaReal) {

  // Set Standard Threshold
  if (!ftThresholdHelper.setThresholds(STANDARD_FT_THRESHOLD)) {
    return 1;
  }

  bool done = false;
  while (!done) {
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
    foodTransform = feedingDemo.getDefaultFoodTransform();

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

      double zForceBeforeSkewering = 0;
      if (adaReal && ftThresholdHelper.startDataCollection(20)) {
        Eigen::Vector3d currentForce, currentTorque;
        while (!ftThresholdHelper.isDataCollectionFinished(currentForce, currentTorque)) {
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        zForceBeforeSkewering = currentForce.z();
      }

      // ===== ROTATE FORQUE ====
      std::cout << std::endl << "\033[1;32mWhat angle do you want to push food at in degrees?\033[0m     > ";
      float angle = 0;
      std::cin >> angle;
      angle *= M_PI / 180.0;
      if (!ros::ok()) {return 0;}

      if (!autoContinueDemo)
      {
        if (!waitForUser("Rotate forque in orientation to push food"))
        {
          return 0;
        }
      }
      feedingDemo.rotateForque(foodTransform, angle, viewer);

      // ===== INTO TO FOOD ====
      if (!autoContinueDemo)
      {
        if (!waitForUser("Move forque into to food"))
        {
          return 0;
        }
      }
      double torqueThreshold = 2;
      if (!ftThresholdHelper.setThresholds(STANDARD_FT_THRESHOLD))
      {
        return 1;
      }
      Eigen::Isometry3d forqueTransform;
      if (adaReal) {
          forqueTransform = perception.getForqueTransform();
      }
      feedingDemo.moveIntoFood();

      // ===== MOVE OUT OF PLATE ====
      if (!autoContinueDemo)
      {
        if (!waitForUser("Move Out of Plate"))
        {
          return 0;
        }
      }
      if (!ftThresholdHelper.setThresholds(AFTER_GRAB_FOOD_FT_THRESHOLD))
      {
        return 1;
      }
      feedingDemo.moveOutOfPlate();

      // keep pushing until user says no, get feedback on how far to move
      // ===== PUSH FOOD ====
      while (1) {
        double pushDist = 0.0;
        std::cout << std::endl << "\033[1;32mHow far do you want to push? (enter 0 to stop)\033[0m     > ";
        std::cin >> pushDist;
        if (pushDist == 0.0) {
          break;
        }

        if (!ftThresholdHelper.setThresholds(PUSH_FOOD_FT_THRESHOLD))
        {
          return 1;
        }
        //feedingDemo.grabFoodWithForque();

        if (adaReal) {
            feedingDemo.pushFood(angle, pushDist, forqueTransform);
        } else {
            feedingDemo.pushFood(angle, pushDist);
        }
      }
      break;
    }
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

    std::string doneResponse;
    std::cout << std::endl << "\033[1;32mShould we keep going? [y/n]\033[0m     > ";
    doneResponse = "";
    std::cin >> doneResponse;
    if (!ros::ok()) {return 0;}
    if (doneResponse == "n") {
      done = true;
    }
  }

  // ===== DONE =====
  waitForUser("Demo finished.");
}

};
