
// #include "feeding/FTThresholdHelper.hpp"
#include "feeding/FeedingDemo.hpp"
#include "feeding/Perception.hpp"
#include "feeding/util.hpp"
#include <ros/ros.h>
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>

namespace feeding {

int demomain(FeedingDemo& feedingDemo,
                // FTThresholdHelper& ftThresholdHelper,
                Perception& perception,
                aikido::rviz::WorldInteractiveMarkerViewerPtr viewer,
                ros::NodeHandle nodeHandle,
                bool autoContinueDemo,
                bool adaReal) 
{

  // feedingDemo.printRobotConfiguration();
  // Eigen::Vector6d source, target;
  // source << -2.3114, 3.26176, 1.7677, -0.675144, 2.02327, 0.156122;
  // target << -2.3114, 3.26176, 1.7677, -0.675144, -2.0, 0.156122;
  // feedingDemo.mAdaMover->moveArmToConfiguration(source);
  // waitForUser("Start Reached");
  // feedingDemo.mAdaMover->moveArmToConfiguration(target);
  // waitForUser("Goal Reached");
  // return 0;


  // std::cout << std::endl << "\033[1;32m      ***** DEMO MODE *****\033[0m" << std::endl;
  

  // std::string foodName = "celery";

  int stepIdx = 1;

  // nodeHandle.setParam("/deep_pose/forceFood", true);
  // nodeHandle.setParam("/deep_pose/forceFoodName", foodName);
  // nodeHandle.setParam("/deep_pose/publish_spnet", (true));
  // nodeHandle.setParam("/deep_pose/spnet_food_name", foodName);
  // nodeHandle.setParam("/deep_pose/invertSPNetDirection", stepIdx == 5);
  // std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // std::cout << std::endl << "\033[1;32mRunning bite transfer study for " << foodName << " beginning on step " << stepIdx << ".\033[0m" << std::endl;


  //   bool angledSkewering = (stepIdx == 2);
  //   bool foodPickedUp = false;
  
  //   // ===== ABOVE PLATE =====
  //   if (!autoContinueDemo)
  //   {
  //     if (!waitForUser("Move forque above plate"))
  //     {
  //       return 0;
  //     }
  //   }
  //   feedingDemo.moveAbovePlate();
  //   std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  //   feedingDemo.moveAboveForque();
  //   waitForUser("Move back to plate");
  //   feedingDemo.moveAbovePlate();
  //   std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  //   // waitForUser("Exit");

  //   // return 0;


  //   // ===== PERCEPTION =====
  //   std::vector<std::string> foodNames = getRosParam<std::vector<std::string>>("/foodItems/names", nodeHandle);
  //   std::vector<double> skeweringForces = getRosParam<std::vector<double>>("/foodItems/forces", nodeHandle);
  //   std::unordered_map<std::string, double> foodSkeweringForces;
  //   for (int i=0; i<foodNames.size(); i++) {
  //     foodSkeweringForces[foodNames[i]] = skeweringForces[i];
  //   }

  //   Eigen::Isometry3d foodTransform;
  //     foodTransform = feedingDemo.getDefaultFoodTransform();

  //   // ===== ABOVE FOOD =====

  //     if (!autoContinueDemo)
  //     {
  //       if (!waitForUser("Move forque above food"))
  //       {
  //         return 0;
  //       }
  //     }
  //     if (angledSkewering) {
  //         feedingDemo.moveAboveFood(foodTransform, 0.25*M_PI, viewer, true);
  //     } else {
  //       feedingDemo.moveAboveFood(foodTransform, 0, viewer, true);
  //     }



  // // ===== IN FRONT OF PERSON =====
  // if (!autoContinueDemo)
  // {
  //   if (!waitForUser("Move forque in front of person"))
  //   {
  //     return 0;
  //   }
  // }
  // nodeHandle.setParam("/feeding/facePerceptionOn", true);
  // feedingDemo.moveInFrontOfPerson();

    
  // bool tilted = (stepIdx != 3);
  //   feedingDemo.moveTowardsPerson();
  

  // // ===== EATING =====
  // ROS_WARN("Human is eating");
  // std::this_thread::sleep_for(
  //     std::chrono::milliseconds(
  //         getRosParam<int>("/feedingDemo/waitMillisecsAtPerson", nodeHandle)));
  // // feedingDemo.ungrabAndDeleteFood();

  // if (!autoContinueDemo)
  //     {
  //       if (!waitForUser("Move away from person 1"))
  //       {
  //         return 0;
  //       }
  //       if (!waitForUser("Move away from person 2"))
  //       {
  //         return 0;
  //       }
  //     }

  // feedingDemo.printRobotConfiguration(); 
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
