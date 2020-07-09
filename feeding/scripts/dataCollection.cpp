
#include <aikido/rviz/InteractiveMarkerViewer.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <libada/util.hpp>

#include "feeding/FeedingDemo.hpp"
#include "feeding/util.hpp"
#include "feeding/action/MoveAbovePlate.hpp"
#include "feeding/action/MoveOutOf.hpp"
#include "feeding/action/MoveInto.hpp"
#include "feeding/action/DetectAndMoveAboveFood.hpp"
#include <cstdlib>
#include <ctime>

#include <yaml-cpp/yaml.h>

using ada::util::getRosParam;
using ada::util::waitForUser;

namespace feeding {

void dataCollection(
    FeedingDemo& feedingDemo,
    std::shared_ptr<Perception>& perception,
    ros::NodeHandle nodeHandle)
{

  ROS_INFO_STREAM("==========  DATA COLLECTION ==========");

  auto ada = feedingDemo.getAda();
  auto workspace = feedingDemo.getWorkspace();
  auto collisionFree = feedingDemo.getCollisionConstraint();
  auto plate = workspace->getPlate()->getRootBodyNode()->getWorldTransform();

  srand(time(NULL));

  std::string dataPath(ros::package::getPath("ada_demos"));
  dataPath = dataPath + getRosParam<std::string>("/acquisitionData/datasetDir", nodeHandle);
  ROS_INFO_STREAM("Writing data to: " << dataPath);

  while (true)
  {
    if (feedingDemo.getFTThresholdHelper())
        feedingDemo.getFTThresholdHelper()->setThresholds(STANDARD_FT_THRESHOLD);
    
    // Get food name
    std::string foodName;
    std::cout << "Food name: ";
    std::cin >> foodName;
    if (foodName == std::string("quit")) {
        break;
    }

    ROS_INFO_STREAM("Running data collection for " << foodName);

    // Get action
    int action = -1;
    double pitch = 0.0;
    double roll = 0.0;
    std::cout << "Action List (all Force 10N, Rotate 0, Liftoff 0)" << std::endl;
    std::cout << "[0] Pitch 0, Roll 0" << std::endl;
    std::cout << "[1] Pitch 0, Roll 90" << std::endl;
    std::cout << "[2] Pitch -0.5, Roll 0" << std::endl;
    std::cout << "[3] Pitch -0.5, Roll 90" << std::endl;
    std::cout << "[4] Pitch 0.4, Roll 0" << std::endl;
    std::cout << "[5] Pitch 0.4, Roll 90" << std::endl;
    while(action < 0 || action > 5) {
        std::cout << std::endl << "Select Action: ";
        std::cin >> action;
    }

    // Construct file name
    std::string fileName = foodName;
    switch(action) {
        case 0:
        fileName += "_0_0_10_0_0/";
        pitch = 0.0;
        roll = 0.0;
        break;
        case 1:
        fileName += "_0_1.57_10_0_0/";
        pitch = 0.0;
        roll = 1.57;
        break;
        case 2:
        fileName += "_-0.5_0_10_0_0/";
        pitch = -0.5;
        roll = 0.0;
        break;
        case 3:
        fileName += "_-0.5_1.57_10_0_0/";
        pitch = -0.5;
        roll = 1.57;
        break;
        case 4:
        fileName += "_0.4_0_10_0_0/";
        pitch = 0.4;
        roll = 0.0;
        break;
        case 5:
        fileName += "_0.4_1.57_10_0_0/";
        pitch = 0.4;
        roll = 1.57;
        break;
    }
    std::string folderName = dataPath + fileName;
    ROS_INFO_STREAM("Saving to folder: " << folderName);

    // Move above plate
    ROS_INFO_STREAM("Move above plate");
    bool abovePlateSuccess = action::moveAbovePlate(
        ada,
        collisionFree,
        plate,
        feedingDemo.getPlateEndEffectorTransform(),
        feedingDemo.mPlateTSRParameters.at("horizontalTolerance"),
        feedingDemo.mPlateTSRParameters.at("verticalTolerance"),
        feedingDemo.mPlateTSRParameters.at("rotationTolerance"),
        feedingDemo.mPlanningTimeout,
        feedingDemo.mMaxNumTrials,
        feedingDemo.mVelocityLimits);
    if (!abovePlateSuccess) {
      ROS_WARN_STREAM("Error moving above plate");
      break;
    }

    // Save cropped image to `image.png`
    std::string imageFile = folderName + "image.png";
    nodeHandle.setParam("/acquisitionData/imageFile", imageFile);
    while(imageFile != "") {
      ROS_INFO_STREAM("Saving image to..." + imageFile);
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      nodeHandle.getParam("/acquisitionData/imageFile", imageFile);
    }

    // Move above food item
    ROS_INFO_STREAM("Detect and move above food");
    auto item = action::detectAndMoveAboveFood(
            ada,
            collisionFree,
            perception,
            foodName,
            feedingDemo.mFoodTSRParameters.at("height"),
            feedingDemo.mFoodTSRParameters.at("horizontalTolerance"),
            feedingDemo.mFoodTSRParameters.at("verticalTolerance"),
            feedingDemo.mFoodTSRParameters.at("rotationTolerance"),
            feedingDemo.mFoodTSRParameters.at("tiltTolerance"),
            feedingDemo.mPlanningTimeout,
            feedingDemo.mMaxNumTrials,
            feedingDemo.mVelocityLimits,
            &feedingDemo,
            nullptr,
            action);

    // Re-tare force, set to move-in threshold
    ROS_INFO_STREAM("Setting force thresholds and re-taring...");
    if (feedingDemo.getFTThresholdHelper()) {
        feedingDemo.getFTThresholdHelper()->setThresholds(10.0, 10.0, true);
    }

    // Start Force Data Collection
    ROS_INFO_STREAM("Starting force data collection");
    if (feedingDemo.getFTThresholdHelper()) {
      feedingDemo.getFTThresholdHelper()->startDataCollection();
    }

    // Move into food item
    ROS_INFO_STREAM("Skewering...");
    auto moveIntoSuccess = action::moveInto(
        ada,
        perception,
        &nodeHandle,
        feedingDemo.getFTThresholdHelper(),
        feedingDemo.mServoVelocity,
        item->getUid());

    if (!moveIntoSuccess)
    {
      ROS_ERROR_STREAM("Failed to skewer food.");
      break;
    }

    // Stop force data collection
    ROS_INFO_STREAM("Stopping force data collection");
    if (feedingDemo.getFTThresholdHelper()) {
      feedingDemo.getFTThresholdHelper()->stopDataCollection();
    }

    // Move out of food item
    ROS_INFO_STREAM("Move out of food item");
    Eigen::Vector3d direction(0, 0, 1);
    action::moveOutOf(
        ada,
        nullptr,
        TargetItem::FOOD,
        feedingDemo.mMoveOufOfFoodLength * 2.0,
        direction,
        feedingDemo.mPlanningTimeout,
        feedingDemo.mEndEffectorOffsetPositionTolerance,
        feedingDemo.mEndEffectorOffsetAngularTolerance,
        feedingDemo.getFTThresholdHelper(),
        feedingDemo.mVelocityLimits);

    // Record success / action / food name to `result.yaml`
    ROS_INFO_STREAM("Wait before determining success...");
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    bool success;
    std::cout << "Did I succeed?";
    std::cin >> success;
    if (success)
    {
      ROS_INFO_STREAM("Recording success...");
    } else {
      ROS_INFO_STREAM("Recording failure...");
    }

    YAML::Node node;
    node["foodName"] = foodName;
    node["action"].push_back(pitch);
    node["action"].push_back(roll);
    node["action"].push_back(10.0);
    node["action"].push_back(0.0);
    node["action"].push_back(0.0);
    node["actionNum"] = action;
    node["success"] = success;

    std::ofstream yamlFile(folderName + "result.yaml");
    if(yamlFile.is_open()) {
      yamlFile << node;
      yamlFile.close();
    } else {
      ROS_ERROR_STREAM("Could not open result.yaml");
      break;
    }

    // Record haptics to forces.csv
    ROS_INFO_STREAM("Writing haptic data...");
    if (feedingDemo.getFTThresholdHelper()) {
      feedingDemo.getFTThresholdHelper()->writeDataToFile(folderName + "forces.csv");
    }

    ROS_INFO("Done! Replace food item...\n\n");

  } // end while

  // ===== DONE =====
  ROS_INFO("Data collection finished.");
}
};
