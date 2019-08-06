#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include <aikido/statespace/Rn.hpp>
#include <mcheck.h>
#include <pr_tsr/plate.hpp>
#include <ros/ros.h>
#include <libada/util.hpp>

#include "feeding/FTThresholdHelper.hpp"
#include "feeding/FeedingDemo.hpp"
#include "feeding/util.hpp"
#include "feeding/perception/Perception.hpp"
#include "feeding/DataCollector.hpp"
#include "feeding/ranker/SuccessRateRanker.hpp"
#include "feeding/ranker/ShortestDistanceRanker.hpp"

#include "experiments.hpp"

using ada::util::getRosParam;
using ada::util::waitForUser;

///
/// OVERVIEW OF FEEDING DEMO CODE
///
/// First, everything is initalized.
/// The FeedingDemo object is responsible for robot and the workspace.
/// The FTThresholdController sets the thresholds in the
/// MoveUntilTouchController
/// The Perception object can perceive food.
///
/// Then the demo is run step by step.
///
int main(int argc, char** argv)
{
  using namespace feeding;


  // ===== STARTUP =====

  // Is the real robot used or simulation?
  bool adaReal = false;

  // Should the demo continue without asking for human input at each step?
  bool autoContinueDemo = false;

  // the FT sensing can stop trajectories if the forces are too big
  bool useFTSensingToStopTrajectories = false;

  bool TERMINATE_AT_USER_PROMPT = true;

  // std::string demoType{"nips"};
  std::string demoType{"ds"};

  // Arguments for data collection.
  std::string foodName{"testItem"};
  std::string dataCollectorPath;
  std::size_t directionIndex{0};
  std::size_t trialIndex{0};
  std::string scenario;

  handleArguments(argc, argv,
    adaReal, autoContinueDemo, useFTSensingToStopTrajectories,
    demoType, foodName, directionIndex, trialIndex, scenario, dataCollectorPath);

  bool useVisualServo = true;
  bool allowRotationFree = true;

  std::cout << "Demo type " << demoType << std::endl;
  bool collect = demoType.rfind("collect") != std::string::npos;

  // If demo type starts with "collect", don't use visualServo
  if (collect)
  {
    useVisualServo = false;
    allowRotationFree = false;
  }

  if (!adaReal)
    ROS_INFO_STREAM("Simulation Mode: " << !adaReal);

  std::cout << "collect " << collect << std::endl;
  if (dataCollectorPath == "" && collect)
    throw std::invalid_argument("Need to provide output path");

  ROS_INFO_STREAM("DemoType: " << demoType);
  ROS_INFO_STREAM("useFTSensingToStopTrajectories " << useFTSensingToStopTrajectories);
  ROS_INFO_STREAM("DataCollectorPath: " << dataCollectorPath);
  ROS_INFO_STREAM("FoodName: " << foodName);

  #ifndef REWD_CONTROLLERS_FOUND
    ROS_WARN_STREAM(
        "Package rewd_controllers not found. The F/T sensor connection is not "
        "going to work.");
  #endif

  // start node
  ros::init(argc, argv, "feeding");
  ros::NodeHandle nodeHandle("~");
  nodeHandle.setParam("/feeding/facePerceptionOn", false);
  ros::AsyncSpinner spinner(2); // 2 threads
  spinner.start();

  std::shared_ptr<FTThresholdHelper> ftThresholdHelper = nullptr;

  if (useFTSensingToStopTrajectories)
  {
    std::cout << "Construct FTThresholdHelper" << std::endl;
    ftThresholdHelper = std::make_shared<FTThresholdHelper>(
    adaReal && useFTSensingToStopTrajectories, nodeHandle);
  }

  // start demo
  std::cout<<"hi"<<std::endl;
  auto feedingDemo = std::make_shared<FeedingDemo>(
    adaReal,
    nodeHandle,
    useFTSensingToStopTrajectories,
    useVisualServo,
    allowRotationFree,
    ftThresholdHelper,
    autoContinueDemo);
  std::cout<<"hi"<<std::endl;

  std::shared_ptr<TargetFoodRanker> ranker;

  if (demoType == "nips" || demoType == "kinova" || demoType == "scoop" || demoType == "ds")
  {
    ranker = std::make_shared<ShortestDistanceRanker>();
  }
  else if (demoType == "spanet" || demoType == "collect_spanet")
  {
    std::cout << "Create SuccessRateRanker" << std::endl;
    ranker = std::make_shared<SuccessRateRanker>();
  }

  auto perception = std::make_shared<Perception>(
      feedingDemo->getWorld(),
      feedingDemo->getAda()->getMetaSkeleton(),
      &nodeHandle,
      ranker,
      demoType != "spanet");

  if (ftThresholdHelper)
    ftThresholdHelper->init();

  feedingDemo->getAda()->closeHand(); 

  feedingDemo->setPerception(perception);

  ROS_INFO_STREAM("Startup complete.");

  // feedingDemo->moveToStartConfiguration();

  if (demoType == "nips")
  {
    demo(*feedingDemo, perception, nodeHandle);
  }
  else if (demoType == "spanet")
  {
    spanetDemo(*feedingDemo, perception, nodeHandle);
  }
  else if (demoType == "collect_spanet")
  {
    ROS_INFO_STREAM("Data will be saved at " << dataCollectorPath << "." << std::endl);
    DataCollector dataCollector(
      feedingDemo, feedingDemo->getAda(), nodeHandle, autoContinueDemo, adaReal, adaReal, dataCollectorPath);

    dataCollector.skewerWithSPANet(foodName, trialIndex, scenario, perception, nodeHandle);

  }
  else if (demoType == "kinova")
  {
    ROS_INFO_STREAM("Start Kinova Scoop Demo.");
    kinovaScoopDemo(*feedingDemo, nodeHandle);
  }
  else if (demoType == "ds")
  {
    ROS_INFO_STREAM("Start Detect and Scoop Demo.");
    DetectScoopDemo(*feedingDemo, nodeHandle);
  }
  else 
  {
    ROS_WARN_STREAM("unknown demoType option");
  }
  // {
    // ROS_INFO_STREAM("Data will be saved at " << dataCollectorPath << "." << std::endl);
    // DataCollector dataCollector(
    //   feedingDemo, feedingDemo->getAda(), nodeHandle, autoContinueDemo, adaReal, perceptionReal, dataCollectorPath);

    // if (StringToAction.find(demoType) == StringToAction.end())
    // {
    //   throw std::invalid_argument(demoType + "not recognized.");
    // }

    // if (StringToAction.at(demoType) == Action::IMAGE_ONLY)
    // {
    //   dataCollector.collect_images(foodName);
    // }
    // else
    // {
    //   dataCollector.collect(StringToAction.at(demoType), foodName, directionIndex, trialIndex);
    // }

  // }

  return 0;
}

