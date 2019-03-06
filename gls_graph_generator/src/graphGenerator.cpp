#include <chrono>
#include <iostream>
#include <Eigen/Dense>
#include <cassert>
#include <cmath>

#include <aikido/statespace/SE2.hpp>
#include <aikido/constraint/Satisfied.hpp>
#include <aikido/constraint/Testable.hpp>
#include <aikido/constraint/dart/CollisionFree.hpp>
#include <aikido/planner/World.hpp>
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include <boost/program_options.hpp>

#include <dart/dart.hpp>
#include <libada/Ada.hpp>
#include <libada/util.hpp>

#include <feeding/Workspace.hpp>
#include "configurations.hpp"

bool romanSim = true;

static const std::string topicName("dart_markers");
static const std::string baseFrameName("map");

static const double planningTimeout{155.};

using dart::dynamics::MetaSkeletonPtr;
using dart::dynamics::SkeletonPtr;
using dart::collision::CollisionDetectorPtr;
using dart::collision::CollisionGroup;
using aikido::constraint::dart::CollisionFree;
using aikido::statespace::dart::MetaSkeletonStateSpacePtr;
using aikido::constraint::dart::CollisionFreePtr;
using aikido::constraint::TestablePtr;

using aikido::statespace::dart::MetaSkeletonStateSpace;

using ada::util::createIsometry;
using ada::util::getRosParam;

namespace po = boost::program_options;

dart::common::Uri adaUrdfUri{
    "package://ada_description/robots_urdf/ada_with_camera.urdf"};
dart::common::Uri adaSrdfUri{
    "package://ada_description/robots_urdf/ada_with_camera.srdf"};

// =====================================================================================
Eigen::VectorXd haltonSequence(
    int index,
    Eigen::VectorXd offset,
    Eigen::VectorXd lowerLimits,
    Eigen::VectorXd upperLimits)
{
  int dimension = lowerLimits.size();
  Eigen::VectorXd difference(dimension);
  difference = upperLimits - lowerLimits;

  // Catering to only ADA and ROMAN for now.
  Eigen::VectorXd bases(dimension);
  bases << 2, 3, 5, 7, 11, 13;

  // Generate the halton config
  Eigen::VectorXd config(dimension);
  for (int i = 0; i < dimension; ++i)
  {
    int base = bases[i];
    int tempIndex = index;
    double result = 0;
    double f = 1;

    while (tempIndex > 0)
    {
      f = f/base;
      result = result + f*(tempIndex % base);
      tempIndex = tempIndex/base;
    }
    config[i] = result;
  }

  // Offset the halton sequence
  config = config + offset;

  // Wrap Around
  for (int i = 0; i < dimension; ++i)
  {
    if (config[i] > 1.0)
      config[i] = config[i] - 1.0;
    if (config[i] < 0.0)
      config[i] = 1.0 + config[i];
  }

  // Scale the configurations
  Eigen::VectorXd scaledConfig(dimension);
  for (int i = 0; i < dimension; ++i)
  {
    scaledConfig(i) = lowerLimits(i) + config(i) * difference(i);
  }
  return scaledConfig;
}

// =====================================================================================
void generateHaltonPoints(const std::shared_ptr<ada::Ada>& robot,
                          const MetaSkeletonStateSpacePtr& space,
                          const MetaSkeletonPtr& skeleton,
                          const TestablePtr& constraint,
                          const std::vector<Eigen::VectorXd>& presetConfigurations,
                          int numSamples,
                          double threshold,
                          double resolution = 0.1)
{
  // GILWOO: Change these to locations you want to save the temporary data to.

  // Holds the vertices. Index is the line number. Content is the configuration.
  std::string vertexFile = "vertices.txt";

  // Holds the edge information. <source vertex ID> <target vertex ID> <length>
  std::string edgesFile = "edges.txt";

  // Holds each edge's source and target configurations. Useful to visualize the edges.
  // Not necessary for graph generation.
  std::string edgesVizFile = "edgesViz.txt";

  int numVertices = 0;
  int index = 1;
  std::srand((unsigned int) time(0));

  // Generate a uniform offset
  Eigen::VectorXd offset = Eigen::VectorXd::Random(space->getDimension());

  auto lowerLimits = skeleton->getPositionLowerLimits();
  auto upperLimits = skeleton->getPositionUpperLimits();

  std::vector<int> continuousJoints{0, 3, 4, 5};
  for (const auto& i : continuousJoints)
  {
    lowerLimits(i) = -M_PI;
    upperLimits(i) = M_PI;
  }

  skeleton->setPositionLowerLimits(lowerLimits);
  skeleton->setPositionUpperLimits(upperLimits);

  std::vector<Eigen::VectorXd> configurations;

  // ==========================================================
  // Graph Generation Begins Here: Sampled generated in a loop.
  // ==========================================================

  for(const auto& config : presetConfigurations)
  {
    auto testState = space->createState();
    space->convertPositionsToState(config, testState);
    if (!constraint->isSatisfied(testState))
      continue;
    configurations.push_back(config);
  }

  std::cout << "Got " << configurations.size() << " from initial seeds" << std::endl;
  while (true)
  {
    // if (numVertices % 100 == 0)
    //   std::cout << "Num vertices " << numVertices << std::endl;

    Eigen::VectorXd config = haltonSequence(index, offset, lowerLimits, upperLimits);
    index++;

    auto testState = space->createState();
    space->convertPositionsToState(config, testState);

    if(!constraint->isSatisfied(testState))
      continue;

    configurations.push_back(config);

    numVertices++;
    if (numVertices == numSamples)
    {
      std::cout << "Sampled " << numSamples << " samples " << std::endl;
      break;
    }
  }

  // Write to file
  std::ofstream vertexLogFile;
  vertexLogFile.open(vertexFile, std::ios_base::app);

  for(const auto& config : configurations)
  {
    for (int i = 0; i < space->getDimension()-1; ++i)
      vertexLogFile << config[i] << " ";
    vertexLogFile << config[space->getDimension()-1] << std::endl;
  }
  vertexLogFile.close();

  // =================================================
  // Vertices generated. Now we try to make the edges.
  // =================================================
  std::ofstream edgesLogFile;
  edgesLogFile.open(edgesFile, std::ios_base::app);

  std::ofstream edgesVizLogFile;
  edgesVizLogFile.open(edgesVizFile, std::ios_base::app);

  assert(configurations.size() == numSamples);

  std::size_t connectedEdges = 0;

  for (std::size_t i = 0; i < configurations.size()-1; ++i)
  {
    if (i % 100 == 0)
      std::cout << "Currently on " << i << std::endl;
    for (std::size_t j = i+1; j < configurations.size(); ++j)
    {
      // Check if vertices are closeby.
      auto distance = (configurations[i] - configurations[j]).norm();

      if (distance > threshold)
        continue;

      // Check if the edge is collision-free with step-size.
      for (int step = 1; step < 10; ++step)
      {
        auto midConfiguration = configurations[i] + resolution*step*(configurations[j] - configurations[i]);
        auto midState = space->createState();
        space->convertPositionsToState(midConfiguration, midState);
        if (!constraint->isSatisfied(midState))
          continue;
      }

      connectedEdges++;

      edgesLogFile << i << " " << j << " " << distance << std::endl;

      for (int index = 0; index < space->getDimension(); ++index)
      {
        edgesVizLogFile << configurations[i](index) << " ";
      }

      for (int index = 0; index < space->getDimension(); ++index)
      {
        edgesVizLogFile << configurations[j](index) << " ";
      }
      edgesVizLogFile << std::endl;

    }
  }
  edgesLogFile.close();
  std::cout << "Total Vertices " << configurations.size() << " Edge " << connectedEdges << std::endl;
}

// =====================================================================================
int main(int argc, char** argv)
{

  po::options_description desc("Robot graph generation options");
  desc.add_options()
      ("help,h", "produce help message")
      ("numSamples,n", po::value<int>()->required(), "number of samples")
      ("threshold,t", po::value<double>()->required(), "radius of connection");

  // // GILWOO: Threshold is provided by the user.
  // auto lowerThreshold = 8*(std::pow(numSamples, -1.0/8.0)) + 1;
  // int threshold = (int)lowerThreshold;
  // std::cout << "Threshold is: " << threshold << std::endl;
  // std::cin.get();

  // Read arguments
  po::variables_map vm;

  // po::store(po::parse_command_line(argc, argv, desc), vm);
  po::store(po::command_line_parser(argc, argv).options(desc).style(
  po::command_line_style::unix_style ^ po::command_line_style::allow_short
  ).run(), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    std::cout << desc << std::endl;
    return 1;
  }

  int numSamples(vm["numSamples"].as<int>());
  double threshold(vm["threshold"].as<double>());

  // Default options for flags
  ROS_INFO("Starting ROS node.");
  ros::init(argc, argv, "ada_demo");
  ros::NodeHandle nh("ada");

  // Create AIKIDO World
  aikido::planner::WorldPtr env(new aikido::planner::World("ada"));

  // Load Ada either in simulation or real based on arguments
  ROS_INFO("Loading Ada...");
  auto robot = std::make_shared<ada::Ada>(env, true, adaUrdfUri, adaSrdfUri);

  auto armSkeleton = robot->getArm()->getMetaSkeleton();
  auto armSpace = robot->getArm()->getStateSpace();

  // static const std::string execTopicName = topicName + "/ada";
  // Start the RViz viewer.
  // ROS_INFO_STREAM(
  //     "You can view ADA in RViz now. \n"
  //     << "Starting viewer. Please subscribe to the '"
  //     << execTopicName
  //     << "' InteractiveMarker topic in RViz.");
  // aikido::rviz::WorldInteractiveMarkerViewer viewer(
  //     env, execTopicName, baseFrameName);
  // Add ADA to the viewer.
  // viewer.setAutoUpdate(true);
  // viewer.addFrame(robot->getRightHand()->getEndEffectorBodyNode(), 0.2, 0.01, 1.0);

  // TODO[GL]: set with seed configurations
  auto presetConfigurations = getSeedConfigurationsForFeeding();

  Eigen::Isometry3d robotPose = createIsometry(
      getRosParam<std::vector<double>>("/ada/baseFramePose", nh));
  auto workspace
      = std::make_shared<feeding::Workspace>(env, robotPose, false, nh);

  // Setting up collisions
  dart::collision::CollisionDetectorPtr collisionDetector
      = dart::collision::FCLCollisionDetector::create();
  std::shared_ptr<dart::collision::CollisionGroup> armCollisionGroup
      = collisionDetector->createCollisionGroup(
          robot->getMetaSkeleton().get(),
          robot->getHand()->getEndEffectorBodyNode());
  std::shared_ptr<dart::collision::CollisionGroup> envCollisionGroup
      = collisionDetector->createCollisionGroup(
          workspace->getTable().get(),
          workspace->getWorkspaceEnvironment().get(),
          workspace->getWheelchair().get());

  auto collisionFreeConstraint
      = std::make_shared<aikido::constraint::dart::CollisionFree>(
          armSpace, armSkeleton, collisionDetector);
  collisionFreeConstraint->addPairwiseCheck(
      armCollisionGroup, envCollisionGroup);
  auto fullCollisionConstraint = robot->getFullCollisionConstraint(
      armSpace, armSkeleton, collisionFreeConstraint);

  generateHaltonPoints(robot, armSpace, armSkeleton, fullCollisionConstraint,
    presetConfigurations, numSamples, threshold);

  return 0;
}