// Standard C++ libraries
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <queue>
#include <vector>
#include <Eigen/Dense>

// Boost libraries
#include <boost/shared_ptr.hpp>
#include <boost/property_map/dynamic_property_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphml.hpp>
#include <boost/function.hpp>
#include <boost/program_options.hpp>

// OMPL base libraries
#include <ompl/base/Planner.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>

// PRL libraries
#include <aikido/constraint/dart/TSR.hpp>
#include <aikido/trajectory/Interpolated.hpp>
#include <aikido/constraint/dart/CollisionFree.hpp>
#include <aikido/constraint/Testable.hpp>
#include <aikido/io/CatkinResourceRetriever.hpp>
#include <aikido/planner/ompl/GeometricStateSpace.hpp>
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/planner/ompl/OMPLConfigurationToConfigurationPlanner.hpp>
#include <aikido/planner/ConfigurationToConfigurationPlanner.hpp>
#include <aikido/planner/ConfigurationToConfiguration.hpp>

#include <dart/dart.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <dart/collision/CollisionGroup.hpp>
#include <dart/collision/CollisionOption.hpp>
#include <dart/collision/CollisionDetector.hpp>

#include <libada/Ada.hpp>
#include <feeding/Workspace.hpp>
#include <libada/util.hpp>

// Custom header files
#include "gls/GLS.hpp"
#include "gls/event/ShortestPathEvent.hpp"
#include "gls/selector/ForwardSelector.hpp"


namespace po = boost::program_options;

using aikido::rviz::WorldInteractiveMarkerViewer;
using aikido::planner::ConfigurationToConfiguration;
using aikido::planner::ompl::OMPLConfigurationToConfigurationPlanner;
using aikido::constraint::dart::CollisionFree;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using dart::collision::CollisionDetectorPtr;
using dart::collision::CollisionGroup;
using dart::dynamics::SkeletonPtr;

using ada::util::createIsometry;
using ada::util::getRosParam;

static const std::string topicName("dart_markers");
static const std::string baseFrameName("map");


dart::common::Uri adaUrdfUri{
    "package://ada_description/robots_urdf/ada_with_camera.urdf"};
dart::common::Uri adaSrdfUri{
    "package://ada_description/robots_urdf/ada_with_camera.srdf"};


// ============================================================================
void waitForUser(std::string message)
{
  std::string completeMessage = message + " Press [Enter] to continue.";
  std::cout << completeMessage << std::endl;
  std::cin.get();
}

// ============================================================================
std::vector<std::vector<double>> readStatesFromFile(std::string filename)
{
  std::ifstream inputFile(filename);

  std::vector<std::vector<double>> configurations;
  if (inputFile)
  {
    while (true)
    {
      std::string line;
      double value;

      std::getline(inputFile, line);

      std::stringstream ss(
            line, std::ios_base::out|std::ios_base::in|std::ios_base::binary);

      if (!inputFile)
        break;
      std::vector<double> row;
      while (ss >> value)
      {
        row.emplace_back(value);
      }
      configurations.emplace_back(row);
    }
  }
  return configurations;
}

int main(int argc, char *argv[])
{
  po::options_description desc("HERB Planner Options");
  desc.add_options()
      ("help,h", "produce help message")
      ("roadmapfile,f", po::value<std::string>()->required(), "Path to Graph")
  ;

  // Read arguments
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    std::cout << desc << std::endl;
    return 1;
  }

  std::string graph_file(vm["roadmapfile"].as<std::string>());

  /// HERB ENVIRONMENT
  ROS_INFO("Starting ROS node.");
  ros::init(argc, argv, "gls_demo");
  ros::NodeHandle nh("ada");

  // Create AIKIDO World
  aikido::planner::WorldPtr env(new aikido::planner::World("gls_demo"));

  // Load ROMAN either in simulation or real based on arguments
  ROS_INFO("Loading Ada...");

  auto robot = std::make_shared<ada::Ada>(env, true, adaUrdfUri, adaSrdfUri);

  Eigen::Isometry3d robotPose = createIsometry(
      getRosParam<std::vector<double>>("/ada/baseFramePose", nh));
  auto workspace
      = std::make_shared<feeding::Workspace>(env, robotPose, false, nh);

  // Visualization topics
  static const std::string execTopicName = topicName + "/gls_demo";

  // Start the RViz viewer.
  ROS_INFO_STREAM("Starting viewer. Please subscribe to the '" << execTopicName << "' InteractiveMarker topic in RViz.");
  WorldInteractiveMarkerViewer viewer(env, execTopicName, baseFrameName);
  viewer.setAutoUpdate(true);

  auto armSkeleton = robot->getArm()->getMetaSkeleton();
  auto armSpace = robot->getArm()->getStateSpace();

  waitForUser("Press key to show graph");

  Eigen::VectorXd currentConfiguration(8);
  auto currentState = armSpace->createState();

  // For each edge, set the position in a loop, draw a black line.
  std::vector<std::vector<double>> edges = readStatesFromFile(graph_file);

  // Visualize the Graph. Visualize only a 400 edges.
  int maxEdges = std::min(200, (int)edges.size());
  for (int siter = 0; siter < maxEdges; ++siter)
  {
    std::vector<double> edge = edges[siter];

    auto edgeTrajectory = std::make_shared<aikido::trajectory::Interpolated>(
         armSpace, std::make_shared<aikido::statespace::GeodesicInterpolator>(armSpace));

    Eigen::VectorXd sourceConfiguration(6);
    sourceConfiguration << edge[0], edge[1], edge[2], edge[3], edge[4], edge[5];
    std::cout << "Position is " << sourceConfiguration.transpose() << std::endl;
    armSkeleton->setPositions(sourceConfiguration);

    Eigen::VectorXd targetConfiguration(6);
    targetConfiguration << edge[6], edge[7], edge[8], edge[9], edge[10], edge[11];

    double maxNum = 5.0;
    for (int idx = 0; idx < maxNum+1; ++idx)
    {
      currentConfiguration = sourceConfiguration + (targetConfiguration - sourceConfiguration)*idx/maxNum;
      armSpace->convertPositionsToState(currentConfiguration, currentState);
      edgeTrajectory->addWaypoint(edgeTrajectory->getNumWaypoints(), currentState);
      std::cout << "Current Position: " << armSkeleton->getPositions() << std::endl;
      auto marker = viewer.addTrajectoryMarker(edgeTrajectory,
                                              armSkeleton,
                                              *(robot->getHand()->getEndEffectorBodyNode()),
                                              Eigen::Vector4d(0.8, 0.8, 0.8, 1.0),
                                              0.001, edgeTrajectory->getNumWaypoints());
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }



  waitForUser("Press enter to exit.");
  return 0;
}
