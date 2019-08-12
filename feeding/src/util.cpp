#include "feeding/util.hpp"
#include <algorithm>
#include <aikido/common/Spline.hpp>
#include <aikido/common/StepSequence.hpp>
#include <aikido/distance/NominalConfigurationRanker.hpp>
#include <aikido/distance/defaults.hpp>
#include <aikido/planner/parabolic/ParabolicTimer.hpp>
#include <aikido/statespace/CartesianProduct.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/trajectory/Interpolated.hpp>
#include <dart/common/StlHelpers.hpp>
#include <tf_conversions/tf_eigen.h>
#include <libada/util.hpp>
#include <cstdlib>
#include "std_msgs/String.h"

static const std::vector<double> weights = {1, 1, 0.01, 0.01, 0.01, 0.01};

using aikido::distance::NominalConfigurationRanker;

namespace feeding {

inline int sgn(double x)
{
  return (x < 0) ? -1 : (x > 0);
}

//==============================================================================
void handleArguments(
    int argc,
    char** argv,
    bool& adaReal,
    bool& autoContinueDemo,
    bool& useFTSensing,
    std::string& demoType,
    std::string& foodName,
    std::size_t& directionIndex,
    std::size_t& trialIndex,
    std::string& dataCollectorPath,
    const std::string& description)
{
  namespace po = boost::program_options;

  // Default options for flags
  po::options_description po_desc(description);
  po_desc.add_options()("help,h", "Produce help message")(
      "adareal,a", po::bool_switch(&adaReal), "Run ADA in real")(
      "continueAuto,c",
      po::bool_switch(&autoContinueDemo),
      "Continue Demo automatically")(
      "ftSensing,f",
      po::bool_switch(&useFTSensing),
      "Use Force/Torque sensing")(
      "demoType,d", po::value<std::string>(&demoType), "Demo type")(
      "foodName",
      po::value<std::string>(&foodName),
      "Name of food (for data collection)")(
      "direction",
      po::value<std::size_t>(&directionIndex),
      "Direction index(for data collection)")(
      "trial",
      po::value<std::size_t>(&trialIndex),
      "Trial index (for data collection)")(
      "output,o",
      po::value<std::string>(&dataCollectorPath),
      "Output directory (for data collection)");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, po_desc), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    std::cout << po_desc << std::endl;
    exit(0);
  }
}

//==============================================================================
void printStateWithTime(
    double t,
    std::size_t dimension,
    Eigen::VectorXd& stateVec,
    Eigen::VectorXd& velocityVec,
    std::ofstream& cout)
{
  cout << t << ",";
  for (std::size_t i = 0; i < dimension; i++)
  {
    cout << stateVec[i] << "," << velocityVec[i];
    if (i < dimension - 1)
    {
      cout << ",";
    }
  }
  cout << std::endl;
  return;
}

//==============================================================================
void dumpSplinePhasePlot(
    const aikido::trajectory::Spline& spline,
    const std::string& filename,
    double timeStep)
{
  std::ofstream phasePlotFile;
  phasePlotFile.open(filename);
  auto stateSpace = spline.getStateSpace();
  std::size_t dim = stateSpace->getDimension();

  aikido::common::StepSequence sequence(
      timeStep, true, true, spline.getStartTime(), spline.getEndTime());
  auto state = stateSpace->createState();
  Eigen::VectorXd stateVec(dim);
  Eigen::VectorXd velocityVec(dim);

  for (std::size_t i = 0; i < sequence.getLength(); i++)
  {
    double t = sequence[i];
    spline.evaluate(t, state);
    spline.evaluateDerivative(t, 1, velocityVec);
    stateSpace->logMap(state, stateVec);
    printStateWithTime(t, dim, stateVec, velocityVec, phasePlotFile);
  }

  phasePlotFile.close();
  return;
}

//==============================================================================
std::string getCurrentTimeDate()
{
  auto now = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);

  std::stringstream ss;
  ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X");
  return ss.str();
}

//==============================================================================
std::string getUserInput(bool food_only, ros::NodeHandle& nodeHandle)
{

  std::string foodName;

  foodName = getUserInputFromAlexa(nodeHandle);
  if (foodName != "")
  {
    ROS_INFO_STREAM("Got " << foodName << " from Alexa.");
    return foodName;
  }


  ROS_INFO_STREAM("Which food item do you want?");
  for (std::size_t i = 0; i < FOOD_NAMES.size(); ++i)
  {
    ROS_INFO_STREAM("(" << i + 1 << ") " << FOOD_NAMES[i] << std::endl);
  }
  if (!food_only)
  {
    for (std::size_t i = 0; i < ACTIONS.size(); ++i)
    {
      ROS_INFO_STREAM(
          "(" << i + FOOD_NAMES.size() + 1 << ") [" << ACTIONS[i] << "]"
              << std::endl);
    }
  }


  int max_id;

  if (!food_only)
    max_id = FOOD_NAMES.size() + ACTIONS.size();
  else
    max_id = FOOD_NAMES.size();

  while (true)
  {
    std::cout << "> ";
    int id;
    std::cin >> id;
    if (id < 1 || id > max_id)
    {
      ROS_WARN_STREAM("Invalid argument. Quitting...");
      return "quit";
    }
    if (id <= FOOD_NAMES.size())
    {
      foodName = FOOD_NAMES[id - 1];
      nodeHandle.setParam("/deep_pose/forceFoodName", foodName);
      nodeHandle.setParam("/deep_pose/spnet_food_name", foodName);
    }
    else
      foodName = ACTIONS[id - FOOD_NAMES.size()];
    return foodName;
  }
}


//==============================================================================
std::string getUserInputFromAlexa(ros::NodeHandle& nodeHandle)
{
  boost::shared_ptr<std_msgs::String const> sharedPtr;
  std_msgs::String rosFoodWord;
  sharedPtr = ros::topic::waitForMessage<std_msgs::String>("/alexa_msgs", ros::Duration(20));
  if (sharedPtr == nullptr)
  {
    ROS_INFO_STREAM("No message from alexa, please input manually");
    return "";
  }
  rosFoodWord = *sharedPtr;
  std::string foodWord = rosFoodWord.data.c_str();
  if (foodWord.compare("~~no_input~~") == 0)
  {
    std_msgs::String rosFoodWord;
    sharedPtr = ros::topic::waitForMessage<std_msgs::String>("/alexa_msgs");
    rosFoodWord = *sharedPtr;
    std::string foodWord = rosFoodWord.data.c_str();
  }
  ROS_INFO_STREAM("Alexa got food " << foodWord);
  //ros::Publisher pub = nodeHandle.advertise<std_msgs::String>("/alexa_msgs", 1, true);
  std_msgs::StringPtr str(new std_msgs::String);
  str->data = "~~no_input~~";
  //pub.publish(str);
  for (std::size_t i = 0; i < FOOD_NAMES.size(); ++i)
  {
    if (FOOD_NAMES[i].compare(foodWord) == 0)
    {
      ROS_INFO_STREAM("Sucessfully returned");
      nodeHandle.setParam("/deep_pose/forceFoodName", foodWord);
      nodeHandle.setParam("/deep_pose/spnet_food_name", foodWord);
      return foodWord;
    }
  }
  return "";
}

//==============================================================================
int getUserInputWithOptions(
    const std::vector<std::string>& optionPrompts, const std::string& prompt)
{
  ROS_INFO_STREAM(prompt);

  for (const auto& option : optionPrompts)
  {
    ROS_INFO_STREAM(option);
  }
  std::cout << "> ";
  int option;
  std::cin.clear();
  std::cin >> option;
  return option;
}

//==============================================================================
std::pair<Eigen::VectorXd, Eigen::VectorXd> setPositionLimits(
    const ::dart::dynamics::MetaSkeletonPtr& metaSkeleton,
    const Eigen::VectorXd& lowerLimits,
    const Eigen::VectorXd& upperLimits,
    const std::vector<std::size_t>& indices)
{
  auto llimits = metaSkeleton->getPositionLowerLimits();
  auto ulimits = metaSkeleton->getPositionUpperLimits();

  Eigen::VectorXd newLowerLimits(llimits);
  Eigen::VectorXd newUpperLimits(ulimits);

  Eigen::VectorXd oldLowerLimits(indices.size());
  Eigen::VectorXd oldUpperLimits(indices.size());

  for (int i = 0; i < indices.size(); ++i)
  {
    newLowerLimits(indices[i]) = lowerLimits[i];
    newUpperLimits(indices[i]) = upperLimits[i];

    oldLowerLimits(i) = llimits[i];
    oldUpperLimits(i) = ulimits[i];
  }
  metaSkeleton->setPositionLowerLimits(newLowerLimits);
  metaSkeleton->setPositionUpperLimits(newUpperLimits);

  return std::make_pair(oldLowerLimits, oldUpperLimits);
}

//==============================================================================
Eigen::Isometry3d getRelativeTransform(
    tf::TransformListener& tfListener,
    const std::string& from,
    const std::string& to)
{
  tf::StampedTransform tfStampedTransform;
  try
  {
    tfListener.lookupTransform(from, to, ros::Time(0), tfStampedTransform);
  }
  catch (tf::TransformException ex)
  {
    throw std::runtime_error(
        "Failed to get TF Transform: " + std::string(ex.what()));
  }

  Eigen::Isometry3d transform;
  tf::transformTFToEigen(tfStampedTransform, transform);
  return transform;
}

//==============================================================================
Eigen::Isometry3d removeRotation(const Eigen::Isometry3d& transform)
{
  Eigen::Isometry3d withoutRotation(Eigen::Isometry3d::Identity());
  withoutRotation.translation() = transform.translation();

  return withoutRotation;
}

//==============================================================================
void printRobotConfiguration(const std::shared_ptr<ada::Ada>& ada)
{
  Eigen::IOFormat CommaInitFmt(
      Eigen::StreamPrecision,
      Eigen::DontAlignCols,
      ", ",
      ", ",
      "",
      "",
      " << ",
      ";");
  auto defaultPose = ada->getArm()->getMetaSkeleton()->getPositions();
  ROS_INFO_STREAM("Current configuration" << defaultPose.format(CommaInitFmt));
}

//==============================================================================
bool isCollisionFree(
    const std::shared_ptr<ada::Ada>& ada,
    const aikido::constraint::dart::CollisionFreePtr& collisionFree)
{
  std::string result;
  auto robotState = ada->getStateSpace()->getScopedStateFromMetaSkeleton(
      ada->getMetaSkeleton().get());
  aikido::constraint::dart::CollisionFreeOutcome collisionCheckOutcome;
  if (!collisionFree->isSatisfied(robotState, &collisionCheckOutcome))
  {
    result = "Robot is in collison: " + collisionCheckOutcome.toString();
    return false;
  }
  result = "Robot is not in collision";
  ROS_INFO_STREAM(result);
  return true;
}

//==============================================================================
double getDistance(
    const Eigen::Isometry3d& item1, const Eigen::Isometry3d& item2)
{
  auto translation = item1.translation() - item2.translation();
  return translation.norm();
}

//==============================================================================
Eigen::Isometry3d getForqueTransform(tf::TransformListener& tfListener)
{
  /*
  return getRelativeTransform(
    tfListener,
    "/map",
    "/j2n6s200_forque_end_effector");
  */
}

//==============================================================================
aikido::distance::ConfigurationRankerPtr getConfigurationRanker(
    const std::shared_ptr<ada::Ada>& ada)
{
  auto space = ada->getArm()->getStateSpace();
  auto metaSkeleton = ada->getArm()->getMetaSkeleton();
  auto nominalState = space->createState();

  nominalState = space->getScopedStateFromMetaSkeleton(metaSkeleton.get());

  return std::make_shared<NominalConfigurationRanker>(
      space, metaSkeleton, weights, std::move(nominalState));
}

//==============================================================================
void talk(const std::string& statement, bool background) {
  std::string cmd;
  if(background) {
    cmd = "aoss swift \"" + statement + "\"" + " &";
  } else {
    cmd = "aoss swift \"" + statement + "\"";
  }
  std::system(cmd.c_str());
}


} // namespace feeding
