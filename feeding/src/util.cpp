#include "feeding/util.hpp"
#include <algorithm>
#include <aikido/common/Spline.hpp>
#include <aikido/common/StepSequence.hpp>
#include <aikido/distance/defaults.hpp>
#include <aikido/planner/parabolic/ParabolicTimer.hpp>
#include <aikido/statespace/CartesianProduct.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/trajectory/Interpolated.hpp>
#include <dart/common/StlHelpers.hpp>
#include <libada/util.hpp>
#include <tf_conversions/tf_eigen.h>

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
      "demoType,d", po::value<std::string>(&demoType), "Demo type");

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

  std::string foodName;
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
      ROS_WARN_STREAM("Invalid argument. Try again.");
      continue;
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

} // namespace feeding
