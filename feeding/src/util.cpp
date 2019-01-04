#include "feeding/util.hpp"
#include <algorithm>
#include <aikido/common/Spline.hpp>
#include <aikido/common/StepSequence.hpp>
#include <aikido/planner/parabolic/ParabolicTimer.hpp>
#include <aikido/trajectory/Interpolated.hpp>
#include <dart/common/StlHelpers.hpp>
#include "aikido/distance/defaults.hpp"
#include "aikido/statespace/CartesianProduct.hpp"
#include "aikido/statespace/dart/MetaSkeletonStateSpace.hpp"

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
      "Use Force/Torque sensing");

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
void infoCallback(
  const sensor_msgs::CameraInfoConstPtr& msg,
  int type, std::string folder,
  std::vector<std::string> foods,
  std::vector<std::string> angleNames)
{

  if (shouldRecordInfo.load() && type == 0 || shouldRecordInfo2.load() && type == 1) {
    ROS_ERROR("recording camera info!");

    std::string s("before");
    if (isAfterPush.load()) {
      s = "after";
    }

    std::string food = foods[curfood.load()];
    std::string direction = angleNames[curdirection.load()];
    int trial = curtrial.load();
    //std::string infoFile = "/home/herb/CameraInfoMsgs/" + return_current_time_and_date() + ".bag";
    std::string infoFile = "/home/herb/Workspace/ryan_ws/CameraInfoMsgs/" + folder +  "/" + food + "-" + direction + "-" + std::to_string(trial) + /*return_current_time_and_date()*/ + "-" + s + ".txt";
    // sstream << imageFile;

    std::ofstream myfile;
    myfile.open(infoFile);
    myfile << "Width: " << msg->width << "\n";
    myfile << "Height: " << msg->height << "\n";
    myfile << "K: " << "\n";
    int i;
    for (i = 0; i < 9; i++) {
      myfile << *(msg->K.data() + i) << "\n";
    }
    rosbag::Bag bag;
    bag.open(infoFile, rosbag::bagmode::Write);
    //bag.write("width", ros::Time::now(), msg->width.data);
    //bag.write("height", ros::Time::now(), msg->height.data);
    //ROS_INFO_STREAM(*(msg->K.data().data()));
    if (type == 0)
      shouldRecordInfo.store(false);
    else
      shouldRecordInfo2.store(false);
  }
}

//==============================================================================
void infoCallback2(const sensor_msgs::CameraInfo::ConstPtr& msg, int type, std::string folder, std::vector<std::string> foods,
                   std::vector<std::string> angleNames)
{

  if (shouldRecordInfo2.load()) {
    ROS_ERROR("recording camera info!");

    std::string s("before");
    if (isAfterPush.load()) {
      s = "after";
    }

    std::string food = foods[curfood.load()];
    std::string direction = angleNames[curdirection.load()];
    int trial = curtrial.load();
    //std::string infoFile = "/home/herb/CameraInfoMsgs/" + return_current_time_and_date() + ".bag";
    std::string infoFile = "/home/herb/Workspace/ryan_ws/CameraInfoMsgs/" + folder +  + "/" + food + "-" + direction + "-" + std::to_string(trial) + /*return_current_time_and_date()*/ + "-" + s + ".bag";
    // sstream << imageFile;

    rosbag::Bag bag;
    bag.open(infoFile, rosbag::bagmode::Write);
    bag.write("camera info", ros::Time::now(), msg);
    shouldRecordInfo2.store(false);
  }
}

//==============================================================================
void imageCallback(const sensor_msgs::ImageConstPtr& msg, int type, std::string folder, std::vector<std::string> foods,
                   std::vector<std::string> angleNames)
{

  if (shouldRecordImage.load() && type == 0 || shouldRecordImage2.load() && type == 1) {
    ROS_ERROR("recording image!");

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      if (type == 0) {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      } else {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);

      }

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // cv::namedWindow("image", WINDOW_AUTOSIZE);
    // cv::imshow("image", cv_ptr->image);
    // cv::waitKey(30);

    static int image_count = 0;
    // std::stringstream sstream;
    std::string s("before");
    if (isAfterPush.load()) {
      s = "after";
    }

    std::string food = foods[curfood.load()];
    std::string direction = angleNames[curdirection.load()];
    int trial = curtrial.load();
    std::string imageFile = "/home/herb/Workspace/ryan_ws/Images/" + folder +  + "/" + food + "-" + direction + "-" + std::to_string(trial) + /*return_current_time_and_date()*/ + "-" + s + ".png";
    // sstream << imageFile;
    bool worked = cv::imwrite( imageFile,  cv_ptr->image );
    image_count++;
    ROS_INFO_STREAM("image saved to " << imageFile << ", worked: " << worked);
    if (type == 0)
      shouldRecordImage.store(false);
    else
      shouldRecordImage2.store(false);
  }

}
} // namespace feeding
