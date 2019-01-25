#include "feeding/DataCollector.hpp"
#include <boost/filesystem/path.hpp>
#include <libada/util.hpp>
#include <yaml-cpp/yaml.h>
#include <boost/date_time.hpp>


using ada::util::getRosParam;
using ada::util::waitForUser;

namespace feeding {

//==============================================================================
void createDirectory(const std::string& directory)
{

  if (!boost::filesystem::is_directory(directory))
  {
    ROS_INFO_STREAM("Create " << directory << std::endl);
    if (!boost::filesystem::create_directories(directory))
    {
      ROS_ERROR_STREAM("Could not create " << directory << std::endl);
    }
  }

}
//==============================================================================
void setupDirectoryPerData(const std::string& root)
{
  std::vector<std::string> folders{"color", "depth"};
  for (auto & folder : folders)
  {
    auto directory = root + "CameraInfoMsgs/" + folder + "/";
    createDirectory(directory);

    directory = root + folder;
    createDirectory(directory);
  }
}

//==============================================================================
void removeDirectory(const std::string directory)
{
  if (!boost::filesystem::remove_all(directory))
  {
    ROS_WARN_STREAM("Failed to remove " << directory);
  }
  else
  {
    ROS_WARN_STREAM("Removed " << directory);
  }
}

//==============================================================================
void DataCollector::infoCallback(
    const sensor_msgs::CameraInfoConstPtr& msg, ImageType imageType)
{
  std::string folder = imageType == COLOR ? "color" : "depth";

  if (mShouldRecordInfo.load())
  {
    ROS_INFO("recording camera info!");

    std::string food = mFoods[mCurrentFood.load()];
    std::string direction = mAngleNames[mCurrentDirection.load()];
    int trial = mCurrentTrial.load();

    auto directory = mDataCollectionPath +
                           "CameraInfoMsgs/"
                           + folder + "/";

    std::string infoFile = directory + "cameraInfo.yaml";

    YAML::Node node;
    node["width"] = msg->width;
    node["height"] = msg->height;

    for (std::size_t i = 0; i < 9; i++)
      node["K"].push_back(*(msg->K.data() + i));

    std::ofstream outFile(infoFile);
    outFile << node;

    ROS_INFO_STREAM("Wrote to " << infoFile);

    mShouldRecordInfo.store(false);
  }
}

//==============================================================================
void DataCollector::imageCallback(
    const sensor_msgs::ImageConstPtr& msg, ImageType imageType)
{
  std::string folder = imageType == COLOR ? "color" : "depth";

  if (mShouldRecordColorImage.load() || mShouldRecordDepthImage.load())
  {
    ROS_INFO("recording image!");

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      if (imageType == ImageType::COLOR)
      {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
      else
      {
        cv_ptr = cv_bridge::toCvCopy(
            msg, sensor_msgs::image_encodings::TYPE_16UC1);
      }
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    auto count = imageType == COLOR ? mColorImageCount.load() : mDepthImageCount.load();

    std::string imageFile
        = mDataCollectionPath + folder + +"/image_" + std::to_string(count) + "-"
          + getCurrentDateTime() + ".png";
    bool worked = cv::imwrite(imageFile, cv_ptr->image);

    if (imageType == COLOR)
    {
      mColorImageCount++;
      mShouldRecordColorImage.store(false);
    }
    else
    {
      mDepthImageCount++;
      mShouldRecordDepthImage.store(false);
    }

    if (worked)
      ROS_INFO_STREAM("image saved to " << imageFile);
    else
      ROS_WARN_STREAM("image saving failed");

  }
}

//==============================================================================
DataCollector::DataCollector(
    std::shared_ptr<FeedingDemo> feedingDemo,
    std::shared_ptr<ada::Ada> ada,
    ros::NodeHandle nodeHandle,
    bool autoContinueDemo,
    bool adaReal,
    bool perceptionReal,
    const std::string& dataCollectionPath)
  : mFeedingDemo(std::move(feedingDemo))
  , mAda(std::move(ada))
  , mNodeHandle(nodeHandle)
  , mAutoContinueDemo(autoContinueDemo)
  , mAdaReal(adaReal)
  , mDataCollectionPath{dataCollectionPath}
  , mPerceptionReal{perceptionReal}
  , mShouldRecordColorImage{false}
  , mShouldRecordDepthImage{false}
  , mShouldRecordInfo{false}
  , mCurrentFood{0}
  , mCurrentDirection{0}
  , mCurrentTrial{0}
  , mColorImageCount{0}
  , mDepthImageCount{0}
{
  // See if we can save force/torque sensor data as well.

  // Set Standard Threshold
  mFeedingDemo->setFTThreshold(STANDARD_FT_THRESHOLD);

  mNumTrials = getRosParam<int>("/data/numTrials", mNodeHandle);
  mFoods = getRosParam<std::vector<std::string>>("/data/foods", mNodeHandle);
  mTiltAngles
      = getRosParam<std::vector<double>>("/data/tiltAngles", mNodeHandle);
  mTiltModes = getRosParam<std::vector<int>>("/data/tiltModes", mNodeHandle);
  mDirections
      = getRosParam<std::vector<double>>("/data/directions", mNodeHandle);
  mAngleNames
      = getRosParam<std::vector<std::string>>("/data/angleNames", mNodeHandle);

  if (mAdaReal || mPerceptionReal)
  {
    image_transport::ImageTransport it(mNodeHandle);
    sub = it.subscribe(
        "/camera/color/image_raw",
        1,
        boost::bind(&DataCollector::imageCallback, this, _1, ImageType::COLOR));
    sub2 = it.subscribe(
        "/camera/aligned_depth_to_color/image_raw",
        1,
        boost::bind(&DataCollector::imageCallback, this, _1, ImageType::DEPTH));
    sub3 = mNodeHandle.subscribe<sensor_msgs::CameraInfo>(
        "/camera/color/camera_info",
        1,
        boost::bind(&DataCollector::infoCallback, this, _1, ImageType::COLOR));
    sub4 = mNodeHandle.subscribe<sensor_msgs::CameraInfo>(
        "/camera/aligned_depth_to_color/camera_info",
        1,
        boost::bind(&DataCollector::infoCallback, this, _1, ImageType::DEPTH));
  }
}

//==============================================================================
void DataCollector::setDataCollectionParams(
    bool pushCompleted, int foodId, int pushDirectionId, int trialId)
{
  if (mAdaReal)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    isAfterPush.store(pushCompleted);

    // Update only when positive.
    if (foodId != -1)
    {
      mCurrentFood.store(foodId);
      mCurrentDirection.store(pushDirectionId);
      mCurrentTrial.store(trialId);
    }

    mShouldRecordInfo.store(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // wait for first stream to be saved
    while (mShouldRecordInfo.load())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
}

//==============================================================================
void DataCollector::collect(Action action,
    const std::string& foodName,
    std::size_t directionIndex,
    std::size_t trialIndex)
{
  if (directionIndex >= mDirections.size())
  {
    std::stringstream ss;
    ss << "Direction index [" << directionIndex
    << "] is greater than the max index [" << mDirections.size() - 1 << "].\n";
    throw std::invalid_argument(ss.str());
  }
  if (trialIndex >= mNumTrials)
  {
    std::stringstream ss;
    ss << "Trial index [" << trialIndex
    << "] is greater than the max index [" << mNumTrials - 1 << "].\n";
    throw std::invalid_argument(ss.str());
  }
  if (std::find(mFoods.begin(), mFoods.end(), foodName) == mFoods.end())
  {
    std::stringstream ss;
    ss << "Food " << foodName << " not in the list of foods.\n" ;
    throw std::invalid_argument(ss.str());
  }

  mDataCollectionPath += foodName + "-" + mAngleNames[directionIndex] + "-" + std::to_string(trialIndex) + "/";
  setupDirectoryPerData(mDataCollectionPath);

  auto foodIndex = std::distance(mFoods.begin(),
    std::find(mFoods.begin(), mFoods.end(), foodName));

  ROS_INFO_STREAM(
      "\nTrial " << trialIndex << ": Food [" << foodName << "] Direction ["
                 << mAngleNames[directionIndex]
                 << "] \n\n");
  mFeedingDemo->waitForUser("Start");

  float rotateForqueAngle = mDirections[directionIndex] * M_PI / 180.0;

  setDataCollectionParams(false, foodIndex, directionIndex, trialIndex);

  if (action == VERTICAL_SKEWER)
  {
    if (!skewer(rotateForqueAngle, TiltStyle::NONE))
    {
      ROS_INFO_STREAM("Terminating.");
      return;
    }
  }
  else if (action == TILTED_VERTICAL_SKEWER)
  {
    if (!skewer(rotateForqueAngle, TiltStyle::VERTICAL))
    {
      ROS_INFO_STREAM("Terminating.");
      return;
    }
  }
  else if (action == TILTED_ANGLED_SKEWER)
  {
    if (!skewer(rotateForqueAngle, TiltStyle::ANGLED))
    {
      ROS_INFO_STREAM("Terminating.");
      return;
    }
  }

  // tiltedSkewer -- TSR

  // Twirling
  else if (action == SCOOP)
  {
    mFeedingDemo->scoop();
  }

  // Move up a bit to test success
  mFeedingDemo->moveInFrontOfPerson();

  recordSuccess();

  ROS_INFO_STREAM("Terminating.");
}

//==============================================================================
bool DataCollector::skewer(float rotateForqueAngle, TiltStyle tiltStyle)
{
  // ===== ROTATE FORQUE ====
  if (!mFeedingDemo->rotateForque(rotateForqueAngle, tiltStyle))
  {
    ROS_ERROR("Rotate Forque failed. Restart.");
    removeDirectory(mDataCollectionPath);
    return false;
  }
  captureFrame();

  // ===== INTO TO FOOD ====
  mFeedingDemo->moveInto(TargetItem::FOOD);
  captureFrame();

  // ===== OUT OF FOOD =====
  mFeedingDemo->setFTThreshold(AFTER_GRAB_FOOD_FT_THRESHOLD);
  mFeedingDemo->moveOutOf(TargetItem::FOOD, true);
  captureFrame();

  return true;
}

//==============================================================================
void DataCollector::recordSuccess()
{
  std::string food = mFoods[mCurrentFood.load()];
  std::string direction = mAngleNames[mCurrentDirection.load()];
  int trial = mCurrentTrial.load();

  auto fileName = mDataCollectionPath + "success/" + food + "-"
          + direction + "-" + std::to_string(trial)
          + "-" + getCurrentDateTime() + ".txt";


  ROS_INFO_STREAM("Record success for " << food << " direction " << direction <<
    " trial " << trial << " [y/n]");

  char input = ' ';
  std::cin.get(input);

  if (input != 'y' && input != 'n')
  {
    ROS_ERROR("Input is not y/n");
    return recordSuccess();
  }

  std::ofstream ss;
  ss.open(fileName);
  ss << input << std::endl;
  ss.close();
}

//==============================================================================
void DataCollector::captureFrame()
{
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  mShouldRecordColorImage.store(true);
  mShouldRecordDepthImage.store(true);
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
}

//==============================================================================
std::string DataCollector::getCurrentDateTime()
{
  std::stringstream ss;
  ss <<  boost::posix_time::second_clock::universal_time();
  return ss.str();
}


} // namespace feeding
