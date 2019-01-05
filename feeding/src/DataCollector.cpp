#include "feeding/DataCollector.hpp"
#include <libada/util.hpp>

using ada::util::getRosParam;
using ada::util::waitForUser;


namespace feeding {

//==============================================================================
void DataCollector::infoCallback(
  const sensor_msgs::CameraInfoConstPtr& msg, ImageType imageType)
{
  std::string folder = imageType == COLOR ? "color" : "depth";

  if (mShouldRecordInfo.load())
  {
    ROS_ERROR("recording camera info!");

    std::string s = isAfterPush.load() ? "after" : "before";
    std::string food = mFoods[mCurrentFood.load()];
    std::string direction = mAngleNames[mCurrentDirection.load()];
    int trial = mCurrentTrial.load();

    std::string infoFile = "/home/herb/Workspace/ryan_ws/CameraInfoMsgs/"
      + folder +  "/" + food + "-" + direction + "-" + std::to_string(trial)
      + "-" + s + ".txt";


    std::ofstream myfile;
    myfile.open(infoFile);
    myfile << "Width: " << msg->width << "\n";
    myfile << "Height: " << msg->height << "\n";
    myfile << "K: " << "\n";
    int i;
    for (i = 0; i < 9; i++) {
      myfile << *(msg->K.data() + i) << "\n";
    }

    // TODO: is this necessary in addition to the file above?
    /*
    rosbag::Bag bag;
    bag.open(infoFile, rosbag::bagmode::Write);
    bag.write("camera info", ros::Time::now(), msg);
    */

    mShouldRecordInfo.store(false);
  }
}

//==============================================================================
void DataCollector::imageCallback(
  const sensor_msgs::ImageConstPtr& msg, ImageType imageType)
{
  std::string folder = imageType == COLOR ? "color" : "depth";

  if (mShouldRecordImage.load())
  {
    ROS_ERROR("recording image!");

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      if (imageType == ImageType::COLOR) {
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

    std::string food = mFoods[mCurrentFood.load()];
    std::string direction = mAngleNames[mCurrentDirection.load()];
    int trial = mCurrentTrial.load();
    std::string imageFile = "/home/herb/Workspace/ryan_ws/Images/" + folder +  + "/" + food + "-" + direction + "-" + std::to_string(trial) + /*return_current_time_and_date()*/ + "-" + s + ".png";
    // sstream << imageFile;
    bool worked = cv::imwrite( imageFile,  cv_ptr->image );
    image_count++;
    ROS_INFO_STREAM("image saved to " << imageFile << ", worked: " << worked);

    mShouldRecordImage.store(false);
  }

}

//==============================================================================
DataCollector::DataCollector(
  std::shared_ptr<FeedingDemo> feedingDemo,
  ros::NodeHandle nodeHandle,
  bool autoContinueDemo,
  bool adaReal)
: mFeedingDemo(std::move(feedingDemo))
, mNodeHandle(nodeHandle)
, mAutoContinueDemo(autoContinueDemo)
, mAdaReal(adaReal)
, mShouldRecordImage{false}
, mShouldRecordInfo{false}
, mCurrentFood{0}
, mCurrentDirection{0}
, mCurrentTrial{0}
{

  // Set Standard Threshold
  mFeedingDemo->setFTThreshold(STANDARD_FT_THRESHOLD);

  mNumTrials = getRosParam<int>("/data/numTrials", mNodeHandle);
  mFoods = getRosParam<std::vector<std::string>>("/data/foods", mNodeHandle);
  mTiltAngles = getRosParam<std::vector<double>>("/data/tiltAngles", mNodeHandle);
  mTiltModes = getRosParam<std::vector<int>>("/data/tiltModes", mNodeHandle);
  mDirections = getRosParam<std::vector<double>>("/data/directions", mNodeHandle);
  mAngleNames = getRosParam<std::vector<std::string>>("/data/angleNames", mNodeHandle);

  image_transport::ImageTransport it(mNodeHandle);

  if (mAdaReal) {
    sub = it.subscribe("/camera/color/image_raw", 1,
      boost::bind(&DataCollector::imageCallback, this, _1, ImageType::COLOR));
    sub2 = it.subscribe("/camera/aligned_depth_to_color/image_raw", 1,
      boost::bind(&DataCollector::imageCallback, this, _1, ImageType::DEPTH));
    sub3 = mNodeHandle.subscribe<sensor_msgs::CameraInfo>(
      "/camera/color/camera_info", 1,
      boost::bind(&DataCollector::infoCallback, this, _1, ImageType::COLOR));
    sub4 = mNodeHandle.subscribe<sensor_msgs::CameraInfo>(
      "/camera/aligned_depth_to_color/camera_info", 1,
      boost::bind(&DataCollector::infoCallback, this, _1, ImageType::DEPTH));
  }
}

//==============================================================================
void DataCollector::setDataCollectionParams(bool pushCompleted,
  int foodId, int pushDirectionId, int trialId)
{

  if (mAdaReal) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    isAfterPush.store(pushCompleted);

    // Update only when positive.
    if (foodId != -1)
    {
      mCurrentFood.store(foodId);
      mCurrentDirection.store(pushDirectionId);
      mCurrentTrial.store(trialId);
    }

    mShouldRecordImage.store(true);
    mShouldRecordInfo.store(true);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // wait for first stream to be saved
    while (mShouldRecordImage.load() || mShouldRecordInfo.load())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
}

//==============================================================================
void DataCollector::pushAndSkewer(
  const std::string& foodName,
  int mode, float rotAngle, float tiltAngle)
{
  mFeedingDemo->pushAndSkewer(foodName, mode, rotAngle, tiltAngle);

  // Pushing complete
  setDataCollectionParams(true, -1, -1, -1);

  // ===== RESET TO ABOVE FOOD =====
  mFeedingDemo->detectAndMoveAboveFood(
    foodName,
    mode,
    rotAngle,
    tiltAngle,
    false);
}

//==============================================================================
void DataCollector::collect(Action action)
{
  for (std::size_t i = 0; i < mFoods.size(); ++i)
  {
    for (std::size_t j = 0; j < mDirections.size(); ++j)
    {
      for (std::size_t k = 0; k < mNumTrials; ++k)
      {
        mFeedingDemo->moveAbovePlate();

        ROS_INFO_STREAM("\nTrial " << k << ": Food / Direction: "
         << mFoods[i] << " / " << mAngleNames[j] << "> \n\n");

        float angle = mDirections[j] * M_PI / 180.0;

        setDataCollectionParams(false, i, j, k);

        if (action == PUSH_AND_SKEWER)
          pushAndSkewer(mFoods[i], mTiltModes[i], angle, mTiltAngles[i]);
        else if (action == SKEWER)
          mFeedingDemo->rotateAndSkewer(mFoods[i], angle);
      }
    }
  }
  // ===== DONE =====
  mFeedingDemo->waitForUser("Data collection finished");
}

} // namespace feeding
