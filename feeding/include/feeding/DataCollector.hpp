#ifndef FEEDING_DATACOLLECTOR_HPP_
#define FEEDING_DATACOLLECTOR_HPP_

#include <pr_tsr/plate.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include <image_transport/image_transport.h>
#include <rosbag/bag.h>
#include <sensor_msgs/CameraInfo.h>
#include <iostream>
#include <fstream>

#include "feeding/FTThresholdHelper.hpp"
#include "feeding/FeedingDemo.hpp"
#include "feeding/perception/Perception.hpp"
#include "feeding/util.hpp"

namespace feeding {


// void infoCallback(
//     const sensor_msgs::CameraInfoConstPtr& msg,
//     int type,
//     std::string folder,
//     std::vector<std::string> foods,
//     std::vector<std::string> angleNames);

// void infoCallback2(
//     const sensor_msgs::CameraInfo::ConstPtr& msg,
//     int type,
//     std::string folder,
//     std::vector<std::string> foods,
//     std::vector<std::string> angleNames);

// void imageCallback(
//   const sensor_msgs::ImageConstPtr& msg,
//   int type,
//   std::string folder,
//   std::vector<std::string> foods,
//   std::vector<std::string> angleNames);


// Action types for data collection
enum Action
{
  PUSH_AND_SKEWER,
  SKEWER
};

enum ImageType
{
  COLOR,
  DEPTH
};

class DataCollector
{
public:
  // Constructor
  explicit DataCollector(
    std::shared_ptr<FeedingDemo> feedingDemo,
    ros::NodeHandle nodeHandle,
    bool autoContinueDemo,
    bool adaReal);

  void collect(Action action);

private:

  void setDataCollectionParams(
    bool pushCompleted,
    int foodId,
    int pushDirectionId,
    int trialId);

  void pushAndSkewer(
    const std::string& foodName,
    int mode,
    float rotAngle,
    float tiltAngle);

  void infoCallback(
    const sensor_msgs::CameraInfoConstPtr& msg,
    ImageType imageType);

  void infoCallback2(
      const sensor_msgs::CameraInfo::ConstPtr& msg,
      ImageType imageType);

  void imageCallback(
    const sensor_msgs::ImageConstPtr& msg,
    ImageType imageType);


  std::shared_ptr<FeedingDemo> mFeedingDemo;

  ros::NodeHandle mNodeHandle;
  const bool mAutoContinueDemo;
  const bool mAdaReal;

  int mNumTrials;
  std::vector<std::string> mFoods;
  std::vector<double> mTiltAngles;
  std::vector<int> mTiltModes;
  std::vector<double> mDirections;
  std::vector<std::string> mAngleNames;

  image_transport::Subscriber sub;
  image_transport::Subscriber sub2;
  ros::Subscriber sub3;
  ros::Subscriber sub4;

  std::atomic<bool> mShouldRecordImage;
  std::atomic<bool> mShouldRecordInfo;
  std::atomic<bool> isAfterPush;
  std::atomic<int> mCurrentFood;
  std::atomic<int> mCurrentDirection;
  std::atomic<int> mCurrentTrial;

};

} // namespace feeding

#endif