
#include "feeding/FTThresholdHelper.hpp"
#include "feeding/FeedingDemo.hpp"
#include "feeding/Perception.hpp"
#include "feeding/util.hpp"
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

namespace feeding {

std::atomic<bool> shouldRecordImage3{false};
std::atomic<bool> shouldRecordImage32{false};
std::atomic<bool> shouldRecordInfo3{false};
std::atomic<bool> shouldRecordInfo32{false};
std::atomic<bool> isAfterPush3{false};
std::atomic<int> curfood3{0};
std::atomic<int> curdirection3{0};
std::atomic<int> curtrial3{0};

std::string return_current_time_and_date2()
{
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X");
    return ss.str();
}

void infoCallback3(const sensor_msgs::CameraInfoConstPtr& msg, int type, std::string folder, std::vector<std::string> foods,
                   std::vector<std::string> angleNames)
{

  if (shouldRecordInfo3.load() && type == 0 || shouldRecordInfo32.load() && type == 1) {
    ROS_ERROR("recording camera info!");

    std::string s("before");
    if (isAfterPush3.load()) {
      s = "after";
    }
    s = "skewer";

    std::string food = foods[curfood3.load()];
    std::string direction = angleNames[curdirection3.load()];
    int trial = curtrial3.load();
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
      shouldRecordInfo3.store(false);
    else
      shouldRecordInfo32.store(false);
  }
}

void imageCallback2(const sensor_msgs::ImageConstPtr& msg, int type, std::string folder, std::vector<std::string> foods,
                   std::vector<std::string> angleNames)
{

  if (shouldRecordImage3.load() && type == 0 || shouldRecordImage32.load() && type == 1) {
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
    if (isAfterPush3.load()) {
      s = "after";
    }
    s = "skewer";

    std::string food = foods[curfood3.load()];
    std::string direction = angleNames[curdirection3.load()];
    int trial = curtrial3.load();
    std::string imageFile = "/home/herb/Workspace/ryan_ws/Images/" + folder +  + "/" + food + "-" + direction + "-" + std::to_string(trial) + /*return_current_time_and_date()*/ + "-" + s + ".png";
    // sstream << imageFile;
    bool worked = cv::imwrite( imageFile,  cv_ptr->image );
    image_count++;
    ROS_INFO_STREAM("image saved to " << imageFile << ", worked: " << worked);
    if (type == 0)
      shouldRecordImage3.store(false);
    else
      shouldRecordImage32.store(false);
  }
}

int datacollectionmainNoPush(FeedingDemo& feedingDemo,
                FTThresholdHelper& ftThresholdHelper,
                Perception& perception,
                aikido::rviz::WorldInteractiveMarkerViewerPtr viewer,
                ros::NodeHandle nodeHandle,
                bool autoContinueDemo,
                bool adaReal) {

  // Set Standard Threshold
  if (!ftThresholdHelper.setThresholds(STANDARD_FT_THRESHOLD)) {
    return 1;
  }

  int numTrials = getRosParam<int>("/data/numTrials", nodeHandle);
  std::vector<std::string> foods = getRosParam<std::vector<std::string>>("/data/foods", nodeHandle);
  std::vector<double> tiltAngles = getRosParam<std::vector<double>>("/data/tiltAngles", nodeHandle);
  std::vector<int> tiltModes = getRosParam<std::vector<int>>("/data/tiltModes", nodeHandle);
  std::vector<double> directions = getRosParam<std::vector<double>>("/data/directions", nodeHandle);
  std::vector<std::string> angleNames = getRosParam<std::vector<std::string>>("/data/angleNames", nodeHandle);


  image_transport::ImageTransport it(nodeHandle);
  image_transport::Subscriber sub;
  image_transport::Subscriber sub2;
  ros::Subscriber sub3;
  ros::Subscriber sub4;
  if (adaReal) {
    sub = it.subscribe("/camera/color/image_raw", 1, boost::bind(imageCallback2, _1, 0, "color", foods, angleNames));
    sub2 = it.subscribe("/camera/aligned_depth_to_color/image_raw", 1, boost::bind(imageCallback2, _1, 1, "depth", foods, angleNames));
    //sub3 = nodeHandle.subscribe("", 1, infoCallback);
    //sub4 = nodeHandle.subscribe("", 1, infoCallback2);
    sub3 = nodeHandle.subscribe<sensor_msgs::CameraInfo>("/camera/color/camera_info", 1, boost::bind(infoCallback3, _1, 0, "color", foods, angleNames));
    sub4 = nodeHandle.subscribe<sensor_msgs::CameraInfo>("/camera/aligned_depth_to_color/camera_info", 1, boost::bind(infoCallback3, _1, 1, "depth", foods, angleNames));
  }

  int i, j, k;
  for (i = 0; i < foods.size(); ++i)
  {
    for (j = 0; j < directions.size(); ++j)
    {
      for (k = 0; k < numTrials; ++k)
      {
        float angle = directions[j] * M_PI / 180.0;
        std::cout << std::endl << "\033[1;32mTrial " << k << ": Food / Direction: " << foods[i] << " / " << angleNames[j] << "\033[0m     > " << std::endl << std::endl;
        // ===== ABOVE PLATE =====
        if (!waitForUser("Move forque above plate"))
        {
          return 0;
        }
        feedingDemo.moveAbovePlate();

        if (adaReal) {
          std::this_thread::sleep_for(std::chrono::milliseconds(2000));
          isAfterPush3.store(false);
          curfood3.store(i);
          curdirection3.store(j);
          curtrial3.store(k);
          shouldRecordImage3.store(true);
          shouldRecordImage32.store(true);
          shouldRecordInfo3.store(true);
          shouldRecordInfo32.store(true);
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
          while (shouldRecordImage32.load() || shouldRecordImage3.load() || shouldRecordInfo3.load() || shouldRecordInfo32.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
          }
        }

        // ===== ABOVE FOOD =====
        std::vector<std::string> foodNames = getRosParam<std::vector<std::string>>("/foodItems/names", nodeHandle);
        std::vector<double> skeweringForces = getRosParam<std::vector<double>>("/foodItems/forces", nodeHandle);
        std::unordered_map<std::string, double> foodSkeweringForces;
        for (int i=0; i<foodNames.size(); i++) {
          foodSkeweringForces[foodNames[i]] = skeweringForces[i];
        }

        Eigen::Isometry3d foodTransform;
        bool foodFound = false;
        std::string foodName = foods[i];

        while (!foodFound) {
          if (!perception.setFoodName(foodName)) {
            std::cout << "\033[1;33mI don't know about any food that's called '" << foodName << ". Wanna get something else?\033[0m" << std::endl;
            continue;
          }

          if (adaReal)
          {
            bool perceptionSuccessful = perception.perceiveFood(foodTransform, true, viewer);
            if (!perceptionSuccessful) {
              std::cout << "\033[1;33mI can't see the " << foodName << "... Wanna try again?\033[0m" << std::endl;
              continue;
            } else {
              foodFound = true;
            }
          }
          else
          {
            foodTransform = feedingDemo.getDefaultFoodTransform();
            foodFound = true;
          }

           std::cout << "\033[1;32mAlright! Let's get the " << foodName << "!\033[0;32m  (Gonna skewer with " << foodSkeweringForces[foodName] << "N)\033[0m" << std::endl << std::endl;

          if (!autoContinueDemo)
          {
            if (!waitForUser("Move forque above food"))
            {
              return 0;
            }
          }
          if(!feedingDemo.moveAboveFood(foodTransform, angle, 0, 0, viewer)) {
            foodFound = false;
            if (!waitForUser("Trajectory failed! Reposition food and try again!"))
            {
              return 0;
            }
          }
        }



        bool foodPickedUp = false;
        while (!foodPickedUp) {

          // ===== ROTATE FORQUE ====
          if (!autoContinueDemo)
          {
            if (!waitForUser("Rotate forque to angle " + angleNames[j] + " to push food"))
            {
              return 0;
            }
          }
          feedingDemo.rotateForque(foodTransform, angle, 0, viewer);

          // ===== INTO TO FOOD ====
          if (!autoContinueDemo)
          {
            if (!waitForUser("Move forque into to food"))
            {
              return 0;
            }
          }

          double torqueThreshold = 2;
          if (!ftThresholdHelper.setThresholds(STANDARD_FT_THRESHOLD))
          {
            return 1;
          }
          Eigen::Isometry3d forqueTransform;
          if (adaReal) {
              forqueTransform = perception.getForqueTransform();
          }
          feedingDemo.moveIntoFood();

          break;
        }

        // ===== OUT OF FOOD =====
        if (!autoContinueDemo)
        {
          if (!waitForUser("Move forque out of food"))
          {
            return 0;
          }
        }
        if (!ftThresholdHelper.setThresholds(AFTER_GRAB_FOOD_FT_THRESHOLD))
        {
          return 1;
        }
        feedingDemo.moveOutOfFood();
      }
    }
  }

  // ===== DONE =====
  waitForUser("Demo finished.");
}

};
