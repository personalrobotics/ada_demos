#include <cmath>

#include "feeding/AcquisitionDetector.hpp"
#include "feeding/util.hpp"

using ada::util::getRosParam;

namespace feeding {

// Calculate phi value of standard normal distribution.
// Integrating from -inf to x.
static double phi(double x);

AcquisitionDetector::AcquisitionDetector(
    bool adaReal,
    std::shared_ptr<ros::NodeHandle> nodeHandle,
    std::shared_ptr<FTThresholdHelper> ftThresholdHelper)
  : mAdaReal(adaReal)
  , mNodeHandle(nodeHandle)
  , mFTThresholdHelper(ftThresholdHelper)
{
  ftTimeout = false;
  zForceAvgDiff = 0.0;
  visionResult = -1;
  numDataPts
      = getRosParam<int>("/acquisitionDetector/numDataPts", *mNodeHandle);
}

AcquisitionDetector::~AcquisitionDetector()
{
}

bool AcquisitionDetector::collectForqueDataBeforeAction()
{
  std::this_thread::sleep_for(std::chrono::milliseconds(getRosParam<int>(
      "/acquisitionDetector/waitMillisecsBeforeCollection", *mNodeHandle)));

  ROS_INFO_STREAM("Collecting forque data before action.");

  if (mFTThresholdHelper)
  {
    bool canCollect = mFTThresholdHelper->startDataCollection(numDataPts);
    if (canCollect)
    {
      ros::Time start_time = ros::Time::now();
      ros::Duration timeout(10.0); // Timeout after 10 seconds
      while (!mFTThresholdHelper->isDataCollectionFinished(
          beforeForceAvg, beforeTorqueAvg))
      {
        ftTimeout = ros::Time::now() - start_time > timeout;
        if (ftTimeout)
          break;
      }
      if (!ftTimeout)
      {
        ROS_INFO_STREAM("Done with FT data collection.");
        ROS_INFO_STREAM("Before average z force: " << beforeForceAvg.z());
      }
    }
    else
    {
      ROS_INFO_STREAM("FT data collection failed. Use Vision only.");
      return false;
    }
  }
  return ftTimeout;
}

bool AcquisitionDetector::collectForqueDataAfterAction()
{
  std::this_thread::sleep_for(std::chrono::milliseconds(getRosParam<int>(
      "/acquisitionDetector/waitMillisecsBeforeCollection", *mNodeHandle)));

  if (mFTThresholdHelper && !ftTimeout)
  {
    bool canCollect = mFTThresholdHelper->startDataCollection(numDataPts);
    if (canCollect)
    {
      ROS_INFO_STREAM("Collecting forque data after action.");
      while (!mFTThresholdHelper->isDataCollectionFinished(
          afterForceAvg, afterTorqueAvg))
      {
      }
      ROS_INFO_STREAM("Done with data collection.");
      ROS_INFO_STREAM("After average z force: " << afterForceAvg.z());
      // Use only z-force
      if (!mAdaReal)
      {
        zForceAvgDiff = afterForceAvg.z(); // use in simulation
      }
      else
      {
        zForceAvgDiff = afterForceAvg.z() - beforeForceAvg.z(); // use in real
      }
      ROS_INFO_STREAM("Difference between average z forces: " << zForceAvgDiff);
    }
    else
    {
      ROS_INFO_STREAM("FT data collection failed.");
      return false;
    }
  }
  else
  {
    ROS_INFO_STREAM("FT data collection failed.");
    return false;
  }
}

int AcquisitionDetector::getResponseFromVision()
{
  ada_demos::DetectAcquisition srv;
  ROS_INFO_STREAM("Calling service...");
  if (ros::service::call("acquisition_detector", srv))
  {
    ROS_INFO_STREAM("Success in calling Vision service.");
    visionResult = srv.response.success;
    ROS_INFO_STREAM("Visual system says: " << visionResult);
  }
  else
  {
    ROS_INFO_STREAM("Failure in calling Vision service.");
  }
  return visionResult;
}

int AcquisitionDetector::autoDetectAcquisition()
{
  double visionOn, visionOff, priorProb;
  double p = 0.5;
  double mu = zForceAvgDiff;
  double weight
      = getRosParam<double>("/acquisitionDetector/weight", *mNodeHandle);
  double forceThreshold = weight * 0.0098; // weight is set on top of file

  if (visionResult != -1) // vision is up
  {
    double tpVision
        = getRosParam<double>("/acquisitionDetector/tpVision", *mNodeHandle);
    double tnVision
        = getRosParam<double>("/acquisitionDetector/tnVision", *mNodeHandle);
    visionOn = (visionResult == 1) ? tpVision : 1 - tpVision;
    visionOff = (visionResult == 1) ? 1 - tnVision : tnVision;
    ROS_INFO_STREAM("Probability given by Vision: " << visionOn);
  }
  else
  {
    ROS_INFO_STREAM("Vision is down...");
  }

  if (mu != 0.0) // haptic is up
  {
    ROS_INFO_STREAM("Demo mean: " << mu);
    double sigma
        = getRosParam<double>("/acquisitionDetector/sigma", *mNodeHandle);
    priorProb = 1 - phi(sqrt(numDataPts) * (forceThreshold - mu) / sigma);
    ROS_INFO_STREAM("Prior probability given by Haptics: " << priorProb);
  }
  else
  {
    ROS_INFO_STREAM("Haptics is down...");
  }

  if (visionResult != -1 && mu != 0.0) // both systems up
  {
    p = (visionOn * priorProb)
        / (visionOn * priorProb + visionOff * (1.0 - priorProb));
  }
  else if (visionResult != -1) // vision up, haptics down
  {
    p = visionOn;
  }
  else if (mu != 0.0) // vision down, haptics up
  {
    p = priorProb;
  } // else: both systems down, default to 0.5 as initialized

  ROS_INFO_STREAM(
      "Probability of ~" << weight << "g of food on fork is: " << p);

  if (p >= getRosParam<double>(
               "/acquisitionDetector/upperThreshold", *mNodeHandle))
  {
    return 1;
  }
  else if (
      p <= getRosParam<double>(
               "/acquisitionDetector/lowerThreshold", *mNodeHandle))
  {
    return 0;
  }
  else
  {
    return -1;
  }
}

static double phi(double x)
{
  // constants
  double a1 = 0.254829592;
  double a2 = -0.284496736;
  double a3 = 1.421413741;
  double a4 = -1.453152027;
  double a5 = 1.061405429;
  double p = 0.3275911;

  // Save the sign of x
  int sign = x < 0 ? -1 : 1;
  x = fabs(x) / sqrt(2.0);

  // A&S formula 7.1.26
  double t = 1.0 / (1.0 + p * x);
  double y
      = 1.0
        - (((((a5 * t + a4) * t) + a3) * t + a2) * t + a1) * t * exp(-x * x);

  return 0.5 * (1.0 + sign * y);
}

} // namespace feeding
