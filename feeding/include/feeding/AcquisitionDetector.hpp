#ifndef ACQUISITION_DETECTOR_HPP_
#define ACQUISITION_DETECTOR_HPP_

#include "feeding/util.hpp"
#include "feeding/FTThresholdHelper.hpp"
#include "ada_demos/DetectAcquisition.h"
#include <libada/util.hpp>

namespace feeding {

// Acquisition Detector is responsible for auto detection of whether food
// has been successfully picked up via fork action.
class AcquisitionDetector
{

public:
  // Constructor for the Acquisition Detector.
  // Takes care of setting up the detector.
  // \param[in] adaReal True if the real robot is used, false it's running in
  // simulation.
  // \param[in] nodeHandle Handle of the ros node.
  // \param[in] ftThresholdHelper manages the forque sensor threshold
  AcquisitionDetector(
      bool adaReal,
      std::shared_ptr<ros::NodeHandle> nodeHandle,
      std::shared_ptr<FTThresholdHelper> ftThresholdHelper = nullptr);

  // Destructor for the Acquisition Detector.
  ~AcquisitionDetector();

  // Collects forque data before fork action.
  //
  // Returns true on success in data collection; returns false otherwise.
  bool collectForqueDataBeforeAction();

  // Collects forque data after fork action.
  //
  // Returns true on success in data collection; returns false otherwise.
  bool collectForqueDataAfterAction();

  // Calls Vision service and gets response for whether Vision system
  // determines there is food on fork or not.
  //
  // Returns 1 on success, 0 on failure, -1 on failure in calling the
  // Vision service. 
  int getResponseFromVision();

  // Detect whether food has been picked up successfully.
  //
  // Returns 1 on success, 0 on failure, -1 when it is undetermined.
  int autoDetectAcquisition();

private:
  bool mAdaReal;
  std::shared_ptr<FTThresholdHelper> mFTThresholdHelper;
  std::shared_ptr<ros::NodeHandle> mNodeHandle;
  bool ftTimeout;
  Eigen::Vector3d beforeForceAvg;
  Eigen::Vector3d beforeTorqueAvg;
  Eigen::Vector3d afterForceAvg;
  Eigen::Vector3d afterTorqueAvg;
  double zForceAvgDiff;
  int visionResult;
  int numDataPts;
};
} // namespace feeding

#endif
