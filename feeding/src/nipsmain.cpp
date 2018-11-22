
#include <ros/ros.h>
#include <aikido/rviz/WorldInteractiveMarkerViewer.hpp>
#include "feeding/FTThresholdHelper.hpp"
#include "feeding/FeedingDemo.hpp"
#include "feeding/Perception.hpp"
#include "feeding/util.hpp"

namespace feeding {

int nipsmain(FeedingDemo& feedingDemo, FTThresholdHelper& ftThresholdHelper,
             Perception& perception,
             ros::NodeHandle nodeHandle, bool autoContinueDemo, bool adaReal) {

  aikido::rviz::WorldInteractiveMarkerViewerPtr viewer = feedingDemo.getViewer();


  while (true) {
    // ===== ABOVE PLATE =====
    if (!autoContinueDemo) {
      if (!waitForUser("Move forque above plate")) {
        return 0;
      }
    }
    feedingDemo.moveAbovePlate(viewer);

    // ===== IN FRONT OF PERSON =====
    if (!autoContinueDemo) {
      if (!waitForUser("Move forque in front of person")) {
        return 0;
      }
    }
    feedingDemo.moveInFrontOfPerson();
  }

  // ===== DONE =====
  waitForUser("Demo finished.");
}

};  // namespace feeding
