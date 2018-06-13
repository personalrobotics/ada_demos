#ifndef PERCEPTION_H
#define PERCEPTION_H

#include <Eigen/Dense>
#include <ros/ros.h>
#include <libada/Ada.hpp>
#include <aikido/perception/PoseEstimatorModule.hpp>

namespace feeding {

class Perception {

  aikido::planner::WorldPtr world;
  ros::NodeHandle& nodeHandle;
  std::unique_ptr<aikido::perception::PoseEstimatorModule> objDetector;

public:

  Perception(aikido::planner::WorldPtr world, ada::Ada& ada, ros::NodeHandle& nodeHandle);

  bool perceiveFood(Eigen::Isometry3d& foodTransform);

};

}

#endif