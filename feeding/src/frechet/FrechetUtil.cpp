#include "feeding/frechet/FrechetUtil.hpp"

#include <dart/math/Geometry.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <aikido/distance/defaults.hpp>
#include <aikido/rviz/InteractiveMarkerViewer.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSaver.hpp>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

using dart::collision::CollisionGroup;
using dart::dynamics::InverseKinematics;
using aikido::constraint::SampleGenerator;
using aikido::constraint::dart::CollisionFree;
using aikido::distance::createDistanceMetric;
using aikido::statespace::CartesianProduct;
using aikido::statespace::dart::MetaSkeletonStateSaver;
using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::trajectory::Interpolated;

namespace feeding {

namespace {

// Helper for `readADAPath`, so kept in anon namespace.
std::vector<Eigen::VectorXd> readVectorsFromFile(
  std::string dirPath,
  int vectorLength
) {
  std::vector<Eigen::VectorXd> readVecs;

  std::ifstream recordFile(dirPath);
  std::string line;

  while(std::getline(recordFile, line))
  {
    Eigen::VectorXd readVector(vectorLength);

    std::stringstream ss;
    ss.str(line);
    std::string stringValue;
    for (int i = 0; i < vectorLength; i++)
    {
      // Break up by whitespace.
      std::getline(ss, stringValue, ' ');
      double numericalValue = boost::lexical_cast<double>(stringValue);
      readVector[i] = numericalValue;
    }

    readVecs.push_back(readVector);
  }

  return readVecs;
}

} // namespace

std::vector<Eigen::Isometry3d> readADAPath(
  std::string pathFile
) {
  std::vector<Eigen::Isometry3d> rawPoses;

  std::vector<Eigen::VectorXd> flatPoses = readVectorsFromFile(pathFile, 12);

  for (auto curFlat : flatPoses)
  {
    Eigen::Isometry3d curPose = Eigen::Isometry3d::Identity();

    curPose.translation()
      = Eigen::Vector3d(curFlat[0], curFlat[1], curFlat[2]);

     Eigen::Matrix3d rot;
     rot << curFlat[3], curFlat[4], curFlat[5],
            curFlat[6], curFlat[7], curFlat[8],
            curFlat[9], curFlat[10], curFlat[11];
    curPose.linear() = rot;

    rawPoses.push_back(curPose);
  }

  return rawPoses;
}

} // namespace feeding
