#ifndef FEEDING_ACQUISITIONACTION_HPP_
#define FEEDING_ACQUISITIONACTION_HPP_

#include <dart/dart.hpp>
#include <Eigen/Core>

namespace feeding {

enum TiltStyle
{
  VERTICAL,
  ANGLED,
  NONE
};

struct AcquisitionAction
{
  TiltStyle tiltStyle; // = TiltStyle::VERTICAL;
  double rotationAngle; // = 0.0;
  double tiltAngle; // = 0.0;
  Eigen::Vector3d moveIntoDirection; //(-1.0, 0.0, 0.0);
};

}
#endif
