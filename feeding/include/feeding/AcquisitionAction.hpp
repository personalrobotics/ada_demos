#ifndef FEEDING_ACQUISITIONACTION_HPP_
#define FEEDING_ACQUISITIONACTION_HPP_

#include <dart/dart.hpp>
#include <Eigen/Core>

namespace feeding {

enum TiltStyle
{
  NONE = 0,
  VERTICAL = 1,
  ANGLED = 2
};

class AcquisitionAction
{
public:
  explicit AcquisitionAction(
    TiltStyle tiltStyle = TiltStyle::NONE,
    double rotationAngle = 0.0,
    double tiltAngle = 0.0,
    Eigen::Vector3d moveIntoDirection = Eigen::Vector3d(-1.0, 0.0, 0.0));

  TiltStyle getTiltStyle() const;

  double getRotationAngle() const;

  double getTiltAngle() const;

  Eigen::Vector3d moveIntoDirection() const;

private:
  TiltStyle mTiltStyle;
  double mRotationAngle;
  double mTiltAngle;
  Eigen::Vector3d mMoveIntoDirection;
};

}

#endif
