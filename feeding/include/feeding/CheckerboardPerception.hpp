#ifndef FEEDING_CHECKERBOARDPERCEPTION_HPP_
#define FEEDING_CHECKERBOARDPERCEPTION_HPP_

#include <Eigen/Dense>
#include <ros/ros.h>


namespace feeding {

class CheckerboardPerception {

public:

  CheckerboardPerception();

  bool perceiveCheckerboard(Eigen::Isometry3d& checkerboardCornerTransform);

};

}

#endif
