#include <cmath>

#include "feeding/onFork.hpp"
#include "feeding/util.hpp"

namespace feeding {

double tpVision = 0.9871309613928841; // P(V1|on) = #(vision:on, true on) / #(trials)
double tnVision = 0.9606164383561644; // P(V0|off) = #(vision:off, true off) / #(trials)
double sigma = 0.1330150445692865 * 2;
double weight = 10;  // in grams, minimum weight threshold
double lowerThreshold = 0.4;
double upperThreshold = 0.7;

// Calculate phi value of standard normal distribution.
// Integrating from -inf to x.
double phi(double x)
{
  // constants
  double a1 =  0.254829592;
  double a2 = -0.284496736;
  double a3 =  1.421413741;
  double a4 = -1.453152027;
  double a5 =  1.061405429;
  double p  =  0.3275911;

  // Save the sign of x
  int sign = x < 0 ? -1 : 1;
  x = fabs(x)/sqrt(2.0);

  // A&S formula 7.1.26
  double t = 1.0/(1.0 + p*x);
  double y = 1.0 - (((((a5*t + a4)*t) + a3)*t + a2)*t + a1)*t*exp(-x*x);

  return 0.5*(1.0 + sign*y);
}

int isFoodOnFork(int fromVision, double zForceAvg, int numPts)
{
    double visionOn, visionOff, priorProb;
    double p = 0.5;
    double mu = zForceAvg;
    double forceThreshold = weight * 0.0098;  // weight is set on top of file

    if (fromVision != -1)  // vision is up
    {
      visionOn = (fromVision == 1) ? tpVision : 1 - tpVision;
      visionOff = (fromVision == 1) ? 1 - tnVision : tnVision;
      ROS_INFO_STREAM("Probability given by Vision: " << visionOn);
    }
    else
    {
      ROS_INFO_STREAM("Vision is down...");
    }

    if (mu != 0.0)  // haptic is up
    {
      ROS_INFO_STREAM("Demo mean: " << mu);
      ROS_INFO_STREAM("Standardized z: " << sqrt(numPts) * (forceThreshold - mu) / sigma);
      priorProb = phi(sqrt(numPts) * (-forceThreshold - mu) / sigma);
      ROS_INFO_STREAM("Prior probability given by Haptics: " << priorProb);
    }
    else
    {
      ROS_INFO_STREAM("Haptics is down...");
    }

    if (fromVision != -1 && mu != 0.0)  // both systems up
    {
      p = (visionOn * priorProb) / (visionOn * priorProb + visionOff * (1.0 - priorProb));
    }
    else if (fromVision != -1)  // vision up, haptics down
    {
      p = visionOn;
    }
    else if (mu != 0.0)  // vision down, haptics up
    {
      p = priorProb;
    } // else: both systems down, default to 0.5 as initialized

    ROS_INFO_STREAM("Probability of ~" << weight << "g of food on fork is: " << p);

    if (p >= upperThreshold)
    {
      return 1;
    }
    else if (p <= lowerThreshold)
    {
      return -1;
    }
    else
    {
      return 0;
    }
}

} // namespace feeding
