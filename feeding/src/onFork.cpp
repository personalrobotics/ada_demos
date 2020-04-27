#include <cmath>

#include "feeding/onFork.hpp"
#include "feeding/util.hpp"

namespace feeding {

double tpVisual = 0.9871309613928841; // P(V1|on) = #(visual:on, true on) / #(trials)
double tnVisual = 0.9606164383561644; // P(V0|off) = #(visual:off, true off) / #(trials)
//double trueOn   = 0.5307352350341502; // P(true on) = #(true on) / #(trials)
double sigma = 0.1330150445692865;
double intersectX = -0.15731022803131997;
double lowerThreshold = 0.3;
double upperThreshold = 0.8;

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

int isFoodOnFork(bool boolFromVisual, double *zForceAvgPtr)
{
    double visualOn = boolFromVisual ? tpVisual : (1 - tpVisual);
    double visualOff = boolFromVisual ? (1 - tnVisual) : tnVisual;
    double priorProb = 0.8;
    if (zForceAvgPtr != NULL)
    {
      double mu = *zForceAvgPtr;
      priorProb = 1 - phi((intersectX - mu) / sigma);
      ROS_INFO_STREAM("Mean: " << mu << "; Prior probability: " << priorProb);
    }
    double p = (visualOn * priorProb) / (visualOn * priorProb + visualOff * (1.0 - priorProb));

    ROS_INFO_STREAM("Probability of food is on: " << p);

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
