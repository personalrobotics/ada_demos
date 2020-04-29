#include <cmath>

#include "feeding/onFork.hpp"
#include "feeding/util.hpp"

namespace feeding {

double tpVisual = 0.9871309613928841; // P(V1|on) = #(visual:on, true on) / #(trials)
double tnVisual = 0.9606164383561644; // P(V0|off) = #(visual:off, true off) / #(trials)
double sigma = 0.1330150445692865;
double weight = 5;  // in grams
double lowerThreshold = 0.3;
double upperThreshold = 0.85;

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

int isFoodOnFork(int fromVisual, double zForceAvg)
{
    double visualOn = 0.5;   // default in case Visual system down
    double visualOff = 0.5;  // default in case Visual system down
    double priorProb = 0.5;
    double p = 0.5;
    double mu = zForceAvg;
    double forceThreshold = weight * 0.0098;

    if (fromVisual == 1)
    {
      visualOn = tpVisual;
      visualOff = 1 - tnVisual;
    }
    else if (fromVisual == 0)
    {
      visualOn = 1 - tpVisual;
      visualOff = tnVisual;
    }

    ROS_INFO_STREAM("Demo mean: " << mu);
    if (mu != 0.0)  // Forque sensor up
      priorProb = 1 - phi((forceThreshold - mu) / sigma);
    ROS_INFO_STREAM("Prior probability given by forque (0.5 if default): " << priorProb);
    ROS_INFO_STREAM("Probability given by vision: " << visualOn);

    p = (visualOn * priorProb) / (visualOn * priorProb + visualOff * (1.0 - priorProb));
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
