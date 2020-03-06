#include "feeding/onFork.hpp"
#include "feeding/util.hpp"

namespace feeding {
    
double tpVisual = 0.972; // accuracy determined by squeezenet
double tnVisual = 0.9;  // P(V0|off)  = #(visual:off, true off) / #(trials)
double tpHaptic = 0.7;  // P(H1|on)   = #(haptic:on, true on)   / #(trials)
double tnHaptic = 0.8;  // P(H0|off)  = #(haptic:off, true off) / #(trials)
double trueOn   = 0.85; // P(true on) = #(true on) / #(trials)
double lowerThreshold = 0.3;
double upperThreshold = 0.8;

int isFoodOnFork(bool boolFromVisual, bool boolFromHaptic)
{
    double visualOn = boolFromVisual ? tpVisual : (1 - tpVisual);
    double visualOff = boolFromVisual ? (1 - tnVisual) : tnVisual;
    double hapticOn = boolFromHaptic ? tpHaptic : (1 - tpHaptic);
    double hapticOff = boolFromHaptic ? (1 - tnHaptic) : tnHaptic;

    double p = (visualOn * hapticOn * trueOn) / (visualOn * hapticOn * trueOn + visualOff * hapticOff * (1.0 - trueOn));
    
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
