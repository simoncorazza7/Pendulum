#ifndef PENDEL_CORE_UNITS_H
#define PENDEL_CORE_UNITS_H

#include <cstdint>

// Hardware-boundary unit conversions (mirror of py/pendel/units.py). SI inside, counts /
// microsteps / STEP toggles only here. Signs are configuration until 0.8 has established +x and
// +theta physically. See docs/model_derivation.md §9.
namespace core {

struct Calib {
    int cpr = 2400;                  // counts/rev
    double c_bottom = 0.0;           // counts, continuous encoder position of the hanging equilibrium (0.7)
    int theta_sign = 1;              // +1 if increasing counts means increasing theta
    double usteps_per_m = 13333.3;   // microsteps/m (PRIOR, 0.8)
    double toggles_per_ustep = 2.0;  // STEP toggles per microstep (issue 3, 0.8)
    int x_sign = 1;                  // +1 if increasing Axis pos means increasing x
    double pos_center = 0.0;         // microsteps, Axis pos at x = 0
};

double wrap(double a);   // into (-pi, pi]

// theta from upright (rad). A count n means the position lies in [n, n+1): its centre is used.
double thetaFromCounts(std::int64_t counts, const Calib& c);
// rad per count, with sign: multiply a count-rate estimate (counts/s) by this.
double omegaScale(const Calib& c);
double xFromPos(std::int64_t posUsteps, const Calib& c);                 // m
double vFromAxisSpeed(std::int64_t curSpeedToggles, const Calib& c);     // m/s, from Axis::getSpeed()
int axisAccelArg(double a, const Calib& c);                             // for Axis::setAcceleration

}

#endif
