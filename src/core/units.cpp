#include "core/units.h"

#include <cmath>

namespace core {

namespace {
constexpr double kPi = 3.141592653589793;
}

double wrap(double a) {
    a = std::fmod(a + kPi, 2 * kPi);
    if (a <= 0) a += 2 * kPi;
    return a - kPi;
}

double thetaFromCounts(std::int64_t counts, const Calib& c) {
    const double u = static_cast<double>(counts) + 0.5;
    return wrap(c.theta_sign * (u - c.c_bottom - c.cpr / 2.0) * 2 * kPi / c.cpr);
}

double omegaScale(const Calib& c) { return c.theta_sign * 2 * kPi / c.cpr; }

double xFromPos(std::int64_t posUsteps, const Calib& c) {
    return c.x_sign * (static_cast<double>(posUsteps) - c.pos_center) / c.usteps_per_m;
}

double vFromAxisSpeed(std::int64_t curSpeedToggles, const Calib& c) {
    return c.x_sign * (-static_cast<double>(curSpeedToggles)) / (c.toggles_per_ustep * c.usteps_per_m);
}

int axisAccelArg(double a, const Calib& c) {
    return static_cast<int>(std::lround(-c.x_sign * a * c.usteps_per_m * c.toggles_per_ustep));
}

}
