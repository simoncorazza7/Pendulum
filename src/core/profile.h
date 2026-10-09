#ifndef PENDEL_CORE_PROFILE_H
#define PENDEL_CORE_PROFILE_H

// Point-to-point motion profile for bring-up moves (Stage 0.8/0.9): trapezoidal velocity (or
// triangular if the distance is too short to reach v_max). Pure; SI units.
namespace core {

struct ProfilePoint {
    double x;   // m, reference position relative to the start
    double v;   // m/s
    double a;   // m/s^2
};

class TrapezoidProfile {
public:
    // distance: signed (m); vmax > 0 (m/s); amax > 0 (m/s^2).
    TrapezoidProfile(double distance, double vmax, double amax);
    ProfilePoint at(double t) const;   // t in s from the start; holds the end point after duration()
    double duration() const { return t_total_; }
    double peakSpeed() const { return v_peak_; }

private:
    double dir_, dist_, amax_, v_peak_, t_acc_, t_cruise_, t_total_;
};

// Tracking law that makes the Axis follow a profile using its own reported state:
// a = a_ref + kv (v_ref - v) + kp (x_ref - x), limited to +-a_limit.
double trackAccel(const ProfilePoint& ref, double x, double v, double kp, double kv, double a_limit);

}

#endif
