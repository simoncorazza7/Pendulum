#include "core/profile.h"

#include <algorithm>
#include <cmath>

namespace core {

TrapezoidProfile::TrapezoidProfile(double distance, double vmax, double amax)
    : dir_(distance < 0 ? -1.0 : 1.0), dist_(std::fabs(distance)), amax_(amax) {
    const double d_acc = vmax * vmax / (2 * amax);           // distance to reach vmax
    if (2 * d_acc >= dist_) {                                 // triangular
        v_peak_ = std::sqrt(dist_ * amax);
        t_acc_ = v_peak_ / amax;
        t_cruise_ = 0;
    } else {
        v_peak_ = vmax;
        t_acc_ = vmax / amax;
        t_cruise_ = (dist_ - 2 * d_acc) / vmax;
    }
    t_total_ = 2 * t_acc_ + t_cruise_;
}

ProfilePoint TrapezoidProfile::at(double t) const {
    double x, v, a;
    if (t <= 0) {
        x = v = a = 0;
    } else if (t < t_acc_) {
        a = amax_;
        v = amax_ * t;
        x = 0.5 * amax_ * t * t;
    } else if (t < t_acc_ + t_cruise_) {
        a = 0;
        v = v_peak_;
        x = 0.5 * amax_ * t_acc_ * t_acc_ + v_peak_ * (t - t_acc_);
    } else if (t < t_total_) {
        const double td = t_total_ - t;                       // time left
        a = -amax_;
        v = amax_ * td;
        x = dist_ - 0.5 * amax_ * td * td;
    } else {
        x = dist_;
        v = a = 0;
    }
    return {dir_ * x, dir_ * v, dir_ * a};
}

double trackAccel(const ProfilePoint& ref, double x, double v, double kp, double kv, double a_limit) {
    const double a = ref.a + kv * (ref.v - v) + kp * (ref.x - x);
    return std::min(a_limit, std::max(-a_limit, a));
}

}
