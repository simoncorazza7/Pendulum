#include "runtime/mover.h"

#include <algorithm>
#include <cmath>

namespace rt {

namespace {
const double kPi = 3.141592653589793;
}

double Mover::shapedAccel(double d, double v, double w0, double a_max, double* v_peak) {
    const double T0 = 2 * kPi / w0;
    d = std::fabs(d);
    double a, vp;
    if (d >= v * T0) { a = v / T0; vp = v; }            // trapezoid, accelerates for exactly T0
    else { a = d / (T0 * T0); vp = d / T0; }             // triangle, accelerates for exactly T0
    if (a > a_max) {                                     // cap: no longer exactly shaped
        a = a_max;
        vp = std::min(v, std::sqrt(d * a_max));
    }
    if (v_peak) *v_peak = vp;
    return a;
}

Mover::Mover(const MoverConfig& c) : c_(c) {}

void Mover::start(std::int64_t target, double v, Side seek, const hw::Sample& s, bool mustFind) {
    const core::Calib& c = c_.calib;
    const double d = c.x_sign * static_cast<double>(target - s.pos_usteps) / c.usteps_per_m;   // m, x frame
    double vp = v;
    const double a = std::fabs(d) > 0 ? shapedAccel(d, v, c_.w0, c_.a_max, &vp) : c_.a_max;
    prof_ = core::TrapezoidProfile(d, std::max(vp, 1e-6), std::max(a, 1e-6));
    a_lim_ = 1.5 * std::max(a, 0.05);
    pos0_ = s.pos_usteps;
    seek_ = seek;
    must_find_ = mustFind;
    found_ = failing_ = sent_prev_ = false;
    ignore_left_ = s.limit_left;
    ignore_right_ = s.limit_right;
    sw_pos_ = 0;
    t_ = 0;
    overruns_ = still_ = brake_ticks_ = 0;
    reason_ = "";
    state_ = MoverState::Moving;
}

void Mover::fail(const char* why) {
    reason_ = why;
    failing_ = true;
    state_ = MoverState::Braking;
    still_ = 0;
}

MoverRecord Mover::tick(hw::Hardware& hw, const hw::Sample& s, bool overrun, bool stop) {
    MoverRecord r;
    const core::Calib& c = c_.calib;
    const double x = c.x_sign * static_cast<double>(s.pos_usteps - pos0_) / c.usteps_per_m;
    const double v = core::vFromAxisSpeed(s.speed_toggles, c);
    overruns_ = overrun ? overruns_ + 1 : 0;

    if (state_ == MoverState::Moving) {
        if (ignore_left_ && !s.limit_left) ignore_left_ = false;
        if (ignore_right_ && !s.limit_right) ignore_right_ = false;
        const bool left = s.limit_left && !ignore_left_, right = s.limit_right && !ignore_right_;
        const core::ProfilePoint ref = prof_.at(t_);
        r.x_ref = ref.x;
        r.v_ref = ref.v;
        if ((seek_ == Side::Left && left) || (seek_ == Side::Right && right)) {
            found_ = true;
            sw_pos_ = s.pos_usteps;
            state_ = MoverState::Braking;
            still_ = 0;
        } else if (left || right) {
            fail(left ? "unexpected limit switch (left)" : "unexpected limit switch (right)");
        } else if ((ignore_left_ || ignore_right_) && std::fabs(x) > c_.release_within) {
            fail("switch pressed at the start is still pressed: driving into the end");
        } else if (stop) {
            fail("stop request");
        } else if (overruns_ >= c_.overrun_streak) {
            fail("loop overrun streak");
        } else if (sent_prev_ && !s.axis_accelerating) {
            fail("Axis not in ACCELERATE");
        } else if (std::fabs(ref.x - x) > c_.track_tol) {
            fail("tracking error");
        } else if (t_ > prof_.duration()) {
            if (seek_ != Side::None && must_find_) fail("switch not found before the give-up position");
            else { state_ = MoverState::Braking; still_ = 0; }
        }
        if (state_ == MoverState::Moving) r.a_cmd = core::trackAccel(ref, x, v, c_.kp, c_.kv, a_lim_);
        t_ += c_.ts;
    }

    if (state_ == MoverState::Braking) {
        r.a_cmd = v != 0.0 ? -std::copysign(std::min(c_.a_brake, std::fabs(v) / c_.ts), v) : 0.0;
        still_ = s.speed_toggles == 0 ? still_ + 1 : 0;
        ++brake_ticks_;
        if (still_ >= 25) {                                  // 50 ms at standstill
            state_ = failing_ ? MoverState::Failed : MoverState::Done;
        } else if (brake_ticks_ * c_.ts > 2.0) {             // never expected: the Axis does not stop
            hw.halt();
            reason_ = "stop timeout (HALT)";
            state_ = MoverState::Failed;
        }
    }

    r.axis_arg = core::axisAccelArg(r.a_cmd, c);
    r.state = static_cast<int>(state_);
    if (state_ != MoverState::Idle) {
        hw.commandAccel(r.axis_arg);
        r.sent = true;
    }
    sent_prev_ = r.sent;
    return r;
}

}
