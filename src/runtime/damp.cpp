#include "runtime/damp.h"

#include <algorithm>
#include <cmath>

namespace rt {

namespace {
const double kPi = 3.141592653589793;
}

double DampRuntime::gainFor(double zeta, double w0, double bJ, double g) {
    return (2 * zeta * w0 - bJ) * g / (w0 * w0);
}

DampRuntime::DampRuntime(const DampConfig& c) : c_(c), est_(c.est_kind, c.ts, c.est_fc) {}

void DampRuntime::abort(const char* why) {
    reason_ = why;
    phase_ = DampPhase::Stopping;
    t_phase_ = 0;
    still_ = 0;
}

DampRecord DampRuntime::tick(hw::Hardware& hw, bool overrun) {
    const hw::Sample s = hw.read();
    DampRecord r;
    r.t_ns = s.t_ns;
    r.counts = s.counts;
    r.pos_usteps = s.pos_usteps;
    r.speed_toggles = s.speed_toggles;
    r.n_edges = s.n_edges;
    r.phi = core::wrap(core::thetaFromCounts(s.counts, c_.calib) - kPi);
    r.omega = est_.update(s.counts) * core::omegaScale(c_.calib);
    r.x = core::xFromPos(s.pos_usteps, c_.calib);
    r.v = core::vFromAxisSpeed(s.speed_toggles, c_.calib);
    r.overrun = overrun;
    r.limit_left = s.limit_left;
    r.limit_right = s.limit_right;
    r.axis_accelerating = s.axis_accelerating;
    overruns_ = overrun ? overruns_ + 1 : 0;

    if (phase_ != DampPhase::Done && !driver_on_) {   // first tick: driver on, hold
        hw.enableDriver(true);
        driver_on_ = true;
    }

    // Aborts (WAIT and ENGAGED). The Axis can only be expected in ACCELERATE after a command.
    if (phase_ == DampPhase::Wait || phase_ == DampPhase::Engaged) {
        const core::SupervisorParams& L = c_.limits;
        const double d = core::stoppingDistance(L, r.v);
        if (stop_.exchange(false)) abort("stop request");
        else if (s.limit_left || s.limit_right) abort("limit switch");
        else if (s.encoder_fault || s.edges_overflow) abort("encoder fault");
        else if (sent_prev_ && !s.axis_accelerating) abort("Axis not in ACCELERATE");
        else if (overruns_ >= L.overrun_streak) abort("loop overrun streak");
        else if (std::fabs(r.phi) > c_.phi_abort) abort("|phi| above phi_abort (growing swing: sign error?)");
        else if (std::fabs(r.v) > c_.v_abort) abort("|v| above v_abort");
        else if (r.x >= L.x_max || r.x <= L.x_min || (r.v > 0 && r.x + d >= L.x_max) ||
                 (r.v < 0 && r.x - d <= L.x_min))
            abort("soft limit within braking distance");
    }

    if (phase_ == DampPhase::Wait) {
        const bool crossing = has_prev_ && ((prev_phi_ < 0) != (r.phi < 0));
        if (crossing && std::fabs(r.phi) < c_.engage_phi_max && std::fabs(r.omega) >= c_.engage_omega_min) {
            phase_ = DampPhase::Engaged;
            t_phase_ = 0;
            t_engage_ = t_;
            phi_engage_ = r.phi;
        } else if (t_phase_ > c_.wait_max) {
            reason_ = "no swing to damp (wait_max)";
            phase_ = DampPhase::Done;
        }
    } else if (phase_ == DampPhase::Engaged && t_phase_ > c_.duration) {
        abort("completed");
    }

    double a = 0.0;
    if (phase_ == DampPhase::Engaged) {
        const double u = -c_.gain * r.omega;
        a = std::clamp(u, -c_.a_max, c_.a_max);
        r.saturated = a != u;
    } else if (phase_ == DampPhase::Stopping) {
        if (r.v != 0.0) a = -std::copysign(std::min(c_.limits.a_brake, std::fabs(r.v) / c_.ts), r.v);
        still_ = s.speed_toggles == 0 ? still_ + 1 : 0;
        if (still_ >= 25) phase_ = DampPhase::Done;   // 50 ms at standstill
        else if (t_phase_ > c_.limits.stop_timeout) {
            reason_ = "stop timeout";
            hw.halt();
            phase_ = DampPhase::Done;
        }
    }
    r.a_cmd = a;
    r.axis_arg = core::axisAccelArg(a, c_.calib);
    r.phase = static_cast<int>(phase_);

    if (phase_ == DampPhase::Done) {
        if (driver_on_) {
            hw.commandAccel(0);
            hw.enableDriver(false);
            driver_on_ = false;
        }
        sent_prev_ = false;
    } else {
        hw.commandAccel(r.axis_arg);
        sent_prev_ = true;
    }
    r.driver_enabled = driver_on_;

    prev_phi_ = r.phi;
    has_prev_ = true;
    t_ += c_.ts;
    t_phase_ += c_.ts;
    return r;
}

}
