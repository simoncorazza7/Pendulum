#include "runtime/swingup.h"

#include <algorithm>
#include <cmath>

namespace rt {

namespace {
const double kPi = 3.141592653589793;
double sgn(double x) { return x > 0 ? 1.0 : (x < 0 ? -1.0 : 0.0); }
}

const char* swingPhaseName(SwingPhase p) {
    switch (p) {
        case SwingPhase::Kick: return "KICK";
        case SwingPhase::Pump: return "PUMP";
        case SwingPhase::Balance: return "BALANCE";
        case SwingPhase::Brake: return "BRAKE";
        case SwingPhase::DampOut: return "DAMP_OUT";
        case SwingPhase::Stopping: return "STOPPING";
        case SwingPhase::Done: return "DONE";
    }
    return "?";
}

SwingUpRuntime::SwingUpRuntime(const SwingUpConfig& c) : c_(c), est_(c.est_kind, c.ts, c.est_fc) {
    // core::VelocityEstimator's BdLp2 biquad (bilinear, prewarped 2nd-order Butterworth at est_fc)
    const double k = std::tan(kPi * c.est_fc * c.ts);
    const double n = 1 / (1 + std::sqrt(2.0) * k + k * k);
    b0_ = k * k * n;
    b1_ = 2 * b0_;
    b2_ = b0_;
    a1_ = 2 * (k * k - 1) * n;
    a2_ = (1 - std::sqrt(2.0) * k + k * k) * n;
}

void SwingUpRuntime::enter(SwingPhase p, const char* why) {
    if (why && !reason_[0]) reason_ = why;     // the first reason is why the run ended
    phase_ = p;
    t_phase_ = 0;
    still_ = 0;
}

// Pumping / damping / kick commands: a_max and the authority taper toward v_max of the balance PID.
double SwingUpRuntime::limitPump(double a, double v) const {
    const core::PidParams& p = c_.pid;
    a = std::clamp(a, -p.a_max, p.a_max);
    const double w = p.v_taper * p.v_max;
    auto taper = [w](double margin) { return w > 0 ? std::clamp(margin / w, 0.0, 1.0) : (margin > 0 ? 1.0 : 0.0); };
    if (a > 0) a *= taper(p.v_max - v);
    if (a < 0) a *= taper(p.v_max + v);
    return a;
}

SwingRecord SwingUpRuntime::tick(hw::Hardware& hw, const hw::Sample& s, bool overrun) {
    SwingRecord r;
    const core::Calib& c = c_.calib;
    r.theta = core::thetaFromCounts(s.counts, c);
    r.omega = est_.update(s.counts) * core::omegaScale(c);
    if (!has_c0_) { c0_ = s.counts; has_c0_ = true; }              // filter state = steady state at the start
    const double xin = static_cast<double>(s.counts - c0_);
    const double yf = b0_ * xin + b1_ * x1_ + b2_ * x2_ - a1_ * y1_ - a2_ * y2_;
    x2_ = x1_; x1_ = xin; y2_ = y1_; y1_ = yf;
    r.theta_f = core::wrap(c.theta_sign * (static_cast<double>(c0_) + yf + 0.5 - c.c_bottom - c.cpr / 2.0) * 2 * kPi /
                           c.cpr);
    r.x = core::xFromPos(s.pos_usteps, c);
    r.v = core::vFromAxisSpeed(s.speed_toggles, c);
    const double w2 = c_.w0 * c_.w0;
    const double E = 0.5 * r.omega * r.omega + w2 * (std::cos(r.theta_f) - 1.0);
    r.energy = E / (2 * w2) + 1.0;
    overruns_ = overrun ? overruns_ + 1 : 0;

    const bool active = phase_ == SwingPhase::Kick || phase_ == SwingPhase::Pump || phase_ == SwingPhase::Balance ||
                        phase_ == SwingPhase::DampOut;
    if (active) {
        max_x_ = std::max(max_x_, std::fabs(r.x));
        peak_v_ = std::max(peak_v_, std::fabs(r.v));
        if (phase_ != SwingPhase::DampOut) peak_e_ = std::max(peak_e_, r.energy);
        const core::SupervisorParams& L = c_.limits;
        const double d = core::stoppingDistance(L, r.v);
        const bool soft = r.x >= L.x_max || r.x <= L.x_min || (r.v > 0 && r.x + d >= L.x_max) ||
                          (r.v < 0 && r.x - d <= L.x_min);
        const bool go = damp_req_.exchange(false);
        if (stop_req_.exchange(false)) enter(SwingPhase::Stopping, "stop request");
        else if (s.limit_left || s.limit_right) enter(SwingPhase::Stopping, "limit switch");
        else if (s.encoder_fault || s.edges_overflow) enter(SwingPhase::Stopping, "encoder fault");
        else if (sent_prev_ && !s.axis_accelerating) enter(SwingPhase::Stopping, "Axis not in ACCELERATE");
        else if (overruns_ >= L.overrun_streak) enter(SwingPhase::Stopping, "loop overrun streak");
        else {
            switch (phase_) {
                case SwingPhase::Kick:
                    if (soft) enter(SwingPhase::Stopping, "soft limit while kicking");
                    else if (go) enter(SwingPhase::DampOut, "operator stop");
                    else if (t_phase_ >= c_.kick_time) enter(SwingPhase::Pump);
                    break;
                case SwingPhase::Pump:
                    if (soft) { after_brake_ = SwingPhase::DampOut; enter(SwingPhase::Brake, "soft limit while pumping"); }
                    else if (go) enter(SwingPhase::DampOut, "operator stop");
                    else if (r.energy - 1.0 > c_.e_abort) enter(SwingPhase::DampOut, "energy above e_abort");
                    else if (t_ > c_.pump_max) enter(SwingPhase::DampOut, "no hand-over within pump_max");
                    else if (std::fabs(r.theta_f) < c_.theta_sw && std::fabs(r.theta_f + r.omega / c_.w0) < c_.u_sw) {
                        t_handover_ = t_;
                        pid_ = core::PidState{};
                        if (c_.preload) pid_.integral = r.omega / w2;   // docs/stage1_drift.md: free motion at engage
                        enter(SwingPhase::Balance);
                    }
                    break;
                case SwingPhase::Balance:
                    after_brake_ = SwingPhase::DampOut;
                    if (soft) enter(SwingPhase::Brake, "balanced until the soft limit");
                    else if (go) enter(SwingPhase::Brake, "operator stop");
                    else if (std::fabs(r.theta) > L.theta_abort) enter(SwingPhase::Brake, "fell (theta_abort)");
                    else if (t_phase_ > c_.balance_max) enter(SwingPhase::Brake, "balance_max reached");
                    break;
                case SwingPhase::DampOut: {
                    const double quiet = -w2 * (1.0 + std::cos(c_.quiet_deg * kPi / 180.0));
                    if (soft) enter(SwingPhase::Stopping, "soft limit while removing energy");
                    else if (E < quiet) { after_brake_ = SwingPhase::Done; enter(SwingPhase::Brake); }
                    else if (t_phase_ > c_.damp_max) enter(SwingPhase::Stopping, "damp_max reached");
                    break;
                }
                default:
                    break;
            }
        }
    }

    double a = 0.0;
    const double sCos = sgn(r.omega * std::cos(r.theta_f));
    switch (phase_) {
        case SwingPhase::Kick:
            a = limitPump(c_.a_pump + centring(r.x, r.v), r.v);
            break;
        case SwingPhase::Pump: {
            const double eRef = c_.e_ref * 2 * w2;
            const double ae = std::clamp(c_.k_e * (E - eRef) * sCos, -c_.a_pump, c_.a_pump);
            a = limitPump(ae + centring(r.x, r.v), r.v);
            break;
        }
        case SwingPhase::Balance: {
            const core::PidResult pr = core::pidStep(c_.pid, pid_, r.theta, r.v, c_.ts, &r.omega);
            pid_ = pr.state;
            a = pr.a_cmd;
            r.saturated = pr.saturated;
            t_balance_ = t_phase_;
            break;
        }
        case SwingPhase::DampOut: {
            const double ae = std::clamp(c_.k_e * (E + 2 * w2) * sCos, -c_.a_damp, c_.a_damp);
            a = limitPump(ae + centring(r.x, r.v), r.v);
            break;
        }
        case SwingPhase::Brake:
        case SwingPhase::Stopping:
            a = r.v != 0.0 ? -std::copysign(std::min(c_.limits.a_brake, std::fabs(r.v) / c_.ts), r.v) : 0.0;
            still_ = s.speed_toggles == 0 ? still_ + 1 : 0;
            if (still_ >= 25) {                                   // 50 ms at standstill
                if (phase_ == SwingPhase::Brake) enter(after_brake_);
                else enter(SwingPhase::Done);
            } else if (t_phase_ > c_.limits.stop_timeout) {       // never expected: the Axis does not stop
                hw.halt();
                enter(SwingPhase::Done, "stop timeout (HALT)");
            }
            break;
        case SwingPhase::Done:
            break;
    }
    r.a_cmd = a;
    r.axis_arg = core::axisAccelArg(a, c);
    r.integral = pid_.integral;
    r.phase = static_cast<int>(phase_);
    hw.commandAccel(r.axis_arg);          // also in DONE: a = 0 (the cart stands still), the caller holds it next
    r.sent = true;
    sent_prev_ = true;
    t_ += c_.ts;
    t_phase_ += c_.ts;
    return r;
}

}
