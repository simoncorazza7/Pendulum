#include "runtime/balance.h"

#include <cmath>
#include <cstdint>

namespace rt {

namespace {
const double kPi = 3.141592653589793;
}

// Each edge is an exact crossing of a count boundary: raising the count to n happened at u = n,
// lowering it to n at u = n + 1 (docs/stage0_7_0_10_fits.md). Keep the two newest as (time, angle).
void BalanceRuntime::trackEdges(const hw::Sample& s) {
    if (!has_count_) {                       // first sample: start from its count, ignore its edges
        prev_count_ = s.counts;
        has_count_ = true;
        return;
    }
    for (int i = 0; i < s.n_edges; ++i) {
        const hw::Edge& e = s.edges[static_cast<std::size_t>(i)];
        const std::int64_t step = e.count_after - prev_count_;
        prev_count_ = e.count_after;
        if (step == 0) continue;
        const double u = static_cast<double>(step > 0 ? e.count_after : e.count_after + 1);
        const core::Calib& c = c_.calib;
        const double th = core::wrap(c.theta_sign * (u - c.c_bottom - c.cpr / 2.0) * 2 * kPi / c.cpr);
        e1_ = e2_;
        gap_e1_ = gap_e2_;
        e2_ = EdgeTheta{e.ts_ns, th};
        gap_e2_ = n_edges_seen_ > 0 ? e.ts_ns - last_edge_ns_ : INT64_MAX;
        last_edge_ns_ = e.ts_ns;
        n_edges_seen_++;
    }
    prev_count_ = s.counts;
}

bool BalanceRuntime::released(std::int64_t t_ns) const {
    if (n_edges_seen_ < 2) return false;
    const double dth = e2_.theta - e1_.theta;   // same direction as the step before, away from upright
    const bool away = dth * e2_.theta > 0 && std::fabs(e2_.theta) < kPi / 2;
    return away && gap_e1_ >= static_cast<std::int64_t>(c_.release_quiet * 1e9) &&
           t_ns - e2_.ts_ns <= static_cast<std::int64_t>(c_.release_fresh * 1e9);
}

// Free motion near upright: theta(t) = A e^{w0 (t - t2)} + B e^{-w0 (t - t2)} through both edges
// (exact times, exact boundary angles), so theta'(t) needs no filtering and no assumption about
// how the hand let go.
double BalanceRuntime::omegaFree(std::int64_t t_ns) const {
    const double w0 = c_.w0;
    const double d = static_cast<double>(e1_.ts_ns - e2_.ts_ns) * 1e-9;   // < 0
    const double tau = static_cast<double>(t_ns - e2_.ts_ns) * 1e-9;      // >= 0
    const double ep = std::exp(w0 * d), em = std::exp(-w0 * d);
    const double det = em - ep;
    if (!(std::fabs(det) > 1e-12)) return 0.0;
    const double A = (e2_.theta * em - e1_.theta) / det;
    const double B = (e1_.theta - e2_.theta * ep) / det;
    return w0 * (A * std::exp(w0 * tau) - B * std::exp(-w0 * tau));
}

BalanceRuntime::BalanceRuntime(const BalanceConfig& c)
    : c_(c), est_(c.est_kind, c.ts, c.est_fc), delay_(static_cast<std::size_t>(c.delay_samples)) {}

TickRecord BalanceRuntime::tick(hw::Hardware& hw, bool overrun) {
    const hw::Sample s = hw.read();
    TickRecord r;
    r.t_ns = s.t_ns;
    r.counts = s.counts;
    r.pos_usteps = s.pos_usteps;
    r.speed_toggles = s.speed_toggles;
    r.n_edges = s.n_edges;
    r.theta = core::thetaFromCounts(s.counts, c_.calib);
    r.omega = est_.update(s.counts) * core::omegaScale(c_.calib);
    r.x = core::xFromPos(s.pos_usteps, c_.calib);
    r.v = core::vFromAxisSpeed(s.speed_toggles, c_.calib);
    r.overrun = overrun;
    r.limit_left = s.limit_left;
    r.limit_right = s.limit_right;
    r.axis_accelerating = s.axis_accelerating;
    trackEdges(s);
    r.released = released(s.t_ns);

    core::SupervisorInputs in;
    in.theta = r.theta;
    in.omega = r.omega;
    in.x = r.x;
    in.v = r.v;
    in.limit_pressed = s.limit_left || s.limit_right;
    // The Axis can only be expected in ACCELERATE once a command has been sent.
    in.axis_accelerating = !sent_prev_ || s.axis_accelerating;
    in.overrun = overrun;
    in.encoder_fault = s.encoder_fault || s.edges_overflow;
    in.arm = arm_.exchange(false);
    in.stop = stop_.exchange(false);
    in.released = r.released;
    const core::SupervisorResult sr = core::supervisorStep(c_.sup, sup_, in, c_.ts);
    sup_ = sr.state;
    const core::SupervisorOutput& o = sr.out;
    r.mode = static_cast<int>(o.mode);
    r.event = static_cast<int>(o.event);

    if (c_.actuate && o.driver_enabled != driver_on_) {
        hw.enableDriver(o.driver_enabled);
        driver_on_ = o.driver_enabled;
    }
    r.driver_enabled = driver_on_;

    if (o.reset_controller) {
        pid_ = core::PidState{};
        delay_.reset();
        // Momentum pre-load: v_inf = v_engage + L_eq theta'(engage) + g int theta dt, and the PID
        // equilibrium forces int theta = -I(0); I(0) = theta'/w0^2 cancels the pendulum's momentum
        // (cart at rest at engage). Only with a detected release: the two edges then bracket free motion.
        if (c_.w0 > 0 && r.released) {
            r.omega_release = omegaFree(s.t_ns);
            pid_.integral = r.omega_release / (c_.w0 * c_.w0);
        }
    }
    double a = 0.0;
    if (o.run_controller) {
        const core::PidResult pr = core::pidStep(c_.pid, pid_, r.theta, r.v, c_.ts, &r.omega);
        pid_ = pr.state;
        r.saturated = pr.saturated;
        r.a_ctrl = delay_.push(pr.a_cmd);
        a = r.a_ctrl;
    } else if (o.mode == core::Mode::Stopping) {
        a = o.a_brake_cmd;
    }
    r.integral = pid_.integral;
    r.a_cmd = a;
    r.axis_arg = core::axisAccelArg(a, c_.calib);

    r.sent = false;
    if (c_.actuate && o.mode != core::Mode::Idle) {
        hw.commandAccel(r.axis_arg);
        r.sent = true;
    }
    sent_prev_ = r.sent;
    return r;
}

}
