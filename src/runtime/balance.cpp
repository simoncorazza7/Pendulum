#include "runtime/balance.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace rt {

namespace {
const double kPi = 3.141592653589793;
}

// Each edge is an exact crossing of a count boundary: raising the count to n happened at u = n,
// lowering it to n at u = n + 1 (docs/stage0_7_0_10_fits.md). Keep the newest as (time, angle).
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
        hist_[head_] = EdgeTheta{e.ts_ns, th};
        head_ = (head_ + 1) % kEdgeHist;
        if (n_hist_ < kEdgeHist) n_hist_++;
    }
    prev_count_ = s.counts;
}

// Free motion near upright: theta(t) = A e^{w0 tau} + B e^{-w0 tau}, tau = t - t_newest. Least squares
// over all newest edges of the current run away from upright (up to kEdgeHist; exact times, exact
// boundary angles). Not the newest n only: a free fall accelerates, so a fixed number of edges spans
// less and less time and would never reach release_span.
bool BalanceRuntime::released(std::int64_t t_ns) {
    if (c_.release_edges < 3 || n_hist_ < 2 || !(c_.w0 > 0)) return false;
    auto at = [this](int back) { return hist_[(head_ - 1 - back + 2 * kEdgeHist) % kEdgeHist]; };   // 0 = newest
    if (!(std::fabs(at(0).theta) < kPi / 2)) return false;
    int n = 1;                               // edges in the run: every step away from upright
    while (n < n_hist_ && (at(n - 1).theta - at(n).theta) * at(n - 1).theta > 0) ++n;
    if (n < c_.release_edges) return false;
    EdgeTheta e[kEdgeHist];
    for (int i = 0; i < n; ++i) e[i] = at(n - 1 - i);   // oldest .. newest
    const EdgeTheta& last = e[n - 1];
    if (t_ns - last.ts_ns > static_cast<std::int64_t>(c_.release_fresh * 1e9)) return false;
    if (last.ts_ns - e[0].ts_ns < static_cast<std::int64_t>(c_.release_span * 1e9)) return false;
    const double w0 = c_.w0;
    double sxx = 0, sxy = 0, syy = 0, sxt = 0, syt = 0;
    for (int i = 0; i < n; ++i) {
        const double tau = static_cast<double>(e[i].ts_ns - last.ts_ns) * 1e-9;
        const double x = std::exp(w0 * tau), y = std::exp(-w0 * tau);
        sxx += x * x; sxy += x * y; syy += y * y;
        sxt += x * e[i].theta; syt += y * e[i].theta;
    }
    const double det = sxx * syy - sxy * sxy;
    if (!(std::fabs(det) > 1e-12)) return false;
    const double A = (sxt * syy - sxy * syt) / det, B = (sxx * syt - sxy * sxt) / det;
    double ss = 0;
    for (int i = 0; i < n; ++i) {
        const double tau = static_cast<double>(e[i].ts_ns - last.ts_ns) * 1e-9;
        const double r = e[i].theta - A * std::exp(w0 * tau) - B * std::exp(-w0 * tau);
        ss += r * r;
    }
    const double q = 2 * kPi / c_.calib.cpr;
    if (std::sqrt(ss / n) / q > c_.release_rms) return false;
    if (!(A * last.theta > 0)) return false;                 // the growing mode carries it away from upright
    if (std::fabs(w0 * (A - B)) < c_.release_omega_min) return false;
    relA_ = A;
    relB_ = B;
    relT_ = last.ts_ns;
    return true;
}

double BalanceRuntime::omegaFit(std::int64_t t_ns) const {
    const double tau = static_cast<double>(t_ns - relT_) * 1e-9;
    return c_.w0 * (relA_ * std::exp(c_.w0 * tau) - relB_ * std::exp(-c_.w0 * tau));
}

BalanceRuntime::BalanceRuntime(const BalanceConfig& c)
    : c_(c), est_(c.est_kind, c.ts, c.est_fc), delay_(static_cast<std::size_t>(c.delay_samples)),
      epoch_(c.epoch_base) {}

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

    // Stage 2b: is the PC's command for this tick (from the sample remote_delay ticks ago) here?
    double a_remote = 0.0;
    bool have_remote = false;
    bool remote_fault = false;
    if (c_.remote) {
        const std::uint32_t want = c_.remote->seq() - static_cast<std::uint32_t>(c_.remote_delay);
        have_remote = c_.remote->command(want, &a_remote) && std::isfinite(a_remote);
        misses_ = (sup_.mode == core::Mode::Engaged && !have_remote) ? misses_ + 1 : 0;
        // The supervisor reads it while arming (refusal), ARMED and ENGAGED (abort); not in STOPPING.
        remote_fault = misses_ >= c_.remote_max_miss || !c_.remote->linkUp();
    }

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
    in.remote_fault = remote_fault;
    const core::SupervisorResult sr = core::supervisorStep(c_.sup, sup_, in, c_.ts);
    sup_ = sr.state;
    const core::SupervisorOutput& o = sr.out;
    r.mode = static_cast<int>(o.mode);
    r.event = static_cast<int>(o.event);

    const bool driverWanted = c_.keep_driver_on || o.driver_enabled;
    if (c_.actuate && driverWanted != driver_on_) {
        hw.enableDriver(driverWanted);
        driver_on_ = driverWanted;
    }
    r.driver_enabled = driver_on_;

    if (o.reset_controller) {
        pid_ = core::PidState{};
        delay_.reset();
        a_hold_ = 0.0;
        ++epoch_;
        // Momentum pre-load: v_inf = v_engage + L_eq theta'(engage) + g int theta dt, and the PID
        // equilibrium forces int theta = -I(0); I(0) = theta'/w0^2 cancels the pendulum's momentum
        // (cart at rest at engage). Only with a detected release: the fitted edges then are free motion.
        if (c_.preload && c_.sup.engage_on_release && r.released) {
            r.omega_release = omegaFit(s.t_ns);
            pid_.integral = r.omega_release / (c_.w0 * c_.w0);
        }
        i0_ = pid_.integral;
    }
    double a = 0.0;
    if (o.run_controller) {
        const core::PidResult pr = core::cascadeStep(c_.pid, pid_, r.theta, r.x, r.v, c_.ts, &r.omega);
        pid_ = pr.state;
        r.saturated = pr.saturated;
        if (c_.remote) {                 // the PC's command from remote_delay ticks ago; the local PID is a shadow
            r.a_local = pr.a_cmd;
            if (have_remote) a_hold_ = std::max(-c_.pid.a_max, std::min(c_.pid.a_max, a_remote));
            else r.remote_miss = true;
            r.a_ctrl = a_hold_;
        } else {
            r.a_ctrl = delay_.push(pr.a_cmd);
        }
        a = r.a_ctrl;
    } else if (o.mode == core::Mode::Stopping) {
        a = o.a_brake_cmd;
    } else if (o.mode == core::Mode::Idle && c_.keep_driver_on && r.v != 0.0) {
        a = -std::copysign(std::min(c_.sup.a_brake, std::fabs(r.v) / c_.ts), r.v);   // hold
    }
    r.integral = pid_.integral;
    r.controller = o.run_controller;
    r.epoch = epoch_;
    r.i0 = i0_;
    r.a_cmd = a;
    r.axis_arg = core::axisAccelArg(a, c_.calib);

    r.sent = false;
    if (c_.actuate && (o.mode != core::Mode::Idle || c_.keep_driver_on)) {
        hw.commandAccel(r.axis_arg);
        r.sent = true;
    }
    sent_prev_ = r.sent;
    return r;
}

}
