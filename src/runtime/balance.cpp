#include "runtime/balance.h"

namespace rt {

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
