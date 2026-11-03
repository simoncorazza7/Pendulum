#include "runtime/params.h"

#include <cmath>
#include <stdexcept>

namespace rt {

namespace {

const double kDeg = 3.141592653589793 / 180.0;

core::VelocityEstimator::Kind estimatorKind(const std::string& s) {
    if (s == "bdlp1") return core::VelocityEstimator::Kind::BdLp1;
    if (s == "bdlp2") return core::VelocityEstimator::Kind::BdLp2;
    throw std::runtime_error("ctrl.estimator must be bdlp1 or bdlp2, not '" + s + "'");
}

// Axis caps in SI (its maxAcceleration as set by pendctl, its default maxSpeed), for brakeDistance.
void axisCaps(const Config& cfg, const core::Calib& c, core::SupervisorParams& p) {
    const double togglesPerM = c.toggles_per_ustep * c.usteps_per_m;
    p.axis_a = static_cast<double>(cfg.getInt("axis.max_accel")) / togglesPerM;
    p.axis_v = static_cast<double>(cfg.getInt("axis.max_speed")) / togglesPerM;
}

}

core::Calib calibFrom(const Config& cfg) {
    core::Calib c;
    c.cpr = static_cast<int>(cfg.getInt("encoder.counts_per_rev"));
    c.theta_sign = static_cast<int>(cfg.getInt("calib.theta_sign"));
    c.x_sign = static_cast<int>(cfg.getInt("calib.x_sign"));
    c.usteps_per_m = cfg.getDouble("axis.usteps_per_m");
    c.toggles_per_ustep = cfg.getDouble("axis.toggles_per_ustep");
    if (c.theta_sign != 1 && c.theta_sign != -1) throw std::runtime_error("calib.theta_sign must be +1 or -1");
    if (c.x_sign != 1 && c.x_sign != -1) throw std::runtime_error("calib.x_sign must be +1 or -1");
    return c;
}

SwingCalibParams swingCalibFrom(const Config& cfg) {
    SwingCalibParams p;
    const double countsPerDeg = static_cast<double>(cfg.getInt("encoder.counts_per_rev")) / 360.0;
    p.w0 = cfg.getDouble("plant.w0");
    p.cpr = static_cast<int>(cfg.getInt("encoder.counts_per_rev"));
    p.sigma = cfg.getDouble("calib.swing_sigma");
    p.window = cfg.getDouble("calib.window");
    p.amp_min = cfg.getDouble("calib.amp_min_deg") * countsPerDeg;
    p.amp_max = cfg.getDouble("calib.amp_max_deg") * countsPerDeg;
    p.rms_max = cfg.getDouble("calib.rms_max");
    p.se_max = cfg.getDouble("calib.se_max");
    p.agree = cfg.getDouble("calib.agree");
    return p;
}

BalanceConfig balanceFrom(const Config& cfg) {
    BalanceConfig b;
    b.ts = cfg.getDouble("loop.ts");
    b.calib = calibFrom(cfg);
    const double w0 = cfg.getDouble("plant.w0"), bJ = cfg.getDouble("plant.bJ");
    const core::Gains g = core::designGains(w0, bJ, cfg.getDouble("ctrl.wc_over_w0") * w0, cfg.getDouble("ctrl.zeta"),
                                            cfg.getDouble("ctrl.p_over_w0") * w0);
    b.pid.kp = g.kp;
    b.pid.ki = g.ki;
    b.pid.kd = g.kd;
    b.pid.a_max = cfg.getDouble("ctrl.a_max");
    b.pid.v_max = cfg.getDouble("ctrl.v_max");
    b.pid.v_taper = cfg.getDouble("ctrl.v_taper");
    b.pid.i_max = cfg.getDouble("ctrl.i_max");
    b.pid.i_delay = cfg.getDouble("ctrl.i_delay");
    b.pid.theta_ref = 0.0;    // the trim is applied through c_bottom (encoder.trim)
    b.pid.vel_fc = cfg.getDouble("ctrl.est_fc");
    const std::string why = core::validate(b.pid);
    if (!why.empty()) throw std::runtime_error("controller parameters: " + why);
    b.est_kind = estimatorKind(cfg.getString("ctrl.estimator"));
    b.est_fc = cfg.getDouble("ctrl.est_fc");
    b.delay_samples = static_cast<int>(cfg.getInt("ctrl.delay_samples"));

    core::SupervisorParams& s = b.sup;
    s.theta_engage = cfg.getDouble("sup.theta_engage_deg") * kDeg;
    s.omega_engage = cfg.getDouble("sup.omega_engage");
    s.engage_samples = static_cast<int>(cfg.getInt("sup.engage_samples"));
    s.theta_abort = cfg.getDouble("sup.theta_abort_deg") * kDeg;
    s.x_min = cfg.getDouble("sup.x_min");
    s.x_max = cfg.getDouble("sup.x_max");
    s.a_brake = cfg.getDouble("sup.a_brake");
    s.t_react = cfg.getDouble("sup.t_react");
    s.brake_margin = cfg.getDouble("sup.brake_margin");
    axisCaps(cfg, b.calib, s);
    s.overrun_streak = static_cast<int>(cfg.getInt("sup.overrun_streak"));
    s.max_run_time = cfg.getDouble("sup.max_run_time");
    s.v_stopped = cfg.getDouble("sup.v_stopped");
    s.stop_timeout = cfg.getDouble("sup.stop_timeout");
    s.hang_tol = cfg.getDouble("sup.hang_tol_deg") * kDeg;
    s.omega_hang = cfg.getDouble("sup.omega_hang");
    s.x_center = 0.0;
    s.center_tol = cfg.getDouble("sup.center_tol");
    s.engage_on_release = cfg.getBool("sup.engage_on_release");
    b.release_edges = static_cast<int>(cfg.getInt("sup.release_edges"));
    b.release_span = cfg.getDouble("sup.release_span");
    b.release_rms = cfg.getDouble("sup.release_rms");
    b.release_omega_min = cfg.getDouble("sup.release_omega_min");
    b.release_fresh = cfg.getDouble("sup.release_fresh");
    b.w0 = w0;
    b.preload = cfg.getBool("ctrl.preload");
    return b;
}

DampConfig dampFrom(const Config& cfg) {
    DampConfig d;
    d.ts = cfg.getDouble("loop.ts");
    d.calib = calibFrom(cfg);
    d.est_kind = estimatorKind(cfg.getString("ctrl.estimator"));
    d.est_fc = cfg.getDouble("ctrl.est_fc");
    d.gain = DampRuntime::gainFor(cfg.getDouble("damp.zeta"), cfg.getDouble("plant.w0"), cfg.getDouble("plant.bJ"));
    d.a_max = cfg.getDouble("damp.a_max");
    d.engage_omega_min = cfg.getDouble("damp.engage_omega_min");
    d.duration = cfg.getDouble("damp.duration");
    d.wait_max = cfg.getDouble("damp.wait_max");
    d.phi_abort = cfg.getDouble("damp.phi_abort_deg") * kDeg;
    d.v_abort = cfg.getDouble("damp.v_abort");
    d.limits.x_min = -cfg.getDouble("damp.x_limit");
    d.limits.x_max = cfg.getDouble("damp.x_limit");
    d.limits.a_brake = cfg.getDouble("damp.a_brake");
    d.limits.t_react = cfg.getDouble("sup.t_react");
    d.limits.brake_margin = cfg.getDouble("sup.brake_margin");
    d.limits.overrun_streak = static_cast<int>(cfg.getInt("sup.overrun_streak"));
    d.limits.stop_timeout = cfg.getDouble("sup.stop_timeout");
    axisCaps(cfg, d.calib, d.limits);
    if (d.gain <= 0 || d.a_max <= 0 || d.limits.a_brake <= 0 || d.limits.x_max <= 0)
        throw std::runtime_error("damp parameters must be positive (damp.zeta, a_max, a_brake, x_limit)");
    if (d.a_max > cfg.getDouble("move.max_accel") || d.limits.a_brake > cfg.getDouble("move.max_accel"))
        throw std::runtime_error("damp.a_max / damp.a_brake above the move.max_accel typo guard");
    return d;
}

namespace { SwingUpConfig swingUpFromSession(const Config& cfg, const SessionConfig& sc); }

SessionConfig sessionFrom(const Config& cfg) {
    SessionConfig s;
    s.balance = balanceFrom(cfg);
    core::PidParams& pid = s.balance.pid;
    pid.a_max = cfg.getDouble("balance.a_max");
    pid.v_max = cfg.getDouble("balance.v_max");
    const std::string why = core::validate(pid);
    if (!why.empty()) throw std::runtime_error("balance.a_max / balance.v_max: " + why);
    const double capA = cfg.getDouble("move.max_accel"), capV = cfg.getDouble("move.max_speed");
    if (pid.a_max > capA || pid.v_max > capV)
        throw std::runtime_error("balance.a_max / balance.v_max above the move.max_accel / max_speed typo guards");

    MoverConfig& m = s.mover;
    m.ts = s.balance.ts;
    m.calib = s.balance.calib;
    m.w0 = cfg.getDouble("plant.w0");
    m.a_max = cfg.getDouble("balance.move_a_max");
    m.a_brake = cfg.getDouble("balance.a_brake");
    m.kp = cfg.getDouble("move.kp");
    m.kv = cfg.getDouble("move.kv");
    m.track_tol = cfg.getDouble("move.track_tol");
    m.release_within = cfg.getDouble("move.release_within");
    m.overrun_streak = static_cast<int>(cfg.getInt("sup.overrun_streak"));

    s.swing = swingCalibFrom(cfg);
    s.trim = cfg.getDouble("encoder.trim");
    s.calib_timeout = cfg.getDouble("calib.timeout");
    s.v_seek = cfg.getDouble("balance.v_seek");
    s.v_travel = cfg.getDouble("balance.v_travel");
    s.seek_max = cfg.getDouble("balance.seek_max");
    s.span_usteps = cfg.getDouble("balance.span_usteps");
    s.span_tol = cfg.getDouble("balance.span_tol");
    s.approach = cfg.getDouble("balance.approach");
    s.touch_tol = cfg.getDouble("balance.touch_tol");
    if (!(s.v_seek > 0 && s.v_travel > 0 && s.v_seek <= capV && s.v_travel <= capV))
        throw std::runtime_error("balance.v_seek / v_travel must be in (0, move.max_speed]");
    if (!(m.a_max > 0 && m.a_brake > 0 && m.a_max <= capA && m.a_brake <= capA))
        throw std::runtime_error("balance.move_a_max / a_brake must be in (0, move.max_accel]");
    if (!(s.span_usteps > 0 && s.span_tol > 0 && s.span_tol < 0.3 && s.approach > 0 && s.touch_tol > 0 && s.seek_max > 0))
        throw std::runtime_error("balance.span_usteps / span_tol / approach / touch_tol / seek_max out of range");
    // The soft limits (about the centre between the switches) plus the braking distance from v_max must stay
    // inside the switches with room for touch_tol of step loss.
    const core::SupervisorParams& sp = s.balance.sup;
    const double half = 0.5 * s.span_usteps / s.balance.calib.usteps_per_m;
    const double reach = std::max(sp.x_max, -sp.x_min) + core::stoppingDistance(sp, pid.v_max) + s.touch_tol;
    if (!(reach < half))
        throw std::runtime_error("sup.x_min/x_max + braking from balance.v_max + touch_tol reach beyond the switches");
    s.swingup_mode = cfg.getBool("balance.swingup");
    s.swingup = swingUpFromSession(cfg, s);
    s.remote_mode = cfg.getBool("balance.remote");
    s.balance.remote_delay = static_cast<int>(cfg.getInt("remote.delay_ticks"));
    s.balance.remote_max_miss = static_cast<int>(cfg.getInt("remote.max_miss"));
    if (s.remote_mode) {
        if (!(s.balance.remote_delay >= 1 && s.balance.remote_delay <= 200))
            throw std::runtime_error("remote.delay_ticks must be in [1, 200] (the reply can't arrive in its own tick)");
        if (!(s.balance.remote_max_miss >= 1 && s.balance.remote_max_miss <= 50))
            throw std::runtime_error("remote.max_miss must be in [1, 50]");
        if (s.balance.delay_samples != 0)
            throw std::runtime_error("balance.remote: ctrl.delay_samples must be 0 (remote.delay_ticks is the delay)");
    }
    return s;
}

namespace {

SwingUpConfig swingUpFromSession(const Config& cfg, const SessionConfig& sc) {
    SwingUpConfig u;
    u.ts = sc.balance.ts;
    u.calib = sc.balance.calib;
    u.est_kind = sc.balance.est_kind;
    u.est_fc = sc.balance.est_fc;
    u.w0 = cfg.getDouble("plant.w0");
    u.pid = sc.balance.pid;
    u.limits = sc.balance.sup;
    u.a_pump = cfg.getDouble("swingup.a_pump");
    u.a_damp = cfg.getDouble("swingup.a_damp");
    u.k_e = cfg.getDouble("swingup.k_e");
    u.e_ref = cfg.getDouble("swingup.e_ref");
    u.e_abort = cfg.getDouble("swingup.e_abort");
    u.kx = cfg.getDouble("swingup.kx");
    u.kv = cfg.getDouble("swingup.kv");
    u.kick_time = cfg.getDouble("swingup.kick_time");
    u.theta_sw = cfg.getDouble("swingup.theta_sw_deg") * kDeg;
    u.u_sw = cfg.getDouble("swingup.u_sw_deg") * kDeg;
    u.pump_max = cfg.getDouble("swingup.pump_max");
    u.balance_max = cfg.getDouble("swingup.balance_max");
    u.damp_max = cfg.getDouble("swingup.damp_max");
    u.quiet_deg = cfg.getDouble("swingup.quiet_deg");
    u.preload = cfg.getBool("swingup.preload");
    if (!(u.a_pump > 0 && u.a_damp > 0 && u.a_pump <= u.pid.a_max && u.a_damp <= u.pid.a_max))
        throw std::runtime_error("swingup.a_pump / a_damp must be in (0, balance.a_max]");
    if (!(u.k_e > 0 && u.e_abort > u.e_ref && u.e_ref > -0.05 && u.kx >= 0 && u.kv >= 0 && u.kick_time > 0))
        throw std::runtime_error("swingup.k_e / e_ref / e_abort / kx / kv / kick_time out of range");
    if (!(u.theta_sw > 0 && u.theta_sw < u.limits.theta_abort && u.u_sw > 0 && u.quiet_deg > 0))
        throw std::runtime_error("swingup.theta_sw_deg / u_sw_deg / quiet_deg out of range (theta_sw < theta_abort)");
    return u;
}

}

SwingUpConfig swingUpFrom(const Config& cfg) { return sessionFrom(cfg).swingup; }

}
