#include "runtime/params.h"

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
    b.release_quiet = cfg.getDouble("sup.release_quiet");
    b.release_fresh = cfg.getDouble("sup.release_fresh");
    b.w0 = cfg.getBool("ctrl.preload") ? w0 : 0.0;
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

}
