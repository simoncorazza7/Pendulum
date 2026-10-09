// config/pendctl.conf -> runtime parameters (the file the rig runs with; tests run from the repo root).

#include <cmath>

#include "check.h"
#include "runtime/params.h"

TEST(params_load_from_the_rig_config) {
    const Config cfg = Config::fromFile("config/pendctl.conf");
    const double g = 9.81, w0 = cfg.getDouble("plant.w0");

    const core::Calib c = rt::calibFrom(cfg);
    CHECK_EQ(c.cpr, 2400);
    CHECK_EQ(c.theta_sign, 1);
    CHECK_EQ(c.x_sign, 1);

    const rt::SwingCalibParams s = rt::swingCalibFrom(cfg);
    CHECK_NEAR(s.amp_min, 2 * 2400.0 / 360, 1e-9);
    CHECK_NEAR(s.amp_max, 9 * 2400.0 / 360, 1e-9);

    // wc = 2 w0, zeta 0.7, p = 0.5 w0: Kp = g (1 + (4 + 1.4)) = 6.4 g
    const rt::BalanceConfig b = rt::balanceFrom(cfg);
    CHECK_NEAR(b.pid.kp, 6.4 * g, 1e-9);
    CHECK(b.pid.kp > g);
    CHECK_NEAR(b.pid.ki, g * 4 * w0 * w0 * 0.5 * w0 / (w0 * w0), 1e-9);
    CHECK_NEAR(b.sup.theta_abort, 22 * 3.141592653589793 / 180, 1e-12);
    CHECK(b.sup.axis_a > 50);                                 // Axis cap far above the controller's range

    const rt::DampConfig d = rt::dampFrom(cfg);
    CHECK_NEAR(d.gain, (2 * 0.2 * w0 - cfg.getDouble("plant.bJ")) * g / (w0 * w0), 1e-12);
    CHECK(d.gain > 0.7 && d.gain < 0.9);                      // ~0.79 (m/s^2)/(rad/s)
    CHECK_NEAR(d.limits.x_max, 0.10, 1e-12);
}

TEST(params_reject_a_bad_sign_or_estimator) {
    Config cfg = Config::fromFile("config/pendctl.conf");
    cfg.set("calib.theta_sign", "0");
    CHECK_THROWS(rt::calibFrom(cfg));
    cfg.set("calib.theta_sign", "1");
    cfg.set("ctrl.estimator", "magic");
    CHECK_THROWS(rt::balanceFrom(cfg));
    cfg.set("ctrl.estimator", "bdlp1");
    cfg.set("damp.a_max", "5");                               // above the move.max_accel typo guard
    CHECK_THROWS(rt::dampFrom(cfg));
}
