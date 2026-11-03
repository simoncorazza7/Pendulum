// Balance session (pendctl --mode balance) and its cart moves on the simulated rig: identified plant,
// switches 6104 usteps apart at 1/8 step (0.458 m, as measured in 0.8c), hard stops 15 mm beyond.
// The operator is scripted: stops the calibration swing by hand, presses Enter, lifts and lets go.

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

#include "check.h"
#include "common/config.h"
#include "hw/sim_hw.h"
#include "runtime/mover.h"
#include "runtime/params.h"
#include "runtime/session.h"

namespace {

const double kPi = 3.141592653589793;
const double kTs = 0.002;
const double kSpanM = 6104 / 13333.0;     // m between the switches

// upmTruth: the step mode the driver really runs (13333 = 1/8 step as configured, 3333 = half step).
hw::SimHwParams rigSim(double upmTruth = 13333.0) {
    hw::SimHwParams p;
    p.w0 = 4.97; p.bJ = 0.054; p.coulomb = 0.168;    // pivot of 2026-11-02 (config plant.*)
    p.axis_a = 400000.0 / (2 * 13333.0); p.axis_v = 120000.0 / (2 * 13333.0);
    p.truth.usteps_per_m = upmTruth;
    p.pos_switch_right = 1000;
    p.pos_switch_left = 1000 + kSpanM * upmTruth;
    p.pos_end_right = 1000 - 0.015 * upmTruth;
    p.pos_end_left = p.pos_switch_left + 0.015 * upmTruth;
    p.truth.pos_center = 1000 + 0.5 * kSpanM * upmTruth;    // truth x = 0 midway between the switches
    return p;
}

rt::SessionConfig rigSession() {
    rt::SessionConfig c = rt::sessionFrom(Config::fromFile("config/pendctl.conf"));
    return c;
}

struct Run {
    hw::SimHw sim;
    rt::Session ses;
    std::vector<rt::SessionRecord> recs;
    Run(const hw::SimHwParams& p, const rt::SessionConfig& c, double theta0, double x0)
        : sim(p, theta0, 0.0, x0), ses(c) { recs.reserve(100000); }
    // Ticks until done(k) or n ticks; script acts before each tick.
    void until(int n, const std::function<bool(int)>& done, const std::function<void(int)>& script = [](int) {}) {
        for (int k = 0; k < n && !done(k); ++k) {
            script(k);
            recs.push_back(ses.tick(sim, false));
            sim.advance(kTs);
        }
    }
    bool sawEvent(core::Reason e) const {
        for (const auto& r : recs) if (r.event == static_cast<int>(e)) return true;
        return false;
    }
};

// Operator hand: move the pendulum to hanging and hold it there for `hold` s, then let go.
void stopSwing(Run& r, double hold = 0.8) {
    const double hang = std::round(r.sim.theta() / (2 * kPi) - 0.5) * 2 * kPi + kPi;   // nearest hanging angle
    r.sim.holdAt(hang, 0.6);
    for (int k = 0; k < static_cast<int>(hold / kTs); ++k) { r.recs.push_back(r.ses.tick(r.sim, false)); r.sim.advance(kTs); }
    r.sim.release();
}

double phiDeg(double theta) { return std::fabs(core::wrap(theta - kPi)) * 180 / kPi; }

// Calibrate (18 deg free swing: calib.swing_sigma matches 12-18 deg), stop the swing, Enter, home.
void calibrateAndHome(Run& r) {
    r.until(20000, [&](int) { return r.ses.phase() != rt::SessionPhase::Calibrate; });
    CHECK(r.ses.phase() == rt::SessionPhase::WaitGo);
    stopSwing(r);
    r.ses.requestGo();
    r.until(30000, [&](int) {
        return r.ses.phase() != rt::SessionPhase::WaitGo && r.ses.phase() != rt::SessionPhase::Home;
    });
}

// From BALANCE: wait for ARMED, lift to 2 deg, let go at ENGAGED; then the script decides.
void liftAndRelease(Run& r) {
    r.until(5000, [&](int) { return r.ses.mode() == core::Mode::Armed; });
    CHECK(r.ses.mode() == core::Mode::Armed);
    r.sim.holdAt(0.035, 0.6);
    r.until(2000, [&](int) {
        if (r.sim.held() && r.ses.mode() == core::Mode::Engaged) r.sim.release();
        return r.ses.mode() == core::Mode::Engaged && !r.sim.held();
    });
    CHECK(r.ses.mode() == core::Mode::Engaged);
}

}

TEST(mover_shaped_profile_accelerates_for_one_period) {
    const double T0 = 2 * kPi / 4.931;
    double vp = 0;
    const double a1 = rt::Mover::shapedAccel(0.3, 0.1, 4.931, 0.3, &vp);     // long: trapezoid
    CHECK_NEAR(a1, 0.1 / T0, 1e-12);
    CHECK_NEAR(vp, 0.1, 1e-12);
    const double a2 = rt::Mover::shapedAccel(-0.05, 0.1, 4.931, 0.3, &vp);   // short: triangle
    CHECK_NEAR(a2, 0.05 / (T0 * T0), 1e-12);
    CHECK_NEAR(vp, 0.05 / T0, 1e-12);
    core::TrapezoidProfile prof(0.05, vp, a2);
    CHECK_NEAR(prof.duration(), 2 * T0, 1e-9);
}

// The claim behind the shaping: a hanging pendulum ends a shaped move at rest; the same move with
// a plain 0.3 m/s^2 trapezoid leaves it swinging.
TEST(mover_shaped_move_leaves_the_pendulum_still) {
    auto residual = [](double aMax, bool shaped) {
        hw::SimHw sim(rigSim(), kPi, 0.0, 0.0);
        rt::SessionConfig sc = rigSession();
        rt::MoverConfig mc = sc.mover;
        mc.a_max = aMax;
        if (!shaped) mc.w0 = 1e3;                 // T0 ~ 6 ms: the cap a_max decides, i.e. no shaping
        rt::Mover m(mc);
        sim.enableDriver(true);
        m.start(sim.read().pos_usteps + 2667, 0.10, rt::Side::None, sim.read());   // 0.2 m
        for (int k = 0; k < 10000 && m.state() != rt::MoverState::Done; ++k) {
            m.tick(sim, sim.read(), false, false);
            sim.advance(kTs);
        }
        CHECK(m.state() == rt::MoverState::Done);
        CHECK_NEAR(sim.x(), 0.2, 0.002);
        double amp = 0;                           // swing amplitude over the next 2 periods
        for (int k = 0; k < 1300; ++k) {
            sim.read();
            sim.advance(kTs);
            amp = std::max(amp, phiDeg(sim.theta()));
        }
        return amp;
    };
    const double shapedAmp = residual(0.3, true), plainAmp = residual(0.3, false);
    std::cout << "    0.2 m at 0.1 m/s: residual swing " << shapedAmp << " deg shaped, " << plainAmp
              << " deg with a plain 0.3 m/s^2 trapezoid\n";
    CHECK(shapedAmp < 0.3);
    CHECK(plainAmp > 1.0);
}

TEST(session_full_flow_two_catches_recentre_quit) {
    hw::SimHwParams p = rigSim();
    rt::SessionConfig c = rigSession();
    c.trim = 1.0;                                  // 1 count reference error: drift into the soft limit
    Run r(p, c, kPi - 0.31, 0.05);                 // 18 deg swing, cart 50 mm left of the middle
    calibrateAndHome(r);
    CHECK(r.ses.phase() == rt::SessionPhase::Balance);
    CHECK(std::llabs(r.ses.posRight() - 1000) <= 2);
    CHECK(std::llabs(r.ses.posLeft() - (1000 + 6104)) <= 2);
    CHECK(std::fabs(r.sim.x()) < 0.002);           // physically centred
    double maxPhiHome = 0;
    for (const auto& x : r.recs)
        if (x.phase == static_cast<int>(rt::SessionPhase::Home)) maxPhiHome = std::max(maxPhiHome, phiDeg(x.theta));
    std::cout << "    homing " << std::count_if(r.recs.begin(), r.recs.end(), [](const rt::SessionRecord& x) {
        return x.phase == static_cast<int>(rt::SessionPhase::Home); }) * kTs << " s, max swing " << maxPhiHome << " deg\n";
    CHECK(maxPhiHome < 3.0);

    // Attempt 1: arms by itself, catch, drifts into the soft limit, controlled stop.
    liftAndRelease(r);
    r.until(10000, [&](int) { return r.ses.phase() == rt::SessionPhase::AfterRun; });
    CHECK(r.ses.phase() == rt::SessionPhase::AfterRun);
    const rt::AttemptStats a1 = r.ses.lastAttempt();
    CHECK_EQ(a1.n, 1);
    CHECK(a1.engaged);
    CHECK_EQ(a1.end_reason, static_cast<int>(core::Reason::SoftLimit));
    CHECK(a1.t_engaged > 2.0);
    CHECK(a1.rmsTheta() < 0.5 * kPi / 180);
    std::cout << "    attempt 1: " << a1.t_engaged << " s balanced, RMS theta " << a1.rmsTheta() * 180 / kPi
              << " deg, x_end " << a1.x_end << " m\n";
    // Held after the stop: the cart must not creep (the Axis keeps its last command).
    const double xHeld = r.sim.x();
    r.until(1000, [](int) { return false; });
    CHECK(std::fabs(r.sim.x() - xHeld) < 0.0005);
    CHECK(r.recs.back().driver_enabled);

    // Pendulum caught and lowered, Enter: touch the nearer switch, re-centre, re-arm.
    stopSwing(r);
    r.ses.requestGo();
    r.until(20000, [&](int) { return r.ses.phase() == rt::SessionPhase::Balance && r.ses.touches() == 1; });
    CHECK(r.ses.phase() == rt::SessionPhase::Balance);
    CHECK(r.ses.lastSwitch() == (a1.x_end > 0 ? rt::Side::Left : rt::Side::Right));
    CHECK(std::llabs(r.ses.lastTouchError()) <= 2);
    r.until(3000, [&](int) { return r.ses.mode() == core::Mode::Armed; });
    CHECK(std::fabs(r.sim.x()) < 0.002);

    // Attempt 2: the operator stops it after 1 s (Enter while ENGAGED), then quits.
    liftAndRelease(r);
    r.until(500, [](int) { return false; });
    r.ses.requestGo();
    r.until(2000, [&](int) { return r.ses.phase() == rt::SessionPhase::AfterRun; });
    CHECK_EQ(r.ses.lastAttempt().n, 2);
    CHECK_EQ(r.ses.lastAttempt().end_reason, static_cast<int>(core::Reason::ManualStop));
    r.ses.requestQuit();
    r.until(2000, [&](int) { return r.ses.finished(); });
    CHECK(r.ses.phase() == rt::SessionPhase::Finished);
    CHECK(!r.recs.back().driver_enabled);
    CHECK(!r.sim.crashed());
    CHECK(!r.sawEvent(core::Reason::LimitSwitch));
    CHECK_EQ(r.ses.attempts(), 2);
}

TEST(session_wrong_step_mode_fails_the_span_check) {
    hw::SimHwParams p = rigSim(3333.3);            // the driver runs half step, the config says 1/8
    Run r(p, rigSession(), kPi - 0.31, 0.0);
    calibrateAndHome(r);
    CHECK(r.ses.phase() == rt::SessionPhase::Failed);
    CHECK(std::string(r.ses.reason()).find("track span") != std::string::npos);
    CHECK(!r.recs.back().driver_enabled);
    CHECK(!r.sim.crashed());
    // physically past the left switch by (seek speed x 4)^2 / (2 a_brake x 4): ~3 mm
    const double past = (r.sim.x() - 0.5 * kSpanM) * 1e3;
    std::cout << "    " << r.ses.reason() << "\n    stopped " << past << " mm past the left switch\n";
    CHECK(past < 5.0);
}

TEST(session_step_loss_is_measured_and_stops_beyond_tolerance) {
    for (double lost : {100.0, 300.0}) {           // 7.5 mm: re-zeroed; 22 mm: beyond touch_tol 15 mm
        Run r(rigSim(), rigSession(), kPi - 0.31, 0.0);
        calibrateAndHome(r);
        liftAndRelease(r);
        r.until(500, [](int) { return false; });
        r.ses.requestGo();                         // stop the run
        r.until(2000, [&](int) { return r.ses.phase() == rt::SessionPhase::AfterRun; });
        stopSwing(r);
        r.sim.loseSteps(lost);
        r.ses.requestGo();
        r.until(20000, [&](int) {
            return r.ses.finished() || (r.ses.phase() == rt::SessionPhase::Balance && r.ses.touches() == 1);
        });
        if (lost < 0.015 * 13333) {
            CHECK(std::fabs(static_cast<double>(r.ses.lastTouchError()) - lost) <= 2);
            CHECK(r.ses.phase() == rt::SessionPhase::Balance);
            r.until(3000, [&](int) { return r.ses.mode() == core::Mode::Armed; });
            CHECK(std::fabs(r.sim.x()) < 0.002);   // re-zeroed: physically centred again
        } else {
            CHECK(r.ses.phase() == rt::SessionPhase::Failed);
            CHECK(std::string(r.ses.reason()).find("step loss") != std::string::npos);   // found too far or not at all
            std::cout << "    " << lost << " usteps lost: " << r.ses.reason() << "\n";
            CHECK(!r.recs.back().driver_enabled);
        }
        CHECK(!r.sim.crashed());
    }
}

TEST(session_quit_during_homing_stops) {
    Run r(rigSim(), rigSession(), kPi - 0.31, 0.0);
    r.until(20000, [&](int) { return r.ses.phase() != rt::SessionPhase::Calibrate; });
    stopSwing(r);
    r.ses.requestGo();
    r.until(1500, [](int) { return false; });      // 3 s into the first seek
    CHECK(r.ses.phase() == rt::SessionPhase::Home);
    r.ses.requestQuit();
    r.until(2000, [&](int) { return r.ses.finished(); });
    CHECK(r.ses.phase() == rt::SessionPhase::Finished);
    CHECK(r.sim.v() == 0.0);
    CHECK(!r.recs.back().driver_enabled);
}

TEST(session_calibration_timeout_moves_nothing) {
    rt::SessionConfig c = rigSession();
    c.calib_timeout = 2.0;
    Run r(rigSim(), c, kPi, 0.0);                  // hanging still: no swing to fit
    r.until(2000, [&](int) { return r.ses.finished(); });
    CHECK(r.ses.phase() == rt::SessionPhase::Failed);
    CHECK(std::string(r.ses.reason()).find("calibration") != std::string::npos);
    for (const auto& x : r.recs) CHECK(!x.driver_enabled && !x.sent);
}

TEST(session_params_from_config) {
    const rt::SessionConfig c = rigSession();
    CHECK_NEAR(c.balance.pid.a_max, 2.0, 1e-12);   // balance.* replaces ctrl.a_max / v_max
    CHECK_NEAR(c.balance.pid.v_max, 0.3, 1e-12);
    CHECK_NEAR(c.span_usteps, 6104, 1e-9);
    CHECK_NEAR(c.mover.w0, 4.97, 1e-12);
    Config bad = Config::fromFile("config/pendctl.conf");
    bad.setFromAssignment("sup.x_max=0.22");      // soft limit + braking beyond the switches
    CHECK_THROWS(rt::sessionFrom(bad));
}
