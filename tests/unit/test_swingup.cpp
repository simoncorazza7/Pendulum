// Energy-based swing-up (rt::SwingUpRuntime) on the simulated rig: identified plant with Coulomb friction,
// switches 6104 usteps apart (0.458 m), hard stops 15 mm beyond, rig config (balance gains, est_fc, limits).

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <string>

#include "check.h"
#include "common/config.h"
#include "hw/sim_hw.h"
#include "runtime/params.h"
#include "runtime/session.h"
#include "runtime/swingup.h"

namespace {

const double kPi = 3.141592653589793;
const double kTs = 0.002;

hw::SimHwParams swingSim() {
    hw::SimHwParams p;
    const double span = 6104 / 13333.0;
    p.w0 = 4.931; p.bJ = 0.024; p.coulomb = 0.0117;
    p.axis_a = 400000.0 / (2 * 13333.0); p.axis_v = 120000.0 / (2 * 13333.0);
    p.pos_switch_right = 1000;
    p.pos_switch_left = 1000 + span * 13333.0;
    p.pos_end_right = 1000 - 0.015 * 13333.0;
    p.pos_end_left = p.pos_switch_left + 0.015 * 13333.0;
    p.truth.pos_center = 1000 + 0.5 * span * 13333.0;
    return p;
}

struct SwingRun {
    hw::SimHw sim;
    rt::SwingUpRuntime rt;
    double max_x = 0, peak_e = 0;
    bool limit_hit = false;
    int ticks = 0;
    SwingRun(const rt::SwingUpConfig& c, const hw::SimHwParams& p) : sim(p, kPi, 0.0, 0.0), rt(c) {
        sim.enableDriver(true);
    }
    void until(int n, const std::function<bool(int)>& done, const std::function<void(int)>& script = [](int) {}) {
        for (int k = 0; k < n && !done(k); ++k, ++ticks) {
            script(ticks);
            const hw::Sample s = sim.read();
            limit_hit = limit_hit || s.limit_left || s.limit_right;
            rt.tick(sim, s, false);
            sim.advance(kTs);
            max_x = std::max(max_x, std::fabs(sim.x()));
        }
    }
};

rt::SwingUpConfig rigSwing(const hw::SimHwParams& p) {
    rt::SwingUpConfig c = rt::swingUpFrom(Config::fromFile("config/pendctl.conf"));
    c.calib.c_bottom = 12.37 - std::floor(12.37 + 2400.0);   // the hanging start: SimHw counts begin at 0
    c.calib.pos_center = p.truth.pos_center;
    c.e_ref = 0.003;            // SimHw loses less energy on the last approach than the rig (config is rig-tuned)
    return c;
}

double swingDeg(const hw::SimHw& sim) {   // amplitude estimate from energy: angle from hanging at the turning point
    const double w2 = 4.931 * 4.931;
    const double E = 0.5 * sim.thetadot() * sim.thetadot() + w2 * (std::cos(sim.theta()) - 1);
    const double c = std::clamp(-E / w2 - 1, -1.0, 1.0);   // cos A at the turning point
    return std::acos(c) * 180 / kPi;
}

}

TEST(swingup_pumps_hands_over_balances_and_brings_it_down) {
    const hw::SimHwParams p = swingSim();
    SwingRun r(rigSwing(p), p);
    r.until(60000, [&](int) { return r.rt.done(); });
    CHECK(r.rt.done());
    CHECK(r.rt.handedOver());
    CHECK(r.rt.handoverTime() < 30.0);
    CHECK(r.rt.balanceTime() > 1.0);
    CHECK(!r.limit_hit);
    CHECK(!r.sim.crashed());
    CHECK(r.max_x < 0.15);
    CHECK(swingDeg(r.sim) < 12.0);              // brought down, not left swinging near upright energy
    CHECK(std::fabs(r.sim.v()) < 1e-3);
    std::cout << "    hand-over after " << r.rt.handoverTime() << " s, balanced " << r.rt.balanceTime() << " s ("
              << r.rt.reason() << "), max |x| " << r.max_x * 1e3 << " mm, peak |v| " << r.rt.peakV()
              << " m/s, peak energy " << r.rt.peakEnergy() << ", done after " << r.ticks * kTs << " s, final swing "
              << swingDeg(r.sim) << " deg\n";
}

TEST(swingup_too_much_energy_is_removed) {
    const hw::SimHwParams p = swingSim();
    rt::SwingUpConfig c = rigSwing(p);
    c.e_ref = 0.3;                               // aims 30 % above upright: must trip e_abort (10 %)
    c.theta_sw = 0.0;                            // and never hand over
    SwingRun r(c, p);
    r.until(60000, [&](int) { return r.rt.done(); });
    CHECK(r.rt.done());
    CHECK(std::string(r.rt.reason()) == "energy above e_abort");
    CHECK(r.rt.peakEnergy() < 1.2);
    CHECK(swingDeg(r.sim) < 15.0);
    CHECK(!r.limit_hit);
    CHECK(!r.sim.crashed());
    std::cout << "    peak energy " << r.rt.peakEnergy() << " (1 = upright), done after " << r.ticks * kTs
              << " s, final swing " << swingDeg(r.sim) << " deg, max |x| " << r.max_x * 1e3 << " mm\n";
}

TEST(swingup_operator_go_removes_energy_and_quit_stops_hard) {
    const hw::SimHwParams p = swingSim();
    {
        SwingRun r(rigSwing(p), p);
        r.until(4000, [](int) { return false; });            // 8 s of pumping
        CHECK(r.rt.phase() == rt::SwingPhase::Pump);
        const double e = swingDeg(r.sim);
        r.rt.requestDampOut();
        r.until(30000, [&](int) { return r.rt.done(); });
        CHECK(r.rt.done());
        CHECK(std::string(r.rt.reason()) == "operator stop");
        CHECK(swingDeg(r.sim) < 15.0);
        std::cout << "    go at a " << e << " deg swing: brought down in " << r.ticks * kTs - 8.0 << " s\n";
    }
    {
        SwingRun r(rigSwing(p), p);
        r.until(4000, [](int) { return false; });
        r.rt.requestStop();
        r.until(1000, [&](int) { return r.rt.done(); });
        CHECK(r.rt.done());
        CHECK(std::string(r.rt.reason()) == "stop request");
        CHECK(std::fabs(r.sim.v()) < 1e-3);                  // hard stop: the cart stands, the pendulum may swing
    }
}

TEST(swingup_params_from_config) {
    const rt::SwingUpConfig c = rt::swingUpFrom(Config::fromFile("config/pendctl.conf"));
    CHECK_NEAR(c.a_pump, 0.75, 1e-12);
    CHECK_NEAR(c.pid.a_max, 2.0, 1e-12);                      // the balance session's reduced limits
    CHECK_NEAR(c.theta_sw * 180 / kPi, 10.0, 1e-9);
    CHECK(c.e_abort > c.e_ref);
}

// The whole swing-up session: calibrate, home, READY, the operator's go, swing-up + hand-over + energy out,
// AFTER_RUN, quit. Same operator script as the hand-catch session tests.
TEST(session_swingup_mode_home_ready_swing_quit) {
    const hw::SimHwParams p = swingSim();
    Config cfg = Config::fromFile("config/pendctl.conf");
    cfg.setFromAssignment("balance.swingup=true");
    cfg.setFromAssignment("swingup.e_ref=0.003");                 // SimHw energy losses (see rigSwing)
    rt::Session ses(rt::sessionFrom(cfg));
    hw::SimHw sim(p, kPi - 0.14, 0.0, 0.03);
    bool ready = false, limit = false;
    double max_x = 0;
    auto tick = [&] {
        const rt::SessionRecord r = ses.tick(sim, false);
        sim.advance(kTs);
        ready = ready || r.sevent == static_cast<int>(rt::SessionEvent::SwingReady);
        if (ses.phase() == rt::SessionPhase::Balance) {          // homing touches the switches by design
            limit = limit || r.limit_left || r.limit_right;
            max_x = std::max(max_x, std::fabs(sim.x()));
        }
    };
    for (int k = 0; k < 20000 && ses.phase() == rt::SessionPhase::Calibrate; ++k) tick();
    CHECK(ses.phase() == rt::SessionPhase::WaitGo);
    sim.holdAt(std::round(sim.theta() / (2 * kPi) - 0.5) * 2 * kPi + kPi, 0.6);   // stop the calibration swing
    for (int k = 0; k < 400; ++k) tick();
    sim.release();
    ses.requestGo();                                                               // home
    for (int k = 0; k < 40000 && !(ses.phase() == rt::SessionPhase::Balance && ses.swingReady()); ++k) tick();
    CHECK(ready);
    CHECK(ses.mode() == core::Mode::Idle);                                         // no hand-catch runtime here
    ses.requestGo();                                                               // start the swing-up
    tick();
    CHECK(ses.swingPhase() == static_cast<int>(rt::SwingPhase::Kick));
    for (int k = 0; k < 60000 && ses.phase() != rt::SessionPhase::AfterRun; ++k) tick();
    CHECK(ses.phase() == rt::SessionPhase::AfterRun);
    const rt::AttemptStats& a = ses.lastAttempt();
    CHECK(a.swingup);
    CHECK(a.engaged);
    CHECK(a.t_engaged > 1.0);
    CHECK(swingDeg(sim) < 15.0);
    std::cout << "    session swing-up: hand-over after " << a.t_handover << " s, balanced " << a.t_engaged << " s ("
              << a.end_text << "), peak energy " << a.peak_energy << ", max |x| " << max_x * 1e3 << " mm\n";
    ses.requestQuit();
    for (int k = 0; k < 2000 && !ses.finished(); ++k) tick();
    CHECK(ses.phase() == rt::SessionPhase::Finished);
    CHECK(!limit);
    CHECK(!sim.crashed());
    CHECK(max_x < 0.15);
}
