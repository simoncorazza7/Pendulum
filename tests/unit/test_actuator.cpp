// Stage 0.9 actuator test (pendctl --mode actuator) on the simulated rig: identified plant (config plant.*),
// switches 6104 usteps apart, hard stops 15 mm beyond, Axis caps from the config. The operator is scripted
// (Enter, s, q); lost steps are injected with SimHw::loseSteps. docs/stage0_9_actuator.md.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <functional>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "check.h"
#include "common/config.h"
#include "hw/sim_hw.h"
#include "runtime/actuator.h"
#include "runtime/params.h"

namespace {

const double kPi = 3.141592653589793;
const double kTs = 0.002;
const double kSpanM = 6104 / 13333.0;     // m between the switches

hw::SimHwParams rigSim() {
    hw::SimHwParams p;
    p.w0 = 4.97; p.bJ = 0.054; p.coulomb = 0.168;    // pivot of 2026-11-02 (config plant.*)
    p.axis_a = 400000.0 / (2 * 13333.0); p.axis_v = 120000.0 / (2 * 13333.0);
    p.truth.usteps_per_m = 13333.0;
    p.pos_switch_right = 1000;
    p.pos_switch_left = 1000 + kSpanM * 13333.0;
    p.pos_end_right = 1000 - 0.015 * 13333.0;
    p.pos_end_left = p.pos_switch_left + 0.015 * 13333.0;
    p.truth.pos_center = 1000 + 0.5 * kSpanM * 13333.0;   // truth x = 0 midway between the switches
    return p;
}

rt::ActuatorConfig actCfg(const std::vector<std::pair<std::string, std::string>>& sets = {}) {
    Config cfg = Config::fromFile("config/pendctl.conf");
    for (const auto& kv : sets) cfg.set(kv.first, kv.second);
    return rt::actuatorFrom(cfg);
}

struct Bench {
    hw::SimHw sim;
    rt::ActuatorTest act;
    std::vector<rt::ActRecord> recs;
    explicit Bench(const rt::ActuatorConfig& c) : sim(rigSim(), kPi, 0.0, 0.0), act(c) { recs.reserve(400000); }
    // Ticks until done(k) (checked before each tick) or n ticks; script acts before each tick.
    void until(int n, const std::function<bool(int)>& done, const std::function<void(int)>& script = [](int) {}) {
        for (int k = 0; k < n && !done(k); ++k) {
            script(k);
            recs.push_back(act.tick(sim, false));
            sim.advance(kTs);
        }
    }
    void home() {
        act.requestGo();
        until(30000, [&](int k) { return k > 0 && (act.phase() == rt::ActPhase::Ready || act.finished()); });
    }
    // Enter at READY, then until READY again (or the end).
    void rung(const std::function<void(int)>& script = [](int) {}) {
        act.requestGo();
        until(300000, [&](int k) { return k > 0 && (act.phase() == rt::ActPhase::Ready || act.finished()); }, script);
    }
    bool sawEvent(rt::ActEvent e) const {
        for (const auto& r : recs) if (r.event == static_cast<int>(e)) return true;
        return false;
    }
};

}

TEST(actuator_params_from_config) {
    const rt::ActuatorConfig c = actCfg();
    CHECK(c.profile == rt::ActProfile::Pair);
    CHECK_EQ(c.speeds.size(), static_cast<std::size_t>(1));
    CHECK_NEAR(c.speeds[0], 0.3, 1e-12);
    CHECK_EQ(c.accels.size(), static_cast<std::size_t>(7));
    CHECK_NEAR(c.accels[6], 10.0, 1e-12);
    CHECK_EQ(c.reps, 2);
    CHECK_NEAR(c.touch_pass, 3, 1e-12);
    CHECK_NEAR(c.touch_fail, 8, 1e-12);
    CHECK_NEAR(c.mover.a_brake, 1.0, 1e-12);                  // balance.a_brake
    CHECK_NEAR(c.hold_a, 1.0, 1e-12);
    const std::vector<rt::ActRung> g = rt::buildActRungs(c);
    CHECK_EQ(g.size(), static_cast<std::size_t>(7));
    for (const rt::ActRung& r : g) CHECK(r.runnable);
    CHECK_NEAR(g[0].leg, 0.3 * 0.3 / 2 + 0.3 * 0.10, 1e-12);  // v^2/a + v cruise
    CHECK_NEAR(g[0].peak_v, 0.3, 1e-9);
    CHECK_NEAR(g[0].peak_a, 2.0, 1e-9);
    CHECK_NEAR(g[0].duration, 2 * kPi / 4.97 + (2 * 0.3 / 2 + 0.10), 1e-9);   // return at T0 + one leg

    CHECK_THROWS(actCfg({{"actuator.speeds", "0.9"}}));      // above v_top
    CHECK_THROWS(actCfg({{"actuator.accels", "2,1"}}));      // not ascending
    CHECK_THROWS(actCfg({{"actuator.accels", "12"}}));       // above a_top
    CHECK_THROWS(actCfg({{"actuator.speeds", "abc"}}));
    CHECK_THROWS(actCfg({{"actuator.speeds", ""}}));
    CHECK_THROWS(actCfg({{"actuator.profile", "zigzag"}}));
    CHECK_THROWS(actCfg({{"axis.max_accel", "100000"}}));    // 3.75 m/s^2: the Axis cap would bite at 4..10
    CHECK_THROWS(actCfg({{"actuator.leg_max", "0.40"}}));    // a 0.2 m half leg reaches within 50 mm of a switch
    const rt::ActuatorConfig c2 = actCfg({{"actuator.speeds", "0.8"}, {"actuator.accels", "2,3"}});
    const std::vector<rt::ActRung> g2 = rt::buildActRungs(c2);
    CHECK(!g2[0].runnable);                                   // 0.40 m leg
    CHECK(g2[1].runnable);                                    // 0.29 m
}

TEST(actuator_trajectories_stay_in_bounds) {
    const double T0 = 2 * kPi / 4.97;
    auto scan = [](const rt::ActTraj& t, double* xMax, double* vMax, double* aMax) {
        *xMax = *vMax = *aMax = 0;
        for (double tt = 0; tt <= t.duration() + kTs; tt += kTs / 4) {
            const core::ProfilePoint p = t.at(tt);
            *xMax = std::max(*xMax, std::fabs(p.x));
            *vMax = std::max(*vMax, std::fabs(p.v));
            *aMax = std::max(*aMax, std::fabs(p.a));
        }
    };
    double xm, vm, am;
    {   // pair: -d/2 -> +d/2, the return leg starts at T0, back to -d/2 at rest
        const rt::ActuatorConfig c = actCfg({{"actuator.speeds", "0.6"}, {"actuator.accels", "4"}});
        const rt::ActRung r = rt::buildActRungs(c)[0];
        const auto t = rt::makeActTraj(c, r, 1);
        const double d = 0.6 * 0.6 / 4 + 0.6 * 0.10;
        scan(*t, &xm, &vm, &am);
        CHECK_NEAR(xm, d / 2, 1e-9);
        CHECK(vm <= 0.6 + 1e-9);
        CHECK(am <= 4 + 1e-9);
        CHECK_NEAR(t->at(0).x, -d / 2, 1e-12);
        CHECK_NEAR(t->at(T0 - 1e-6).x, d / 2, 1e-9);
        CHECK_NEAR(t->at(T0 - 1e-6).v, 0.0, 1e-9);
        CHECK(t->at(T0 + 0.05).v < 0);
        CHECK_NEAR(t->at(t->duration() + 0.01).x, -d / 2, 1e-9);
        CHECK_NEAR(t->at(t->duration() + 0.01).v, 0.0, 1e-12);
    }
    {   // oneway: odd reps toward +x, even reps back
        const rt::ActuatorConfig c = actCfg({{"actuator.profile", "oneway"}, {"actuator.speeds", "0.45"}, {"actuator.accels", "6"}});
        const rt::ActRung r = rt::buildActRungs(c)[0];
        const double d = 0.45 * 0.45 / 6 + 0.45 * 0.10;
        const auto t1 = rt::makeActTraj(c, r, 1), t2 = rt::makeActTraj(c, r, 2);
        CHECK_NEAR(t1->at(0).x, -d / 2, 1e-12);
        CHECK_NEAR(t1->at(t1->duration() + 0.01).x, d / 2, 1e-9);
        CHECK_NEAR(t2->at(0).x, d / 2, 1e-12);
        CHECK_NEAR(t2->at(t2->duration() + 0.01).x, -d / 2, 1e-9);
    }
    {   // bang: speed between +-v, x within +-v P / 8, ends at rest where it started
        const rt::ActuatorConfig c = actCfg({{"actuator.profile", "bang"}, {"actuator.speeds", "0.45"}, {"actuator.accels", "6"}});
        const rt::ActRung r = rt::buildActRungs(c)[0];
        const auto t = rt::makeActTraj(c, r, 1);
        scan(*t, &xm, &vm, &am);
        const double P = 4 * 0.45 / 6;
        CHECK_NEAR(xm, 0.45 * P / 8, 1e-6);
        CHECK_NEAR(vm, 0.45, 1e-6);
        CHECK_NEAR(am, 6, 1e-9);
        CHECK(t->duration() <= 10.0 + 1e-9 && t->duration() > 9.0);
        CHECK_NEAR(t->at(t->duration() + 0.01).x, t->at(0).x, 1e-9);
        CHECK_NEAR(t->at(t->duration() - 1e-9).v, 0.0, 1e-6);
    }
    {   // chirp: |v| <= v, |x| <= stream_x_max, starts and ends at rest in the centre, amplitude ~a
        const rt::ActuatorConfig c = actCfg({{"actuator.profile", "chirp"}, {"actuator.speeds", "0.45"}, {"actuator.accels", "4"}});
        const rt::ActRung r = rt::buildActRungs(c)[0];
        CHECK(r.runnable);
        CHECK(r.f0 >= 4 / (2 * kPi * 0.45) - 1e-9);
        const auto t = rt::makeActTraj(c, r, 1);
        scan(*t, &xm, &vm, &am);
        CHECK(vm <= 0.45 + 1e-9);
        CHECK(xm <= 0.10 + 1e-9);
        CHECK(am > 3.6 && am < 4.6);
        CHECK_NEAR(t->at(0).x, 0.0, 1e-12);
        CHECK_NEAR(t->at(t->duration()).v, 0.0, 1e-12);
        std::cout << "    chirp 0.45 m/s, 4 m/s^2: from " << r.f0 << " Hz, peak |x| " << xm * 1e3 << " mm, |v| " << vm
                  << " m/s, |a| " << am << " m/s^2\n";
    }
}

TEST(actuator_table_loads_and_rejects_bad_files) {
    const char* path = "build/test_actuator_table.csv";
    { std::ofstream f(path); f << "# test table\nt,x,v,a\n0,0,0,0\n0.002,0.000001,0.001,1\n0.004,0.000004,0.002,0\n"; }
    std::vector<core::ProfilePoint> p;
    std::string err;
    CHECK(rt::loadActTable(path, 0.002, &p, &err));
    CHECK_EQ(p.size(), static_cast<std::size_t>(3));
    if (p.size() == 3) CHECK_NEAR(p[1].a, 1.0, 1e-12);
    CHECK(!rt::loadActTable(path, 0.001, &p, &err));          // rows not loop.ts apart
    { std::ofstream f(path); f << "t,x,v,a\n0,0,0,0\n0.002,1,2\n"; }
    CHECK(!rt::loadActTable(path, 0.002, &p, &err));          // a row with three numbers
    { std::ofstream f(path); f << "x,v,a\n0,0,0\n"; }
    CHECK(!rt::loadActTable(path, 0.002, &p, &err));          // wrong header
    CHECK(!rt::loadActTable("build/no_such_table.csv", 0.002, &p, &err));
    std::remove(path);
}

// One speed, three rungs, two reps each, on an ideal actuator: homing, every touch within 1 ustep, every rung
// passes, the pendulum is left nearly still by the out-and-back pairs, nothing hits a switch or a stop.
TEST(actuator_ladder_on_the_simulated_rig) {
    Bench b(actCfg({{"actuator.speeds", "0.45"}, {"actuator.accels", "2,6,10"}}));
    b.home();
    CHECK(b.act.phase() == rt::ActPhase::Ready);
    CHECK(std::llabs(b.act.posLeft() - b.act.posRight() - 6104) <= 2);
    for (int i = 0; i < 3; ++i) b.rung();
    CHECK(b.act.phase() == rt::ActPhase::Ready);
    CHECK_EQ(b.act.nextRung(), -1);
    CHECK_EQ(b.act.results().size(), static_cast<std::size_t>(6));
    for (const rt::RepResult& r : b.act.results()) {
        CHECK(r.verdict == rt::Verdict::Pass);
        CHECK(std::llabs(r.touch_err) <= 1);
        CHECK_EQ(r.side, 1);                                  // pairs end at -d/2: the right switch
        CHECK(!r.settle_timeout);
        CHECK(r.peak_a_cmd <= 1.1 * r.a + 1e-9);
        CHECK(r.peak_a_cmd >= 0.95 * r.a);
        CHECK(r.peak_v <= r.v + 0.01);
        CHECK(r.max_track < 0.001);
    }
    for (rt::Verdict v : b.act.rungVerdicts()) CHECK(v == rt::Verdict::Pass);
    double swingAtTouch = 0;
    bool switchInRun = false;
    for (const rt::ActRecord& r : b.recs) {
        if (r.event == static_cast<int>(rt::ActEvent::Touched)) swingAtTouch = std::max(swingAtTouch, r.swing_deg);
        if (r.phase == static_cast<int>(rt::ActPhase::Run) && (r.limit_left || r.limit_right)) switchInRun = true;
    }
    CHECK(swingAtTouch < 4.0);
    CHECK(!switchInRun);
    CHECK(!b.sim.crashed());
    std::cout << "    0.45 m/s at 2 / 6 / 10 m/s^2, 2 reps each: largest swing at a touch " << swingAtTouch << " deg\n";
    // Enter after the last rung ends the block: hold to standstill, driver off.
    b.rung();
    CHECK(b.act.phase() == rt::ActPhase::Finished);
    CHECK(!b.recs.back().driver_enabled);
}

// Steps lost in a rep (one slip = 32 usteps): that rep fails, the rest of that speed is skipped, the next
// speed runs normally after the re-zero.
TEST(actuator_lost_steps_fail_the_rung_and_skip_the_speed) {
    Bench b(actCfg({{"actuator.speeds", "0.3,0.45"}, {"actuator.accels", "2,4,6"}}));
    b.home();
    bool lost = false;
    auto inject = [&](int) {
        if (!lost && b.act.currentRung() == 1 && b.act.phase() == rt::ActPhase::Run) { b.sim.loseSteps(32); lost = true; }
    };
    while (b.act.phase() == rt::ActPhase::Ready && b.act.nextRung() >= 0) b.rung(inject);
    CHECK(lost);
    const std::vector<rt::Verdict>& v = b.act.rungVerdicts();
    CHECK(v[0] == rt::Verdict::Pass);
    CHECK(v[1] == rt::Verdict::Fail);
    CHECK(v[2] == rt::Verdict::Skipped);
    CHECK(v[3] == rt::Verdict::Pass && v[4] == rt::Verdict::Pass && v[5] == rt::Verdict::Pass);
    bool sawSlip = false;
    for (const rt::RepResult& r : b.act.results()) {
        if (r.rung == 1) {                                    // +-1: the seek reads the switch once per tick
            CHECK(std::llabs(r.touch_err - 32) <= 1);
            CHECK(r.verdict == rt::Verdict::Fail);
            sawSlip = true;
        } else {
            CHECK(std::llabs(r.touch_err) <= 1);
        }
    }
    CHECK(sawSlip);
    CHECK(!b.sim.crashed());
}

// A touch error between touch_pass and touch_fail repeats the rep once; a clean repeat passes the rung.
TEST(actuator_marginal_touch_repeats_the_rep) {
    Bench b(actCfg({{"actuator.speeds", "0.3"}, {"actuator.accels", "2"}}));
    b.home();
    bool lost = false;
    b.rung([&](int) {
        if (!lost && b.act.phase() == rt::ActPhase::Run) { b.sim.loseSteps(5); lost = true; }
    });
    const std::vector<rt::RepResult>& r = b.act.results();
    CHECK_EQ(r.size(), static_cast<std::size_t>(3));          // rep 1 marginal, rep 1 again, rep 2
    if (r.size() == 3) {
        CHECK(r[0].verdict == rt::Verdict::Marginal && r[0].rep == 1 && r[0].touch_err == 5);
        CHECK(r[1].verdict == rt::Verdict::Pass && r[1].rep == 1 && r[1].repeat);
        CHECK(r[2].verdict == rt::Verdict::Pass && r[2].rep == 2 && !r[2].repeat);
    }
    CHECK(b.act.rungVerdicts()[0] == rt::Verdict::Pass);
}

// More than touch_tol (15 mm) lost: the position reference is gone, the block ends FAILED with the driver off.
// Injected after the run: on the rig a slip leaves the step count alone, so only the touch can see it (a jump
// of the reported position during RUN would trip the tracking check instead, which the real slip never does).
TEST(actuator_large_loss_fails_the_block) {
    Bench b(actCfg({{"actuator.speeds", "0.3"}, {"actuator.accels", "2"}}));
    b.home();
    bool lost = false;
    b.rung([&](int) {
        if (!lost && b.act.phase() == rt::ActPhase::Brake) { b.sim.loseSteps(300); lost = true; }
    });
    CHECK(b.act.phase() == rt::ActPhase::Failed);
    CHECK(std::string(b.act.reason()).find("step loss") != std::string::npos);
    CHECK(!b.recs.back().driver_enabled);
    CHECK(!b.sim.crashed());
}

// q during a leg brakes at the rung's own acceleration: by step count the cart stops before the leg end.
TEST(actuator_quit_during_a_run_brakes_at_the_rung_accel) {
    Bench b(actCfg({{"actuator.speeds", "0.6"}, {"actuator.accels", "8"}}));
    b.home();
    const double d = 0.6 * 0.6 / 8 + 0.6 * 0.10;
    bool quit = false;
    b.rung([&](int) {
        if (!quit && b.act.phase() == rt::ActPhase::Run && std::fabs(b.recs.back().v) > 0.59) { b.act.requestQuit(); quit = true; }
    });
    CHECK(quit);
    CHECK(b.act.phase() == rt::ActPhase::Finished);
    double xMax = -1, aBrake = 0;
    bool braking = false;
    for (const rt::ActRecord& r : b.recs) {
        if (r.phase == static_cast<int>(rt::ActPhase::Run) || r.phase == static_cast<int>(rt::ActPhase::Brake))
            xMax = std::max(xMax, r.x);
        if (r.phase == static_cast<int>(rt::ActPhase::Brake)) { braking = true; aBrake = std::max(aBrake, std::fabs(r.a_cmd)); }
    }
    CHECK(braking);
    CHECK(xMax <= d / 2 + 0.001);
    CHECK_NEAR(aBrake, 8.0, 0.01);
    CHECK_EQ(b.act.results().size(), static_cast<std::size_t>(1));
    CHECK(b.act.results().back().verdict == rt::Verdict::Aborted);
    CHECK(b.act.rungVerdicts()[0] == rt::Verdict::Aborted);
    CHECK(!b.recs.back().driver_enabled);
}

// s at READY skips the rest of the next rung's speed; Enter after the last rung ends the block.
TEST(actuator_skip_and_end) {
    Bench b(actCfg({{"actuator.speeds", "0.3,0.45"}, {"actuator.accels", "2,3"}}));
    b.home();
    b.act.requestSkip();
    b.until(5, [](int) { return false; });
    CHECK(b.sawEvent(rt::ActEvent::Skipped));
    CHECK_EQ(b.act.nextRung(), 2);
    b.rung();                                                 // (0.45, 2)
    CHECK(b.act.rungVerdicts()[2] == rt::Verdict::Pass);
    b.act.requestSkip();
    b.until(5, [](int) { return false; });
    CHECK_EQ(b.act.nextRung(), -1);
    b.rung();
    CHECK(b.act.phase() == rt::ActPhase::Finished);
}

// One-way legs alternate direction and touch the switch ahead; the pendulum settles before each leg.
TEST(actuator_oneway_alternates_sides) {
    Bench b(actCfg({{"actuator.profile", "oneway"}, {"actuator.speeds", "0.45"}, {"actuator.accels", "4"}}));
    b.home();
    b.rung();
    const std::vector<rt::RepResult>& r = b.act.results();
    CHECK_EQ(r.size(), static_cast<std::size_t>(2));
    if (r.size() == 2) {
        CHECK_EQ(r[0].side, 2);                               // toward +x: the left switch
        CHECK_EQ(r[1].side, 1);
        CHECK(r[0].verdict == rt::Verdict::Pass && r[1].verdict == rt::Verdict::Pass);
        CHECK(!r[1].settle_timeout);
        CHECK(r[1].swing_start_deg < 3.0);
    }
    CHECK(!b.sim.crashed());
}

// Bang-bang and chirp streams run, stay inside their bounds and pass the audit.
TEST(actuator_streams_run_and_pass) {
    for (const char* prof : {"bang", "chirp"}) {
        Bench b(actCfg({{"actuator.profile", prof}, {"actuator.speeds", "0.45"}, {"actuator.accels", "6"},
                        {"actuator.reps", "1"}}));
        b.home();
        b.rung();
        CHECK(b.act.rungVerdicts()[0] == rt::Verdict::Pass);
        double xMax = 0, vMax = 0;
        for (const rt::ActRecord& r : b.recs)
            if (r.phase == static_cast<int>(rt::ActPhase::Run)) { xMax = std::max(xMax, std::fabs(r.x)); vMax = std::max(vMax, std::fabs(r.v)); }
        CHECK(xMax <= 0.10 + 0.001);
        CHECK(vMax <= 0.45 + 0.01);
        CHECK_EQ(b.act.results().size(), static_cast<std::size_t>(1));
        if (!b.act.results().empty()) CHECK(std::llabs(b.act.results()[0].touch_err) <= 1);
        CHECK(!b.sim.crashed());
        std::cout << "    " << prof << " 0.45 m/s, 6 m/s^2: peak |x| " << xMax * 1e3 << " mm, |v| " << vMax << " m/s\n";
    }
}
