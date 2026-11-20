// Upright-offset estimate (docs/offset_estimator.md): core::DriftIdentityEstimator against its Python reference
// (py/offset/identity.py via py/offset_golden.py), offset.* parsing, and the balance session on the simulated rig:
// watch changes no command; apply starts the next hand catch and the next swing-up run trimmed.

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "check.h"
#include "common/config.h"
#include "core/offset_est.h"
#include "hw/sim_hw.h"
#include "runtime/offset_params.h"
#include "runtime/params.h"
#include "runtime/session.h"

namespace {

const double kPi = 3.141592653589793;
const double kTs = 0.002;
const double kCount = 2 * kPi / 2400;      // rad per encoder count

std::vector<std::vector<std::string>> readCsv(const std::string& path) {
    std::ifstream in(path);
    std::vector<std::vector<std::string>> rows;
    std::string line;
    bool header = true;
    while (std::getline(in, line)) {
        if (header) { header = false; continue; }
        if (line.empty()) continue;
        std::vector<std::string> f;
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ',')) f.push_back(cell);
        rows.push_back(f);
    }
    return rows;
}

double num(const std::string& s) { return std::strtod(s.c_str(), nullptr); }

// The session test rig (as tests/unit/test_session.cpp): the 2026-11-02 pivot, switches 6104 usteps apart.
hw::SimHwParams rig() {
    hw::SimHwParams p;
    const double span = 6104 / 13333.0;
    p.w0 = 4.97; p.bJ = 0.054; p.coulomb = 0.168;
    p.axis_a = 400000.0 / (2 * 13333.0); p.axis_v = 120000.0 / (2 * 13333.0);
    p.pos_switch_right = 1000;
    p.pos_switch_left = 1000 + span * 13333.0;
    p.pos_end_right = 1000 - 0.015 * 13333.0;
    p.pos_end_left = p.pos_switch_left + 0.015 * 13333.0;
    p.truth.pos_center = 1000 + 0.5 * span * 13333.0;
    return p;
}

struct Run {
    hw::SimHw sim;
    rt::Session ses;
    std::vector<rt::SessionRecord> recs;
    Run(const rt::SessionConfig& c, double theta0) : sim(rig(), theta0, 0.0, 0.0), ses(c) { recs.reserve(200000); }
    void until(int n, const std::function<bool(int)>& done) {
        for (int k = 0; k < n && !done(k); ++k) {
            recs.push_back(ses.tick(sim, false));
            sim.advance(kTs);
        }
    }
    void hold(double s) {       // the operator stops the swing by hand
        sim.holdAt(std::round(sim.theta() / (2 * kPi) - 0.5) * 2 * kPi + kPi, 0.6);
        until(static_cast<int>(s / kTs), [](int) { return false; });
        sim.release();
    }
    void calibrateAndHome() {
        until(20000, [&](int) { return ses.phase() != rt::SessionPhase::Calibrate; });
        hold(0.8);
        ses.requestGo();
        until(40000, [&](int) { return ses.phase() != rt::SessionPhase::WaitGo && ses.phase() != rt::SessionPhase::Home; });
    }
    void liftAndRelease() {     // from BALANCE: wait for ARMED, lift to 2 deg, let go at ENGAGED
        until(5000, [&](int) { return ses.mode() == core::Mode::Armed; });
        sim.holdAt(0.035, 0.6);
        until(2000, [&](int) {
            if (sim.held() && ses.mode() == core::Mode::Engaged) sim.release();
            return ses.mode() == core::Mode::Engaged && !sim.held();
        });
    }
};

rt::SessionConfig session(const std::string& mode, bool swingup = false) {
    Config cfg = Config::fromFile("config/pendctl.conf");
    cfg.setFromAssignment("offset.mode=" + mode);
    if (swingup) {
        cfg.setFromAssignment("balance.swingup=true");
        cfg.setFromAssignment("swingup.e_ref=0.003");      // SimHw energy losses (tests/unit/test_swingup.cpp)
    }
    rt::SessionConfig c = rt::sessionFrom(cfg);
    c.offset = rt::offsetFrom(cfg, c.remote_mode);
    c.trim = 1.0;               // a 1-count upright reference error: theta_true = theta_measured + 1 count
    return c;
}

}

// ---- golden: C++ must reproduce the Python reference (py/offset_golden.py) ----
TEST(offset_golden_matches_python) {
    const auto params = readCsv("tests/golden/offset_params.csv");
    const auto rows = readCsv("tests/golden/offset_golden.csv");
    CHECK(params.size() == 2 && rows.size() > 10000);
    double worst = 0;
    int n = 0, stretch_mismatch = 0;
    for (const auto& pr : params) {
        core::OffsetParams p;
        p.w0 = num(pr[1]); p.bJ = num(pr[2]); p.ts = num(pr[3]); p.g = num(pr[4]); p.t_settle = num(pr[5]);
        p.theta_max = num(pr[6]); p.lag = num(pr[7]); p.smooth = num(pr[8]); p.step = num(pr[9]);
        p.min_stretch = num(pr[10]); p.delta_max = num(pr[11]);
        core::DriftIdentityEstimator e(p, num(pr[12]), num(pr[13]));
        for (const auto& r : rows) {
            if (r[0] != pr[0]) continue;
            e.step(num(r[2]), num(r[3]), num(r[4]), num(r[5]), r[6] == "1");
            if (r[7] == "1") e.endBalance();
            const double ref[3] = {num(r[8]), num(r[9]), num(r[10])};
            const double got[3] = {e.slope(), e.weight(), e.deltaHat()};
            for (int i = 0; i < 3; ++i) {
                const double d = std::fabs(got[i] - ref[i]) / (1e-12 + std::fabs(ref[i]));
                worst = std::max(worst, d);
                if (!(std::fabs(got[i] - ref[i]) <= 1e-12 + 1e-9 * std::fabs(ref[i])))
                    check::fail(__FILE__, __LINE__, "case " + pr[0] + " tick " + r[1] + " output " + std::to_string(i));
            }
            if (e.stretches() != std::atoi(r[11].c_str())) ++stretch_mismatch;
            ++n;
        }
    }
    CHECK_EQ(stretch_mismatch, 0);
    std::cout << "    golden: " << n << " ticks, worst relative difference " << worst << "\n";
}

TEST(offset_validate_rejects_bad_params) {
    CHECK(core::validate(core::OffsetParams{}).empty());
    core::OffsetParams p;
    p.lag = 0.001;              // shorter than a tick
    CHECK(!core::validate(p).empty());
    p = core::OffsetParams{};
    p.step = 0;
    CHECK(!core::validate(p).empty());
}

TEST(offset_prior_alone_is_the_estimate_and_end_keeps_it) {
    core::DriftIdentityEstimator none;
    CHECK_EQ(none.weight(), 0.0);
    CHECK_EQ(none.deltaHat(), 0.0);
    core::DriftIdentityEstimator e(core::OffsetParams{}, 1.3 * kCount, 20.0);
    CHECK_NEAR(e.deltaHat(), 1.3 * kCount, 1e-15);
    CHECK_NEAR(e.weight(), 20.0, 1e-12);
    for (int k = 0; k < 1000; ++k) e.step(k * kTs, 0.0, 0.0, 0.0, k < 500);   // nothing quiet enough to learn from
    e.endBalance();
    CHECK_NEAR(e.deltaHat(), 1.3 * kCount, 1e-15);
}

TEST(offset_params_from_config) {
    Config cfg = Config::fromFile("config/pendctl.conf");
    rt::OffsetUse u = rt::offsetFrom(cfg, false);
    CHECK_EQ(u.mode, static_cast<int>(rt::OffsetUse::Off));            // default off
    CHECK_NEAR(u.est.w0, cfg.getDouble("plant.w0"), 1e-12);
    CHECK_NEAR(u.est.theta_max, 3.0 * kPi / 180, 1e-15);
    CHECK_NEAR(u.est.delta_max, 5 * kCount, 1e-15);
    CHECK_NEAR(u.w_use, 3.0, 1e-12);
    cfg.setFromAssignment("offset.mode=apply");
    CHECK_EQ(rt::offsetFrom(cfg, false).mode, static_cast<int>(rt::OffsetUse::Apply));
    CHECK_THROWS(rt::offsetFrom(cfg, true));                           // apply needs the controller on the Pi
    cfg.setFromAssignment("offset.mode=watch");
    CHECK_EQ(rt::offsetFrom(cfg, true).mode, static_cast<int>(rt::OffsetUse::Watch));
    cfg.setFromAssignment("offset.mode=on");
    CHECK_THROWS(rt::offsetFrom(cfg, false));
    cfg.setFromAssignment("offset.mode=watch");
    cfg.setFromAssignment("offset.step=0");
    CHECK_THROWS(rt::offsetFrom(cfg, false));
}

// Watch logs the estimate and changes nothing: the same session with offset off sends the same commands.
TEST(offset_watch_session_changes_no_command) {
    Run off(session("off"), kPi - 0.31), watch(session("watch"), kPi - 0.31);
    for (Run* r : {&off, &watch}) {
        r->calibrateAndHome();
        r->liftAndRelease();
        r->until(7500, [](int) { return false; });                      // 15 s balanced
    }
    CHECK_EQ(off.recs.size(), watch.recs.size());
    bool same = off.recs.size() == watch.recs.size();
    for (std::size_t k = 0; same && k < off.recs.size(); ++k)
        same = off.recs[k].a_cmd == watch.recs[k].a_cmd && off.recs[k].axis_arg == watch.recs[k].axis_arg;
    CHECK(same);
    const rt::SessionRecord& w = watch.recs.back();
    std::cout << "    watch: estimate " << w.offset_delta / kCount << " counts from " << w.offset_weight
              << " s (set error 1 count), trim " << w.offset_trim << "\n";
    CHECK(w.offset_weight > 8.0);
    CHECK(w.offset_delta / kCount > 0.4 && w.offset_delta / kCount < 1.7);   // 1 count + up to 0.5 quantisation
    CHECK_EQ(w.offset_trim, 0.0);
    CHECK_EQ(off.recs.back().offset_weight, 0.0);
}

// Apply: the first catch starts untrimmed and learns; after the re-centre the second catch starts with
// theta_ref = -delta_hat, held for the whole catch, and after the catch transient (the kick sets the first
// ~0.1 m; the offset sets what follows while the cart integral builds up) the cart stays closer to the centre.
TEST(offset_apply_session_trims_the_next_catch) {
    Run r(session("apply"), kPi - 0.31);
    r.calibrateAndHome();
    CHECK(r.ses.phase() == rt::SessionPhase::Balance);
    r.liftAndRelease();
    double max1 = 0;
    r.until(7500, [&](int k) { if (k >= 2000) max1 = std::max(max1, std::fabs(r.sim.x())); return false; });   // 15 s
    CHECK_EQ(r.ses.offsetTrim(), 0.0);                                  // the first catch had nothing to go on
    r.ses.requestGo();                                                  // stop the run
    r.until(3000, [&](int) { return r.ses.phase() == rt::SessionPhase::AfterRun; });
    const double learnt = r.ses.offsetEstimate().deltaHat();
    CHECK(r.ses.offsetEstimate().weight() >= 3.0);
    r.hold(0.8);
    r.ses.requestGo();                                                  // re-centre, re-arm
    r.until(30000, [&](int) { return r.ses.phase() == rt::SessionPhase::Balance && r.ses.mode() == core::Mode::Armed; });
    CHECK(r.ses.mode() == core::Mode::Armed);
    CHECK_EQ(r.ses.offsetTrim(), learnt);                               // set when the runtime was created
    r.liftAndRelease();
    const std::size_t k0 = r.recs.size();
    double max2 = 0;
    r.until(7500, [&](int k) { if (k >= 2000) max2 = std::max(max2, std::fabs(r.sim.x())); return false; });
    bool held = true;
    for (std::size_t k = k0; k < r.recs.size(); ++k) held = held && r.recs[k].offset_trim == learnt;
    CHECK(held);
    CHECK(r.ses.mode() == core::Mode::Engaged);
    std::cout << "    apply: learnt " << learnt / kCount << " counts in catch 1; max |x| from 4 s to 15 s after engage "
              << max1 * 1e3 << " mm untrimmed, " << max2 * 1e3 << " mm trimmed\n";
    CHECK(learnt / kCount > 0.4 && learnt / kCount < 1.7);
    CHECK(max2 < max1);
}

// Apply with swing-ups: the first run's balance teaches the estimator; the second run starts trimmed.
TEST(offset_apply_swingup_session_trims_the_next_run) {
    Run r(session("apply", true), kPi - 0.31);
    r.calibrateAndHome();
    auto runOnce = [&] {
        r.until(40000, [&](int) { return r.ses.phase() == rt::SessionPhase::Balance && r.ses.swingReady(); });
        CHECK(r.ses.swingReady());
        r.ses.requestGo();
        r.until(80000, [&](int) { return r.ses.phase() == rt::SessionPhase::AfterRun; });
        CHECK(r.ses.phase() == rt::SessionPhase::AfterRun);
    };
    runOnce();
    const rt::AttemptStats a1 = r.ses.lastAttempt();
    CHECK(a1.engaged && a1.t_engaged > 5.0);
    CHECK_EQ(r.ses.offsetTrim(), 0.0);
    const double learnt = r.ses.offsetEstimate().deltaHat();
    CHECK(r.ses.offsetEstimate().weight() >= 3.0);
    r.hold(0.8);
    r.ses.requestGo();                                                  // re-centre, then READY again
    r.until(40000, [&](int) { return r.ses.phase() == rt::SessionPhase::Balance && r.ses.swingReady(); });
    r.ses.requestGo();
    r.until(1, [](int) { return false; });
    CHECK(r.ses.swingPhase() == static_cast<int>(rt::SwingPhase::Kick));
    CHECK_EQ(r.ses.offsetTrim(), learnt);                               // the run's balance PID starts trimmed
    std::cout << "    swing-up apply: run 1 balanced " << a1.t_engaged << " s, learnt " << learnt / kCount
              << " counts; run 2 starts with that trim\n";
    CHECK(learnt / kCount > 0.3 && learnt / kCount < 1.8);
    CHECK(!r.sim.crashed());
}
