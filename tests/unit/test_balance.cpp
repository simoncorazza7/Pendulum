// Closed-loop tests of the balance runtime on the simulated hardware: the whole raw-unit chain
// (SimHw -> units -> estimator -> supervisor -> PID -> delay line -> Axis argument -> SimHw).
// PLACEHOLDER plant (1 s hanging period); the behaviours checked here must hold for any plant.

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <vector>

#include "check.h"
#include "hw/sim_hw.h"
#include "common/config.h"
#include "runtime/balance.h"
#include "runtime/params.h"

namespace {

const double kPi = 3.141592653589793;
const double kTs = 0.002;

struct Rig {
    hw::SimHwParams hp;
    hw::SimHw sim;
    rt::BalanceConfig cfg;

    explicit Rig(double theta0 = kPi, double x0 = 0.0) : Rig(hw::SimHwParams(), theta0, x0) {}
    Rig(const hw::SimHwParams& p, double theta0 = kPi, double x0 = 0.0) : hp(p), sim(hp, theta0, 0.0, x0) {
        cfg.calib = hp.truth;
        cfg.calib.c_bottom = sim.trueCBottomRelative();   // as the start-up swing fit would find it
        const auto g = core::designGains(hp.w0, hp.bJ, 2 * hp.w0, 0.7, 0.5 * hp.w0);
        cfg.pid.kp = g.kp; cfg.pid.ki = g.ki; cfg.pid.kd = g.kd;
        cfg.pid.a_max = 5.0; cfg.pid.v_max = 0.8; cfg.pid.i_delay = 0.1; cfg.pid.i_max = 1.0;
        cfg.sup.axis_a = hp.axis_a; cfg.sup.axis_v = hp.axis_v;
    }
};

// Runs ticks; `script(k, rt, sim)` may act before each tick (operator, hand). Returns all records.
std::vector<rt::TickRecord> run(Rig& rig, rt::BalanceRuntime& rt, int ticks,
                                const std::function<void(int, rt::BalanceRuntime&, hw::SimHw&)>& script,
                                const std::function<bool(int)>& overrunAt = [](int) { return false; }) {
    std::vector<rt::TickRecord> recs;
    recs.reserve(static_cast<std::size_t>(ticks));
    for (int k = 0; k < ticks; ++k) {
        script(k, rt, rig.sim);
        recs.push_back(rt.tick(rig.sim, overrunAt(k)));
        rig.sim.advance(kTs);
    }
    return recs;
}

// Operator arms while hanging, lifts to 2.3 deg over 0.4 s and holds; lets go as soon as the
// runtime engages.
void armLiftRelease(int k, rt::BalanceRuntime& rt, hw::SimHw& sim, double liftTheta = 0.04) {
    if (k == 10) rt.requestArm();
    if (k == 50) sim.holdAt(liftTheta);
    if (sim.held() && rt.mode() == core::Mode::Engaged) sim.release();
}

int firstEvent(const std::vector<rt::TickRecord>& recs, core::Reason e) {
    for (std::size_t i = 0; i < recs.size(); ++i)
        if (recs[i].event == static_cast<int>(e)) return static_cast<int>(i);
    return -1;
}

double maxAbs(const std::vector<rt::TickRecord>& recs, int from, int to, double rt::TickRecord::*f) {
    double m = 0;
    for (int i = from; i < to && i < static_cast<int>(recs.size()); ++i)
        m = std::max(m, std::fabs(recs[static_cast<std::size_t>(i)].*f));
    return m;
}

}

TEST(balance_catches_holds_and_stops_on_request) {
    Rig rig;
    rt::BalanceRuntime rt(rig.cfg);
    auto recs = run(rig, rt, 2200, [](int k, rt::BalanceRuntime& r, hw::SimHw& s) {
        armLiftRelease(k, r, s);
        if (k == 1800) r.requestStop();
    });
    const int armed = firstEvent(recs, core::Reason::ArmedEvent);
    const int engaged = firstEvent(recs, core::Reason::EngagedEvent);
    const int stopped = firstEvent(recs, core::Reason::Stopped);
    CHECK_EQ(armed, 10);
    CHECK(engaged > 250 && engaged < 250 + 60);               // engages within ~0.1 s of reaching upright
    CHECK(firstEvent(recs, core::Reason::ThetaAbort) < 0);
    CHECK(firstEvent(recs, core::Reason::SoftLimit) < 0);
    // balanced: |theta| below 1 deg from 1 s after engage until the stop request
    const double thMax = maxAbs(recs, engaged + 500, 1800, &rt::TickRecord::theta);
    CHECK(thMax < 1.0 * kPi / 180);
    CHECK(stopped > 1800 && stopped < 1800 + 250);           // controlled stop within 0.5 s
    CHECK(!recs.back().driver_enabled);
    CHECK(!rig.sim.crashed());
    std::cout << "    engaged " << (engaged - 250) * 2 << " ms after reaching upright; |theta| max "
              << thMax * 180 / kPi << " deg after 1 s; cart x at stop " << recs[static_cast<std::size_t>(stopped)].x
              << " m\n";
}

TEST(balance_too_low_gain_falls_and_aborts_safely) {
    Rig rig;
    rig.cfg.pid.kp = 0.5 * 9.81;   // below the Kp > g limit
    rig.cfg.pid.ki = 0;
    rt::BalanceRuntime rt(rig.cfg);
    auto recs = run(rig, rt, 1500, [](int k, rt::BalanceRuntime& r, hw::SimHw& s) { armLiftRelease(k, r, s); });
    // Kp < g diverges slowly (the cart chases the pendulum), so whichever safety abort comes first,
    // theta or the soft limit (here: the soft limit), must end in a controlled stop.
    const int th = firstEvent(recs, core::Reason::ThetaAbort), sl = firstEvent(recs, core::Reason::SoftLimit);
    const int abort = th > 0 && (sl < 0 || th < sl) ? th : sl;
    CHECK(abort > 0);
    CHECK(firstEvent(recs, core::Reason::Stopped) > abort);
    CHECK(!recs.back().driver_enabled);
    CHECK(!rig.sim.crashed());
}

TEST(balance_reference_error_drifts_into_soft_limit_and_stops_inside_switches) {
    Rig rig;
    rig.cfg.calib.c_bottom += 2.0;   // upright reference off by 2 counts: steady accel g*2q toward +x
    rig.cfg.pid.ki = 0;              // PD: drift g*delta*Kp/(Kp-g)
    rt::BalanceRuntime rt(rig.cfg);
    auto recs = run(rig, rt, 6000, [](int k, rt::BalanceRuntime& r, hw::SimHw& s) { armLiftRelease(k, r, s); });
    const int soft = firstEvent(recs, core::Reason::SoftLimit);
    const int stopped = firstEvent(recs, core::Reason::Stopped);
    CHECK(soft > 0);
    CHECK(stopped > soft);
    const auto& last = recs.back();
    CHECK(last.x > 0);                                         // drift toward +x, as derived
    CHECK(last.pos_usteps < 7750 && last.pos_usteps > 250);    // stopped before either switch
    CHECK(!rig.sim.crashed());
    CHECK(firstEvent(recs, core::Reason::LimitSwitch) < 0);
    std::cout << "    soft limit after " << (soft - firstEvent(recs, core::Reason::EngagedEvent)) * kTs
              << " s balanced; stopped at x = " << last.x << " m (soft limit " << rig.cfg.sup.x_max << ")\n";
}

TEST(balance_shadow_mode_never_touches_the_hardware) {
    Rig rig;
    rig.cfg.actuate = false;
    rt::BalanceRuntime rt(rig.cfg);
    auto recs = run(rig, rt, 800, [](int k, rt::BalanceRuntime& r, hw::SimHw& s) { armLiftRelease(k, r, s); });
    CHECK(firstEvent(recs, core::Reason::EngagedEvent) > 0);   // the logic runs
    bool anySent = false, anyCtrl = false;
    for (const auto& r : recs) {
        anySent = anySent || r.sent || r.driver_enabled;
        anyCtrl = anyCtrl || r.a_ctrl != 0.0;
    }
    CHECK(!anySent);
    CHECK(anyCtrl);                                            // a_cmd was computed and logged
    CHECK_EQ(rig.sim.x(), 0.0);                                // the cart never moved
    CHECK(firstEvent(recs, core::Reason::ThetaAbort) > 0);     // unactuated pendulum falls -> abort
}

TEST(balance_delay_line_beyond_margin_destabilises) {
    for (int n : {5, 30}) {        // 10 ms holds; 60 ms is beyond the ~40 ms delay margin
        Rig rig;
        rig.cfg.delay_samples = n;
        rt::BalanceRuntime rt(rig.cfg);
        auto recs = run(rig, rt, 2500, [](int k, rt::BalanceRuntime& r, hw::SimHw& s) { armLiftRelease(k, r, s); });
        const bool aborted = firstEvent(recs, core::Reason::ThetaAbort) > 0 ||
                             firstEvent(recs, core::Reason::SoftLimit) > 0;
        CHECK(n == 5 ? !aborted : aborted);
    }
}

TEST(balance_overrun_streak_aborts) {
    Rig rig;
    rt::BalanceRuntime rt(rig.cfg);
    auto recs = run(
        rig, rt, 1200, [](int k, rt::BalanceRuntime& r, hw::SimHw& s) { armLiftRelease(k, r, s); },
        [](int k) { return k >= 700 && k < 703; });
    CHECK_EQ(firstEvent(recs, core::Reason::Overrun), 702);    // third consecutive overrun
}

TEST(balance_arm_refused_at_limit_switch) {
    Rig rig(kPi, (250.0 - 4000.0) / 13333.3);                  // cart sitting on the right switch
    rt::BalanceRuntime rt(rig.cfg);
    auto recs = run(rig, rt, 50, [](int k, rt::BalanceRuntime& r, hw::SimHw&) { if (k == 10) r.requestArm(); });
    CHECK_EQ(recs[10].event, static_cast<int>(core::Reason::LimitSwitch));
    CHECK_EQ(recs.back().mode, static_cast<int>(core::Mode::Idle));
    CHECK(!recs.back().driver_enabled);
}

TEST(balance_wrong_angle_sign_is_caught) {
    Rig rig;
    rig.cfg.calib.theta_sign = -rig.cfg.calib.theta_sign;      // sign error in the configuration
    rig.cfg.calib.c_bottom = rig.sim.trueCBottomRelative();   // (hanging position unchanged)
    rt::BalanceRuntime rt(rig.cfg);
    auto recs = run(rig, rt, 1500, [](int k, rt::BalanceRuntime& r, hw::SimHw& s) { armLiftRelease(k, r, s); });
    const int engaged = firstEvent(recs, core::Reason::EngagedEvent);
    const int abort = firstEvent(recs, core::Reason::ThetaAbort);
    CHECK(engaged > 0);
    CHECK(abort > engaged && abort - engaged < 250);           // positive feedback: aborts within 0.5 s
    CHECK(!rig.sim.crashed());
}

TEST(balance_bdlp2_estimator_also_balances) {
    Rig rig;
    rig.cfg.est_kind = core::VelocityEstimator::Kind::BdLp2;
    rt::BalanceRuntime rt(rig.cfg);
    auto recs = run(rig, rt, 1500, [](int k, rt::BalanceRuntime& r, hw::SimHw& s) { armLiftRelease(k, r, s); });
    const int engaged = firstEvent(recs, core::Reason::EngagedEvent);
    CHECK(engaged > 0);
    CHECK(firstEvent(recs, core::Reason::ThetaAbort) < 0);
    CHECK(maxAbs(recs, engaged + 500, 1500, &rt::TickRecord::theta) < 1.0 * kPi / 180);
}

// Stage 0.11 on the identified plant with the rig's own config (gains, limits, supervisor): the
// estimator chosen from real data (BD+LP2 20 Hz, py/estimator_real.py) against the one the shadow run
// used (BD+LP1 30 Hz, +-2.4 m/s^2 Kd*omega spikes per count step). Both must balance; LP2 20 Hz must
// cut the command noise in the quantisation limit cycle. (Both drift ~0.2 m in 8 s: angle-only control.)
TEST(balance_identified_plant_bdlp2_20hz_cuts_command_noise) {
    struct Out { double a_rms, a_p99, th_rms; bool ok; };
    auto run1 = [](core::VelocityEstimator::Kind kind, double fc) {
        hw::SimHwParams p;
        p.w0 = 4.931; p.bJ = 0.024; p.coulomb = 0.0117;           // 0.10
        p.axis_a = 400000.0 / (2 * 13333.3); p.axis_v = 120000.0 / (2 * 13333.3);   // config axis.max_*
        Rig rig(p);
        rig.cfg = rt::balanceFrom(Config::fromFile("config/pendctl.conf"));
        rig.cfg.calib = p.truth;
        rig.cfg.calib.c_bottom = rig.sim.trueCBottomRelative();
        rig.cfg.est_kind = kind;
        rig.cfg.est_fc = fc;
        rig.cfg.sup.engage_on_release = false;                    // engage while held, released on engage
        rig.cfg.preload = false;
        rt::BalanceRuntime rt(rig.cfg);
        auto recs = run(rig, rt, 2200, [](int k, rt::BalanceRuntime& r, hw::SimHw& s) { armLiftRelease(k, r, s); });
        const int e = firstEvent(recs, core::Reason::EngagedEvent);
        std::vector<double> a;
        double th2 = 0;
        bool ok = e > 0;
        // 1 s to 3.5 s after engage: the limit cycle, before the angle-only drift reaches a soft limit
        const std::size_t from = static_cast<std::size_t>(std::max(e, 0) + 500), to = from + 1250;
        for (std::size_t i = from; i < to && i < recs.size(); ++i) {
            if (ok && recs[i].mode != static_cast<int>(core::Mode::Engaged))
                std::cout << "    left ENGAGED at tick " << i << ": " << core::reasonName(static_cast<core::Reason>(recs[i].event))
                          << ", x " << recs[i].x << " m" << std::endl;
            ok = ok && recs[i].mode == static_cast<int>(core::Mode::Engaged);
            a.push_back(std::fabs(recs[i].a_cmd));
            th2 += recs[i].theta * recs[i].theta;
        }
        double a2 = 0;
        for (double v : a) a2 += v * v;
        std::sort(a.begin(), a.end());
        const double n = static_cast<double>(a.size());
        return Out{std::sqrt(a2 / n), a[static_cast<std::size_t>(0.99 * n)], std::sqrt(th2 / n) * 180 / kPi, ok};
    };
    const Out lp1 = run1(core::VelocityEstimator::Kind::BdLp1, 30.0);
    const Out lp2 = run1(core::VelocityEstimator::Kind::BdLp2, 20.0);
    std::cout << "    BD+LP1 30 Hz: RMS a " << lp1.a_rms << ", p99 " << lp1.a_p99 << " m/s^2, RMS theta " << lp1.th_rms
              << " deg" << "\n";
    std::cout << "    BD+LP2 20 Hz: RMS a " << lp2.a_rms << ", p99 " << lp2.a_p99 << " m/s^2, RMS theta " << lp2.th_rms
              << " deg" << "\n";
    CHECK(lp1.ok && lp2.ok);                                     // both balance for 4 s
    CHECK(lp2.a_rms < 0.7 * lp1.a_rms);
    CHECK(lp2.a_p99 < 0.7 * lp1.a_p99);
    CHECK(lp2.th_rms < 0.2 && lp1.th_rms < 0.2);
}

// Drift budget (docs/stage1_drift.md): the operator's hand still on the rod after ENGAGED. The rigid-hand
// model (SimHw::holdAt) keeps the pendulum at the lift angle while the controller accelerates the cart;
// that momentum stays after the release. Prints the residual cart velocity for hold times and the
// integrator enable delay; checks only the sanity case (immediate release, i_delay 0).
TEST(balance_drift_budget_hand_hold) {
    auto residual = [](double hold, double iDelay, double* xAt) {
        hw::SimHwParams p;
        p.w0 = 4.931; p.bJ = 0.024; p.coulomb = 0.0117;
        p.axis_a = 400000.0 / (2 * 13333.3); p.axis_v = 120000.0 / (2 * 13333.3);
        Rig rig(p);
        rig.cfg = rt::balanceFrom(Config::fromFile("config/pendctl.conf"));
        rig.cfg.calib = p.truth;
        rig.cfg.calib.c_bottom = rig.sim.trueCBottomRelative();
        rig.cfg.pid.i_delay = iDelay;
        rig.cfg.sup.engage_on_release = false;                    // the old logic: engage while held
        rig.cfg.preload = false;
        rig.cfg.sup.x_min = -0.27; rig.cfg.sup.x_max = 0.27;      // inside the simulated switches
        rt::BalanceRuntime rt(rig.cfg);
        int engagedAt = -1, releasedAt = -1;
        auto recs = run(rig, rt, 2500, [&](int k, rt::BalanceRuntime& r, hw::SimHw& s) {
            if (k == 10) r.requestArm();
            if (k == 50) s.holdAt(0.04);
            if (engagedAt < 0 && r.mode() == core::Mode::Engaged) engagedAt = k;
            if (s.held() && engagedAt >= 0 && k >= engagedAt + static_cast<int>(hold / kTs)) { s.release(); releasedAt = k; }
        });
        if (releasedAt < 0) return 1e9;
        double v = 0;
        int n = 0;
        for (int i = releasedAt + 500; i < releasedAt + 1000 && i < static_cast<int>(recs.size()); ++i, ++n)
            v += recs[static_cast<std::size_t>(i)].v;
        const int iEnd = std::min(releasedAt + 1000, static_cast<int>(recs.size()) - 1);
        *xAt = recs[static_cast<std::size_t>(iEnd)].x;
        return n ? v / n : 1e9;
    };
    std::cout << "    hold after engage | i_delay 0.1: v (mm/s), x(+2 s) (mm) | i_delay 0: v, x(+2 s)\n";
    double v0sane = 0;
    for (double hold : {0.0, 0.05, 0.1, 0.2, 0.3}) {
        double xa = 0, xb = 0;
        const double va = residual(hold, 0.1, &xa), vb = residual(hold, 0.0, &xb);
        if (hold == 0.0) v0sane = vb;
        std::cout << "    " << hold << " s | " << va * 1e3 << ", " << xa * 1e3 << " | " << vb * 1e3 << ", " << xb * 1e3 << "\n";
    }
    CHECK(std::fabs(v0sane) < 0.02);   // immediate release + full integration: residual < 20 mm/s
}

// The fix (docs/stage1_drift.md): engage on release + momentum pre-load + i_delay 0 (the rig config).
// The hand lifts the pendulum to 1 or 2 deg and holds it still for 1 s: the runtime must not engage and
// the cart must not move while it is held; after the release it engages within 0.25 s, the two-edge
// estimate matches the true theta' and the cart keeps no momentum.
TEST(balance_engage_on_release_leaves_no_cart_momentum) {
    for (double lift : {0.0175, 0.035}) {
        hw::SimHwParams p;
        p.w0 = 4.931; p.bJ = 0.024; p.coulomb = 0.0117;
        p.axis_a = 400000.0 / (2 * 13333.3); p.axis_v = 120000.0 / (2 * 13333.3);
        Rig rig(p);
        rig.cfg = rt::balanceFrom(Config::fromFile("config/pendctl.conf"));
        rig.cfg.calib = p.truth;
        rig.cfg.calib.c_bottom = rig.sim.trueCBottomRelative();
        rig.cfg.sup.x_min = -0.27; rig.cfg.sup.x_max = 0.27;
        rig.cfg.sup.engage_on_release = true;                     // config default is off until validated
        CHECK(rig.cfg.preload && rig.cfg.pid.i_delay == 0);
        rt::BalanceRuntime rt(rig.cfg);
        const int releaseAt = 50 + 200 + 500;                     // lift 0.4 s, hold 1 s
        int engagedAt = -1;
        double trueOmega = 0, estOmega = 0;
        auto recs = run(rig, rt, 2500, [&](int k, rt::BalanceRuntime& r, hw::SimHw& s) {
            if (k == 10) r.requestArm();
            if (k == 50) s.holdAt(lift);
            if (k == releaseAt) s.release();
            if (engagedAt < 0 && r.mode() == core::Mode::Engaged) { engagedAt = k; trueOmega = s.thetadot(); }
        });
        double xHeld = 0;
        for (int i = 0; i < releaseAt; ++i) xHeld = std::max(xHeld, std::fabs(recs[static_cast<std::size_t>(i)].x));
        for (const auto& rec : recs) if (rec.event == static_cast<int>(core::Reason::EngagedEvent)) estOmega = rec.omega_release;
        double v = 0;
        int n = 0;
        for (int i = engagedAt + 500; engagedAt > 0 && i < engagedAt + 1000 && i < static_cast<int>(recs.size()); ++i, ++n)
            v += recs[static_cast<std::size_t>(i)].v;
        v = n ? v / n : 1e9;
        std::cout << "    lift " << lift * 180 / kPi << " deg: engaged " << (engagedAt - releaseAt) * kTs * 1e3
                  << " ms after release, theta' true " << trueOmega << " est " << estOmega << " rad/s, residual v "
                  << v * 1e3 << " mm/s" "\n";
        CHECK(engagedAt > releaseAt);                                 // never while held
        CHECK((engagedAt - releaseAt) * kTs < 0.35);
        CHECK(xHeld == 0.0);                                          // the cart did not move while held
        CHECK(std::fabs(estOmega - trueOmega) < 0.02);
        // 2 deg: the longer release window engages at ~3.5 deg, where a_max 3 (PRIOR) saturates and the
        // integrator hold leaves ~45 mm/s; 1 deg stays clear of saturation.
        CHECK(std::fabs(v) < (lift < 0.02 ? 0.02 : 0.06));
        CHECK(recs.back().mode == static_cast<int>(core::Mode::Engaged));
    }
}

// Hand creep (shadow run 14-38-54: one count every 0.1-0.35 s while held) must never count as a release:
// the hand moves the pendulum away from upright at ~0.02 rad/s for 2 s.
TEST(balance_hand_creep_is_not_a_release) {
    hw::SimHwParams p;
    p.w0 = 4.931; p.bJ = 0.024; p.coulomb = 0.0117;
    Rig rig(p);
    rig.cfg = rt::balanceFrom(Config::fromFile("config/pendctl.conf"));
    rig.cfg.calib = p.truth;
    rig.cfg.calib.c_bottom = rig.sim.trueCBottomRelative();
    rig.cfg.sup.engage_on_release = true;
    rt::BalanceRuntime rt(rig.cfg);
    bool released = false;
    auto recs = run(rig, rt, 1800, [&](int k, rt::BalanceRuntime& r, hw::SimHw& s) {
        if (k == 10) r.requestArm();
        if (k == 50) s.holdAt(0.0175);                            // lift to 1 deg
        if (k == 300) s.holdAt(0.0525, 2.0);                      // creep to 3 deg over 2 s
    });
    for (const auto& rec : recs) released = released || rec.released;
    CHECK(!released);
    CHECK(recs.back().mode == static_cast<int>(core::Mode::Armed));
}
