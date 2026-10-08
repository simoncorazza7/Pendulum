// Closed-loop tests of the balance runtime on the simulated hardware: the whole raw-unit chain
// (SimHw -> units -> estimator -> supervisor -> PID -> delay line -> Axis argument -> SimHw).
// PLACEHOLDER plant (1 s hanging period); the behaviours checked here must hold for any plant.

#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>

#include "check.h"
#include "hw/sim_hw.h"
#include "runtime/balance.h"

namespace {

const double kPi = 3.141592653589793;
const double kTs = 0.002;

struct Rig {
    hw::SimHwParams hp;
    hw::SimHw sim;
    rt::BalanceConfig cfg;

    explicit Rig(double theta0 = kPi, double x0 = 0.0) : hp(), sim(hp, theta0, 0.0, x0) {
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
