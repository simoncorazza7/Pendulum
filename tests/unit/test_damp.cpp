// Start-up swing calibration and the hanging active-damping test, closed loop on the simulated
// hardware with the identified plant (0.10: w0 = 4.931 rad/s, b/J = 0.024 1/s, Coulomb 0.0117 rad/s^2).

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "check.h"
#include "hw/sim_hw.h"
#include "runtime/damp.h"
#include "runtime/swing_calib.h"

namespace {

const double kPi = 3.141592653589793;
const double kTs = 0.002;
const double kDeg = kPi / 180;

hw::SimHwParams plant() {
    hw::SimHwParams p;
    p.w0 = 4.931;
    p.bJ = 0.024;
    p.coulomb = 0.0117;
    return p;
}

rt::SwingCalibParams calibParams(double /*amp*/) {
    rt::SwingCalibParams c;
    c.w0 = 4.931;                         // the calibrator corrects w for the amplitude
    c.sigma = 0.03;
    return c;
}

// Feeds samples to the calibrator until it accepts (or maxTicks). Returns ticks used (-1: never).
int calibrate(hw::SimHw& sim, rt::SwingCalibrator& cal, int maxTicks) {
    for (int k = 0; k < maxTicks; ++k) {
        if (cal.update(sim.read())) return k;
        sim.advance(kTs);
    }
    return -1;
}

rt::DampConfig dampConfig(const hw::SimHwParams& hp, double cBottom) {
    rt::DampConfig c;
    c.calib = hp.truth;
    c.calib.c_bottom = cBottom;
    c.gain = rt::DampRuntime::gainFor(0.2, hp.w0, hp.bJ);
    c.a_max = 1.0;
    c.limits.x_min = -0.10;
    c.limits.x_max = 0.10;
    c.limits.a_brake = 1.0;
    c.limits.axis_a = hp.axis_a;
    c.limits.axis_v = hp.axis_v;
    return c;
}

struct DampRun {
    std::vector<rt::DampRecord> recs;
    std::vector<double> th, x;   // ground truth per tick
};

DampRun runDamp(hw::SimHw& sim, rt::DampRuntime& rt, int maxTicks) {
    DampRun d;
    for (int k = 0; k < maxTicks && rt.phase() != rt::DampPhase::Done; ++k) {
        d.recs.push_back(rt.tick(sim, false));
        d.th.push_back(sim.theta());
        d.x.push_back(sim.x());
        sim.advance(kTs);
    }
    return d;
}

// c_bottom error modulo one revolution (the fit finds the nearest hanging position).
double cbErr(double c, double truth) {
    double d = std::fmod(c - truth, 2400.0);
    if (d > 1200) d -= 2400;
    if (d < -1200) d += 2400;
    return d;
}

// Peak |theta - pi| (true) over ticks [from, to).
double peakPhi(const DampRun& d, int from, int to) {
    double m = 0;
    for (int i = std::max(0, from); i < to && i < static_cast<int>(d.th.size()); ++i)
        m = std::max(m, std::fabs(core::wrap(d.th[static_cast<std::size_t>(i)] - kPi)));
    return m;
}

}

TEST(swing_calib_finds_c_bottom_from_a_small_free_swing) {
    const double amp = 5 * kDeg;
    hw::SimHw sim(plant(), kPi + amp);
    rt::SwingCalibrator cal(calibParams(amp));
    const int k = calibrate(sim, cal, 5000);
    CHECK(k > 0);
    CHECK(k * kTs < 5.0);                                       // window 3 s + two fits
    CHECK_NEAR(cbErr(cal.status().c_bottom, sim.trueCBottomRelative()), 0.0, 0.05);
    CHECK(cal.status().fit.rms < 0.6);
}

TEST(swing_calib_waits_while_the_hand_is_on_the_pendulum) {
    const double amp = 5 * kDeg;
    hw::SimHw sim(plant(), kPi + amp);
    rt::SwingCalibrator cal(calibParams(amp));
    int accepted = -1;
    for (int k = 0; k < 8000 && accepted < 0; ++k) {
        if (k == 1000) sim.holdAt(kPi + 3 * kDeg, 0.2);         // hand stops the swing at 3 deg ...
        if (k == 2000) sim.release();                           // ... and lets go 1.8 s later
        if (cal.update(sim.read())) accepted = k;
        sim.advance(kTs);
    }
    CHECK(accepted > 2000 + static_cast<int>(0.8 * 3.0 / kTs));  // only once the window is hand-free
    CHECK_NEAR(cbErr(cal.status().c_bottom, sim.trueCBottomRelative()), 0.0, 0.05);
}

TEST(swing_calib_accepts_a_15_deg_swing) {
    const double amp = 15 * kDeg;
    hw::SimHw sim(plant(), kPi + amp);
    rt::SwingCalibrator cal(calibParams(amp));
    CHECK(calibrate(sim, cal, 5000) > 0);
    CHECK_NEAR(cbErr(cal.status().c_bottom, sim.trueCBottomRelative()), 0.0, 0.15);
}

TEST(swing_calib_rejects_a_swing_that_is_too_large) {
    const double amp = 30 * kDeg;
    hw::SimHw sim(plant(), kPi + amp);
    rt::SwingCalibrator cal(calibParams(5 * kDeg));
    CHECK(calibrate(sim, cal, 3000) < 0);
    CHECK(cal.status().amplitude > cal.params().amp_max);
}

TEST(damp_engages_at_a_zero_crossing_and_damps_as_designed) {
    const auto hp = plant();
    const double amp = 6 * kDeg;
    hw::SimHw sim(hp, kPi + amp);
    rt::SwingCalibrator cal(calibParams(amp));
    CHECK(calibrate(sim, cal, 5000) > 0);
    rt::DampRuntime rt(dampConfig(hp, cal.status().c_bottom));
    DampRun d = runDamp(sim, rt, 8000);

    CHECK(rt.phase() == rt::DampPhase::Done);
    CHECK(std::string(rt.reason()) == "completed");
    CHECK(rt.engagedAt() >= 0 && rt.engagedAt() < 0.7);         // first crossing within half a period
    CHECK(std::fabs(rt.phiEngage()) < 0.02);
    const int e = static_cast<int>(rt.engagedAt() / kTs);
    const int per = static_cast<int>(2 * kPi / hp.w0 / kTs);     // one period in ticks
    const double a0 = peakPhi(d, e, e + per);
    const double a2 = peakPhi(d, e + static_cast<int>(2.0 / kTs), e + static_cast<int>(2.0 / kTs) + per);
    const double predicted = std::exp(-0.2 * hp.w0 * 2.0);       // zeta_h w0 decay over 2 s: 0.14
    CHECK(a2 / a0 > 0.6 * predicted && a2 / a0 < 1.6 * predicted);
    double xmax = 0;
    for (double xi : d.x) xmax = std::max(xmax, std::fabs(xi));
    // Engaged at a zero crossing, x = -gain * integral(phi) swings between 0 and 2 gain A/w0 (~2.8 cm)
    // and settles at gain A/w0 as the swing dies out.
    std::cout << "    a(2 s)/a0 = " << a2 / a0 << " (predicted " << predicted << "), max |x| = " << xmax * 1e3
              << " mm, final x = " << d.x.back() * 1e3 << " mm, c_bottom error "
              << cbErr(cal.status().c_bottom, sim.trueCBottomRelative()) << " counts\n";
    CHECK(xmax < 0.045);
    CHECK(!d.recs.back().driver_enabled);
    CHECK(sim.v() == 0.0);
    CHECK(!sim.crashed());
}

TEST(damp_with_a_sign_error_grows_the_swing_and_aborts) {
    const auto hp = plant();
    const double amp = 6 * kDeg;
    hw::SimHw sim(hp, kPi + amp);
    rt::SwingCalibrator cal(calibParams(amp));
    CHECK(calibrate(sim, cal, 5000) > 0);
    rt::DampConfig c = dampConfig(hp, cal.status().c_bottom);
    c.calib.theta_sign = -c.calib.theta_sign;                    // sign error in the configuration
    rt::DampRuntime rt(c);
    DampRun d = runDamp(sim, rt, 8000);

    CHECK(rt.phase() == rt::DampPhase::Done);
    CHECK(std::string(rt.reason()) != "completed");
    CHECK(peakPhi(d, 0, static_cast<int>(d.th.size())) > 2 * amp * 0.85);   // grew instead of decaying
    CHECK(!d.recs.back().driver_enabled);
    CHECK(!sim.crashed());
}

TEST(damp_without_a_swing_never_moves_the_cart) {
    const auto hp = plant();
    hw::SimHw sim(hp, kPi);
    rt::DampConfig c = dampConfig(hp, sim.trueCBottomRelative());
    c.wait_max = 1.0;
    rt::DampRuntime rt(c);
    DampRun d = runDamp(sim, rt, 2000);
    CHECK(rt.phase() == rt::DampPhase::Done);
    CHECK(std::string(rt.reason()) == "no swing to damp (wait_max)");
    double xmax = 0;
    for (double xi : d.x) xmax = std::max(xmax, std::fabs(xi));
    CHECK(xmax == 0.0);
    CHECK(!d.recs.back().driver_enabled);
}

TEST(damp_stop_request_brakes_and_disables) {
    const auto hp = plant();
    const double amp = 6 * kDeg;
    hw::SimHw sim(hp, kPi + amp);
    rt::SwingCalibrator cal(calibParams(amp));
    CHECK(calibrate(sim, cal, 5000) > 0);
    rt::DampRuntime rt(dampConfig(hp, cal.status().c_bottom));
    int k = 0;
    for (; k < 6000 && rt.phase() != rt::DampPhase::Done; ++k) {
        if (k == 700) rt.requestStop();
        rt.tick(sim, false);
        sim.advance(kTs);
    }
    CHECK(std::string(rt.reason()) == "stop request");
    CHECK(k < 700 + 400);                                        // stopped within 0.8 s
    CHECK(sim.v() == 0.0);
}
