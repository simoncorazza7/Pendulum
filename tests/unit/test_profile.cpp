#include <cmath>

#include "check.h"
#include "core/profile.h"
#include "core/units.h"
#include "hw/sim_hw.h"

TEST(profile_trapezoid_and_triangle) {
    core::TrapezoidProfile tr(0.10, 0.02, 0.1);   // 10 cm at 2 cm/s, 0.1 m/s^2: trapezoid
    CHECK_NEAR(tr.peakSpeed(), 0.02, 1e-12);
    CHECK_NEAR(tr.duration(), 0.10 / 0.02 + 0.02 / 0.1, 1e-12);   // d/v + v/a
    CHECK_NEAR(tr.at(tr.duration()).x, 0.10, 1e-12);
    CHECK_NEAR(tr.at(0.5 * tr.duration()).v, 0.02, 1e-12);
    core::TrapezoidProfile tri(-0.002, 0.05, 0.1);   // 2 mm backwards: triangle
    CHECK(tri.peakSpeed() < 0.05);
    CHECK_NEAR(tri.at(tri.duration()).x, -0.002, 1e-12);
    CHECK_NEAR(tri.at(0.5 * tri.duration()).v, -tri.peakSpeed(), 1e-9);
    CHECK_EQ(tri.at(10).v, 0.0);
    // continuity: position and velocity have no jumps
    double maxJumpX = 0, maxJumpV = 0;
    for (double t = 0; t < tr.duration() + 0.01; t += 1e-4) {
        maxJumpX = std::max(maxJumpX, std::fabs(tr.at(t + 1e-4).x - tr.at(t).x));
        maxJumpV = std::max(maxJumpV, std::fabs(tr.at(t + 1e-4).v - tr.at(t).v));
    }
    CHECK(maxJumpX <= 0.02 * 1e-4 + 1e-12);
    CHECK(maxJumpV <= 0.1 * 1e-4 + 1e-12);
}

TEST(profile_tracking_on_simulated_axis_ends_on_target) {
    // The move mode's tracking law on SimHw through the raw-unit chain: ends within one microstep.
    hw::SimHwParams p;
    hw::SimHw sim(p, 3.141592653589793);
    core::Calib cal = p.truth;
    sim.enableDriver(true);
    core::TrapezoidProfile prof(0.03, 0.02, 0.1);     // the planned first move: 3 cm at 2 cm/s
    const double x0 = core::xFromPos(sim.read().pos_usteps, cal);
    double maxErr = 0;
    for (int k = 0; k < static_cast<int>((prof.duration() + 0.5) / 0.002); ++k) {
        const hw::Sample s = sim.read();
        const double x = core::xFromPos(s.pos_usteps, cal) - x0, v = core::vFromAxisSpeed(s.speed_toggles, cal);
        const core::ProfilePoint ref = prof.at(k * 0.002);
        maxErr = std::max(maxErr, std::fabs(ref.x - x));
        sim.commandAccel(core::axisAccelArg(core::trackAccel(ref, x, v, 25.0, 10.0, 0.3), cal));
        sim.advance(0.002);
    }
    const hw::Sample s = sim.read();
    CHECK(std::fabs(core::xFromPos(s.pos_usteps, cal) - x0 - 0.03) <= 1.0 / cal.usteps_per_m + 1e-9);
    CHECK(std::fabs(core::vFromAxisSpeed(s.speed_toggles, cal)) < 1e-3);
    CHECK(maxErr < 0.5e-3);                            // tracks within 0.5 mm
    std::cout << "    3 cm move: max tracking error " << maxErr * 1e3 << " mm\n";
}
