#ifndef PENDEL_HW_SIM_HW_H
#define PENDEL_HW_SIM_HW_H

#include <cstdint>
#include <vector>

#include "core/units.h"
#include "hw/hardware.h"

// Simulated hardware in raw units (mirror of py/pendel/simhw.py, golden-tested).
//
// Plant: nonlinear pendulum (theta from upright, unwrapped) with viscous and Coulomb friction,
// driven by the realised cart acceleration. Cart: follows the commanded acceleration up to the
// Axis cap A(1 - |v|/V), applied compute_delay after commandAccel; HALT brakes at A; with the
// driver off the cart coasts at coast_decel (approximation: the real free cart would also be pushed
// by the pendulum). Encoder: exact edge times from the RK4 substeps; counts start at 0.
// Track: limit switches and hard stops in Axis microsteps (right = low pos, as on the rig).
// The "true" calibration (truth) may differ from the runtime's, to test calibration errors.
namespace hw {

struct SimHwParams {
    double w0 = 6.283185307179586;  // rad/s, hanging small-angle frequency
    double bJ = 0.1;                // 1/s, viscous pivot friction b/J
    double coulomb = 0.0;           // rad/s^2, Coulomb friction acceleration
    double g = 9.81;                // m/s^2
    double axis_a = 4.5;            // m/s^2, Axis cap A(1 - |v|/V) and HALT deceleration
    double axis_v = 4.5;            // m/s
    double compute_delay = 0.0005;  // s, a command takes effect this long after commandAccel
    double coast_decel = 0.5;       // m/s^2, cart deceleration while the driver is off
    int substeps = 20;              // RK4 steps per advance(dt)
    double pos_switch_right = 250;  // usteps: right switch closed for pos <= this
    double pos_switch_left = 7750;  // usteps: left switch closed for pos >= this
    double pos_end_right = 0;       // usteps: hard stop
    double pos_end_left = 8000;     // usteps: hard stop
    core::Calib truth = [] { core::Calib c; c.c_bottom = 12.37; c.pos_center = 4000.0; return c; }();
};

class SimHw : public Hardware {
public:
    SimHw(const SimHwParams& p, double theta0 = 3.141592653589793, double thetadot0 = 0.0, double x0 = 0.0);

    Sample read() override;
    void commandAccel(int axisArg) override;
    void enableDriver(bool on) override { driver_ = on; }
    void halt() override { halted_ = true; }
    bool isStopped() override { return v_ == 0.0; }

    void advance(double dt);                 // move simulated time

    // Test hook (C++ only, not in the Python mirror): a hand moves the pendulum linearly to theta
    // over `duration` s, then holds it there (thetadot 0) until release(). The encoder edges of the
    // move are emitted as it happens (an instantaneous move would overflow the edge buffer, which the
    // runtime treats as an encoder fault). The cart keeps following commands meanwhile.
    void holdAt(double theta, double duration = 0.4);
    void release() { held_ = false; }
    bool held() const { return held_; }
    // Test hook (C++ only): lost steps. From now on the reported Axis pos is the physical position
    // + usteps (the switches and hard stops stay physical), as after steps the motor did not make.
    void loseSteps(double usteps) { pos_offset_ += usteps; }
    // Test hook (C++ only): the Axis stall guard (Axis::setCommandWatchdog). Without a commandAccel for
    // `timeout` s the cart brakes to standstill at `brake` m/s^2 until the next command. 0 = off.
    void setCommandWatchdog(double timeout, double brake) { wd_timeout_ = timeout; wd_brake_ = brake; }
    int watchdogTrips() const { return wd_trips_; }

    // Ground truth for tests and logging.
    double theta() const { return th_; }     // rad from upright, unwrapped
    double thetadot() const { return thd_; }
    double x() const { return x_; }          // m (truth coordinates)
    double v() const { return v_; }
    double time() const { return t_; }
    bool crashed() const { return crashed_; }
    double trueCBottomRelative() const;      // c_bottom in the decoder's counts

private:
    double u(double th) const;               // continuous encoder position, counts (absolute)
    double pos(double x) const;              // Axis position, usteps (continuous)
    double realised(double v, double h) const;
    void f(const double s[4], double a, double out[4]) const;
    void emitEdges(double thOld, double thNew, double t0, double h);

    SimHwParams p_;
    double th_, thd_, x_, v_ = 0.0, t_ = 0.0;
    double a_applied_ = 0.0, a_pending_ = 0.0;
    bool has_pending_ = false;
    bool driver_ = false, halted_ = false, commanded_ = false, crashed_ = false, held_ = false;
    double lift_from_ = 0, lift_to_ = 0, lift_t0_ = 0, lift_T_ = 0;   // hand motion
    double pos_offset_ = 0;                                           // usteps, loseSteps()
    double wd_timeout_ = 0, wd_brake_ = 0, t_cmd_ = 0;                // stall guard (setCommandWatchdog)
    bool wd_active_ = false;
    int wd_trips_ = 0;
    std::int64_t u0_floor_, c_prev_;
    std::vector<Edge> edges_;
};

}

#endif
