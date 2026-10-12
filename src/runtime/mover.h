#ifndef PENDEL_RUNTIME_MOVER_H
#define PENDEL_RUNTIME_MOVER_H

#include <cstdint>

#include "core/profile.h"
#include "core/units.h"
#include "hw/hardware.h"

// Cart moves for the balance session (homing, re-centring), on any hw::Hardware. Portable: the
// caller reads the sample once per tick and passes it in.
//
// A segment drives from the current position toward `target` (Axis microsteps, raw frame) on a
// trapezoid whose acceleration phases last exactly one pendulum period T0 = 2 pi / w0: a hanging
// pendulum driven by a constant acceleration for a whole period ends that phase at rest, so a move
// leaves no swing behind (linear model; docs/stage1_demo.md). Tracked with core::trackAccel on the
// Axis's own position and speed, as in pendctl --mode move.
//   seek segment: ends when its switch closes; the sample's pos is the switch position. Reaching
//                 the target first = switch not found (fails), unless the segment only watches
//                 for the switch (mustFind = false: ends at the target, or earlier at the switch).
//   move segment: ends at the target.
// Aborts (fail): any other switch, a switch pressed at the start that is still pressed after
// release_within of travel (driving into the end), tracking error, Axis not in ACCELERATE, overrun
// streak, stop request. Every segment ends with a brake (a = -v/ts, |a| <= a_brake) until the Axis
// speed is exactly 0 for 50 ms, so the last command it keeps executing is a = 0.
namespace rt {

struct MoverConfig {
    double ts = 0.002;              // s
    core::Calib calib;              // usteps_per_m, toggles_per_ustep, x_sign (c_bottom, pos_center unused)
    double w0 = 4.931;              // rad/s; acceleration phases last 2 pi / w0
    double a_max = 0.3;             // m/s^2, cap on the profile acceleration (shaping normally uses less)
    double a_brake = 1.0;           // m/s^2, stop at a switch and on aborts
    double kp = 25.0;               // 1/s^2, tracking gain (position)
    double kv = 10.0;               // 1/s, tracking gain (velocity)
    double track_tol = 0.003;       // m, |x_ref - x| above this aborts
    double release_within = 0.01;   // m
    int overrun_streak = 3;
};

enum class Side { None = 0, Right = 1, Left = 2 };   // Right = switch at the low-pos end (GPIO2)

enum class MoverState { Idle = 0, Moving = 1, Braking = 2, Done = 3, Failed = 4 };

struct MoverRecord {
    double x_ref = 0;     // m, reference relative to the segment start (x frame)
    double v_ref = 0;     // m/s
    double a_cmd = 0;     // m/s^2 (x frame)
    int axis_arg = 0;
    int state = 0;        // MoverState after this tick
    bool sent = false;
};

class Mover {
public:
    explicit Mover(const MoverConfig& c);

    // Starts a segment from the current sample. v: cruise speed (m/s, > 0).
    void start(std::int64_t target, double v, Side seek, const hw::Sample& s, bool mustFind = true);
    // One control period: commands the hardware (always, while busy). stop: operator request.
    MoverRecord tick(hw::Hardware& hw, const hw::Sample& s, bool overrun, bool stop);

    MoverState state() const { return state_; }
    bool busy() const { return state_ == MoverState::Moving || state_ == MoverState::Braking; }
    bool found() const { return found_; }              // the seek switch closed
    std::int64_t switchPos() const { return sw_pos_; } // pos at the first sample with it closed
    const char* reason() const { return reason_; }     // why it failed ("" otherwise)
    double duration() const { return prof_.duration(); }

    // Shaped profile parameters for a move of |d| m at cruise speed v (exposed for tests):
    // returns the acceleration; *v_peak the reached speed.
    static double shapedAccel(double d, double v, double w0, double a_max, double* v_peak);

private:
    void fail(const char* why);

    MoverConfig c_;
    MoverState state_ = MoverState::Idle;
    core::TrapezoidProfile prof_{0.0, 1.0, 1.0};
    std::int64_t pos0_ = 0, sw_pos_ = 0;
    Side seek_ = Side::None;
    bool found_ = false, failing_ = false, sent_prev_ = false, must_find_ = true;
    bool ignore_left_ = false, ignore_right_ = false;
    double t_ = 0, a_lim_ = 0;
    int overruns_ = 0, still_ = 0, brake_ticks_ = 0;
    const char* reason_ = "";
};

}

#endif
