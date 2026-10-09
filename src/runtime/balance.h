#ifndef PENDEL_RUNTIME_BALANCE_H
#define PENDEL_RUNTIME_BALANCE_H

#include <atomic>
#include <cstdint>

#include "core/estimator.h"
#include "core/pid.h"
#include "core/supervisor.h"
#include "core/units.h"
#include "hw/hardware.h"

// One control period of the Stage 1 balance runtime, on any hw::Hardware (SimHw on the PC,
// PiHw on the rig). Portable: no threads, clocks or I/O of its own; the caller provides the fixed-
// rate loop (pendctl on the Pi, a plain for-loop in simulation) and stores the returned records.
//
// Per tick: read -> SI (core::units) -> omega estimate -> supervisor (operator requests + safety
// inputs) -> driver on/off -> controller (ENGAGED, through the Stage 2a delay line) or brake
// (STOPPING, not delayed: it stays local to the Pi) or 0 (ARMED) -> Axis argument -> command.
namespace rt {

struct BalanceConfig {
    double ts = 0.002;                         // s, control period
    core::Calib calib;                         // runtime's calibration (c_bottom from the start-up fit)
    core::PidParams pid;
    core::SupervisorParams sup;
    core::VelocityEstimator::Kind est_kind = core::VelocityEstimator::Kind::BdLp1;
    double est_fc = 30.0;                      // Hz
    int delay_samples = 0;                     // Stage 2a delay on the controller output
    bool actuate = true;                       // false = shadow mode: compute, never enable or command
    // Release detection and momentum pre-load (docs/stage1_drift.md). `released` (supervisor input) is true
    // when the two newest encoder edges step the same way, away from upright, the first one after
    // >= release_quiet s without edges (the still hold) and the newest one <= release_fresh s old.
    double release_quiet = 0.15;               // s
    double release_fresh = 0.1;                // s
    double w0 = 0.0;                           // rad/s; > 0: at engage the integrator is pre-loaded with
                                               // theta'/w0^2 from the two release edges (free-motion fit)
};

struct TickRecord {
    std::int64_t t_ns = 0;          // hardware sample time, ns
    std::int64_t counts = 0;        // raw encoder count
    std::int64_t pos_usteps = 0;    // raw Axis pos
    std::int64_t speed_toggles = 0; // raw Axis curSpeed (internal sign)
    int n_edges = 0;
    double theta = 0;               // rad from upright
    double omega = 0;               // rad/s (estimator)
    double x = 0;                   // m
    double v = 0;                   // m/s
    int mode = 0;                   // core::Mode after this tick
    int event = 0;                  // core::Reason of this tick
    double a_ctrl = 0;              // m/s^2, controller output after the delay line (ENGAGED)
    double a_cmd = 0;               // m/s^2, acceleration sent (or that would be sent in shadow mode)
    int axis_arg = 0;               // sent argument (toggles/s^2, internal sign)
    bool sent = false;              // a command was sent to the hardware this tick
    bool saturated = false;
    bool overrun = false;
    bool limit_left = false, limit_right = false;
    bool axis_accelerating = false;
    bool driver_enabled = false;
    double integral = 0;            // rad*s, controller integrator
    bool released = false;          // release detected this sample (supervisor input)
    double omega_release = 0;       // rad/s, free-motion velocity from the two release edges (at engage)
};

class BalanceRuntime {
public:
    explicit BalanceRuntime(const BalanceConfig& c);

    TickRecord tick(hw::Hardware& hw, bool overrun);

    // Operator requests, safe to call from another thread; consumed by the next tick.
    void requestArm() { arm_ = true; }
    void requestStop() { stop_ = true; }

    core::Mode mode() const { return sup_.mode; }
    const BalanceConfig& config() const { return c_; }

private:
    BalanceConfig c_;
    core::SupervisorState sup_;
    core::PidState pid_;
    core::VelocityEstimator est_;
    core::DelayLine delay_;
    std::atomic<bool> arm_{false}, stop_{false};
    bool driver_on_ = false;
    bool sent_prev_ = false;
    struct EdgeTheta { std::int64_t ts_ns; double theta; };   // edge time, boundary angle (rad)
    EdgeTheta e1_{0, 0}, e2_{0, 0};      // the two newest edges (e2_ newest)
    int n_edges_seen_ = 0;
    std::int64_t gap_e1_ = 0, gap_e2_ = 0;   // ns without edges before e1_ / e2_
    std::int64_t last_edge_ns_ = 0, prev_count_ = 0;
    bool has_count_ = false;
    void trackEdges(const hw::Sample& s);
    bool released(std::int64_t t_ns) const;
    double omegaFree(std::int64_t t_ns) const;
};

}

#endif
