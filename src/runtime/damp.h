#ifndef PENDEL_RUNTIME_DAMP_H
#define PENDEL_RUNTIME_DAMP_H

#include <atomic>
#include <cstdint>

#include "core/estimator.h"
#include "core/supervisor.h"
#include "core/units.h"
#include "hw/hardware.h"

// Hanging active-damping test (brief §6, Stage 1 hardware step 2), on any hw::Hardware.
//
// Around the hanging position, with phi = wrap(theta - pi), the linearised model is
//     phi'' = -w0^2 phi - (b/J) phi' + (w0^2/g) a          (input gain sign flipped vs upright)
// and the law a = -gain * phi' gives phi'' = -w0^2 phi - (b/J + gain w0^2/g) phi', i.e. a damping
// ratio zeta_h with gain = (2 zeta_h w0 - b/J) g / w0^2 (docs/stage1_damp.md). Physically: the cart
// follows the bob. A sign error anywhere (theta_sign, x_sign, Axis direction) turns the damping into
// anti-damping, the swing grows, and the phi/v/soft-limit aborts stop it: the test is self-checking.
//
// Sequence: WAIT (driver on, a = 0) -> ENGAGED at a zero crossing of phi with |phi'| above
// engage_omega_min (so the cart's velocity v = -gain (phi - phi_engage) ends near 0) -> after
// `duration` or on any abort: STOPPING (brake at a_brake) -> DONE (driver off).
namespace rt {

enum class DampPhase { Wait = 0, Engaged = 1, Stopping = 2, Done = 3 };

struct DampConfig {
    double ts = 0.002;                     // s
    core::Calib calib;                     // c_bottom from the start-up swing fit
    core::VelocityEstimator::Kind est_kind = core::VelocityEstimator::Kind::BdLp1;
    double est_fc = 30.0;                  // Hz
    double gain = 0.0;                     // (m/s^2)/(rad/s): a = -gain * phi'
    double a_max = 1.0;                    // m/s^2, command saturation
    double engage_omega_min = 0.15;        // rad/s at the zero crossing (~1.7 deg amplitude)
    double engage_phi_max = 0.02;          // rad, |phi| at the crossing sample
    double duration = 8.0;                 // s engaged, then a normal stop
    double wait_max = 20.0;                // s in WAIT without a crossing -> done, no motion
    double phi_abort = 0.436;              // rad (25 deg)
    double v_abort = 0.3;                  // m/s
    core::SupervisorParams limits;         // x_min/x_max, a_brake, t_react, brake_margin, axis_a/v,
                                           // overrun_streak, stop_timeout (other fields unused)
};

struct DampRecord {
    std::int64_t t_ns = 0;
    std::int64_t counts = 0, pos_usteps = 0, speed_toggles = 0;
    int n_edges = 0;
    double phi = 0;           // rad from hanging
    double omega = 0;         // rad/s
    double x = 0, v = 0;      // m, m/s
    int phase = 0;            // DampPhase after this tick
    double a_cmd = 0;         // m/s^2
    int axis_arg = 0;
    bool saturated = false, overrun = false, limit_left = false, limit_right = false;
    bool axis_accelerating = false, driver_enabled = false;
};

class DampRuntime {
public:
    explicit DampRuntime(const DampConfig& c);

    DampRecord tick(hw::Hardware& hw, bool overrun);
    void requestStop() { stop_ = true; }

    DampPhase phase() const { return phase_; }
    const char* reason() const { return reason_; }      // why it stopped ("" while running)
    double engagedAt() const { return t_engage_; }      // s since the first tick (-1: never)
    double phiEngage() const { return phi_engage_; }    // rad at engage

    static double gainFor(double zeta, double w0, double bJ, double g = 9.81);

private:
    void abort(const char* why);

    DampConfig c_;
    core::VelocityEstimator est_;
    DampPhase phase_ = DampPhase::Wait;
    const char* reason_ = "";
    std::atomic<bool> stop_{false};
    bool driver_on_ = false, sent_prev_ = false, has_prev_ = false;
    double prev_phi_ = 0, t_ = 0, t_phase_ = 0, t_engage_ = -1, phi_engage_ = 0;
    int overruns_ = 0, still_ = 0;
};

}

#endif
