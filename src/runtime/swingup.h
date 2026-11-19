#ifndef PENDEL_RUNTIME_SWINGUP_H
#define PENDEL_RUNTIME_SWINGUP_H

#include <atomic>
#include <cstdint>

#include "core/estimator.h"
#include "core/pid.h"
#include "core/supervisor.h"
#include "core/units.h"
#include "hw/hardware.h"
#include "runtime/remote.h"

// Energy-based swing-up and hand-over to the balance PID (brief Stage 4, pulled forward on
// 2026-10-14; docs/swingup_notes.md). One run from the hanging pendulum, on any hw::Hardware; portable.
// The caller passes the sample each tick (one read per tick, as rt::Mover) and owns the driver enable.
//
// Energy per unit inertia, zero at upright rest: E = theta'^2/2 + w0^2 (cos theta - 1), hanging: -2 w0^2;
// dE/dt = -(w0^2/g) a theta' cos theta - (b/J) theta'^2. theta' comes from the BD+LP2 estimator, which lags
// ~16 ms at 14 Hz; mixing it with the raw angle makes E wobble by up to ~8 % within a swing, while the catch
// needs E within ~0.3 %. The estimate is exactly the derivative of LP2(theta), so E and the switching use the
// angle through the same LP2 (theta_f): the energy of the trajectory 16 ms ago, which is constant along a swing.
// Phases:
//   KICK      a = +a_pump for kick_time: the law below cannot start from rest (theta' = 0)
//   PUMP      a = sat_{a_pump}(k_e (E - E_ref) sign(theta' cos theta)) - kx x - kv x' (Astrom-Furuta + slow
//             cart centring); E_ref a hair above upright. Hand-over to BALANCE when |theta| < theta_sw and
//             |theta + theta'/w0| < u_sw (arriving along the stable manifold) and the cart speed predicted after
//             the catch |v + (g/w0)(theta + theta'/w0)| < v_sw (Stage 3b gate); integrator pre-loaded theta'/w0^2.
//   BALANCE   core::cascadeStep (the balance gains, a_max, v_max taper, plus the Stage 3/3b cart loop; with the cart
//             gains at 0 it is the Stage 1 angle-only PID, and the cart keeps its hand-over velocity until the soft
//             limit). With `remote` (Stage 2b, docs/remote.md)
//             the command is the PC's, computed from the sample remote_delay ticks earlier (the local PID runs as a
//             shadow, a_local); a late reply holds the previous command; remote_max_miss in a row, or the link
//             going down, -> BRAKE ("remote link"). The pre-load goes to the PC as the epoch's i0.
//   BRAKE     brake the cart to standstill, then DAMP_OUT (the pendulum must not be left near upright energy)
//   DAMP_OUT  the pumping law toward the hanging energy, capped at a_damp, with the same centring, until the
//             swing is below quiet_deg; then a final brake -> DONE
//   STOPPING  hard stop: brake to standstill -> DONE (the pendulum keeps whatever energy it has)
// Ends of a run: PUMP: energy above e_abort, pump_max, operator go, remote link down -> DAMP_OUT. BALANCE:
// |theta| > theta_abort, soft limit, balance_max, operator go, remote link -> BRAKE -> DAMP_OUT. Hard (any phase):
// limit switch, encoder fault, Axis not in ACCELERATE, overrun streak, operator quit, soft limit during KICK /
// DAMP_OUT -> STOPPING.
namespace rt {

enum class SwingPhase { Kick = 0, Pump = 1, Balance = 2, Brake = 3, DampOut = 4, Stopping = 5, Done = 6 };
const char* swingPhaseName(SwingPhase p);

struct SwingUpConfig {
    double ts = 0.002;                  // s
    core::Calib calib;                  // c_bottom (incl. trim) and pos_center set by the caller
    core::VelocityEstimator::Kind est_kind = core::VelocityEstimator::Kind::BdLp2;
    double est_fc = 14.0;               // Hz
    double w0 = 4.931;                  // rad/s
    double g = 9.81;                    // m/s^2
    core::PidParams pid;                // balance after the hand-over (its a_max, v_max, v_taper also cap pumping)
    core::SupervisorParams limits;      // x_min/x_max, a_brake, t_react, brake_margin, axis_a/v, theta_abort,
                                        // overrun_streak, stop_timeout (other fields unused)
    double a_pump = 0.75;               // m/s^2, cap of the pumping term (cart swing ~ +-52 mm per m/s^2)
    double a_damp = 0.5;                // m/s^2, cap of the energy-removal term
    double k_e = 1.0;                   // (m/s^2)/(rad^2/s^2): bang-bang until E is within ~1.5 % of the target
    double e_ref = 0.001;               // fraction of 2 w0^2 above the upright energy (0 = exactly upright)
    double e_abort = 0.10;              // fraction of 2 w0^2 above upright: too much energy -> DAMP_OUT
    double kx = 4.0;                    // 1/s^2, cart centring while pumping / damping (2 rad/s, below w0)
    double kv = 2.8;                    // 1/s
    double kick_time = 0.25;            // s
    double theta_sw = 0.17453292519943295;   // rad (10 deg)
    double u_sw = 0.10471975511965977;       // rad (6 deg), |theta + theta'/w0|
    double v_sw = 0.0;                  // m/s, and |v + (g/w0)(theta + theta'/w0)| < v_sw: the predicted cart speed
                                        // after the catch (each rad of u costs L_eq w0 = g/w0); 0: no speed gate
    double pump_max = 40.0;             // s in KICK + PUMP
    double balance_max = 30.0;          // s in BALANCE (normal end, then energy out)
    double damp_max = 25.0;             // s in DAMP_OUT, then a plain stop
    double quiet_deg = 10.0;            // deg, swing amplitude that ends DAMP_OUT
    bool preload = true;                // integrator pre-load theta'/w0^2 at the hand-over
    // Stage 2b: BALANCE runs on the PC (nullptr: locally). As BalanceConfig's remote fields.
    RemoteSource* remote = nullptr;
    int remote_delay = 1;               // ticks (>= 1)
    int remote_max_miss = 5;            // consecutive BALANCE ticks without the command
    std::uint32_t epoch_base = 0;       // the hand-over starts epoch epoch_base + 1 (the PC resets its PID)
};

struct SwingRecord {
    int phase = 0;                      // SwingPhase after this tick
    double theta = 0, omega = 0;        // rad from upright (wrapped), rad/s
    double theta_f = 0;                 // rad, theta through the estimator's LP2 (energy, switching)
    double x = 0, v = 0;                // m, m/s
    double energy = 0;                  // 0 = hanging at rest, 1 = upright at rest (E / (2 w0^2) + 1)
    double a_cmd = 0;                   // m/s^2
    int axis_arg = 0;
    double integral = 0;
    bool saturated = false, sent = false;
    bool controller = false;            // the balance PID ran on this sample (BALANCE): the PC computes it
    std::uint32_t epoch = 0;            // epoch_base, epoch_base + 1 from the hand-over
    double i0 = 0;                      // rad*s, the integrator the epoch started with (the pre-load)
    double a_local = 0;                 // m/s^2, remote: the local (shadow) PID's output
    bool remote_miss = false;           // remote, BALANCE: the command for this tick had not arrived (held)
};

class SwingUpRuntime {
public:
    explicit SwingUpRuntime(const SwingUpConfig& c);

    SwingRecord tick(hw::Hardware& hw, const hw::Sample& s, bool overrun);
    void requestDampOut() { damp_req_ = true; }   // operator "go": end gracefully (energy out)
    void requestStop() { stop_req_ = true; }      // operator quit: hard stop

    SwingPhase phase() const { return phase_; }
    bool done() const { return phase_ == SwingPhase::Done; }
    const char* reason() const { return reason_; }      // why PUMP / BALANCE ended
    bool handedOver() const { return t_handover_ >= 0; }
    double handoverTime() const { return t_handover_; } // s from the start (-1: never)
    double balanceTime() const { return t_balance_; }   // s in BALANCE
    double maxAbsX() const { return max_x_; }
    double peakV() const { return peak_v_; }
    double peakEnergy() const { return peak_e_; }       // 0..1+ (1 = upright)

private:
    void enter(SwingPhase p, const char* why = nullptr);
    double centring(double x, double v) const { return -c_.kx * x - c_.kv * v; }
    double limitPump(double a, double v) const;

    SwingUpConfig c_;
    core::VelocityEstimator est_;
    double b0_ = 0, b1_ = 0, b2_ = 0, a1_ = 0, a2_ = 0;          // LP2 on counts, same as the estimator
    double x1_ = 0, x2_ = 0, y1_ = 0, y2_ = 0;
    std::int64_t c0_ = 0;
    bool has_c0_ = false;
    core::PidState pid_;
    std::uint32_t epoch_ = 0;
    double i0_ = 0, a_hold_ = 0;                                  // remote: the epoch's i0, last applied command
    int misses_ = 0;                                              // remote: consecutive BALANCE ticks without it
    SwingPhase phase_ = SwingPhase::Kick, after_brake_ = SwingPhase::DampOut;
    std::atomic<bool> damp_req_{false}, stop_req_{false};
    const char* reason_ = "";
    bool sent_prev_ = false;
    int overruns_ = 0, still_ = 0;
    double t_ = 0, t_phase_ = 0, t_handover_ = -1, t_balance_ = 0;
    double max_x_ = 0, peak_v_ = 0, peak_e_ = 0;
};

}

#endif
