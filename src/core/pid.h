#ifndef PENDEL_CORE_PID_H
#define PENDEL_CORE_PID_H

#include <cstddef>
#include <string>
#include <vector>

// Balance controller core (Stage 1). Pure and deterministic: params + state + measurement + dt
// -> command + new state. No I/O, no clock, no allocation after construction. The same code runs
// in simulation, on the Pi and later on a PC.
//
// Mirrors py/pendel/control.py operation for operation; tests/golden/ checks that the two agree
// to floating-point tolerance.
//
// Sign convention (docs/model_derivation.md): theta > 0 when the top leans toward +x; a > 0
// accelerates the cart toward +x; positive gains move the cart under the pendulum.
namespace core {

struct PidParams {
    double kp = 0;           // (m/s^2)/rad
    double ki = 0;           // (m/s^2)/(rad*s)
    double kd = 0;           // (m/s^2)/(rad/s)
    double a_max = 0;        // m/s^2, command saturation
    double v_max = 0;        // m/s, cart speed limit (authority toward it tapers to 0)
    double v_taper = 0.25;   // fraction of v_max over which that taper happens
    double i_max = 0;        // m/s^2, clamp of |ki * integral| (0: no clamp)
    double i_delay = 0;      // s after engage before the integrator runs
    double theta_ref = 0;    // rad, upright reference trim
    double vel_fc = 30;      // Hz, 1st-order low-pass on the backward-difference velocity
};

struct PidState {
    double integral = 0;     // rad*s
    double t_engaged = 0;    // s since engage
    double vel_f = 0;        // rad/s, filtered velocity estimate
    double prev_theta = 0;   // rad, previous measured angle (valid if has_prev)
    bool has_prev = false;
};

struct PidResult {
    double a_cmd;            // m/s^2, saturated command
    PidState state;          // state for the next sample
    double u;                // m/s^2, unsaturated controller output
    double e;                // rad, error theta - theta_ref
    double vel_raw;          // rad/s, backward difference
    bool saturated;
};

// theta: measured angle from upright, unwrapped (rad). v_cart: realised cart velocity (m/s),
// only used for the v_max taper. dt: sample period (s).
PidResult pidStep(const PidParams& p, const PidState& s, double theta, double v_cart, double dt);

// Empty string if the parameters are usable, otherwise the reason.
std::string validate(const PidParams& p);

struct Gains { double kp, ki, kd; };
// Pole placement (docs/model_derivation.md §6): poles at the roots of
// (s^2 + 2 zeta wc s + wc^2)(s + p). p = 0 gives PD.
Gains designGains(double w0, double bJ, double wc, double zeta, double p, double g = 9.81);

// N-sample delay on the command path (Stage 2a). N = 0 passes through. The first N outputs are 0.
class DelayLine {
public:
    explicit DelayLine(std::size_t n) : buf_(n, 0.0) {}
    double push(double a) {
        if (buf_.empty()) return a;
        double out = buf_[head_];
        buf_[head_] = a;
        head_ = (head_ + 1) % buf_.size();
        return out;
    }
    std::size_t length() const { return buf_.size(); }

private:
    std::vector<double> buf_;
    std::size_t head_ = 0;
};

}

#endif
