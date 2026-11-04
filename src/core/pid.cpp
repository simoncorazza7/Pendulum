#include "core/pid.h"

#include <algorithm>
#include <cmath>

namespace core {
namespace {

constexpr double kPi = 3.141592653589793;   // same value as Python's math.pi

// 1 when margin >= width, linear to 0 at margin <= 0.
double taper(double margin, double width) {
    if (width <= 0) return margin > 0 ? 1.0 : 0.0;
    return std::min(1.0, std::max(0.0, margin / width));
}

}

PidResult pidStep(const PidParams& p, const PidState& s, double theta, double v_cart, double dt,
                  const double* velEst) {
    // Derivative of the measurement (not of the error), filtered.
    const double vel_raw = s.has_prev ? (theta - s.prev_theta) / dt : 0.0;
    const double alpha = dt / (dt + 1.0 / (2.0 * kPi * p.vel_fc));
    const double vel_f = velEst ? *velEst : s.vel_f + alpha * (vel_raw - s.vel_f);

    const double e = theta - p.theta_ref;
    const double u = p.kp * e + p.ki * s.integral + p.kd * vel_f;

    // Saturation with authority taper near +-v_max: no more acceleration toward the limit.
    const double w = p.v_taper * p.v_max;
    const double a_hi = p.a_max * taper(p.v_max - v_cart, w);
    const double a_lo = -p.a_max * taper(p.v_max + v_cart, w);
    const double a_cmd = std::min(a_hi, std::max(a_lo, u));
    const bool saturated = a_cmd != u;

    // Conditional integration: hold the integrator while it would push further into saturation,
    // and during the enable delay. Clamp its contribution.
    double integral = s.integral;
    if (s.t_engaged >= p.i_delay && p.ki != 0.0) {
        const bool pushes_out = saturated && (e * (u - a_cmd) > 0);
        if (!pushes_out) integral += e * dt;
        if (p.i_max > 0) {
            const double lim = p.i_max / std::fabs(p.ki);
            integral = std::min(lim, std::max(-lim, integral));
        }
    }

    PidState next;
    next.integral = integral;
    next.t_engaged = s.t_engaged + dt;
    next.vel_f = vel_f;
    next.prev_theta = theta;
    next.has_prev = true;
    return PidResult{a_cmd, next, u, e, vel_raw, saturated};
}

double cartThetaRef(const PidParams& p, double x_cart, double v_cart, double x_int, bool* clamped) {
    if (clamped) *clamped = false;
    if (p.cart_kx == 0.0 && p.cart_kv == 0.0 && p.cart_ki == 0.0) return 0.0;
    const double raw = (-p.cart_kx * x_cart - p.cart_kv * v_cart - p.cart_ki * x_int) / kGCascade;
    const double ref = std::min(p.cart_theta_max, std::max(-p.cart_theta_max, raw));
    if (clamped) *clamped = ref != raw;
    return ref;
}

PidResult cascadeStep(const PidParams& p, const PidState& s, double theta, double x_cart, double v_cart,
                      double dt, const double* velEst) {
    bool clamped = false;
    PidParams q = p;
    q.theta_ref = p.theta_ref + cartThetaRef(p, x_cart, v_cart, s.x_int, &clamped);
    PidResult r = pidStep(q, s, theta, v_cart, dt, velEst);
    double x_int = s.x_int;
    if (p.cart_ki != 0.0 && !clamped) {
        x_int += x_cart * dt;
        const double lim = kGCascade * p.cart_i_max / p.cart_ki;
        x_int = std::min(lim, std::max(-lim, x_int));
    }
    r.state.x_int = x_int;
    return r;
}

std::string validate(const PidParams& p) {
    if (!(p.kp > 0)) return "kp must be > 0 (and > g for stability)";
    if (p.ki < 0 || p.kd < 0) return "ki and kd must be >= 0";
    if (!(p.a_max > 0)) return "a_max must be > 0";
    if (!(p.v_max > 0)) return "v_max must be > 0";
    if (!(p.v_taper > 0 && p.v_taper <= 1)) return "v_taper must be in (0, 1]";
    if (p.i_max < 0 || p.i_delay < 0) return "i_max and i_delay must be >= 0";
    if (!(p.vel_fc > 0)) return "vel_fc must be > 0";
    if (!std::isfinite(p.theta_ref)) return "theta_ref must be finite";
    if (!(p.cart_kx >= 0 && p.cart_kv >= 0 && p.cart_ki >= 0)) return "cart_kx, cart_kv and cart_ki must be >= 0";
    if (!(p.cart_theta_max >= 0 && p.cart_theta_max <= 0.175)) return "cart_theta_max must be in [0, 10 deg]";
    if ((p.cart_kx > 0 || p.cart_kv > 0 || p.cart_ki > 0) && !(p.cart_theta_max > 0))
        return "cart_theta_max must be > 0 when the outer loop is on (cart_kx, cart_kv or cart_ki > 0)";
    if (!(p.cart_i_max >= 0 && p.cart_i_max <= p.cart_theta_max)) return "cart_i_max must be in [0, cart_theta_max]";
    if (p.cart_ki > 0 && !(p.cart_i_max > 0)) return "cart_i_max must be > 0 when cart_ki > 0";
    return "";
}

Gains designGains(double w0, double bJ, double wc, double zeta, double p, double g) {
    const double kp = g * (1 + (wc * wc + 2 * zeta * wc * p) / (w0 * w0));
    const double kd = (2 * zeta * wc + p - bJ) * g / (w0 * w0);
    const double ki = g * wc * wc * p / (w0 * w0);
    return {kp, ki, kd};
}

}
