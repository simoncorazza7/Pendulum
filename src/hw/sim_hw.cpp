#include "hw/sim_hw.h"

#include <algorithm>
#include <cmath>

namespace hw {
namespace {
constexpr double kPi = 3.141592653589793;
}

SimHw::SimHw(const SimHwParams& p, double theta0, double thetadot0, double x0)
    : p_(p), th_(theta0), thd_(thetadot0), x_(x0) {
    u0_floor_ = static_cast<std::int64_t>(std::floor(u(theta0)));
    c_prev_ = u0_floor_;
    edges_.reserve(4096);
}

double SimHw::u(double th) const {
    const auto& c = p_.truth;
    return c.c_bottom + c.cpr / 2.0 + c.theta_sign * th * c.cpr / (2 * kPi);
}

double SimHw::pos(double x) const {
    const auto& c = p_.truth;
    return c.pos_center + c.x_sign * x * c.usteps_per_m;
}

double SimHw::trueCBottomRelative() const { return p_.truth.c_bottom - static_cast<double>(u0_floor_); }

Sample SimHw::read() {
    const auto& c = p_.truth;
    Sample s;
    const double ps = pos(x_);
    const double speedProtocol = c.x_sign * v_ * c.usteps_per_m * c.toggles_per_ustep;
    s.t_ns = std::llround(t_ * 1e9);
    s.counts = c_prev_ - u0_floor_;
    s.n_edges = static_cast<int>(std::min<std::size_t>(edges_.size(), Sample::kMaxEdges));
    for (int i = 0; i < s.n_edges; ++i) s.edges[static_cast<std::size_t>(i)] = edges_[static_cast<std::size_t>(i)];
    s.edges_overflow = edges_.size() > static_cast<std::size_t>(Sample::kMaxEdges);
    s.pos_usteps = std::llround(ps + pos_offset_);
    s.speed_toggles = -std::llround(speedProtocol);
    s.limit_right = ps <= p_.pos_switch_right;
    s.limit_left = ps >= p_.pos_switch_left;
    s.axis_accelerating = driver_ && commanded_ && !halted_;
    s.encoder_fault = false;
    s.driver_enabled = driver_;
    edges_.clear();
    return s;
}

void SimHw::commandAccel(int axisArg) {
    const auto& c = p_.truth;
    if (halted_) return;
    a_pending_ = -c.x_sign * axisArg / (c.usteps_per_m * c.toggles_per_ustep);
    has_pending_ = true;
    commanded_ = true;
}

double SimHw::realised(double v, double h) const {
    if (!driver_) return v != 0.0 ? -std::copysign(std::min(p_.coast_decel, std::fabs(v) / h), v) : 0.0;
    if (halted_) return v != 0.0 ? -std::copysign(std::min(p_.axis_a, std::fabs(v) / h), v) : 0.0;
    const double cap = p_.axis_a * std::max(0.0, 1.0 - std::fabs(v) / p_.axis_v);
    return std::min(cap, std::max(-cap, a_applied_));
}

void SimHw::f(const double s[4], double a, double out[4]) const {
    const double w2 = p_.w0 * p_.w0;
    double acc = w2 * std::sin(s[0]) - (w2 / p_.g) * a * std::cos(s[0]) - p_.bJ * s[1];
    if (p_.coulomb != 0.0 && s[1] != 0.0) acc -= std::copysign(p_.coulomb, s[1]);
    out[0] = s[1];
    out[1] = acc;
    out[2] = s[3];
    out[3] = a;
}

void SimHw::advance(double dt) {
    const double h = dt / p_.substeps;
    const int dSub = static_cast<int>(std::lround(p_.compute_delay / h));
    for (int j = 0; j < p_.substeps; ++j) {
        if (j == dSub && has_pending_) {
            a_applied_ = a_pending_;
            has_pending_ = false;
        }
        const double a = realised(v_, h);
        const double s0[4] = {th_, thd_, x_, v_};
        double k1[4], k2[4], k3[4], k4[4], s1[4], s2[4], s3[4];
        f(s0, a, k1);
        for (int i = 0; i < 4; ++i) s1[i] = s0[i] + 0.5 * h * k1[i];
        f(s1, a, k2);
        for (int i = 0; i < 4; ++i) s2[i] = s0[i] + 0.5 * h * k2[i];
        f(s2, a, k3);
        for (int i = 0; i < 4; ++i) s3[i] = s0[i] + h * k3[i];
        f(s3, a, k4);
        double n[4];
        for (int i = 0; i < 4; ++i) n[i] = s0[i] + h / 6 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
        const double thOld = th_;
        th_ = n[0]; thd_ = n[1]; x_ = n[2]; v_ = n[3];
        if (held_) {
            const double tEnd = t_ + (j + 1) * h;
            const double frac = lift_T_ > 0 ? std::min(1.0, (tEnd - lift_t0_) / lift_T_) : 1.0;
            th_ = lift_from_ + (lift_to_ - lift_from_) * frac;
            thd_ = frac < 1.0 ? (lift_to_ - lift_from_) / lift_T_ : 0.0;
        }
        if ((halted_ || !driver_) && v_ * s0[3] < 0) v_ = 0.0;   // braking/coasting stops at 0
        const double ps = pos(x_);
        if (ps <= p_.pos_end_right || ps >= p_.pos_end_left) {     // hard stop: crash
            const double end = ps <= p_.pos_end_right ? p_.pos_end_right : p_.pos_end_left;
            x_ = (end - p_.truth.pos_center) / (p_.truth.x_sign * p_.truth.usteps_per_m);
            v_ = 0.0;
            crashed_ = true;
        }
        emitEdges(thOld, th_, t_ + j * h, h);
    }
    t_ += dt;
}

void SimHw::emitEdges(double thOld, double thNew, double t0, double h) {
    const double u0 = u(thOld), u1 = u(thNew);
    const auto cNew = static_cast<std::int64_t>(std::floor(u1));
    if (cNew == c_prev_) return;
    const bool up = cNew > c_prev_;
    for (std::int64_t b = up ? c_prev_ + 1 : c_prev_; up ? b <= cNew : b > cNew; b += up ? 1 : -1) {
        const double ts = t0 + h * (static_cast<double>(b) - u0) / (u1 - u0);
        edges_.push_back(Edge{std::llround(ts * 1e9), (up ? b : b - 1) - u0_floor_});
    }
    c_prev_ = cNew;
}

void SimHw::holdAt(double theta, double duration) {
    lift_from_ = th_;
    lift_to_ = theta;
    lift_t0_ = t_;
    lift_T_ = duration;
    held_ = true;
    if (duration <= 0) {               // instantaneous: all edges at the current time
        emitEdges(th_, theta, t_, 0.0);
        th_ = theta;
        thd_ = 0.0;
    }
}

}
