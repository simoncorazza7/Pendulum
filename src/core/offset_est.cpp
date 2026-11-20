#include "core/offset_est.h"

#include <algorithm>
#include <cmath>

namespace core {

std::string validate(const OffsetParams& p) {
    if (!(p.w0 > 0)) return "offset: w0 must be > 0";
    if (!(p.bJ >= 0)) return "offset: bJ must be >= 0";
    if (!(p.ts > 0)) return "offset: ts must be > 0";
    if (!(p.g > 0)) return "offset: g must be > 0";
    if (!(p.t_settle >= 0)) return "offset: t_settle must be >= 0";
    if (!(p.theta_max > 0)) return "offset: theta_max must be > 0";
    if (!(p.lag >= p.ts)) return "offset: lag must be >= ts";
    if (!(p.smooth >= p.ts)) return "offset: smooth must be >= ts";
    if (!(p.step > 0)) return "offset: step must be > 0";
    if (!(p.min_stretch > 0)) return "offset: min_stretch must be > 0";
    if (!(p.delta_max > 0)) return "offset: delta_max must be > 0";
    return "";
}

DriftIdentityEstimator::DriftIdentityEstimator(const OffsetParams& p, double prior, double prior_weight)
    : p_(p), L_(p.g / (p.w0 * p.w0)),
      nlag_(static_cast<std::size_t>(std::max(1L, std::lround(p.lag / p.ts)))),
      nsm_(static_cast<std::size_t>(std::max(1L, std::lround(p.smooth / p.ts)))),
      pool_ws_(prior_weight * p.g * prior), pool_w_(prior_weight) {
    buf_.init(nlag_ + 1);            // push, then pop above nlag
    sm_.init(nsm_ + 1);
    sm_hist_.init(nlag_ + 2);
    resetStretch(false, 0.0, 0.0, 0.0);
}

void DriftIdentityEstimator::resetStretch(bool open, double theta, double omega, double v) {
    open_ = open;
    if (open) { th_s_ = theta; om_s_ = omega; v_s_ = v; }
    k_ = 0;
    integ_ = 0.0;
    th_prev_ = open ? theta : 0.0;
    buf_.clear();
    sm_.clear();
    sm_sum_ = 0.0;
    sm_hist_.clear();
    n_ = 0;
    st_ = sr_ = stt_ = str_ = 0.0;
}

bool DriftIdentityEstimator::stretchSlope(double* sl) const {
    if (stretchLen() < p_.min_stretch || n_ < 3) return false;
    const double n = static_cast<double>(n_);
    const double den = stt_ - st_ * st_ / n;
    if (den <= 0) return false;
    *sl = (str_ - st_ * sr_ / n) / den;
    return true;
}

void DriftIdentityEstimator::close() {
    if (open_) {
        double sl;
        if (stretchSlope(&sl)) {
            const double T = stretchLen();
            pool_ws_ += T * sl;
            pool_w_ += T;
            ++n_stretches_;
        }
    }
    resetStretch(false, 0.0, 0.0, 0.0);
}

void DriftIdentityEstimator::endBalance() { close(); }

void DriftIdentityEstimator::step(double t_engaged, double theta, double omega, double v, bool balancing) {
    const bool ok = balancing && t_engaged >= p_.t_settle && std::fabs(theta) < p_.theta_max;
    if (!ok) {
        if (open_) close();
        return;
    }
    if (!open_) {
        resetStretch(true, theta, omega, v);
        return;
    }
    ++k_;
    integ_ += 0.5 * (theta + th_prev_) * p_.ts;
    th_prev_ = theta;
    const double t = static_cast<double>(k_) * p_.ts;
    const double r = (v - v_s_) - p_.g * integ_ + L_ * (omega - om_s_) + L_ * p_.bJ * (theta - th_s_);
    // step detector on the moving mean of the residual, against the current slope estimate
    sm_.push(r);
    sm_sum_ += r;
    if (sm_.n > nsm_) sm_sum_ -= sm_.pop();
    const double m = sm_sum_ / static_cast<double>(sm_.n);
    sm_hist_.push(m);
    if (sm_hist_.n > nlag_ + 1) sm_hist_.pop();
    if (sm_hist_.n == nlag_ + 1 && sm_.n == nsm_) {
        const double jump = m - sm_hist_.front() - slope() * p_.lag;
        if (std::fabs(jump) > p_.step) {
            close();                                    // the push's samples are discarded
            resetStretch(true, theta, omega, v);        // a new stretch, new intercept
            return;
        }
    }
    // commit late
    buf_.push(Sample{t, r});
    if (buf_.n > nlag_) {
        const Sample c = buf_.pop();
        ++n_;
        st_ += c.t;
        sr_ += c.r;
        stt_ += c.t * c.t;
        str_ += c.t * c.r;
    }
}

double DriftIdentityEstimator::slope() const {
    double ws = pool_ws_, w = pool_w_, sl;
    if (open_ && stretchSlope(&sl)) {
        const double T = stretchLen();
        ws += T * sl;
        w += T;
    }
    return w > 0 ? ws / w : 0.0;
}

double DriftIdentityEstimator::weight() const {
    double w = pool_w_, sl;
    if (open_ && stretchSlope(&sl)) w += stretchLen();
    return w;
}

double DriftIdentityEstimator::deltaHat() const {
    const double d = slope() / p_.g;
    return std::max(-p_.delta_max, std::min(p_.delta_max, d));
}

}
