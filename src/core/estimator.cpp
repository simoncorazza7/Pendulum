#include "core/estimator.h"

#include <cmath>

namespace core {
namespace {
constexpr double kPi = 3.141592653589793;
}

VelocityEstimator::VelocityEstimator(Kind kind, double ts, double fc) : kind_(kind), ts_(ts) {
    alpha_ = ts / (ts + 1.0 / (2 * kPi * fc));
    const double k = std::tan(kPi * fc * ts);
    const double n = 1 / (1 + std::sqrt(2.0) * k + k * k);
    b0_ = k * k * n;
    b1_ = 2 * b0_;
    b2_ = b0_;
    a1_ = 2 * (k * k - 1) * n;
    a2_ = (1 - std::sqrt(2.0) * k + k * k) * n;
}

void VelocityEstimator::reset() {
    has_prev_ = false;
    v_ = x1_ = x2_ = y1_ = y2_ = 0;
}

double VelocityEstimator::update(std::int64_t counts) {
    const double x = has_prev_ ? static_cast<double>(counts - prev_) / ts_ : 0.0;
    has_prev_ = true;
    prev_ = counts;
    if (kind_ == Kind::BdLp1) {
        v_ += alpha_ * (x - v_);
        return v_;
    }
    const double y = b0_ * x + b1_ * x1_ + b2_ * x2_ - a1_ * y1_ - a2_ * y2_;
    x2_ = x1_; x1_ = x;
    y2_ = y1_; y1_ = y;
    return y;
}

}
