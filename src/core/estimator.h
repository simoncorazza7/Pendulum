#ifndef PENDEL_CORE_ESTIMATOR_H
#define PENDEL_CORE_ESTIMATOR_H

#include <cstdint>

// Velocity estimator on cumulative encoder counts (Stage 0.11 candidates (a)), mirror of
// py/pendel/estimators.py BdLp1 / BdLp2. Output in counts/s; multiply by core::omegaScale() for
// rad/s. Pure state machine: one update() per sample, no allocation.
namespace core {

class VelocityEstimator {
public:
    enum class Kind { BdLp1, BdLp2 };   // backward difference + 1st-order / 2nd-order Butterworth LP

    VelocityEstimator(Kind kind, double ts, double fc);   // ts: s, fc: Hz
    double update(std::int64_t counts);                   // counts/s
    void reset();

private:
    Kind kind_;
    double ts_;
    double alpha_;                                        // BdLp1
    double b0_, b1_, b2_, a1_, a2_;                       // BdLp2
    bool has_prev_ = false;
    std::int64_t prev_ = 0;
    double v_ = 0, x1_ = 0, x2_ = 0, y1_ = 0, y2_ = 0;
};

}

#endif
