#ifndef PENDEL_CORE_OFFSET_EST_H
#define PENDEL_CORE_OFFSET_EST_H

#include <cstddef>
#include <string>
#include <vector>

// Upright-offset estimate from the drift identity, carried between catches (docs/offset_estimator.md).
// Mirrors py/offset/identity.py DriftIdentityEstimator operation for operation; the golden test
// (tests/unit/test_offset_est.cpp, tests/golden/offset_*.csv from py/offset_golden.py) holds them together.
//
// Along any balance, whatever the controller does:
//   r(t) = (v - v_s) - g int theta dt + L_eq (omega - omega_s) + L_eq (b/J)(theta - theta_s) = g delta (t - t_s) + noise
// with delta = theta_true - theta the upright reference error (rad) and _s the start of a stretch. A stretch starts
// t_settle after engage and ends when |theta| reaches theta_max, when the balance ends, or at a push: the residual's
// `smooth` s mean moves by more than `step` within `lag` s beyond the fitted slope. Samples are committed `lag` s late,
// so a push, a fall or an abort discards the samples that saw it. Every stretch with >= min_stretch s committed adds
// its slope; the estimate is the length-weighted mean of the slopes (plus an optional prior), / g, clamped to
// +-delta_max. The pool persists between balances: that is the carry. Fixed-size buffers: step() never allocates.
namespace core {

struct OffsetParams {
    double w0 = 4.97;                          // rad/s, plant.w0
    double bJ = 0.054;                         // 1/s, plant.bJ
    double ts = 0.002;                         // s, loop.ts
    double g = 9.81;                           // m/s^2
    double t_settle = 1.5;                     // s after engage before a stretch may start
    double theta_max = 0.05235987755982989;    // rad (3 deg): |theta| at or above this ends the stretch
    double lag = 0.3;                          // s, commit delay = step-detection window
    double smooth = 0.25;                      // s, moving mean of the residual for the step detector
    double step = 0.03;                        // m/s, residual step that marks a push
    double min_stretch = 1.0;                  // s committed before a stretch's slope counts
    double delta_max = 0.013089969389957471;   // rad (5 counts at 2400/rev), clamp of the estimate
};

// Empty if usable, else what is wrong.
std::string validate(const OffsetParams& p);

class DriftIdentityEstimator {
public:
    // prior: rad, an offset known from elsewhere; prior_weight: s of quiet balance it counts as (0 = none).
    explicit DriftIdentityEstimator(const OffsetParams& p = OffsetParams{}, double prior = 0.0, double prior_weight = 0.0);

    // One control tick. t_engaged: s since engage (or hand-over); theta: measured angle from upright (rad);
    // omega: the loop's rate estimate (rad/s); v: cart speed (m/s); balancing: the balance controller is in charge.
    void step(double t_engaged, double theta, double omega, double v, bool balancing);
    // The balance ended (abort, soft limit, operator, end of the swing-up balance): close the open stretch.
    void endBalance();

    double slope() const;           // m/s^2, estimate of g delta
    double weight() const;          // s of quiet balance (and prior) behind it
    double deltaHat() const;        // rad, theta_true ~ theta + deltaHat
    int stretches() const { return n_stretches_; }
    const OffsetParams& params() const { return p_; }

private:
    // Fixed-capacity FIFO (capacity set once in the constructor).
    template <class T>
    struct Ring {
        std::vector<T> buf;
        std::size_t head = 0, n = 0;
        void init(std::size_t cap) { buf.assign(cap, T{}); head = 0; n = 0; }
        void clear() { head = 0; n = 0; }
        void push(const T& x) { buf[(head + n) % buf.size()] = x; ++n; }
        T pop() { T x = buf[head]; head = (head + 1) % buf.size(); --n; return x; }
        const T& front() const { return buf[head]; }
    };
    struct Sample { double t = 0, r = 0; };

    void resetStretch(bool open, double theta, double omega, double v);
    void close();
    double stretchLen() const { return static_cast<double>(n_) * p_.ts; }
    bool stretchSlope(double* sl) const;

    OffsetParams p_;
    double L_;                       // m, g / w0^2
    std::size_t nlag_, nsm_;
    double pool_ws_, pool_w_;        // sum of T_j slope_j (m/s) and of T_j (s), prior included
    int n_stretches_ = 0;
    // open stretch
    bool open_ = false;
    double th_s_ = 0, om_s_ = 0, v_s_ = 0;
    long k_ = 0;                     // ticks since the stretch start
    double integ_ = 0, th_prev_ = 0;
    Ring<Sample> buf_;               // not yet committed
    Ring<double> sm_;                // last nsm residuals
    double sm_sum_ = 0;
    Ring<double> sm_hist_;           // moving means of the last nlag + 1 ticks
    long n_ = 0;                     // committed: n, sum t, sum r, sum t^2, sum t r
    double st_ = 0, sr_ = 0, stt_ = 0, str_ = 0;
};

}

#endif
