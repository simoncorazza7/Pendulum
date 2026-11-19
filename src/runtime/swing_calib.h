#ifndef PENDEL_RUNTIME_SWING_CALIB_H
#define PENDEL_RUNTIME_SWING_CALIB_H

#include <cstdint>
#include <vector>

#include "core/swingfit.h"
#include "hw/hardware.h"

// Start-up upright reference (Stage 0.7 procedure, run at every program start: the encoder has no
// index). The operator gives the hanging pendulum a small swing and lets go; this collects the edge
// crossings and repeatedly fits u = c + e^{-sigma t}(a cos wt + b sin wt) (core::fitOffset, sigma
// from identification) over the last `window` seconds, with the pendulum's amplitude-dependent
// frequency w = w0 (1 - A^2/16) for the window's amplitude A (rad). On the 2026-10-09 free decay
// (exact edges) this keeps c within ~0.13 counts up to 20 deg (fixed w: 0.6 counts at 20-30 deg).
// It accepts c_bottom when
//   - the swing amplitude is within [amp_min, amp_max],
//   - the residual RMS is below rms_max (a hand on the pendulum breaks the free-swing model),
//   - the standard error of c is below se_max, and
//   - two consecutive fits (`interval` apart) agree within `agree`.
// Portable (no clock or I/O): feed every hw::Sample in order. Raw units: counts, ns.
namespace rt {

struct SwingCalibParams {
    double w0 = 4.931;         // rad/s, small-angle hanging frequency (config plant.w0)
    double sigma = 0.03;       // 1/s, small-swing decay rate (calib.swing_sigma)
    double window = 3.0;       // s of edges per fit (~2.3 periods)
    double interval = 0.5;     // s between fits
    double amp_min = 13.0;     // counts (2 deg)
    double amp_max = 133.3;    // counts (20 deg)
    double rms_max = 2.0;      // counts (free decay: <= 1.2 up to 20 deg; hand contact: 7-27)
    double se_max = 0.05;      // counts
    double agree = 0.2;        // counts, consecutive fits
    int cpr = 2400;            // counts/rev (amplitude in rad for the frequency correction)
};

struct SwingCalibStatus {
    bool done = false;
    double c_bottom = 0;       // counts (continuous position of the hanging equilibrium), valid if done
    core::OffsetFit fit{false, 0, 0, 0};   // last fit
    double amplitude = 0;      // counts, half the peak-to-peak range in the last window
    double w = 0;              // rad/s, frequency used in the last fit
    int n_edges = 0;           // edges in the last window
    const char* why = "waiting for edges";   // reason the last fit was not accepted
};

class SwingCalibrator {
public:
    explicit SwingCalibrator(const SwingCalibParams& p);

    // Feed one sample (its edges, oldest first). Fits at most every `interval`. Returns status().done.
    bool update(const hw::Sample& s);
    const SwingCalibStatus& status() const { return st_; }
    const SwingCalibParams& params() const { return p_; }

private:
    void fitNow(std::int64_t t_ns);

    SwingCalibParams p_;
    SwingCalibStatus st_;
    std::int64_t prev_count_ = 0;             // counts start at 0 at program start
    std::vector<std::int64_t> t_;             // edge times, ns (pruned to the window)
    std::vector<double> u_;                   // edge positions, counts
    std::vector<double> tf_;                  // fit scratch: s from the first edge in the window
    std::int64_t next_fit_ns_ = 0;
    bool has_prev_fit_ = false;
    double prev_c_ = 0;
};

}

#endif
