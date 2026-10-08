#ifndef PENDEL_CORE_SWINGFIT_H
#define PENDEL_CORE_SWINGFIT_H

#include <cstddef>

// Start-up upright reference (Stage 0.7), mirror of swingfit.fit_offset_linear in Python.
// The encoder has no index and counts from 0 at program start, so c_bottom must be found again
// after every start: give the hanging pendulum a small swing, collect its edge crossings
// (t_i in s, u_i = continuous position in counts at the crossing), then fit
//     u = c + e^{-sigma t} (a cos(w t) + b sin(w t))
// with w, sigma known from identification (0.10): linear least squares in (c, a, b).
namespace core {

struct OffsetFit {
    bool ok;          // false if the normal equations are singular or n < 4
    double c;         // counts, equilibrium position (c_bottom)
    double se;        // counts, standard error of c
    double rms;       // counts, residual RMS
};

OffsetFit fitOffset(const double* t, const double* u, std::size_t n, double w, double sigma);

}

#endif
