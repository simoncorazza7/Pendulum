#include "core/swingfit.h"

#include <cmath>

namespace core {

OffsetFit fitOffset(const double* t, const double* u, std::size_t n, double w, double sigma) {
    OffsetFit f{false, 0, 0, 0};
    if (n < 4) return f;
    // Normal equations N p = r for basis (1, e cos wt, e sin wt); centre t for conditioning is not
    // needed at these time scales (seconds) with double precision.
    double N[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, r[3] = {0, 0, 0};
    for (std::size_t i = 0; i < n; ++i) {
        const double e = std::exp(-sigma * t[i]);
        const double phi[3] = {1.0, e * std::cos(w * t[i]), e * std::sin(w * t[i])};
        for (int j = 0; j < 3; ++j) {
            r[j] += phi[j] * u[i];
            for (int k = 0; k < 3; ++k) N[j][k] += phi[j] * phi[k];
        }
    }
    // Inverse of the symmetric 3x3 matrix by cofactors.
    const double det = N[0][0] * (N[1][1] * N[2][2] - N[1][2] * N[2][1])
                     - N[0][1] * (N[1][0] * N[2][2] - N[1][2] * N[2][0])
                     + N[0][2] * (N[1][0] * N[2][1] - N[1][1] * N[2][0]);
    if (!(std::fabs(det) > 1e-12 * N[0][0] * N[1][1] * N[2][2])) return f;
    double inv[3][3];
    inv[0][0] = (N[1][1] * N[2][2] - N[1][2] * N[2][1]) / det;
    inv[0][1] = (N[0][2] * N[2][1] - N[0][1] * N[2][2]) / det;
    inv[0][2] = (N[0][1] * N[1][2] - N[0][2] * N[1][1]) / det;
    inv[1][0] = (N[1][2] * N[2][0] - N[1][0] * N[2][2]) / det;
    inv[1][1] = (N[0][0] * N[2][2] - N[0][2] * N[2][0]) / det;
    inv[1][2] = (N[0][2] * N[1][0] - N[0][0] * N[1][2]) / det;
    inv[2][0] = (N[1][0] * N[2][1] - N[1][1] * N[2][0]) / det;
    inv[2][1] = (N[0][1] * N[2][0] - N[0][0] * N[2][1]) / det;
    inv[2][2] = (N[0][0] * N[1][1] - N[0][1] * N[1][0]) / det;
    double p[3];
    for (int j = 0; j < 3; ++j) p[j] = inv[j][0] * r[0] + inv[j][1] * r[1] + inv[j][2] * r[2];

    double ss = 0;
    for (std::size_t i = 0; i < n; ++i) {
        const double e = std::exp(-sigma * t[i]);
        const double res = u[i] - (p[0] + p[1] * e * std::cos(w * t[i]) + p[2] * e * std::sin(w * t[i]));
        ss += res * res;
    }
    const double s2 = ss / static_cast<double>(n - 3);
    f.ok = true;
    f.c = p[0];
    f.se = std::sqrt(inv[0][0] * s2);
    f.rms = std::sqrt(s2);
    return f;
}

}
