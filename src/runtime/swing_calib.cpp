#include "runtime/swing_calib.h"

#include <algorithm>
#include <cmath>

namespace rt {

SwingCalibrator::SwingCalibrator(const SwingCalibParams& p) : p_(p) {
    t_.reserve(16384);
    u_.reserve(16384);
    tf_.reserve(16384);
}

bool SwingCalibrator::update(const hw::Sample& s) {
    if (st_.done) return true;
    for (int i = 0; i < s.n_edges; ++i) {
        const hw::Edge& e = s.edges[static_cast<std::size_t>(i)];
        const std::int64_t step = e.count_after - prev_count_;
        prev_count_ = e.count_after;
        if (step == 0) continue;
        // count = floor(u): an edge that raised the count to n happened at u = n, one that lowered
        // it to n at u = n + 1 (docs/stage0_7_0_10_fits.md).
        if (t_.size() < t_.capacity()) {
            t_.push_back(e.ts_ns);
            u_.push_back(static_cast<double>(step > 0 ? e.count_after : e.count_after + 1));
        }
    }
    prev_count_ = s.counts;   // resync if edges were dropped (the runtime treats that as a fault anyway)
    if (next_fit_ns_ == 0) next_fit_ns_ = s.t_ns + static_cast<std::int64_t>(p_.interval * 1e9);
    if (s.t_ns >= next_fit_ns_) {
        next_fit_ns_ = s.t_ns + static_cast<std::int64_t>(p_.interval * 1e9);
        fitNow(s.t_ns);
    }
    return st_.done;
}

void SwingCalibrator::fitNow(std::int64_t t_ns) {
    const std::int64_t from = t_ns - static_cast<std::int64_t>(p_.window * 1e9);
    const auto first = std::lower_bound(t_.begin(), t_.end(), from);
    const auto drop = first - t_.begin();
    t_.erase(t_.begin(), first);
    u_.erase(u_.begin(), u_.begin() + drop);

    const std::size_t n = t_.size();
    st_.n_edges = static_cast<int>(n);
    st_.fit = core::OffsetFit{false, 0, 0, 0};
    const auto reject = [this](const char* why) {
        st_.why = why;
        has_prev_fit_ = false;
    };
    if (n < 40) return reject("too few edges: give the hanging pendulum a small swing");
    const auto mm = std::minmax_element(u_.begin(), u_.end());
    st_.amplitude = 0.5 * (*mm.second - *mm.first);
    if (st_.amplitude < p_.amp_min) return reject("swing too small");
    if (st_.amplitude > p_.amp_max) return reject("swing too large (stop it and give a smaller one)");
    if (static_cast<double>(t_.back() - t_.front()) * 1e-9 < 0.8 * p_.window)
        return reject("swing too short so far");

    tf_.resize(n);
    for (std::size_t i = 0; i < n; ++i) tf_[i] = static_cast<double>(t_[i] - t_.front()) * 1e-9;
    st_.fit = core::fitOffset(tf_.data(), u_.data(), n, p_.w, p_.sigma);
    if (!st_.fit.ok) return reject("fit singular");
    if (st_.fit.rms > p_.rms_max) return reject("not a free swing (hand contact?)");
    if (st_.fit.se > p_.se_max) return reject("fit too uncertain");
    if (has_prev_fit_ && std::fabs(st_.fit.c - prev_c_) < p_.agree) {
        st_.done = true;
        st_.c_bottom = st_.fit.c;
        st_.why = "accepted";
        return;
    }
    has_prev_fit_ = true;
    prev_c_ = st_.fit.c;
    st_.why = "waiting for a second consistent fit";
}

}
