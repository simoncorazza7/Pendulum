#ifndef PENDEL_RUNTIME_OFFSET_PARAMS_H
#define PENDEL_RUNTIME_OFFSET_PARAMS_H

#include <stdexcept>
#include <string>

#include "common/config.h"
#include "core/offset_est.h"
#include "runtime/session.h"

// offset.* (docs/offset_estimator.md): the balance session's upright-offset estimate. Every key is optional; the
// defaults used are recorded in config_used.conf. Kept apart from runtime/params.cpp so the feature stays a separate,
// switchable method (./offset.sh sets offset.mode).
//   offset.mode              off | watch | apply   off: nothing. watch: estimate and log only (the command is the same
//                                                   as off). apply: each catch / swing-up run starts with
//                                                   theta_ref = theta_ref_config - delta_hat once offset.w_use s of
//                                                   quiet balance stand behind the estimate; held for that catch.
//   offset.w_use             s      3      quiet balance before apply uses the estimate
//   offset.t_settle          s      1.5    after engage / hand-over before a stretch may start
//   offset.theta_max_deg     deg    3      |theta| at or above this ends a stretch
//   offset.lag               s      0.3    commit delay = step-detection window
//   offset.smooth            s      0.25   moving mean of the residual for the step detector
//   offset.step              m/s    0.03   residual step that marks a push
//   offset.min_stretch       s      1.0    committed before a stretch counts
//   offset.delta_max_counts  counts 5      clamp of the estimate
// plant.w0, plant.bJ, loop.ts and encoder.counts_per_rev come from their own keys.
namespace rt {

inline OffsetUse offsetFrom(const Config& cfg, bool remote) {
    OffsetUse u;
    const std::string mode = cfg.getString("offset.mode", "off");
    if (mode == "off") u.mode = OffsetUse::Off;
    else if (mode == "watch") u.mode = OffsetUse::Watch;
    else if (mode == "apply") u.mode = OffsetUse::Apply;
    else throw std::runtime_error("offset.mode must be off, watch or apply, not '" + mode + "'");
    const double kPi = 3.141592653589793;
    core::OffsetParams& p = u.est;
    p.w0 = cfg.getDouble("plant.w0");
    p.bJ = cfg.getDouble("plant.bJ");
    p.ts = cfg.getDouble("loop.ts");
    p.t_settle = cfg.getDouble("offset.t_settle", 1.5);
    p.theta_max = cfg.getDouble("offset.theta_max_deg", 3.0) * kPi / 180.0;
    p.lag = cfg.getDouble("offset.lag", 0.3);
    p.smooth = cfg.getDouble("offset.smooth", 0.25);
    p.step = cfg.getDouble("offset.step", 0.03);
    p.min_stretch = cfg.getDouble("offset.min_stretch", 1.0);
    p.delta_max = cfg.getDouble("offset.delta_max_counts", 5.0) * 2 * kPi /
                  static_cast<double>(cfg.getInt("encoder.counts_per_rev"));
    u.w_use = cfg.getDouble("offset.w_use", 3.0);
    const std::string e = core::validate(p);
    if (!e.empty()) throw std::runtime_error(e);
    if (!(u.w_use >= 0)) throw std::runtime_error("offset.w_use must be >= 0");
    if (u.mode == OffsetUse::Apply && remote)
        throw std::runtime_error("offset.mode=apply needs the balance controller on the Pi: with balance.remote the PC "
                                 "computes the command from its own PARAMS (theta_ref included). Use offset.mode=watch "
                                 "(docs/offset_estimator.md)");
    return u;
}

}

#endif
