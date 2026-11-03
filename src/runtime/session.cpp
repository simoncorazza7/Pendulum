#include "runtime/session.h"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace rt {

const char* sessionPhaseName(SessionPhase p) {
    switch (p) {
        case SessionPhase::Calibrate: return "CALIBRATE";
        case SessionPhase::WaitGo: return "WAIT_GO";
        case SessionPhase::Home: return "HOME";
        case SessionPhase::Balance: return "BALANCE";
        case SessionPhase::AfterRun: return "AFTER_RUN";
        case SessionPhase::Recentre: return "RECENTRE";
        case SessionPhase::Finishing: return "FINISHING";
        case SessionPhase::Finished: return "FINISHED";
        case SessionPhase::Failed: return "FAILED";
    }
    return "?";
}

double AttemptStats::rmsTheta() const { return n_th ? std::sqrt(sum_th2 / n_th) : 0.0; }
double AttemptStats::satFraction() const { return n_eng ? static_cast<double>(n_sat) / n_eng : 0.0; }

Session::Session(const SessionConfig& c)
    : c_(c), cal_(c.swing), mover_(c.mover), est_(c.balance.est_kind, c.balance.ts, c.balance.est_fc),
      calib_(c.balance.calib) {}

void Session::driver(hw::Hardware& hw, bool on) {
    if (on != driver_on_) {
        if (!on) hw.commandAccel(0);
        hw.enableDriver(on);
        driver_on_ = on;
    }
}

void Session::failWith(hw::Hardware& hw, const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(reason_, sizeof reason_, fmt, ap);
    va_end(ap);
    driver(hw, false);
    phase_ = SessionPhase::Failed;
}

void Session::startBalance() {
    if (c_.swingup_mode) {                     // swing-up: hold the cart, wait for READY + the operator's go
        swr_.reset();
        ready_ = false;
        t_phase_ = 0;
        phase_ = SessionPhase::Balance;
        return;
    }
    BalanceConfig bc = c_.balance;
    bc.calib = calib_;
    bc.actuate = true;
    bc.keep_driver_on = true;
    bc.epoch_base = static_cast<std::uint32_t>(attempt_);   // unique epochs across re-centres (remote PC resets)
    brt_ = std::make_unique<BalanceRuntime>(bc);
    run_started_ = false;
    prev_mode_ = core::Mode::Idle;
    next_arm_ = 0;
    t_phase_ = 0;
    phase_ = SessionPhase::Balance;
}

// Touch the nearer switch: travel at v_travel to `approach` before where the step count says it is
// (closing earlier also counts as found), then seek at v_seek up to touch_tol + 5 mm beyond it.
void Session::startRecentre(const hw::Sample& s) {
    const double upm = calib_.usteps_per_m;
    side_ = (s.pos_usteps - pos_r_) <= (pos_l_ - s.pos_usteps) ? Side::Right : Side::Left;
    const std::int64_t dir = side_ == Side::Right ? -1 : 1;           // toward the switch, in pos
    expected_ = side_ == Side::Right ? pos_r_ : pos_l_;
    const std::int64_t approachPos = expected_ - dir * std::llround(c_.approach * upm);
    if (dir * (approachPos - s.pos_usteps) > 0) {
        mover_.start(approachPos, c_.v_travel, side_, s, false);   // stops early if the switch closes
        step_ = 0;
    } else {                                                          // already that close: seek
        mover_.start(expected_ + dir * std::llround((c_.touch_tol + 0.005) * upm), c_.v_seek, side_, s);
        step_ = 1;
    }
    phase_ = SessionPhase::Recentre;
    t_phase_ = 0;
}

void Session::moverDone(hw::Hardware& hw, const hw::Sample& s, SessionRecord& r) {
    const double upm = calib_.usteps_per_m;
    if (phase_ == SessionPhase::Home) {
        if (step_ == 0) {                                             // right switch found
            pos_r_ = mover_.switchPos();
            last_side_ = Side::Right;
            r.sevent = static_cast<int>(SessionEvent::SwitchFound);
            mover_.start(pos_r_ + std::llround(c_.span_usteps * (1 + c_.span_tol) + 0.01 * upm), c_.v_seek,
                         Side::Left, s);
            step_ = 1;
        } else if (step_ == 1) {                                      // left switch found: span check
            pos_l_ = mover_.switchPos();
            last_side_ = Side::Left;
            r.sevent = static_cast<int>(SessionEvent::SwitchFound);
            const double span = static_cast<double>(pos_l_ - pos_r_);
            if (std::fabs(span - c_.span_usteps) > c_.span_tol * c_.span_usteps) {
                failWith(hw, "track span %.0f usteps (%.3f m at %.0f usteps/m), expected %.0f +- %.0f %%: the "
                         "step mode is not the configured one (loose MS1/MS2 wire?)", span, span / upm, upm,
                         c_.span_usteps, c_.span_tol * 100);
                r.sevent = static_cast<int>(SessionEvent::Failed);
                return;
            }
            centre_ = pos_r_ + (pos_l_ - pos_r_) / 2;
            mover_.start(centre_, c_.v_travel, Side::None, s);
            step_ = 2;
        } else {                                                      // centred
            calib_.pos_center = static_cast<double>(centre_);
            r.sevent = static_cast<int>(SessionEvent::Homed);
            startBalance();
        }
        return;
    }
    // RECENTRE
    if (step_ == 0 && !mover_.found()) {                              // at the approach point: seek
        const std::int64_t dir = side_ == Side::Right ? -1 : 1;
        mover_.start(expected_ + dir * std::llround((c_.touch_tol + 0.005) * upm), c_.v_seek, side_, s);
        step_ = 1;
    } else if (step_ <= 1) {                                          // switch touched
        touch_err_ = mover_.switchPos() - expected_;
        last_side_ = side_;
        ++touches_;
        r.sevent = static_cast<int>(SessionEvent::SwitchFound);
        if (std::fabs(static_cast<double>(touch_err_)) > c_.touch_tol * upm) {
            failWith(hw, "%s switch found %.1f mm from where the step count puts it (tolerance %.0f mm): step loss "
                     "or a step-mode change", side_ == Side::Right ? "right" : "left", touch_err_ / upm * 1e3,
                     c_.touch_tol * 1e3);
            r.sevent = static_cast<int>(SessionEvent::Failed);
            return;
        }
        pos_r_ += touch_err_;                                         // re-zero on the switch
        pos_l_ += touch_err_;
        centre_ += touch_err_;
        mover_.start(centre_, c_.v_travel, Side::None, s);
        step_ = 2;
    } else {                                                          // centred
        calib_.pos_center = static_cast<double>(centre_);
        r.sevent = static_cast<int>(SessionEvent::Recentred);
        startBalance();
    }
}

SessionRecord Session::tick(hw::Hardware& hw, bool overrun) {
    SessionRecord r;
    r.overrun = overrun;
    const bool go = go_.exchange(false);
    const bool quit = quit_.load();
    const double ts = c_.balance.ts;
    t_phase_ += ts;

    if (c_.swingup_mode && (phase_ == SessionPhase::Balance || phase_ == SessionPhase::AfterRun))
        return swingTick(hw, overrun, go, quit);
    if (phase_ == SessionPhase::Balance || phase_ == SessionPhase::AfterRun) {
        BalanceRuntime& b = *brt_;
        if (phase_ == SessionPhase::Balance) {
            const core::Mode m = b.mode();
            if (m == core::Mode::Idle && !run_started_ && !quit && t_phase_ >= c_.arm_delay && t_phase_ >= next_arm_) {
                b.requestArm();
                next_arm_ = t_phase_ + 0.5;
            }
            if ((go || quit) && (m == core::Mode::Armed || m == core::Mode::Engaged)) b.requestStop();
        }
        const TickRecord t = b.tick(hw, overrun);
        driver_on_ = t.driver_enabled;
        r.t_ns = t.t_ns; r.counts = t.counts; r.pos_usteps = t.pos_usteps; r.speed_toggles = t.speed_toggles;
        r.n_edges = t.n_edges; r.theta = t.theta; r.omega = t.omega; r.x = t.x; r.v = t.v;
        r.mode = t.mode; r.event = t.event; r.a_cmd = t.a_cmd; r.integral = t.integral; r.axis_arg = t.axis_arg;
        r.sent = t.sent; r.saturated = t.saturated; r.limit_left = t.limit_left; r.limit_right = t.limit_right;
        r.axis_accelerating = t.axis_accelerating; r.driver_enabled = t.driver_enabled;
        r.controller = t.controller; r.epoch = t.epoch; r.a_local = t.a_local; r.remote_miss = t.remote_miss;
        r.i0 = t.i0;
        est_.update(t.counts);
        const core::Mode m = static_cast<core::Mode>(t.mode);
        if (phase_ == SessionPhase::Balance) {
            if (t.event == static_cast<int>(core::Reason::ArmedEvent)) {
                run_started_ = true;
                cur_ = AttemptStats{};
                cur_.n = ++attempt_;
            }
            if (run_started_) {
                if (t.event == static_cast<int>(core::Reason::EngagedEvent)) cur_.engaged = true;
                if (m == core::Mode::Engaged) {
                    ++cur_.n_eng;
                    cur_.t_engaged = cur_.n_eng * ts;
                    if (cur_.t_engaged > 1.0) { cur_.sum_th2 += t.theta * t.theta; ++cur_.n_th; }
                    cur_.peak_a = std::max(cur_.peak_a, std::fabs(t.a_cmd));
                    cur_.peak_v = std::max(cur_.peak_v, std::fabs(t.v));
                    if (t.saturated) ++cur_.n_sat;
                }
                if (m != core::Mode::Idle) cur_.max_abs_x = std::max(cur_.max_abs_x, std::fabs(t.x));
                if (m == core::Mode::Stopping && prev_mode_ != core::Mode::Stopping) cur_.end_reason = t.event;
                if (m == core::Mode::Idle) {
                    cur_.x_end = t.x;
                    last_ = cur_;
                    r.sevent = static_cast<int>(SessionEvent::RunEnded);
                    phase_ = SessionPhase::AfterRun;
                    t_phase_ = 0;
                }
            }
            if (quit && m == core::Mode::Idle) { phase_ = SessionPhase::Finishing; still_ = 0; t_phase_ = 0; }
        } else if (quit) {
            phase_ = SessionPhase::Finishing;
            still_ = 0;
            t_phase_ = 0;
        } else if (go) {
            phase_ = SessionPhase::Recentre;  // the mover starts on the next tick, from its own sample
            step_ = -1;
            t_phase_ = 0;
            r.sevent = static_cast<int>(SessionEvent::RecentreStarted);
        }
        prev_mode_ = m;
        r.phase = static_cast<int>(phase_);
        r.attempt = attempt_;
        return r;
    }

    const hw::Sample s = hw.read();
    if (!has_start_) {                         // x before HOME: from the start position
        calib_.pos_center = static_cast<double>(s.pos_usteps);
        has_start_ = true;
    }
    r.t_ns = s.t_ns; r.counts = s.counts; r.pos_usteps = s.pos_usteps; r.speed_toggles = s.speed_toggles;
    r.n_edges = s.n_edges; r.limit_left = s.limit_left; r.limit_right = s.limit_right;
    r.axis_accelerating = s.axis_accelerating;
    r.theta = core::thetaFromCounts(s.counts, calib_);
    r.omega = est_.update(s.counts) * core::omegaScale(calib_);
    r.x = core::xFromPos(s.pos_usteps, calib_);
    r.v = core::vFromAxisSpeed(s.speed_toggles, calib_);

    switch (phase_) {
        case SessionPhase::Calibrate:
            if (quit) { phase_ = SessionPhase::Finished; r.sevent = static_cast<int>(SessionEvent::Finished); break; }
            if (cal_.update(s)) {
                calib_.c_bottom = cal_.status().c_bottom + c_.trim;
                phase_ = SessionPhase::WaitGo;
                r.sevent = static_cast<int>(SessionEvent::Calibrated);
            } else if (t_phase_ > c_.calib_timeout) {
                failWith(hw, "calibration: no accepted fit within %.0f s (nothing was moved)", c_.calib_timeout);
                r.sevent = static_cast<int>(SessionEvent::Failed);
            }
            break;
        case SessionPhase::WaitGo:
            if (quit) { phase_ = SessionPhase::Finished; r.sevent = static_cast<int>(SessionEvent::Finished); break; }
            if (go) {
                driver(hw, true);
                mover_.start(s.pos_usteps - std::llround(c_.seek_max * calib_.usteps_per_m), c_.v_seek, Side::Right, s);
                step_ = 0;
                phase_ = SessionPhase::Home;
                t_phase_ = 0;
                r.sevent = static_cast<int>(SessionEvent::HomeStarted);
            }
            break;
        case SessionPhase::Home:
        case SessionPhase::Recentre: {
            if (phase_ == SessionPhase::Recentre && step_ == -1) startRecentre(s);   // deferred start
            const MoverRecord m = mover_.tick(hw, s, overrun, quit);
            r.a_cmd = m.a_cmd; r.x_ref = m.x_ref; r.axis_arg = m.axis_arg; r.sent = m.sent;
            if (mover_.state() == MoverState::Done) {
                moverDone(hw, s, r);
            } else if (mover_.state() == MoverState::Failed) {
                if (quit && std::strcmp(mover_.reason(), "stop request") == 0) {
                    driver(hw, false);
                    phase_ = SessionPhase::Finished;
                    r.sevent = static_cast<int>(SessionEvent::Finished);
                } else if (phase_ == SessionPhase::Recentre && step_ == 1 && !mover_.found() &&
                           std::strcmp(mover_.reason(), "switch not found before the give-up position") == 0) {
                    failWith(hw, "%s switch not within %.0f mm of where the step count puts it: step loss or a "
                             "step-mode change", side_ == Side::Right ? "right" : "left", (c_.touch_tol + 0.005) * 1e3);
                    r.sevent = static_cast<int>(SessionEvent::Failed);
                } else {
                    failWith(hw, "%s: %s", phase_ == SessionPhase::Home ? "homing" : "re-centring", mover_.reason());
                    r.sevent = static_cast<int>(SessionEvent::Failed);
                }
            }
            break;
        }
        case SessionPhase::Finishing: {
            const double a = r.v != 0.0 ? -std::copysign(std::min(c_.mover.a_brake, std::fabs(r.v) / ts), r.v) : 0.0;
            r.a_cmd = a;
            r.axis_arg = core::axisAccelArg(a, calib_);
            if (driver_on_) { hw.commandAccel(r.axis_arg); r.sent = true; }
            still_ = s.speed_toggles == 0 ? still_ + 1 : 0;
            if (still_ >= 25 || t_phase_ > 2.0) {
                if (t_phase_ > 2.0 && still_ < 25) hw.halt();
                driver(hw, false);
                phase_ = SessionPhase::Finished;
                r.sevent = static_cast<int>(SessionEvent::Finished);
            }
            break;
        }
        default:
            driver(hw, false);
            break;
    }
    r.driver_enabled = driver_on_;
    r.phase = static_cast<int>(phase_);
    r.attempt = attempt_;
    return r;
}

// Swing-up mode: BALANCE = hold + wait for READY and the go, or a running rt::SwingUpRuntime; AFTER_RUN = hold.
SessionRecord Session::swingTick(hw::Hardware& hw, bool overrun, bool go, bool quit) {
    SessionRecord r;
    r.overrun = overrun;
    const double ts = c_.balance.ts;
    const hw::Sample s = hw.read();
    r.t_ns = s.t_ns; r.counts = s.counts; r.pos_usteps = s.pos_usteps; r.speed_toggles = s.speed_toggles;
    r.n_edges = s.n_edges; r.limit_left = s.limit_left; r.limit_right = s.limit_right;
    r.axis_accelerating = s.axis_accelerating;
    r.theta = core::thetaFromCounts(s.counts, calib_);
    r.omega = est_.update(s.counts) * core::omegaScale(calib_);
    r.x = core::xFromPos(s.pos_usteps, calib_);
    r.v = core::vFromAxisSpeed(s.speed_toggles, calib_);

    if (phase_ == SessionPhase::Balance && swr_) {
        if (quit) swr_->requestStop();
        else if (go) swr_->requestDampOut();
        const SwingRecord w = swr_->tick(hw, s, overrun);
        r.a_cmd = w.a_cmd; r.axis_arg = w.axis_arg; r.integral = w.integral; r.saturated = w.saturated;
        r.sent = w.sent; r.swing_phase = w.phase; r.energy = w.energy;
        r.controller = w.controller; r.epoch = w.epoch; r.i0 = w.i0; r.a_local = w.a_local;
        r.remote_miss = w.remote_miss;
        if (swr_->done()) {
            cur_.engaged = swr_->handedOver();
            cur_.t_handover = swr_->handoverTime();
            cur_.t_engaged = swr_->balanceTime();
            cur_.max_abs_x = swr_->maxAbsX();
            cur_.peak_v = swr_->peakV();
            cur_.peak_energy = swr_->peakEnergy();
            cur_.end_text = swr_->reason();
            cur_.x_end = r.x;
            last_ = cur_;
            swr_.reset();
            r.sevent = static_cast<int>(SessionEvent::RunEnded);
            phase_ = quit ? SessionPhase::Finishing : SessionPhase::AfterRun;
            still_ = 0;
            t_phase_ = 0;
        }
    } else {
        const double a = r.v != 0.0 ? -std::copysign(std::min(c_.mover.a_brake, std::fabs(r.v) / ts), r.v) : 0.0;
        r.a_cmd = a;
        r.axis_arg = core::axisAccelArg(a, calib_);
        hw.commandAccel(r.axis_arg);           // hold the cart (the Axis keeps executing its last command)
        r.sent = true;
        if (quit) {
            phase_ = SessionPhase::Finishing;
            still_ = 0;
            t_phase_ = 0;
        } else if (phase_ == SessionPhase::Balance) {
            const core::SupervisorParams& p = c_.balance.sup;
            const bool still = std::fabs(core::wrap(r.theta - 3.141592653589793)) < p.hang_tol &&
                               std::fabs(r.omega) < p.omega_hang && std::fabs(r.v) < p.v_stopped &&
                               std::fabs(r.x) < p.center_tol && !s.limit_left && !s.limit_right &&
                               t_phase_ > c_.arm_delay && (!c_.balance.remote || c_.balance.remote->linkUp());
            if (still && !ready_) r.sevent = static_cast<int>(SessionEvent::SwingReady);
            ready_ = still;
            if (go && ready_) {
                SwingUpConfig u = c_.swingup;
                u.calib = calib_;
                u.remote = c_.balance.remote;
                u.remote_delay = c_.balance.remote_delay;
                u.remote_max_miss = c_.balance.remote_max_miss;
                u.epoch_base = static_cast<std::uint32_t>(attempt_);   // unique epochs across runs (the PC resets)
                swr_ = std::make_unique<SwingUpRuntime>(u);
                cur_ = AttemptStats{};
                cur_.n = ++attempt_;
                cur_.swingup = true;
                ready_ = false;
                r.sevent = static_cast<int>(SessionEvent::SwingStarted);
            } else if (go) {
                r.sevent = static_cast<int>(SessionEvent::SwingNotReady);
            }
        } else if (go) {                      // AFTER_RUN: re-centre, then READY again
            phase_ = SessionPhase::Recentre;
            step_ = -1;
            t_phase_ = 0;
            r.sevent = static_cast<int>(SessionEvent::RecentreStarted);
        }
    }
    r.driver_enabled = driver_on_;
    r.phase = static_cast<int>(phase_);
    r.attempt = attempt_;
    return r;
}

}
