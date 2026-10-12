#ifndef PENDEL_RUNTIME_SESSION_H
#define PENDEL_RUNTIME_SESSION_H

#include <atomic>
#include <cstdint>
#include <memory>

#include "core/estimator.h"
#include "core/supervisor.h"
#include "core/units.h"
#include "hw/hardware.h"
#include "runtime/balance.h"
#include "runtime/mover.h"
#include "runtime/swing_calib.h"

// Balance session (pendctl --mode balance; brief §6 hardware step 3, first closed-loop catches): one
// program run, many catches. Portable like BalanceRuntime: the caller owns the fixed-rate loop and
// the operator input (requestGo, requestQuit). docs/stage1_demo.md.
//
//   CALIBRATE  driver off; c_bottom from the free swing (rt::SwingCalibrator)
//   WAIT_GO    driver off; the operator's go starts the first motion
//   HOME       seek the right switch, seek the left switch (the measured span checks the step mode:
//              1/8 step ~6100 usteps, half step ~1500), move to the centre between them
//   BALANCE    rt::BalanceRuntime with keep_driver_on; arm requests while IDLE until a run starts;
//              go = stop the run; ARMED -> ENGAGED -> STOPPING -> IDLE ends the attempt
//   AFTER_RUN  cart held; the operator's go re-centres
//   RECENTRE   touch the nearer switch (it must be within touch_tol of where the step count says:
//              step-loss audit), re-zero on it, move to the centre -> BALANCE
//   FINISHING  quit: hold to standstill, driver off -> FINISHED
//   FAILED     a move failed or the span check failed: braked to standstill, driver off (terminal:
//              the position reference is lost)
// The driver stays on from HOME until FINISHED/FAILED so the step count stays the cart position.
namespace rt {

enum class SessionPhase {
    Calibrate = 0, WaitGo = 1, Home = 2, Balance = 3, AfterRun = 4, Recentre = 5, Finishing = 6,
    Finished = 7, Failed = 8
};
const char* sessionPhaseName(SessionPhase p);

enum class SessionEvent {
    None = 0, Calibrated = 1, HomeStarted = 2, SwitchFound = 3, Homed = 4, RunEnded = 5,
    RecentreStarted = 6, Recentred = 7, Failed = 8, Finished = 9
};

struct SessionConfig {
    BalanceConfig balance;          // calib.c_bottom and calib.pos_center are set by the session
    MoverConfig mover;
    SwingCalibParams swing;
    double trim = 0.0;              // counts, added to the fitted c_bottom (upright reference trim)
    double calib_timeout = 120.0;   // s without an accepted fit -> FAILED (nothing was moved)
    double v_seek = 0.04;           // m/s, toward a switch
    double v_travel = 0.10;         // m/s, other moves
    double seek_max = 0.55;         // m, the first seek (right switch) gives up after this
    double span_usteps = 6104;      // usteps, expected switch-to-switch span at the configured step mode
    double span_tol = 0.05;         // fraction
    double approach = 0.03;         // m: re-centring travels at v_travel to this far before the switch
    double touch_tol = 0.015;       // m: the switch found further than this from where expected fails
    double arm_delay = 0.2;         // s after BALANCE starts before the first arm request (estimator)
};

struct AttemptStats {
    int n = 0;                      // attempt number (1-based)
    bool engaged = false;
    int end_reason = 0;             // core::Reason that ended ENGAGED (or ARMED)
    double t_engaged = 0;           // s
    double max_abs_x = 0, x_end = 0;    // m
    double sum_th2 = 0;             // rad^2, from 1 s after engage
    int n_th = 0, n_eng = 0, n_sat = 0;
    double peak_a = 0, peak_v = 0;  // m/s^2, m/s while engaged
    double rmsTheta() const;
    double satFraction() const;
};

struct SessionRecord {
    std::int64_t t_ns = 0, counts = 0, pos_usteps = 0, speed_toggles = 0;
    int n_edges = 0;
    int phase = 0;                  // SessionPhase after this tick
    int attempt = 0;
    int mode = -1;                  // core::Mode in BALANCE / AFTER_RUN, else -1
    int event = 0;                  // core::Reason of this tick (balance runtime)
    int sevent = 0;                 // SessionEvent of this tick
    double theta = 0, omega = 0;    // rad, rad/s (valid after CALIBRATE)
    double x = 0, v = 0;            // m from the centre (before HOME: from the start position)
    double a_cmd = 0, x_ref = 0, integral = 0;
    int axis_arg = 0;
    bool sent = false, saturated = false, overrun = false, limit_left = false, limit_right = false;
    bool axis_accelerating = false, driver_enabled = false;
};

class Session {
public:
    explicit Session(const SessionConfig& c);

    SessionRecord tick(hw::Hardware& hw, bool overrun);

    void requestGo() { go_ = true; }       // operator: next step (home, stop the run, re-centre)
    void requestQuit() { quit_ = true; }   // operator: controlled stop, driver off, end

    SessionPhase phase() const { return phase_; }
    bool finished() const { return phase_ == SessionPhase::Finished || phase_ == SessionPhase::Failed; }
    core::Mode mode() const { return brt_ ? brt_->mode() : core::Mode::Idle; }
    const char* reason() const { return reason_; }           // why FAILED
    int attempts() const { return attempt_; }
    const AttemptStats& lastAttempt() const { return last_; }
    const SwingCalibStatus& calibStatus() const { return cal_.status(); }
    double cBottom() const { return calib_.c_bottom; }       // counts, incl. trim
    std::int64_t posRight() const { return pos_r_; }
    std::int64_t posLeft() const { return pos_l_; }
    std::int64_t centre() const { return centre_; }
    Side lastSwitch() const { return last_side_; }
    std::int64_t lastTouchError() const { return touch_err_; }  // usteps, switch found - expected
    int touches() const { return touches_; }
    const SessionConfig& config() const { return c_; }

private:
    void startBalance();
    void startRecentre(const hw::Sample& s);
    void moverDone(hw::Hardware& hw, const hw::Sample& s, SessionRecord& r);
    void failWith(hw::Hardware& hw, const char* fmt, ...);
    void driver(hw::Hardware& hw, bool on);

    SessionConfig c_;
    SessionPhase phase_ = SessionPhase::Calibrate;
    SwingCalibrator cal_;
    Mover mover_;
    core::VelocityEstimator est_;
    std::unique_ptr<BalanceRuntime> brt_;
    core::Calib calib_;
    std::atomic<bool> go_{false}, quit_{false};
    bool driver_on_ = false, run_started_ = false, has_start_ = false;
    int step_ = 0, attempt_ = 0, still_ = 0, touches_ = 0;
    double t_phase_ = 0, next_arm_ = 0;
    core::Mode prev_mode_ = core::Mode::Idle;
    std::int64_t pos_r_ = 0, pos_l_ = 0, centre_ = 0, expected_ = 0, touch_err_ = 0;
    Side side_ = Side::None, last_side_ = Side::None;
    AttemptStats cur_, last_;
    char reason_[240] = "";
};

}

#endif
