#ifndef PENDEL_RUNTIME_ACTUATOR_H
#define PENDEL_RUNTIME_ACTUATOR_H

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "core/profile.h"
#include "core/units.h"
#include "hw/hardware.h"
#include "runtime/mover.h"

// Stage 0.9 actuator test (pendctl --mode actuator, ./actuator.sh; docs/stage0_9_actuator.md): the cart's
// reliable acceleration and speed with the pendulum mounted (hanging). Portable like rt::Session: the
// caller owns the fixed-rate loop and the operator input (requestGo / requestSkip / requestQuit).
//
//   WAIT_GO    driver off; the operator's go starts homing
//   HOME       right switch, left switch (span = step-mode check), centre, as in the balance session
//   READY      cart held at the centre; go runs the next rung, skip drops the rest of its speed
//   TO_START   shaped move (rt::Mover) to where the next rep starts
//   SETTLE     cart held until the hanging pendulum swings less than settle_deg (at most settle_max)
//   RUN        the rep's reference trajectory, tracked on the Axis's own position (feedforward + PD)
//   BRAKE      the trajectory has ended: hold until the Axis speed is 0
//   TOUCH      the switch on the side where the rep ended: travel to `approach` before it, seek at
//              v_seek; touch error = switch found - switch expected (the step-loss audit), re-zero
//   CENTRE     back to the centre after a rung, then READY
//   FINISHING  hold to standstill, driver off -> FINISHED
//   FAILED     driver off: a failed move or span check, a touch beyond touch_tol, an abort during RUN
//
// A rung is (v, a), run `reps` times with the configured profile:
//   pair    out-and-back legs (accelerate at a to v, cruise, brake at a) centred on the track; the return
//           starts n T0 after the outbound leg (n = smallest with n T0 >= leg time), so the two legs' kicks to
//           the hanging pendulum cancel (linear model). Ends at -d/2: touches the right switch.
//   oneway  one leg per rep, alternating direction; touches the switch ahead.
//   bang    +-a square wave whose speed swings between +-v, centred, ending at rest where it started.
//   chirp   acceleration amplitude a, exponential sweep up to chirp_f1, cosine-tapered ends; it starts
//           high enough that |v| <= v and |x| <= stream_x_max.
//   table   a reference trajectory from a file (py/actuator_replay.py); one rung, labelled with its peaks.
// Aborts during RUN (a switch, overrun streak, Axis not in ACCELERATE, encoder fault, tracking error, stop
// request) brake at the rung's own acceleration, so by step count the cart never passes the planned end.
// Touch error within touch_pass: the rep passes. touch_fail or more: it fails, and the rest of that
// speed's ladder is skipped. In between: the rep runs once more (a second marginal one fails).
namespace rt {

enum class ActProfile { Pair = 0, OneWay = 1, Bang = 2, Chirp = 3, Table = 4 };
const char* actProfileName(ActProfile p);
bool parseActProfile(const char* s, ActProfile* p);

enum class ActPhase {
    WaitGo = 0, Home = 1, Ready = 2, ToStart = 3, Settle = 4, Run = 5, Brake = 6, Touch = 7, Centre = 8,
    Finishing = 9, Finished = 10, Failed = 11
};
const char* actPhaseName(ActPhase p);

enum class ActEvent {
    None = 0, HomeStarted = 1, SwitchFound = 2, Homed = 3, RepStarted = 4, RunStarted = 5, SettleTimeout = 6,
    RunEnded = 7, Touched = 8, RungDone = 9, Ready = 10, Skipped = 11, Failed = 12, Finished = 13
};

enum class Verdict { NotRun = -1, Pass = 0, Marginal = 1, Fail = 2, Aborted = 3, Skipped = 4 };
const char* verdictName(Verdict v);

struct ActRung {
    double v = 0;                   // m/s, speed (pair, oneway: cruise; bang: peak; chirp: velocity bound)
    double a = 0;                   // m/s^2, acceleration (pair, oneway, bang) or amplitude (chirp)
    int speed_index = 0;            // which entry of `speeds`: a failed rung skips the rest of that speed
    double leg = 0;                 // m: pair/oneway leg length, bang/chirp/table x span (peak |x|)
    double duration = 0;            // s of RUN per rep
    double peak_v = 0, peak_a = 0;  // of the reference (sampled at ts)
    double f0 = 0;                  // Hz, chirp: start frequency (found once, before the loop runs)
    bool runnable = true;
    const char* why = "";           // why not runnable
};

struct RepResult {
    int rung = 0;                   // index into rungs()
    int rep = 0;                    // 1-based; a repeated (marginal) rep keeps its number
    bool repeat = false;            // the second run of a marginal rep
    double v = 0, a = 0;
    int side = 0;                   // switch touched: 1 right, 2 left
    std::int64_t touch_err = 0;     // usteps, switch found - expected
    Verdict verdict = Verdict::NotRun;
    double peak_a_cmd = 0;          // m/s^2, |a_cmd| during RUN
    double peak_v = 0;              // m/s, |v| (Axis) during RUN
    double max_track = 0;           // m, |x_ref - x| during RUN
    double swing_start_deg = 0;     // pendulum swing amplitude when RUN started
    bool settle_timeout = false;
    double t_run = 0;               // s
};

struct ActuatorConfig {
    double ts = 0.002;              // s
    core::Calib calib;              // usteps_per_m, toggles_per_ustep, x_sign, cpr (pos_center: set here)
    MoverConfig mover;              // homing, travel and touch moves (balance.*, move.*)
    double w0 = 4.97;               // rad/s, hanging frequency: T0 for the pair timing and the swing window
    double v_seek = 0.04;           // m/s, toward a switch
    double v_travel = 0.10;         // m/s, other moves
    double seek_max = 0.55;         // m, the first seek gives up after this
    double span_usteps = 6104;      // usteps, expected switch-to-switch span
    double span_tol = 0.05;         // fraction
    double approach = 0.03;         // m, a touch travels at v_travel to this far before the switch
    double touch_tol = 0.015;       // m, a switch found further than this from where expected: FAILED

    ActProfile profile = ActProfile::Pair;
    std::vector<double> speeds;     // m/s, one ladder per speed, in this order
    std::vector<double> accels;     // m/s^2, the ladder (ascending)
    int reps = 2;
    double cruise = 0.10;           // s of cruise per leg
    double leg_max = 0.30;          // m, longest leg (legs are centred)
    double stream_seconds = 10.0;   // s, bang and chirp
    double stream_x_max = 0.10;     // m, peak |x| of a stream
    double chirp_f1 = 40.0;         // Hz
    double chirp_taper = 1.0;       // s, cosine ramp at each end of the chirp
    std::vector<core::ProfilePoint> table;   // profile table: x (from the centre), v, a every ts
    double kp = 25.0, kv = 10.0;    // 1/s^2, 1/s: tracking of the reference on the Axis's own state
    double track_tol = 0.003;       // m, |x_ref - x| above this during RUN aborts
    double a_margin = 1.1;          // the command is limited to a_margin * the rung's acceleration
    int overrun_streak = 3;
    double touch_pass = 3;          // usteps
    double touch_fail = 8;          // usteps (one full step at 1/8)
    double settle_deg = 3.0;        // deg, swing amplitude below which a rep may start
    double settle_max = 40.0;       // s, then the rep starts anyway (flagged)
    double hold_a = 1.0;            // m/s^2, cap of the hold law (balance.a_brake)
};

// The trajectory of one rep, x from the track centre (x frame). Exposed for tests and the app's preview.
class ActTraj {
public:
    virtual ~ActTraj() = default;
    virtual core::ProfilePoint at(double t) const = 0;
    virtual double duration() const = 0;
};
std::unique_ptr<ActTraj> makeActTraj(const ActuatorConfig& c, const ActRung& r, int rep);
// Profile `table`: a CSV with the header t,x,v,a (s; m from the track centre; m/s; m/s^2), one row every ts,
// '#' lines ignored (written by py/actuator_replay.py). False, with *err set, on a malformed file.
bool loadActTable(const std::string& path, double ts, std::vector<core::ProfilePoint>* out, std::string* err);
// The rungs of the configured ladder (speeds x accels, or the one table rung), with their sizes; a rung whose
// leg exceeds leg_max (or whose chirp cannot start below chirp_f1) is not runnable.
std::vector<ActRung> buildActRungs(const ActuatorConfig& c);

struct ActRecord {
    std::int64_t t_ns = 0, counts = 0, pos_usteps = 0, speed_toggles = 0;
    int n_edges = 0;
    int phase = 0;                  // ActPhase after this tick
    int event = 0;                  // ActEvent of this tick
    int rung = -1, rep = 0;         // the rung / rep being run (-1: none yet)
    double x = 0, v = 0;            // m from the centre (before HOME: from the start), m/s
    double x_ref = 0, v_ref = 0, a_ref = 0;   // RUN: the reference
    double a_cmd = 0;               // m/s^2 (x frame)
    int axis_arg = 0;
    double swing_deg = 0;           // swing amplitude of the hanging pendulum over the last T0
    bool sent = false, overrun = false, limit_left = false, limit_right = false;
    bool axis_accelerating = false, driver_enabled = false, encoder_fault = false;
};

class ActuatorTest {
public:
    explicit ActuatorTest(const ActuatorConfig& c);

    ActRecord tick(hw::Hardware& hw, bool overrun);

    void requestGo() { go_ = true; }       // WAIT_GO: home; READY: run the next rung (or end after the last)
    void requestSkip() { skip_ = true; }   // READY: skip the rest of the next rung's speed
    void requestQuit() { quit_ = true; }   // controlled stop (a RUN brakes at its acceleration), driver off

    ActPhase phase() const { return phase_; }
    bool finished() const { return phase_ == ActPhase::Finished || phase_ == ActPhase::Failed; }
    const char* reason() const { return reason_; }
    const std::vector<ActRung>& rungs() const { return rungs_; }
    const std::vector<Verdict>& rungVerdicts() const { return rung_verdict_; }
    int nextRung() const;                  // the rung READY runs next, -1 when the ladder is done
    int currentRung() const { return rung_; }
    const std::vector<RepResult>& results() const { return results_; }
    std::int64_t posRight() const { return pos_r_; }
    std::int64_t posLeft() const { return pos_l_; }
    std::int64_t centre() const { return centre_; }
    Side lastSwitch() const { return last_side_; }
    double swingDeg() const { return swing_deg_; }
    const ActuatorConfig& config() const { return c_; }

private:
    void driver(hw::Hardware& hw, bool on);
    void failWith(hw::Hardware& hw, const char* fmt, ...);
    void startRung(int i);
    void startRep(const hw::Sample& s);
    void startTouch(const hw::Sample& s);
    void afterTouch(hw::Hardware& hw, const hw::Sample& s, ActRecord& r);
    void brakeTick(hw::Hardware& hw, const hw::Sample& s, ActRecord& r, bool quit);
    void finishRep(Verdict v);
    double hold(hw::Hardware& hw, double v);      // hold law; returns the command (x frame)
    std::int64_t posAt(double x) const;           // Axis pos of x (m from the centre)
    void updateSwing(std::int64_t counts);

    ActuatorConfig c_;
    std::vector<ActRung> rungs_;
    std::vector<Verdict> rung_verdict_;
    std::vector<RepResult> results_;
    ActPhase phase_ = ActPhase::WaitGo;
    Mover mover_;
    core::Calib calib_;
    std::unique_ptr<ActTraj> traj_;
    std::atomic<bool> go_{false}, skip_{false}, quit_{false};
    bool driver_on_ = false, has_start_ = false, aborting_ = false, sent_prev_ = false;
    int step_ = 0, rung_ = -1, rep_ = 0, still_ = 0, overruns_ = 0, brake_ticks_ = 0;
    bool repeat_ = false;                 // this rep is the repeat of a marginal one
    double t_phase_ = 0, t_run_ = 0;
    RepResult cur_;
    std::int64_t pos_r_ = 0, pos_l_ = 0, centre_ = 0, expected_ = 0;
    Side side_ = Side::None, last_side_ = Side::None;
    std::vector<std::int64_t> window_;    // counts over the last T0 (ring)
    std::size_t win_n_ = 0, win_i_ = 0;
    double swing_deg_ = 0;
    char reason_[240] = "";
};

}

#endif
