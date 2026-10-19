// balance (Stage 1 hardware step 3, first closed-loop catches): the balance session, rt::Session on
// hw::PiHw. HARDWARE ACTION: it homes the cart from switch to switch, balances and re-centres.
// docs/stage1_demo.md; ./demo.sh runs it on the rig after a checklist.
//
//   1. calibration (driver off): Simon swings the hanging pendulum; c_bottom from the free swing
//   2. Enter: homing (right switch, left switch: span = step-mode check, centre)
//   3. arms by itself when the pendulum hangs still; Simon lifts it; ENGAGED rings the terminal bell
//   4. a run ends at the soft limit, theta_abort, Enter (stop) or any other abort: controlled stop,
//      cart held; Enter re-centres (touches the nearer switch: step-loss audit) and re-arms
//   q + Enter or Ctrl-C: controlled stop, driver off, logs. Refuses the real chip unless
//   balance.allow_real_chip, and refuses to start on a pressed switch.

#include <poll.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "app/modes.h"
#include "app/remote_link.h"
#include "app/rig.h"
#include "common/runlog.h"
#include "hw/pi_hw.h"
#include "rt/ringlog.h"
#include "rt/rt.h"
#include "runtime/params.h"
#include "runtime/session.h"

namespace app {
namespace {

struct BalRec {
    std::uint32_t tick;
    std::int64_t late_ns;
    rt::SessionRecord r;
};

struct EdgeRec {
    std::int64_t ts_ns;           // kernel edge timestamp (CLOCK_MONOTONIC)
    std::int64_t count_after;     // cumulative count after the edge
    std::uint32_t tick;           // control tick that read it
};

// Passes everything through and keeps the last sample (its edges go to edges.csv).
class RecordingHw : public hw::Hardware {
public:
    explicit RecordingHw(hw::Hardware& h) : h_(h) {}
    hw::Sample read() override { last_ = h_.read(); return last_; }
    void commandAccel(int a) override { h_.commandAccel(a); }
    void enableDriver(bool on) override { h_.enableDriver(on); }
    void halt() override { h_.halt(); }
    bool isStopped() override { return h_.isStopped(); }
    const hw::Sample& last() const { return last_; }

private:
    hw::Hardware& h_;
    hw::Sample last_;
};

const double kRad2Deg = 180.0 / 3.141592653589793;

std::string fmt(const char* f, double v) {
    char b[64];
    std::snprintf(b, sizeof b, f, v);
    return b;
}

}

int runBalance(const Config& cfg, const std::string& configPath, const std::string& cmdLine) {
    const std::string chip = cfg.getString("balance.chip");
    const bool allowReal = cfg.getBool("balance.allow_real_chip");
    const double logSeconds = cfg.getDouble("balance.log_seconds");      // s
    const double maxSession = cfg.getDouble("balance.max_session");      // s
    const double statusPeriod = cfg.getDouble("balance.status_period");  // s
    const bool lockMem = cfg.getBool("rt.lock_memory"), rtOn = cfg.getBool("rt.enable");
    const std::string logDir = cfg.getString("log.dir");
    rt::SessionConfig sc = rt::sessionFrom(cfg);                         // throws on bad parameters
    const double ts = sc.balance.ts;
    const double upm = sc.balance.calib.usteps_per_m;
    const hw::PiRt rtc = rtFrom(cfg);

    // Stage 2b: the PC's controller over UDP (docs/remote.md). Opened before any GPIO, so a busy port ends here.
    std::unique_ptr<RemoteLink> link;
    if (sc.remote_mode) {
        link = std::make_unique<RemoteLink>(static_cast<int>(cfg.getInt("remote.port")), sc.balance.remote_delay,
                                            cfg.getDouble("remote.link_timeout"),
                                            static_cast<std::size_t>(logSeconds / ts));
        std::string e;
        if (!link->open(&e)) { std::cerr << "balance: " << e << "\n"; return 1; }
        proto::Params pp;
        const core::PidParams& g = sc.balance.pid;
        pp.ts = ts; pp.kp = g.kp; pp.ki = g.ki; pp.kd = g.kd; pp.a_max = g.a_max; pp.v_max = g.v_max;
        pp.v_taper = g.v_taper; pp.i_max = g.i_max; pp.i_delay = g.i_delay; pp.theta_ref = g.theta_ref; pp.vel_fc = g.vel_fc;
        link->setParams(pp);
        sc.balance.remote = link.get();
    }

    RingLog<BalRec> log(static_cast<std::size_t>(logSeconds / ts), true);   // keeps the newest
    RingLog<EdgeRec> edgeLog(1500000, true);
    std::vector<rt::AttemptStats> attempts;
    attempts.reserve(1000);
    std::vector<long long> touchErrs;
    touchErrs.reserve(1000);
    std::string err;
    if (lockMem && !rt::lockMemory(512 * 1024, &err)) std::cerr << "warning: " << err << "\n";

    hw::PiHw pi(chip, pinsFrom(cfg), allowReal, static_cast<int>(cfg.getInt("axis.max_accel")));   // throws if refused
    activeAxis = pi.axis();
    if (!pi.start(rtc, &err)) { std::cerr << "balance: " << err << "\n"; activeAxis = nullptr; return 1; }
    if (rtOn && !rt::makeCurrentThreadRealtime(static_cast<int>(cfg.getInt("rt.loop_prio")),
                                               static_cast<int>(cfg.getInt("rt.loop_cpu")), &err))
        std::cerr << "warning: " << err << "\n";

    const hw::Sample s0 = pi.read();
    if (s0.limit_left || s0.limit_right) {
        std::cerr << "balance: a limit switch is pressed at start; refusing (push the cart off it by hand, "
                     "roughly to the middle)\n";
        activeAxis = nullptr;
        return 2;
    }
    {
        const core::PidParams& p = sc.balance.pid;
        const core::SupervisorParams& sp = sc.balance.sup;
        char b[700];
        std::snprintf(b, sizeof b,
                      "balance session on %s (%s). PID Kp %.2f Ki %.2f Kd %.3f, a_max %.1f m/s^2, v_max %.2f m/s, soft "
                      "limits %+.0f/%+.0f mm about the track centre, engage |theta| < %.0f deg, abort > %.0f deg.\n"
                      "Enter = next step (home / stop the run / re-centre), q + Enter or Ctrl-C = stop and end.\n",
                      chip.c_str(), pi.chipLabel().c_str(), p.kp, p.ki, p.kd, p.a_max, p.v_max, sp.x_min * 1e3,
                      sp.x_max * 1e3, sp.theta_engage * kRad2Deg, sp.theta_abort * kRad2Deg);
        std::cout << b;
        if (sc.swingup_mode) {
            const rt::SwingUpConfig& u = sc.swingup;
            std::snprintf(b, sizeof b,
                          "SWING-UP MODE: pumping capped at %.2f m/s^2 (cart swings ~+-%.0f mm), energy target %+.1f %% of "
                          "upright, hand-over at |theta| < %.0f deg and |theta + theta'/w0| < %.0f deg, then the PID above.\n"
                          "The pendulum sweeps the WHOLE CIRCLE around the pivot: stay outside it during a run. During a run: "
                          "Enter = bring it down (energy out), q / Ctrl-C = hard stop (the cart stops, the pendulum keeps "
                          "swinging).\n",
                          u.a_pump, u.a_pump * 52.0, u.e_ref * 100, u.theta_sw * kRad2Deg, u.u_sw * kRad2Deg);
            std::cout << b;
        }
        if (link) {
            std::snprintf(b, sizeof b,
                          "REMOTE MODE: the controller runs on the PC (py/remote_ctrl.py, UDP port %d). Constant delay "
                          "%d ticks = %.1f ms; a run aborts after %d missing commands in a row or %.0f ms without replies; "
                          "arming waits for the PC. Everything else (limits, stops, homing, keys) stays on the Pi.\n",
                          static_cast<int>(cfg.getInt("remote.port")), sc.balance.remote_delay,
                          sc.balance.remote_delay * ts * 1e3, sc.balance.remote_max_miss,
                          cfg.getDouble("remote.link_timeout") * 1e3);
            std::cout << b;
        }
        std::cout << "calib: give the HANGING pendulum a small swing (3-8 deg) and let go. Hands off until "
                     "'calib: accepted' (~5 s). Driver OFF.\n" << std::flush;
    }

    rt::Session ses(sc);
    RecordingHw hw(pi);
    std::atomic<bool> inputRun{true}, quitReq{false};
    std::thread input([&] {                    // operator input, not real-time
        std::string line;
        char buf[64];
        while (inputRun.load()) {
            pollfd p{STDIN_FILENO, POLLIN, 0};
            if (poll(&p, 1, 100) <= 0) continue;
            const ssize_t k = read(STDIN_FILENO, buf, sizeof buf);
            if (k <= 0) break;                 // EOF: no more operator input (Ctrl-C still works)
            for (ssize_t i = 0; i < k; ++i) {
                if (buf[i] != '\n') { if (buf[i] != '\r' && buf[i] != ' ') line += buf[i]; continue; }
                if (line == "q" || line == "quit") quitReq = true; else ses.requestGo();
                line.clear();
            }
        }
    });

    TickClock clk(ts);
    flags.motionSafe = false;
    bool quitSent = false, halted = false;
    double tQuit = 0, nextStatus = 0, nextCal = 1.0, nextMove = 0;
    int lastMode = -1, lastRefusal = -1, lastSwing = -1;
    bool linkWasUp = false;
    double nextLink = 5.0;
    long remoteMiss = 0;
    const char* afterCentre = sc.swingup_mode
                                  ? "Let the pendulum hang still; at READY, Enter starts the swing-up."
                                  : "Arms by itself when the pendulum hangs still.";
    while (true) {
        const std::int64_t late = clk.wait();
        const double t = clk.seconds();
        if (!quitSent && (flags.stop || quitReq || t > maxSession)) {
            if (t > maxSession) std::cout << "balance: balance.max_session reached\n";
            ses.requestQuit();
            quitSent = true;
            tQuit = t;
        }
        if (quitSent && t - tQuit > 5.0) {     // never expected: every phase ends within ~2 s
            std::cerr << "balance: no controlled end within 5 s, HALT\n";
            pi.halt();
            halted = true;
            break;
        }
        if (link) link->beginTick(clk.tick(), rt::monoNs());   // replies first: this tick may need one
        const rt::SessionRecord r = ses.tick(hw, clk.overrun());
        log.push({clk.tick(), late, r});
        if (link) {
            proto::Sample s;
            s.flags = static_cast<std::uint16_t>((r.controller ? proto::kSampleController : 0) |
                                                 (quitSent ? proto::kSampleFinishing : 0));
            s.epoch = r.epoch;
            s.t_ns = r.t_ns;
            s.counts = static_cast<std::int32_t>(r.counts);
            s.pos_usteps = static_cast<std::int32_t>(r.pos_usteps);
            s.theta = r.theta;
            s.omega = r.omega;
            s.x = r.x;
            s.v = r.v;
            s.mode = static_cast<std::uint8_t>(r.mode < 0 ? 255 : r.mode);
            s.phase = static_cast<std::uint8_t>(r.phase);
            link->sendSample(s, r.a_local);
            if (r.remote_miss) ++remoteMiss;
            if (link->linkUp() != linkWasUp) {
                linkWasUp = link->linkUp();
                std::cout << fmt("%7.2f s  ", t)
                          << (linkWasUp ? "remote: link UP, PC " + link->peer() : std::string("remote: link DOWN (no replies)"))
                          << "\n" << std::flush;
            }
            if (r.mode != static_cast<int>(core::Mode::Engaged) && t >= nextLink) {
                nextLink = t + 5.0;
                std::uint64_t n, lateN;
                std::int64_t mx;
                link->takeWindow(&n, &mx, &lateN);
                char s2[200];
                std::snprintf(s2, sizeof s2, "  remote: link %s, PC %s; last 5 s: %llu replies, worst round trip %.2f ms "
                              "(delay %.1f ms), %llu late", link->linkUp() ? "up" : "DOWN", link->peer().c_str(),
                              static_cast<unsigned long long>(n), mx * 1e-6, sc.balance.remote_delay * ts * 1e3,
                              static_cast<unsigned long long>(lateN));
                if (static_cast<rt::SessionPhase>(r.phase) != rt::SessionPhase::Calibrate) std::cout << s2 << "\n" << std::flush;
            }
        }
        const hw::Sample& ls = hw.last();
        for (int i = 0; i < ls.n_edges; ++i)
            edgeLog.push({ls.edges[static_cast<std::size_t>(i)].ts_ns, ls.edges[static_cast<std::size_t>(i)].count_after, clk.tick()});

        const auto phase = static_cast<rt::SessionPhase>(r.phase);
        char b[600];
        b[0] = 0;
        switch (static_cast<rt::SessionEvent>(r.sevent)) {
            case rt::SessionEvent::Calibrated: {
                const rt::SwingCalibStatus& st = ses.calibStatus();
                std::snprintf(b, sizeof b,
                              "calib: accepted c_bottom = %.3f counts (se %.3f, rms %.2f, amplitude %.1f deg).\n"
                              "Stop the swing by hand and let the pendulum hang still. Hands off the cart and belt.\n"
                              "Press Enter to HOME: the cart drives at %.0f mm/s to the RIGHT switch, then to the LEFT "
                              "switch (step-mode check), then to the centre (~25 s).",
                              st.c_bottom, st.fit.se, st.fit.rms, st.amplitude * 360.0 / sc.balance.calib.cpr,
                              sc.v_seek * 1e3);
                break;
            }
            case rt::SessionEvent::HomeStarted:
                std::snprintf(b, sizeof b, "home: driver ON, seeking the RIGHT switch");
                break;
            case rt::SessionEvent::SwitchFound:
                if (phase == rt::SessionPhase::Home && ses.lastSwitch() == rt::Side::Right)
                    std::snprintf(b, sizeof b, "home: right switch at pos %lld; seeking the LEFT switch",
                                  static_cast<long long>(ses.posRight()));
                else if (phase == rt::SessionPhase::Home)
                    std::snprintf(b, sizeof b, "home: left switch at pos %lld: span %lld usteps = %.3f m (expected %.0f "
                                  "+- %.0f %%): step mode OK; moving to the centre",
                                  static_cast<long long>(ses.posLeft()), static_cast<long long>(ses.posLeft() - ses.posRight()),
                                  (ses.posLeft() - ses.posRight()) / upm, sc.span_usteps, sc.span_tol * 100);
                else {
                    touchErrs.push_back(ses.lastTouchError());
                    std::snprintf(b, sizeof b, "re-centre: %s switch found %+.1f mm from where the step count puts it "
                                  "(step-loss audit); moving to the centre",
                                  ses.lastSwitch() == rt::Side::Right ? "right" : "left", ses.lastTouchError() / upm * 1e3);
                }
                break;
            case rt::SessionEvent::Homed:
                std::snprintf(b, sizeof b, "home: centred at pos %lld. %s", static_cast<long long>(ses.centre()),
                              afterCentre);
                break;
            case rt::SessionEvent::RunEnded: {
                const rt::AttemptStats& a = ses.lastAttempt();
                attempts.push_back(a);
                if (a.swingup)
                    std::snprintf(b, sizeof b,
                                  "\nswing-up run %d: %s. %s, peak energy %.3f of upright, max |x| %.0f mm, peak |v| %.2f m/s, "
                                  "stopped at x %+.0f mm.\nEnter = re-centre (touches the nearer switch) and get READY again; "
                                  "q + Enter = end.",
                                  a.n, a.end_text[0] ? a.end_text : "done",
                                  a.engaged ? fmt("handed over after %.1f s", a.t_handover).append(fmt(", balanced %.1f s",
                                                                                                         a.t_engaged)).c_str()
                                            : "no hand-over",
                                  a.peak_energy, a.max_abs_x * 1e3, a.peak_v, a.x_end * 1e3);
                else if (a.engaged)
                    std::snprintf(b, sizeof b,
                                  "run %d: %s after %.2f s balanced. RMS theta %.2f deg (after 1 s), peak |a| %.2f m/s^2 "
                                  "(saturated %.0f %%), peak |v| %.2f m/s, max |x| %.0f mm, stopped at x %+.0f mm.\n"
                                  "Catch the pendulum and let it hang still. Enter = re-centre (touches the nearer switch, "
                                  "<= %.0f mm/s) and re-arm; q + Enter = end.",
                                  a.n, core::reasonName(static_cast<core::Reason>(a.end_reason)), a.t_engaged,
                                  a.rmsTheta() * kRad2Deg, a.peak_a, a.satFraction() * 100, a.peak_v, a.max_abs_x * 1e3,
                                  a.x_end * 1e3, sc.v_travel * 1e3);
                else
                    std::snprintf(b, sizeof b, "run %d: ended while ARMED: %s. Enter = re-centre and re-arm; q + Enter = end.",
                                  a.n, core::reasonName(static_cast<core::Reason>(a.end_reason)));
                break;
            }
            case rt::SessionEvent::RecentreStarted:
                std::snprintf(b, sizeof b, "re-centre: driver stays ON; to the nearer switch, then the centre");
                break;
            case rt::SessionEvent::Recentred:
                std::snprintf(b, sizeof b, "re-centre: centred. %s", afterCentre);
                break;
            case rt::SessionEvent::SwingReady:
                std::snprintf(b, sizeof b, "\nREADY (swing-up): stand outside the pendulum's circle. Enter = start.");
                break;
            case rt::SessionEvent::SwingStarted:
                std::snprintf(b, sizeof b, "\a%7.2f s  SWING-UP run %d: kick, then pumping. Enter = bring it down, "
                              "q = hard stop.", t, r.attempt);
                break;
            case rt::SessionEvent::SwingNotReady:
                std::snprintf(b, sizeof b, "not READY: the pendulum must hang still (within %.0f deg, slower than %.1f "
                              "rad/s) with the cart centred", sc.balance.sup.hang_tol * kRad2Deg, sc.balance.sup.omega_hang);
                break;
            case rt::SessionEvent::Failed:
                std::snprintf(b, sizeof b, "FAILED: %s. Driver off; the session ends.", ses.reason());
                break;
            case rt::SessionEvent::Finished:
                std::snprintf(b, sizeof b, "session finished: driver off.");
                break;
            default:
                break;
        }
        if (b[0]) std::cout << b << "\n" << std::flush;

        // Balance runtime: transitions, refusals (once per change), status while ENGAGED.
        if (r.mode >= 0) {
            const auto m = static_cast<core::Mode>(r.mode);
            if (r.mode != lastMode) {
                if (m == core::Mode::Armed)
                    std::cout << "\nARMED (run " << r.attempt << "): lift the pendulum to upright and hold the rod lightly "
                                 "near the top with your fingertips, hand still. At the beep (ENGAGED) the cart moves "
                                 "1-2 cm under your hand: let go cleanly about 1 s later. Enter = stop the run.\n";
                else if (m == core::Mode::Engaged)
                    std::cout << "\a" << fmt("%7.2f s  ", t) << "ENGAGED (theta " << fmt("%+.1f", r.theta * kRad2Deg)
                              << " deg): let go in about 1 s\n";
                else if (m == core::Mode::Stopping)
                    std::cout << fmt("%7.2f s  ", t) << "STOPPING: "
                              << core::reasonName(static_cast<core::Reason>(r.event)) << "\n";
                std::cout << std::flush;
                lastRefusal = -1;
            }
            const bool refusal = r.event == static_cast<int>(core::Reason::ArmNotHanging) ||
                                 r.event == static_cast<int>(core::Reason::ArmMoving) ||
                                 r.event == static_cast<int>(core::Reason::ArmNotCentred) ||
                                 r.event == static_cast<int>(core::Reason::LimitSwitch) ||
                                 r.event == static_cast<int>(core::Reason::RemoteFault);
            if (m == core::Mode::Idle && refusal && r.event != lastRefusal && phase == rt::SessionPhase::Balance) {
                std::cout << "waiting to arm: " << core::reasonName(static_cast<core::Reason>(r.event)) << "\n" << std::flush;
                lastRefusal = r.event;
            }
            if (m == core::Mode::Engaged && t >= nextStatus) {
                nextStatus = t + statusPeriod;
                char s[200];
                std::snprintf(s, sizeof s, "%7.2f s  theta %+6.2f deg  x %+6.1f mm  v %+6.3f m/s  a %+5.2f m/s^2%s", t,
                              r.theta * kRad2Deg, r.x * 1e3, r.v, r.a_cmd, r.saturated ? " (sat)" : "");
                std::cout << s;
                if (link) {
                    std::uint64_t n, lateN;
                    std::int64_t mx;
                    link->takeWindow(&n, &mx, &lateN);
                    std::snprintf(s, sizeof s, "  | PC: worst %.2f ms, %llu late, %ld missed", mx * 1e-6,
                                  static_cast<unsigned long long>(lateN), remoteMiss);
                    std::cout << s;
                }
                std::cout << "\n" << std::flush;
            }
            lastMode = r.mode;
        } else {
            lastMode = -1;
        }
        // Swing-up runtime: phase changes and a status line every status period.
        if (r.swing_phase >= 0) {
            const auto sp = static_cast<rt::SwingPhase>(r.swing_phase);
            if (r.swing_phase != lastSwing) {
                std::cout << (sp == rt::SwingPhase::Balance ? "\a" : "") << fmt("%7.2f s  ", t) << rt::swingPhaseName(sp)
                          << fmt("  (energy %.3f of upright", r.energy) << fmt(", theta %+.1f deg", r.theta * kRad2Deg)
                          << fmt(", x %+.0f mm)", r.x * 1e3) << "\n" << std::flush;
                lastSwing = r.swing_phase;
            }
            if (t >= nextStatus) {
                nextStatus = t + statusPeriod;
                char s[200];
                std::snprintf(s, sizeof s, "%7.2f s  %-8s energy %5.3f  theta %+7.1f deg  x %+6.1f mm  v %+6.3f m/s  a %+5.2f",
                              t, rt::swingPhaseName(sp), r.energy, r.theta * kRad2Deg, r.x * 1e3, r.v, r.a_cmd);
                std::cout << s << "\n" << std::flush;
            }
        } else {
            lastSwing = -1;
        }
        if (phase == rt::SessionPhase::Calibrate && t >= nextCal) {
            nextCal = t + 1.0;
            const rt::SwingCalibStatus& st = ses.calibStatus();
            char s[240];
            std::snprintf(s, sizeof s, "calib: %4.0f s  amplitude %4.1f deg, %4d edges, rms %5.2f, se %5.3f: %s", t,
                          st.amplitude * 360.0 / sc.balance.calib.cpr, st.n_edges, st.fit.ok ? st.fit.rms : 0.0,
                          st.fit.ok ? st.fit.se : 0.0, st.why);
            std::cout << s << "\n" << std::flush;
        }
        if ((phase == rt::SessionPhase::Home || phase == rt::SessionPhase::Recentre) && t >= nextMove) {
            nextMove = t + 2.0;
            char s[160];
            std::snprintf(s, sizeof s, "  %s: pos %lld, v %+.3f m/s, pendulum %+.1f deg from hanging",
                          rt::sessionPhaseName(phase), static_cast<long long>(r.pos_usteps), r.v,
                          core::wrap(r.theta - 3.141592653589793) * kRad2Deg);
            std::cout << s << "\n" << std::flush;
        }
        if (ses.finished()) break;
    }
    if (!halted) pi.commandAccel(0);
    pi.enableDriver(false);
    flags.motionSafe = true;
    inputRun = false;
    input.join();
    const hw::Sample sEnd = pi.read();
    activeAxis = nullptr;

    const std::string dir = runlog::makeRunDir(logDir, "balance");
    writeCsv(log, dir + "/data.csv",
             "tick,late_ns,t_ns,phase,attempt,mode,event,sevent,counts,pos_usteps,speed_toggles,n_edges,theta,omega,x,v,"
             "a_cmd,x_ref,integral,axis_arg,sent,saturated,overrun,lim_l,lim_r,axis_acc,driver,swing_phase,energy,"
             "ctrl,epoch,a_local,r_miss",
             [](std::ostream& o, const BalRec& x) {
                 const rt::SessionRecord& r = x.r;
                 char b[480];
                 std::snprintf(b, sizeof b,
                               "%u,%lld,%lld,%d,%d,%d,%d,%d,%lld,%lld,%lld,%d,%.7f,%.6f,%.7f,%.7f,%.6f,%.7f,%.7f,%d,%d,%d,%d,%d,%d,%d,%d,"
                               "%d,%.5f,%d,%u,%.6f,%d",
                               x.tick, static_cast<long long>(x.late_ns), static_cast<long long>(r.t_ns), r.phase, r.attempt,
                               r.mode, r.event, r.sevent, static_cast<long long>(r.counts),
                               static_cast<long long>(r.pos_usteps), static_cast<long long>(r.speed_toggles), r.n_edges,
                               r.theta, r.omega, r.x, r.v, r.a_cmd, r.x_ref, r.integral, r.axis_arg, r.sent ? 1 : 0,
                               r.saturated ? 1 : 0, r.overrun ? 1 : 0, r.limit_left ? 1 : 0, r.limit_right ? 1 : 0,
                               r.axis_accelerating ? 1 : 0, r.driver_enabled ? 1 : 0, r.swing_phase, r.energy,
                               r.controller ? 1 : 0, r.epoch, r.a_local, r.remote_miss ? 1 : 0);
                 o << b;
             });
    if (link) writeRemoteCsv(*link, dir + "/remote.csv");
    writeCsv(edgeLog, dir + "/edges.csv", "ts_ns,count_after,tick", [](std::ostream& o, const EdgeRec& e) {
        char b[96];
        std::snprintf(b, sizeof b, "%lld,%lld,%u", static_cast<long long>(e.ts_ns), static_cast<long long>(e.count_after),
                      e.tick);
        o << b;
    });
    // Per-attempt summary: n engaged t_engaged end_reason rms_theta_deg peak_a sat_fraction peak_v max_abs_x x_end
    std::string runs, touches;
    for (const rt::AttemptStats& a : attempts) {
        char b[260];
        if (a.swingup) {   // swing-up: n handed_over t_handover t_balanced end peak_energy peak_v max_abs_x x_end
            std::snprintf(b, sizeof b, "%sswingup %d %d %.3f %.3f %s %.4f %.3f %.4f %.4f", runs.empty() ? "" : "; ", a.n,
                          a.engaged ? 1 : 0, a.t_handover, a.t_engaged, a.end_text, a.peak_energy, a.peak_v, a.max_abs_x,
                          a.x_end);
            runs += b;
            continue;
        }
        std::snprintf(b, sizeof b, "%s%d %d %.3f %s %.3f %.2f %.3f %.3f %.4f %.4f", runs.empty() ? "" : "; ", a.n,
                      a.engaged ? 1 : 0, a.t_engaged, core::reasonName(static_cast<core::Reason>(a.end_reason)),
                      a.rmsTheta() * kRad2Deg, a.peak_a, a.satFraction(), a.peak_v, a.max_abs_x, a.x_end);
        runs += b;
    }
    for (long long e : touchErrs) touches += (touches.empty() ? "" : " ") + std::to_string(e);
    const rt::SwingCalibStatus& st = ses.calibStatus();
    const std::string result = std::string(rt::sessionPhaseName(ses.phase())) + (ses.reason()[0] ? std::string(": ") + ses.reason() : std::string()) +
                               (halted ? " (HALT)" : "");
    runlog::Fields meta{{"chip", chip + " (" + pi.chipLabel() + ")"},
                       {"result", result},
                       {"c_bottom_fit_counts", fmt("%.4f", st.c_bottom)},
                       {"c_bottom_se_counts", fmt("%.4f", st.fit.se)},
                       {"c_bottom_fit_rms_counts", fmt("%.4f", st.fit.rms)},
                       {"calib_amplitude_counts", fmt("%.2f", st.amplitude)},
                       {"c_bottom_used_counts", fmt("%.4f", ses.cBottom())},
                       {"kp", fmt("%.4f", sc.balance.pid.kp)},
                       {"ki", fmt("%.4f", sc.balance.pid.ki)},
                       {"kd", fmt("%.4f", sc.balance.pid.kd)},
                       {"a_max", fmt("%.3f", sc.balance.pid.a_max)},
                       {"v_max", fmt("%.3f", sc.balance.pid.v_max)},
                       {"pos_right_usteps", std::to_string(ses.posRight())},
                       {"pos_left_usteps", std::to_string(ses.posLeft())},
                       {"span_usteps", std::to_string(ses.posLeft() - ses.posRight())},
                       {"centre_usteps", std::to_string(ses.centre())},
                       {"attempts", std::to_string(attempts.size())},
                       {"runs", runs},
                       {"touch_errors_usteps", touches},
                       {"pos_start_usteps", std::to_string(s0.pos_usteps)},
                       {"pos_end_usteps", std::to_string(sEnd.pos_usteps)},
                       {"encoder_same_type", std::to_string(pi.encoder().sameType())},
                       {"encoder_full_batches", std::to_string(pi.encoder().fullBatches())},
                       {"records", std::to_string(log.size())},
                       {"records_overwritten", std::to_string(log.overwritten())}};
    if (link) {
        const runlog::Fields rf = remoteMetaFields(*link, sc.balance.remote_delay, ts);
        meta.insert(meta.end(), rf.begin(), rf.end());
        meta.push_back({"remote_missed_ticks", std::to_string(remoteMiss)});
        const LinkStats& ls2 = link->stats();
        std::cout << fmt("remote: worst round trip %.2f ms", ls2.rtt_max_ns * 1e-6) << " (delay "
                  << fmt("%.1f ms)", sc.balance.remote_delay * ts * 1e3) << ", " << ls2.late << " late replies, "
                  << remoteMiss << " missed ticks, max |PC - local PID| " << ls2.max_abs_diff << " m/s^2\n";
    }
    runlog::writeMeta(dir, "balance", cmdLine, cfg, configPath, meta);
    std::cout << "balance: " << result << "; " << attempts.size() << " runs\n  -> " << dir << "\n";
    return ses.phase() == rt::SessionPhase::Finished ? 0 : 3;
}

}
