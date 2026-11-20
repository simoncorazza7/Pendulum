// actuator (Stage 0.9, actuator capability): rt::ActuatorTest on hw::PiHw. HARDWARE ACTION: it homes the cart
// from switch to switch, then runs a ladder of hard moves with the pendulum hanging, touching a switch after
// every rep (step-loss audit). docs/stage0_9_actuator.md; ./actuator.sh runs it after a checklist.
//
//   1. Enter: homing (right switch, left switch: span = step-mode check, centre)
//   2. READY prints the next rung (what moves, how far, how fast, how hard); Enter runs it, s + Enter skips the
//      rest of that speed
//   3. each rep: move to its start, wait for the pendulum to settle, run, brake, touch a switch, verdict
//   q + Enter or Ctrl-C: controlled stop (a running rep brakes at its own acceleration), driver off, logs.
//   Refuses the real chip unless actuator.allow_real_chip, and refuses to start on a pressed switch.

#include <poll.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "app/modes.h"
#include "app/rig.h"
#include "common/runlog.h"
#include "hw/pi_hw.h"
#include "rt/ringlog.h"
#include "rt/rt.h"
#include "runtime/actuator.h"
#include "runtime/params.h"

namespace app {
namespace {

struct ActRec {
    std::uint32_t tick;
    std::int64_t late_ns;
    rt::ActRecord r;
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

std::string fmt(const char* f, double v) {
    char b[64];
    std::snprintf(b, sizeof b, f, v);
    return b;
}

// What the rung moves, how far, fast and hard, how it ends and how to stop it (CLAUDE.md: before each run).
std::string rungText(const rt::ActuatorTest& act, int i) {
    const rt::ActuatorConfig& c = act.config();
    const rt::ActRung& g = act.rungs()[static_cast<std::size_t>(i)];
    char b[700];
    const char* what = "";
    std::string where;
    switch (c.profile) {
        case rt::ActProfile::Pair:
            what = "out-and-back legs";
            where = fmt("legs of %.0f mm centred (x -", g.leg * 1e3) + fmt("%.0f..+", g.leg * 500) + fmt("%.0f mm)", g.leg * 500);
            break;
        case rt::ActProfile::OneWay:
            what = "one-way legs, alternating direction";
            where = fmt("legs of %.0f mm centred (x -", g.leg * 1e3) + fmt("%.0f..+", g.leg * 500) + fmt("%.0f mm)", g.leg * 500);
            break;
        case rt::ActProfile::Bang:
            what = "a bang-bang stream";
            where = fmt("x within +-%.0f mm", g.leg * 1e3);
            break;
        case rt::ActProfile::Chirp:
            what = "a chirp stream";
            where = fmt("x within +-%.0f mm", g.leg * 1e3) + fmt(", %.2f Hz", g.f0) + fmt(" to %.0f Hz", c.chirp_f1);
            break;
        case rt::ActProfile::Table:
            what = "a replayed table";
            where = fmt("x within +-%.0f mm", g.leg * 1e3);
            break;
    }
    std::snprintf(b, sizeof b,
                  "NEXT: rung %d of %zu, %s: |v| <= %.2f m/s, |a| <= %.1f m/s^2, %s, %d rep%s of %.1f s each.\n"
                  "  Each rep travels to its start at %.0f mm/s, waits for the pendulum to settle below %.0f deg, runs,\n"
                  "  then touches a switch at %.0f mm/s (step-loss audit). Enter = run, s + Enter = skip the rest of\n"
                  "  %.2f m/s, q + Enter = stop (a running rep brakes at %.1f m/s^2), motor cut = emergency.",
                  i + 1, act.rungs().size(), what, g.peak_v, g.peak_a, where.c_str(), c.reps, c.reps == 1 ? "" : "s",
                  g.duration, c.v_travel * 1e3, c.settle_deg, c.v_seek * 1e3, g.v, g.a);
    return b;
}

std::string readyText(const rt::ActuatorTest& act) {
    const int n = act.nextRung();
    if (n < 0) return "Ladder done. Enter or q + Enter ends the block (cart held at the centre until then).";
    return rungText(act, n);
}

}

int runActuator(const Config& cfg, const std::string& configPath, const std::string& cmdLine) {
    const std::string chip = cfg.getString("actuator.chip");
    const bool allowReal = cfg.getBool("actuator.allow_real_chip");
    const double logSeconds = cfg.getDouble("actuator.log_seconds");      // s
    const double statusPeriod = cfg.getDouble("actuator.status_period");  // s
    const double vref = cfg.getDouble("actuator.vref");                   // V, 0 = not entered
    const bool lockMem = cfg.getBool("rt.lock_memory"), rtOn = cfg.getBool("rt.enable");
    const std::string logDir = cfg.getString("log.dir");
    rt::ActuatorConfig ac = rt::actuatorFrom(cfg);                       // throws on bad parameters
    const std::string tablePath = cfg.getString("actuator.table");
    if (ac.profile == rt::ActProfile::Table) {
        std::string e;
        if (tablePath.empty()) { std::cerr << "actuator: profile table needs actuator.table\n"; return 2; }
        if (!rt::loadActTable(tablePath, ac.ts, &ac.table, &e)) { std::cerr << "actuator: " << e << "\n"; return 2; }
    }
    const double ts = ac.ts;
    const double upm = ac.calib.usteps_per_m;
    const hw::PiRt rtc = rtFrom(cfg);

    rt::ActuatorTest act(ac);                  // sizes every rung before any GPIO is touched
    int runnable = 0;
    std::cout << "actuator test, profile " << rt::actProfileName(ac.profile) << ", " << ac.reps << " rep"
              << (ac.reps == 1 ? "" : "s") << " per rung:\n";
    for (std::size_t i = 0; i < act.rungs().size(); ++i) {
        const rt::ActRung& g = act.rungs()[i];
        char b[220];
        if (g.runnable) {
            ++runnable;
            std::snprintf(b, sizeof b, "  rung %2zu: v %.2f m/s, a %4.1f m/s^2: x span %3.0f mm, %.1f s, peak |v| %.2f, |a| %.1f",
                          i + 1, g.v, g.a, g.leg * 1e3, g.duration, g.peak_v, g.peak_a);
        } else {
            std::snprintf(b, sizeof b, "  rung %2zu: v %.2f m/s, a %4.1f m/s^2: not run (%s)", i + 1, g.v, g.a, g.why);
        }
        std::cout << b << "\n";
    }
    if (runnable == 0) { std::cerr << "actuator: no runnable rung\n"; return 2; }

    RingLog<ActRec> log(static_cast<std::size_t>(logSeconds / ts), true);   // keeps the newest
    RingLog<EdgeRec> edgeLog(2000000, true);
    std::string err;
    if (lockMem && !rt::lockMemory(512 * 1024, &err)) std::cerr << "warning: " << err << "\n";

    hw::PiHw pi(chip, pinsFrom(cfg), allowReal, static_cast<int>(cfg.getInt("axis.max_accel")));   // throws if refused
    activeAxis = pi.axis();
    if (!pi.start(rtc, &err)) { std::cerr << "actuator: " << err << "\n"; activeAxis = nullptr; return 1; }
    if (rtOn && !rt::makeCurrentThreadRealtime(static_cast<int>(cfg.getInt("rt.loop_prio")),
                                               static_cast<int>(cfg.getInt("rt.loop_cpu")), &err))
        std::cerr << "warning: " << err << "\n";

    const hw::Sample s0 = pi.read();
    if (s0.limit_left || s0.limit_right) {
        std::cerr << "actuator: a limit switch is pressed at start; refusing (push the cart off it by hand, "
                     "roughly to the middle)\n";
        activeAxis = nullptr;
        return 2;
    }
    std::cout << "actuator test on " << chip << " (" << pi.chipLabel() << "). Let the pendulum HANG STILL, hands off the "
              << "cart, belt and pendulum.\nPress Enter to HOME: the cart drives at " << fmt("%.0f", ac.v_seek * 1e3)
              << " mm/s to the RIGHT switch, then to the LEFT switch (step-mode check), then to the centre (~25 s).\n"
              << "q + Enter or Ctrl-C = controlled stop at any time.\n" << std::flush;

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
                if (line == "q" || line == "quit") quitReq = true;
                else if (line == "s" || line == "skip") act.requestSkip();
                else act.requestGo();
                line.clear();
            }
        }
    });

    TickClock clk(ts);
    flags.motionSafe = false;
    bool quitSent = false, halted = false;
    double tQuit = 0, nextStatus = 0;
    while (true) {
        const std::int64_t late = clk.wait();
        const double t = clk.seconds();
        if (!quitSent && (flags.stop || quitReq)) {
            act.requestQuit();
            quitSent = true;
            tQuit = t;
        }
        if (quitSent && t - tQuit > 5.0) {     // never expected: every phase ends within ~3 s
            std::cerr << "actuator: no controlled end within 5 s, HALT\n";
            pi.halt();
            halted = true;
            break;
        }
        const rt::ActRecord r = act.tick(hw, clk.overrun());
        log.push({clk.tick(), late, r});
        const hw::Sample& ls = hw.last();
        for (int i = 0; i < ls.n_edges; ++i)
            edgeLog.push({ls.edges[static_cast<std::size_t>(i)].ts_ns, ls.edges[static_cast<std::size_t>(i)].count_after, clk.tick()});

        const auto phase = static_cast<rt::ActPhase>(r.phase);
        const rt::ActRung* g = r.rung >= 0 ? &act.rungs()[static_cast<std::size_t>(r.rung)] : nullptr;
        char b[900];
        b[0] = 0;
        switch (static_cast<rt::ActEvent>(r.event)) {
            case rt::ActEvent::HomeStarted:
                std::snprintf(b, sizeof b, "home: driver ON, seeking the RIGHT switch");
                break;
            case rt::ActEvent::SwitchFound:
                if (act.lastSwitch() == rt::Side::Right)
                    std::snprintf(b, sizeof b, "home: right switch at pos %lld; seeking the LEFT switch",
                                  static_cast<long long>(act.posRight()));
                else
                    std::snprintf(b, sizeof b, "home: left switch at pos %lld: span %lld usteps = %.3f m (expected %.0f +- "
                                  "%.0f %%): step mode OK; moving to the centre", static_cast<long long>(act.posLeft()),
                                  static_cast<long long>(act.posLeft() - act.posRight()),
                                  (act.posLeft() - act.posRight()) / upm, ac.span_usteps, ac.span_tol * 100);
                break;
            case rt::ActEvent::Homed:
                std::snprintf(b, sizeof b, "home: centred at pos %lld.\n%s", static_cast<long long>(act.centre()),
                              readyText(act).c_str());
                break;
            case rt::ActEvent::RepStarted:
                std::snprintf(b, sizeof b, "%7.2f s  rung %d (v %.2f m/s, a %.1f m/s^2): to the start of rep 1", t,
                              r.rung + 1, g ? g->v : 0.0, g ? g->a : 0.0);
                break;
            case rt::ActEvent::RunStarted:
            case rt::ActEvent::SettleTimeout: {
                const bool again = !act.results().empty() && act.results().back().rung == r.rung &&
                                   act.results().back().rep == r.rep && act.results().back().verdict == rt::Verdict::Marginal;
                std::snprintf(b, sizeof b, "%s%7.2f s  RUN rung %d rep %d%s (pendulum swing %.1f deg)",
                              r.event == static_cast<int>(rt::ActEvent::SettleTimeout) ? "pendulum did not settle: " : "",
                              t, r.rung + 1, r.rep, again ? " again (the last touch was marginal)" : "", r.swing_deg);
                break;
            }
            case rt::ActEvent::RunEnded:
                if (phase == rt::ActPhase::Run || phase == rt::ActPhase::Brake) {
                    if (!act.results().empty() && act.results().back().verdict == rt::Verdict::Aborted &&
                        act.results().back().rung == r.rung)
                        std::snprintf(b, sizeof b, "%7.2f s  %s: braking at %.1f m/s^2", t, act.reason(), g ? g->a : 0.0);
                    else
                        std::snprintf(b, sizeof b, "%7.2f s  run done; braking, then touching a switch", t);
                }
                break;
            case rt::ActEvent::Touched:
            case rt::ActEvent::RungDone:
            case rt::ActEvent::Failed: {
                if (!act.results().empty() && r.event != static_cast<int>(rt::ActEvent::Failed)) {
                    const rt::RepResult& x = act.results().back();
                    std::snprintf(b, sizeof b, "%7.2f s  rung %d rep %d: %s switch %+lld usteps (%+.2f mm) -> %s; peak |a_cmd| "
                                  "%.2f m/s^2, peak |v| %.3f m/s, max tracking %.2f mm", t, x.rung + 1, x.rep,
                                  x.side == 1 ? "right" : "left", static_cast<long long>(x.touch_err), x.touch_err / upm * 1e3,
                                  rt::verdictName(x.verdict), x.peak_a_cmd, x.peak_v, x.max_track * 1e3);
                    std::cout << b << "\n";
                    b[0] = 0;
                }
                if (r.event == static_cast<int>(rt::ActEvent::RungDone) && g) {
                    const rt::Verdict v = act.rungVerdicts()[static_cast<std::size_t>(r.rung)];
                    std::snprintf(b, sizeof b, "rung %d (v %.2f m/s, a %.1f m/s^2): %s%s; back to the centre", r.rung + 1, g->v,
                                  g->a, rt::verdictName(v),
                                  v == rt::Verdict::Fail ? " -> the rest of this speed is skipped" : "");
                } else if (r.event == static_cast<int>(rt::ActEvent::Failed)) {
                    std::snprintf(b, sizeof b, "FAILED: %s. Driver off; the block ends.", act.reason());
                }
                break;
            }
            case rt::ActEvent::Ready:
                std::snprintf(b, sizeof b, "\a%s", readyText(act).c_str());
                break;
            case rt::ActEvent::Skipped:
                std::snprintf(b, sizeof b, "skipped.\n%s", readyText(act).c_str());
                break;
            case rt::ActEvent::Finished:
                std::snprintf(b, sizeof b, "block finished: driver off.");
                break;
            default:
                break;
        }
        if (b[0]) std::cout << b << "\n" << std::flush;

        if (t >= nextStatus) {
            char s[220];
            s[0] = 0;
            if (phase == rt::ActPhase::Run) {
                nextStatus = t + statusPeriod;
                std::snprintf(s, sizeof s, "%7.2f s  RUN  x %+6.1f mm (ref %+6.1f)  v %+6.3f m/s  a %+6.2f m/s^2  swing %4.1f deg",
                              t, r.x * 1e3, r.x_ref * 1e3, r.v, r.a_cmd, r.swing_deg);
            } else if (phase == rt::ActPhase::Settle) {
                nextStatus = t + 1.0;
                std::snprintf(s, sizeof s, "%7.2f s  SETTLE  pendulum swing %4.1f deg (starts below %.0f deg)", t, r.swing_deg,
                              ac.settle_deg);
            } else if (phase == rt::ActPhase::Home || phase == rt::ActPhase::ToStart || phase == rt::ActPhase::Touch ||
                       phase == rt::ActPhase::Centre) {
                nextStatus = t + 2.0;
                std::snprintf(s, sizeof s, "  %s: pos %lld, v %+.3f m/s, pendulum swing %.1f deg", rt::actPhaseName(phase),
                              static_cast<long long>(r.pos_usteps), r.v, r.swing_deg);
            }
            if (s[0]) std::cout << s << "\n" << std::flush;
        }
        if (act.finished()) break;
    }
    if (!halted) pi.commandAccel(0);
    pi.enableDriver(false);
    flags.motionSafe = true;
    inputRun = false;
    input.join();
    const hw::Sample sEnd = pi.read();
    activeAxis = nullptr;

    const std::string dir = runlog::makeRunDir(logDir, "actuator");
    writeCsv(log, dir + "/data.csv",
             "tick,late_ns,t_ns,phase,event,rung,rep,counts,pos_usteps,speed_toggles,n_edges,x,v,x_ref,v_ref,a_ref,a_cmd,"
             "axis_arg,swing_deg,sent,overrun,lim_l,lim_r,axis_acc,driver,enc_fault",
             [](std::ostream& o, const ActRec& x) {
                 const rt::ActRecord& r = x.r;
                 char b[400];
                 std::snprintf(b, sizeof b,
                               "%u,%lld,%lld,%d,%d,%d,%d,%lld,%lld,%lld,%d,%.7f,%.7f,%.7f,%.7f,%.5f,%.5f,%d,%.3f,%d,%d,%d,%d,%d,%d,%d",
                               x.tick, static_cast<long long>(x.late_ns), static_cast<long long>(r.t_ns), r.phase, r.event,
                               r.rung, r.rep, static_cast<long long>(r.counts), static_cast<long long>(r.pos_usteps),
                               static_cast<long long>(r.speed_toggles), r.n_edges, r.x, r.v, r.x_ref, r.v_ref, r.a_ref,
                               r.a_cmd, r.axis_arg, r.swing_deg, r.sent ? 1 : 0, r.overrun ? 1 : 0, r.limit_left ? 1 : 0,
                               r.limit_right ? 1 : 0, r.axis_accelerating ? 1 : 0, r.driver_enabled ? 1 : 0,
                               r.encoder_fault ? 1 : 0);
                 o << b;
             });
    writeCsv(edgeLog, dir + "/edges.csv", "ts_ns,count_after,tick", [](std::ostream& o, const EdgeRec& e) {
        char b[96];
        std::snprintf(b, sizeof b, "%lld,%lld,%u", static_cast<long long>(e.ts_ns), static_cast<long long>(e.count_after),
                      e.tick);
        o << b;
    });
    {   // one row per rep and per rung (py/actuator_report.py)
        std::ofstream f(dir + "/reps.csv");
        f << "rung,rep,repeat,v,a,side,touch_err_usteps,verdict,peak_a_cmd,peak_v,max_track_m,swing_start_deg,"
             "settle_timeout,t_run\n";
        for (const rt::RepResult& x : act.results()) {
            char b[260];
            std::snprintf(b, sizeof b, "%d,%d,%d,%.4f,%.4f,%d,%lld,%s,%.4f,%.4f,%.6f,%.2f,%d,%.3f\n", x.rung + 1, x.rep,
                          x.repeat ? 1 : 0, x.v, x.a, x.side, static_cast<long long>(x.touch_err), rt::verdictName(x.verdict),
                          x.peak_a_cmd, x.peak_v, x.max_track, x.swing_start_deg, x.settle_timeout ? 1 : 0, x.t_run);
            f << b;
        }
        std::ofstream g(dir + "/rungs.csv");
        g << "rung,speed_index,v,a,leg_m,duration_s,peak_v,peak_a,f0_hz,runnable,verdict\n";
        for (std::size_t i = 0; i < act.rungs().size(); ++i) {
            const rt::ActRung& x = act.rungs()[i];
            char b[220];
            std::snprintf(b, sizeof b, "%zu,%d,%.4f,%.4f,%.4f,%.3f,%.4f,%.4f,%.3f,%d,%s\n", i + 1, x.speed_index, x.v, x.a,
                          x.leg, x.duration, x.peak_v, x.peak_a, x.f0, x.runnable ? 1 : 0,
                          rt::verdictName(act.rungVerdicts()[i]));
            g << b;
        }
    }
    std::string rungs, touches;
    for (std::size_t i = 0; i < act.rungs().size(); ++i) {
        const rt::Verdict v = act.rungVerdicts()[i];
        if (v == rt::Verdict::NotRun || (v == rt::Verdict::Skipped && !act.rungs()[i].runnable)) continue;
        rungs += (rungs.empty() ? "" : "; ") + fmt("%.2f", act.rungs()[i].v) + " " + fmt("%.1f", act.rungs()[i].a) + " " +
                 rt::verdictName(v);
    }
    for (const rt::RepResult& x : act.results()) touches += (touches.empty() ? "" : " ") + std::to_string(x.touch_err);
    const std::string result = std::string(rt::actPhaseName(act.phase())) +
                               (act.phase() == rt::ActPhase::Failed ? std::string(": ") + act.reason() : std::string()) +
                               (halted ? " (HALT)" : "");
    runlog::writeMeta(dir, "actuator", cmdLine, cfg, configPath,
                      {{"chip", chip + " (" + pi.chipLabel() + ")"},
                       {"result", result},
                       {"profile", rt::actProfileName(ac.profile)},
                       {"vref_v", fmt("%.3f", vref)},
                       {"table", ac.profile == rt::ActProfile::Table ? tablePath : std::string()},
                       {"pos_right_usteps", std::to_string(act.posRight())},
                       {"pos_left_usteps", std::to_string(act.posLeft())},
                       {"span_usteps", std::to_string(act.posLeft() - act.posRight())},
                       {"centre_usteps", std::to_string(act.centre())},
                       {"rungs", rungs},
                       {"reps", std::to_string(act.results().size())},
                       {"touch_errors_usteps", touches},
                       {"pos_start_usteps", std::to_string(s0.pos_usteps)},
                       {"pos_end_usteps", std::to_string(sEnd.pos_usteps)},
                       {"encoder_same_type", std::to_string(pi.encoder().sameType())},
                       {"encoder_full_batches", std::to_string(pi.encoder().fullBatches())},
                       {"records", std::to_string(log.size())},
                       {"records_overwritten", std::to_string(log.overwritten())}});
    std::cout << "actuator: " << result << "; " << act.results().size() << " reps\n";
    for (std::size_t i = 0; i < act.rungs().size(); ++i) {
        const rt::Verdict v = act.rungVerdicts()[i];
        if (v == rt::Verdict::NotRun) continue;
        std::cout << fmt("  rung %2.0f: ", static_cast<double>(i + 1)) << fmt("v %.2f m/s, ", act.rungs()[i].v)
                  << fmt("a %4.1f m/s^2: ", act.rungs()[i].a) << rt::verdictName(v) << "\n";
    }
    std::cout << "  -> " << dir << "\n";
    return act.phase() == rt::ActPhase::Finished ? 0 : 3;
}

}
