// remote-sim (Stage 2b, docs/remote.md): the remote controller end to end with a SIMULATED pendulum. No GPIO
// is opened and nothing moves: hw::SimHw (the identified rig plant) runs in real time inside the fixed-rate
// loop, rt::BalanceRuntime with the real UDP link (app::RemoteLink) applies the PC's commands with the constant
// delay remote.delay_ticks, exactly as balance.remote does on the rig. Run py/remote_ctrl.py on the PC first.
//
//   1. waits for the PC (link up), arms while the simulated pendulum hangs at the track centre
//   2. a simulated hand lifts it to upright (1 s) and holds it still: ENGAGED; the hand lets go 0.5 s later
//   3. taps of remote.sim_tap_deg every remote.sim_tap_period s; after remote.sim_seconds engaged: stop
// Ends on any abort too. Logs as the balance session: data.csv (+ true theta), remote.csv, meta.json.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <string>

#include "app/modes.h"
#include "app/remote_link.h"
#include "common/runlog.h"
#include "hw/sim_hw.h"
#include "rt/ringlog.h"
#include "rt/rt.h"
#include "runtime/params.h"
#include "runtime/session.h"

namespace app {
namespace {

struct SimRec {
    std::uint32_t tick;
    std::int64_t late_ns;
    rt::TickRecord r;
    double true_theta;
};

constexpr double kDeg = 3.141592653589793 / 180.0;

}

int runRemoteSim(const Config& cfg, const std::string& configPath, const std::string& cmdLine) {
    rt::SessionConfig sc = rt::sessionFrom(cfg);          // the balance session's gains and limits
    if (!sc.remote_mode) sc.remote_mode = true;           // remote is the point of this mode
    const double ts = sc.balance.ts;
    const double simSeconds = cfg.getDouble("remote.sim_seconds");
    const double tapPeriod = cfg.getDouble("remote.sim_tap_period");
    const double tapDeg = cfg.getDouble("remote.sim_tap_deg");
    const double waitPc = cfg.getDouble("remote.sim_wait_pc");
    const bool rtOn = cfg.getBool("rt.enable");
    const std::string logDir = cfg.getString("log.dir");
    if (sc.balance.remote_delay < 1) throw std::runtime_error("remote.delay_ticks must be >= 1");

    // The identified rig plant on a 20 m track (switches +-10 m, soft limits +-9.5 m): the angle-only PID's cart
    // drift after a catch (docs/stage1_drift.md) would end a run on the real 0.3 m track within seconds; this mode
    // tests the link and measures the latency over minutes. Everything else is the session's configuration.
    rt::BalanceConfig bc = sc.balance;
    bc.sup.x_min = -9.5;
    bc.sup.x_max = 9.5;
    hw::SimHwParams p;
    p.w0 = cfg.getDouble("plant.w0");
    p.bJ = cfg.getDouble("plant.bJ");
    p.coulomb = cfg.getDouble("plant.coulomb");
    p.axis_a = bc.sup.axis_a;
    p.axis_v = bc.sup.axis_v;
    p.truth = bc.calib;
    p.truth.c_bottom = 12.37;
    const double upm = bc.calib.usteps_per_m;
    p.truth.pos_center = 200000.0;
    p.pos_switch_right = p.truth.pos_center - 10.0 * upm;
    p.pos_switch_left = p.truth.pos_center + 10.0 * upm;
    p.pos_end_right = p.pos_switch_right - 0.015 * upm;
    p.pos_end_left = p.pos_switch_left + 0.015 * upm;
    hw::SimHw sim(p, 3.141592653589793, 0.0, 0.0);        // hanging at rest at the centre
    bc.calib.c_bottom = sim.trueCBottomRelative();
    bc.calib.pos_center = p.truth.pos_center;
    bc.actuate = true;
    bc.keep_driver_on = true;

    RemoteLink link(static_cast<int>(cfg.getInt("remote.port")), bc.remote_delay, cfg.getDouble("remote.link_timeout"),
                    static_cast<std::size_t>(cfg.getDouble("balance.log_seconds") / ts));
    std::string err;
    if (!link.open(&err)) { std::cerr << err << "\n"; return 1; }
    {
        proto::Params pp;
        const core::PidParams& g = bc.pid;
        pp.ts = ts; pp.kp = g.kp; pp.ki = g.ki; pp.kd = g.kd; pp.a_max = g.a_max; pp.v_max = g.v_max;
        pp.v_taper = g.v_taper; pp.i_max = g.i_max; pp.i_delay = g.i_delay; pp.theta_ref = g.theta_ref; pp.vel_fc = g.vel_fc;
        link.setParams(pp);
    }
    bc.remote = &link;
    rt::BalanceRuntime brt(bc);

    if (rtOn && !rt::makeCurrentThreadRealtime(static_cast<int>(cfg.getInt("rt.loop_prio")),
                                               static_cast<int>(cfg.getInt("rt.loop_cpu")), &err))
        std::cerr << "warning: " << err << " (timing without SCHED_FIFO)\n";
    {
        char b[400];
        std::snprintf(b, sizeof b,
                      "remote-sim: SIMULATED pendulum (rig plant, 20 m track), no GPIO. UDP port %d, constant delay %d "
                      "ticks = %.1f ms, abort "
                      "after %d missing commands in a row. PID Kp %.2f Ki %.2f Kd %.3f. Waiting up to %.0f s for the PC "
                      "(py/remote_ctrl.py --host <this Pi>).\n",
                      static_cast<int>(cfg.getInt("remote.port")), bc.remote_delay, bc.remote_delay * ts * 1e3,
                      bc.remote_max_miss, bc.pid.kp, bc.pid.ki, bc.pid.kd, waitPc);
        std::cout << b << std::flush;
    }

    RingLog<SimRec> log(static_cast<std::size_t>(cfg.getDouble("balance.log_seconds") / ts), true);
    const std::int64_t period = static_cast<std::int64_t>(std::llround(ts * 1e9));
    std::int64_t next = rt::monoNs() + period;
    std::uint32_t k = 0;
    int overruns = 0;
    double t = 0, tArm = -1, tEng = -1, tRelease = -1, nextTap = -1, tapEnd = -1, nextStatus = 0, tEnd = -1;
    bool lifted = false, stopSent = false, linkWasUp = false;
    int tapSign = 1;
    double sumTh2 = 0;
    long nTh = 0, nMiss = 0, nEng = 0;
    int endReason = 0;
    const char* result = "";
    while (true) {
        rt::sleepUntilNs(next);
        const std::int64_t now = rt::monoNs();
        const std::int64_t late = now - next;
        next += period;
        const bool overrun = late > period;
        if (overrun) ++overruns;
        t = k * ts;
        link.beginTick(k, now);
        if (link.linkUp() != linkWasUp) {
            std::cout << (link.linkUp() ? "remote: link UP, peer " + link.peer() : std::string("remote: link DOWN")) << "\n"
                      << std::flush;
            linkWasUp = link.linkUp();
        }
        const core::Mode m0 = brt.mode();
        if (flags.stop && !stopSent) { brt.requestStop(); stopSent = true; result = "stopped by signal"; }
        if (m0 == core::Mode::Idle && tEng < 0 && link.linkUp() && (tArm < 0 || t - tArm > 0.5)) {
            brt.requestArm();
            tArm = t;
        }
        if (m0 == core::Mode::Idle && tEng < 0 && t > waitPc && !link.linkUp()) { result = "no PC"; break; }
        const rt::TickRecord r = brt.tick(sim, overrun);
        const auto m = static_cast<core::Mode>(r.mode);
        if (m == core::Mode::Armed && !lifted) {          // the hand lifts the pendulum to just off upright
            sim.holdAt(0.5 * kDeg, 1.0);
            lifted = true;
            std::cout << "remote-sim: ARMED; the simulated hand lifts the pendulum (1 s)\n" << std::flush;
        }
        if (r.event == static_cast<int>(core::Reason::EngagedEvent)) {
            // A hand that holds the rod rigidly after ENGAGED makes the cart run away (docs/stage1_drift.md):
            // the simulated hand lets go at once, as the brief asks of the operator ("let go cleanly").
            tEng = t;
            tRelease = t;
            std::cout << "remote-sim: ENGAGED; the hand lets go\n" << std::flush;
        }
        if (tRelease > 0 && t >= tRelease) {
            sim.release();
            tRelease = -1;
            nextTap = tapPeriod > 0 ? t + tapPeriod : -1;
        }
        if (nextTap > 0 && t >= nextTap && m == core::Mode::Engaged) {
            // A push on the rod: tap_deg over 0.1 s, held still for 20 ms, let go (a displacement, no velocity).
            // A push released while moving adds theta'/w0 to what the catch must absorb (2 deg in 40 ms ~ 12 deg).
            sim.holdAt(sim.theta() + tapSign * tapDeg * kDeg, 0.1);
            tapSign = -tapSign;
            tapEnd = t + 0.12;
            nextTap = t + tapPeriod;
        }
        if (tapEnd > 0 && t >= tapEnd) { sim.release(); tapEnd = -1; }
        if (m == core::Mode::Engaged) {
            ++nEng;
            if (t - tEng > 1.5) { sumTh2 += r.theta * r.theta; ++nTh; }
            if (r.remote_miss) ++nMiss;
            if (t - tEng >= simSeconds && !stopSent) { brt.requestStop(); stopSent = true; result = "completed"; }
        }
        if (m == core::Mode::Stopping && endReason == 0) endReason = r.event;
        sim.advance(ts);

        proto::Sample s;
        s.flags = static_cast<std::uint16_t>((r.controller ? proto::kSampleController : 0) |
                                             (stopSent ? proto::kSampleFinishing : 0));
        s.epoch = r.epoch;
        s.t_ns = now;                                      // wall clock of the sample (SimHw's own t is sim time)
        s.counts = static_cast<std::int32_t>(r.counts);
        s.pos_usteps = static_cast<std::int32_t>(r.pos_usteps);
        s.theta = r.theta;
        s.omega = r.omega;
        s.x = r.x;
        s.v = r.v;
        s.mode = static_cast<std::uint8_t>(r.mode);
        s.phase = static_cast<std::uint8_t>(rt::SessionPhase::Balance);
        link.sendSample(s, r.a_local);
        log.push({k, late, r, sim.theta()});

        if (m == core::Mode::Engaged && t >= nextStatus) {
            nextStatus = t + 1.0;
            std::uint64_t n, lateN;
            std::int64_t mx;
            link.takeWindow(&n, &mx, &lateN);
            char b[220];
            std::snprintf(b, sizeof b, "%7.2f s  theta %+6.2f deg  x %+6.1f mm  a %+5.2f  | %llu replies, worst round trip "
                          "%.2f ms, %llu late, %ld missed", t, r.theta / kDeg, r.x * 1e3, r.a_cmd,
                          static_cast<unsigned long long>(n), mx * 1e-6, static_cast<unsigned long long>(lateN), nMiss);
            std::cout << b << "\n" << std::flush;
        }
        if (tEnd < 0 && ((tEng >= 0 && m == core::Mode::Idle) || sim.crashed())) {   // the run ended (stop or abort)
            if (sim.crashed()) result = "simulated cart hit an end stop";
            tEnd = t;
            stopSent = true;                               // the PC sees FINISHING on the last samples
        }
        if (tEnd >= 0 && t - tEnd > 0.3) break;
        ++k;
    }

    const std::string dir = runlog::makeRunDir(logDir, "remote-sim");
    writeCsv(log, dir + "/data.csv",
             "tick,late_ns,t_ns,mode,event,counts,pos_usteps,theta,omega,x,v,a_cmd,a_local,r_miss,ctrl,epoch,integral,"
             "saturated,overrun,true_theta",
             [](std::ostream& o, const SimRec& x) {
                 const rt::TickRecord& r = x.r;
                 char b[360];
                 std::snprintf(b, sizeof b, "%u,%lld,%lld,%d,%d,%lld,%lld,%.7f,%.6f,%.7f,%.7f,%.6f,%.6f,%d,%d,%u,%.7f,%d,%d,%.7f",
                               x.tick, static_cast<long long>(x.late_ns), static_cast<long long>(r.t_ns), r.mode, r.event,
                               static_cast<long long>(r.counts), static_cast<long long>(r.pos_usteps), r.theta, r.omega, r.x,
                               r.v, r.a_cmd, r.a_local, r.remote_miss ? 1 : 0, r.controller ? 1 : 0, r.epoch, r.integral,
                               r.saturated ? 1 : 0, r.overrun ? 1 : 0, x.true_theta);
                 o << b;
             });
    writeRemoteCsv(link, dir + "/remote.csv");
    const LinkStats& st = link.stats();
    const double engaged = static_cast<double>(nEng) * ts;
    if (!result[0]) result = tEng < 0 ? "never engaged" : "aborted";
    char b[160];
    std::snprintf(b, sizeof b, "%.3f", nTh ? std::sqrt(sumTh2 / nTh) / kDeg : 0.0);
    runlog::Fields f = remoteMetaFields(link, bc.remote_delay, ts);
    f.insert(f.begin(), {{"result", result},
                         {"end_reason", core::reasonName(static_cast<core::Reason>(endReason))},
                         {"engaged_s", std::to_string(engaged)},
                         {"rms_theta_deg", b},
                         {"missed_ticks", std::to_string(nMiss)},
                         {"overruns", std::to_string(overruns)},
                         {"kp", std::to_string(bc.pid.kp)},
                         {"ki", std::to_string(bc.pid.ki)},
                         {"kd", std::to_string(bc.pid.kd)}});
    runlog::writeMeta(dir, "remote-sim", cmdLine, cfg, configPath, f);
    std::snprintf(b, sizeof b, "%.2f", st.rtt_max_ns * 1e-6);
    std::cout << "remote-sim: " << result << " (" << core::reasonName(static_cast<core::Reason>(endReason)) << "), engaged "
              << engaged << " s, missed " << nMiss << " ticks, worst round trip " << b << " ms, max |a_remote - a_local| "
              << st.max_abs_diff << " m/s^2\n  -> " << dir << "\n";
    const bool ok = std::string(result) == "completed" && nMiss == 0;
    return ok ? 0 : 3;
}

}
