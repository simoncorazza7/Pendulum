// shadow (Stage 1 hardware step 1): the full balance runtime on the real sensors, motor never touched.
//
// hw::SensorHw requests only the encoder and limit-switch lines (never STEP/DIR/EN/MS), and the
// runtime runs with actuate = false, so nothing can move. Sequence:
//   1. start-up calibration: c_bottom from a small free swing (app::calibrateAtStart);
//   2. the runtime arms automatically whenever IDLE (refused unless the pendulum hangs still);
//      the operator lifts it to upright -> ENGAGED; tilting it shows the would-be command a_cmd
//      (top toward +x = toward the LEFT switch must give a_cmd > 0); beyond theta_abort -> STOPPING
//      -> IDLE; lowering it re-arms.
// Logs every tick (TickRecord) including the calibration phase. Refuses the real chip unless
// shadow.allow_real_chip.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <iostream>

#include "app/modes.h"
#include "app/rig.h"
#include "common/runlog.h"
#include "hw/sensor_hw.h"
#include "rt/ringlog.h"
#include "rt/rt.h"
#include "runtime/balance.h"
#include "runtime/params.h"

namespace app {
namespace {

struct EdgeRec {
    std::int64_t ts_ns;           // kernel edge timestamp (CLOCK_MONOTONIC)
    std::int64_t count_after;     // cumulative count after the edge
    std::uint32_t tick;           // control tick that read it
};

struct ShadowRec {
    std::uint32_t tick;
    std::int64_t late_ns;
    std::uint8_t calibrating;     // 1 during the start-up calibration (theta etc. not yet valid)
    rt::TickRecord r;
};

const double kRad2Deg = 180.0 / 3.141592653589793;

}

int runShadow(const Config& cfg, const std::string& configPath, const std::string& cmdLine) {
    const std::string chip = cfg.getString("shadow.chip");
    const bool allowReal = cfg.getBool("shadow.allow_real_chip");
    const double duration = cfg.getDouble("shadow.duration");          // s after calibration
    const double statusPeriod = cfg.getDouble("shadow.status_period"); // s
    const double calTimeout = cfg.getDouble("calib.timeout");          // s
    const double trim = cfg.getDouble("encoder.trim");                 // counts
    const bool lockMem = cfg.getBool("rt.lock_memory"), rtOn = cfg.getBool("rt.enable");
    const std::string logDir = cfg.getString("log.dir");
    rt::BalanceConfig bc = rt::balanceFrom(cfg);                       // throws on bad parameters
    bc.actuate = false;
    rt::SwingCalibrator cal(rt::swingCalibFrom(cfg));
    const hw::PiRt rtc = rtFrom(cfg);

    const std::size_t capacity = static_cast<std::size_t>((duration + calTimeout + 10) / bc.ts);
    RingLog<ShadowRec> log(capacity);
    RingLog<EdgeRec> edgeLog(2000000);                                 // every edge with its kernel timestamp
    auto logEdges = [&edgeLog](const hw::Sample& s, std::uint32_t tick) {
        for (int i = 0; i < s.n_edges; ++i)
            edgeLog.push({s.edges[static_cast<std::size_t>(i)].ts_ns, s.edges[static_cast<std::size_t>(i)].count_after, tick});
    };
    std::string err;
    if (lockMem && !rt::lockMemory(512 * 1024, &err)) std::cerr << "warning: " << err << "\n";

    hw::SensorHw hw(chip, pinsFrom(cfg), allowReal);                  // throws if refused
    if (!hw.start(rtc.enc_prio, rtc.enc_cpu, &err)) { std::cerr << "shadow: " << err << "\n"; return 1; }
    if (rtOn && !rt::makeCurrentThreadRealtime(static_cast<int>(cfg.getInt("rt.loop_prio")),
                                               static_cast<int>(cfg.getInt("rt.loop_cpu")), &err))
        std::cerr << "warning: " << err << "\n";
    {
        char b[260];
        std::snprintf(b, sizeof b, "shadow on %s (%s): SENSORS ONLY, the motor lines are not requested. "
                      "Kp %.2f Ki %.2f Kd %.3f, a_max %.1f m/s^2", chip.c_str(), hw.chipLabel().c_str(),
                      bc.pid.kp, bc.pid.ki, bc.pid.kd, bc.pid.a_max);
        std::cout << b << "\n" << std::flush;
    }

    TickClock clk(bc.ts);
    const bool calibrated = calibrateAtStart(hw, cal, clk, calTimeout, [&](const hw::Sample& s, std::int64_t late) {
        logEdges(s, clk.tick());
        ShadowRec rec{clk.tick(), late, 1, rt::TickRecord{}};
        rec.r.t_ns = s.t_ns;
        rec.r.counts = s.counts;
        rec.r.n_edges = s.n_edges;
        rec.r.limit_left = s.limit_left;
        rec.r.limit_right = s.limit_right;
        log.push(rec);
    });

    std::string result = calibrated ? "completed" : (flags.stop ? "stopped during calibration" : "calibration timeout");
    int engagedCount = 0;
    double maxAbsA = 0;
    if (calibrated) {
        bc.calib.c_bottom = cal.status().c_bottom + trim;
        bc.calib.pos_center = 0;
        rt::BalanceRuntime brt(bc);
        std::cout << "shadow: arming automatically whenever IDLE and the pendulum hangs still. Then lift it to "
                     "upright; with sup.engage_on_release it engages only after you let go (catch it). Top toward the LEFT switch (+x): a_cmd > 0.\n"
                  << std::flush;
        const double t0 = clk.seconds();
        // First arm request after 0.2 s: the estimator reads 0 on its first sample, which would pass
        // the "hanging still" check while the pendulum still swings (seen in the mock run).
        double nextStatus = t0 + statusPeriod, nextArm = t0 + 0.2;
        int lastEvent = -1, lastMode = -1;
        while (!flags.stop && clk.seconds() - t0 < duration) {
            const std::int64_t late = clk.wait();
            const double t = clk.seconds() - t0;
            if (brt.mode() == core::Mode::Idle && clk.seconds() >= nextArm) {
                brt.requestArm();
                nextArm = clk.seconds() + 0.5;
            }
            const rt::TickRecord r = brt.tick(hw, clk.overrun());
            logEdges(hw.lastSample(), clk.tick());
            log.push({clk.tick(), late, 0, r});
            if (r.mode == static_cast<int>(core::Mode::Engaged)) maxAbsA = std::max(maxAbsA, std::fabs(r.a_cmd));
            if (r.event == static_cast<int>(core::Reason::EngagedEvent)) engagedCount++;
            // Print transitions and changed refusals (refusals repeat every 0.5 s while IDLE).
            const bool modeChanged = r.mode != lastMode, newEvent = r.event != 0 && r.event != lastEvent;
            if (modeChanged || newEvent) {
                char b[200];
                std::snprintf(b, sizeof b, "%7.2f s  %-9s %s  (theta %+6.1f deg, omega %+5.2f rad/s)", t,
                              core::modeName(static_cast<core::Mode>(r.mode)),
                              core::reasonName(static_cast<core::Reason>(r.event)), r.theta * kRad2Deg, r.omega);
                std::cout << b << "\n" << std::flush;
            }
            if (modeChanged) lastEvent = -1;
            if (r.event != 0) lastEvent = r.event;
            lastMode = r.mode;
            if (clk.seconds() >= nextStatus) {
                nextStatus += statusPeriod;
                char b[200];
                std::snprintf(b, sizeof b, "%7.2f s  %-9s theta %+6.2f deg  omega %+6.2f rad/s  a_cmd %+6.2f m/s^2%s", t,
                              core::modeName(static_cast<core::Mode>(r.mode)), r.theta * kRad2Deg, r.omega, r.a_cmd,
                              r.saturated ? " (sat)" : "");
                std::cout << b << "\n" << std::flush;
            }
        }
        if (flags.stop) result = "stopped by signal";
    }
    const hw::Sample sEnd = hw.read();

    const std::string dir = runlog::makeRunDir(logDir, "shadow");
    writeCsv(log, dir + "/data.csv",
             "tick,late_ns,calibrating,t_ns,counts,n_edges,theta,omega,mode,event,a_ctrl,a_cmd,saturated,overrun,"
             "lim_l,lim_r,integral,released,omega_release",
             [](std::ostream& o, const ShadowRec& x) {
                 const rt::TickRecord& r = x.r;
                 char b[320];
                 std::snprintf(b, sizeof b, "%u,%lld,%u,%lld,%lld,%d,%.7f,%.6f,%d,%d,%.6f,%.6f,%d,%d,%d,%d,%.7f,%d,%.5f", x.tick,
                               static_cast<long long>(x.late_ns), x.calibrating, static_cast<long long>(r.t_ns),
                               static_cast<long long>(r.counts), r.n_edges, r.theta, r.omega, r.mode, r.event, r.a_ctrl,
                               r.a_cmd, r.saturated ? 1 : 0, r.overrun ? 1 : 0, r.limit_left ? 1 : 0,
                               r.limit_right ? 1 : 0, r.integral, r.released ? 1 : 0, r.omega_release);
                 o << b;
             });
    writeCsv(edgeLog, dir + "/edges.csv", "ts_ns,count_after,tick", [](std::ostream& o, const EdgeRec& e) {
        char b[96];
        std::snprintf(b, sizeof b, "%lld,%lld,%u", static_cast<long long>(e.ts_ns), static_cast<long long>(e.count_after),
                      e.tick);
        o << b;
    });
    const rt::SwingCalibStatus& st = cal.status();
    char num[64];
    auto f = [&num](double v) { std::snprintf(num, sizeof num, "%.4f", v); return std::string(num); };
    runlog::writeMeta(dir, "shadow", cmdLine, cfg, configPath,
                      {{"chip", chip + " (" + hw.chipLabel() + ")"},
                       {"result", result},
                       {"c_bottom_fit_counts", f(st.c_bottom)},
                       {"c_bottom_se_counts", f(st.fit.se)},
                       {"c_bottom_fit_rms_counts", f(st.fit.rms)},
                       {"calib_amplitude_counts", f(st.amplitude)},
                       {"trim_counts", f(trim)},
                       {"kp", f(bc.pid.kp)},
                       {"ki", f(bc.pid.ki)},
                       {"kd", f(bc.pid.kd)},
                       {"engaged_count", std::to_string(engagedCount)},
                       {"max_abs_a_cmd_engaged", f(maxAbsA)},
                       {"motor_commands_ignored", std::to_string(hw.ignoredCommands())},
                       {"encoder_counts_end", std::to_string(sEnd.counts)},
                       {"encoder_same_type", std::to_string(hw.encoder().sameType())},
                       {"encoder_full_batches", std::to_string(hw.encoder().fullBatches())},
                       {"records", std::to_string(log.size())}});
    std::cout << "shadow: " << result << "; engaged " << engagedCount << " times; motor commands ignored "
              << hw.ignoredCommands() << " (must be 0)\n  -> " << dir << "\n";
    return calibrated ? 0 : 3;
}

}
