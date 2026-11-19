// damp (Stage 1 hardware step 2): hanging active-damping test, rt::DampRuntime on hw::PiHw.
//
// 1. Start-up calibration with the driver OFF (c_bottom from the small free swing the operator starts).
// 2. Driver on, cart holds (a = 0) until the next zero crossing of the swing, then a = -gain * phi'
//    for damp.duration s: the swing should die out at zeta_h * w0 (open loop it barely decays).
// 3. Controlled stop, driver off. Any abort (switch, phi or v too large, soft limit, encoder fault,
//    Axis not in ACCELERATE, overruns, SIGINT) brakes at damp.a_brake and switches the driver off.
// Refuses the real chip unless damp.allow_real_chip, and refuses to start on a pressed switch.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <iostream>

#include "app/modes.h"
#include "app/rig.h"
#include "common/runlog.h"
#include "hw/pi_hw.h"
#include "rt/ringlog.h"
#include "rt/rt.h"
#include "runtime/damp.h"
#include "runtime/params.h"

namespace app {
namespace {

struct DampRec {
    std::uint32_t tick;
    std::int64_t late_ns;
    rt::DampRecord r;          // r.phase = -1 during the start-up calibration
};

const double kRad2Deg = 180.0 / 3.141592653589793;
const char* kPhaseName[] = {"WAIT", "ENGAGED", "STOPPING", "DONE"};

}

int runDamp(const Config& cfg, const std::string& configPath, const std::string& cmdLine) {
    const std::string chip = cfg.getString("damp.chip");
    const bool allowReal = cfg.getBool("damp.allow_real_chip");
    const double calTimeout = cfg.getDouble("calib.timeout");          // s
    const bool lockMem = cfg.getBool("rt.lock_memory"), rtOn = cfg.getBool("rt.enable");
    const std::string logDir = cfg.getString("log.dir");
    const double w0 = cfg.getDouble("plant.w0"), zeta = cfg.getDouble("damp.zeta");
    rt::DampConfig dc = rt::dampFrom(cfg);                             // throws on bad parameters
    rt::SwingCalibrator cal(rt::swingCalibFrom(cfg));
    const hw::PiRt rtc = rtFrom(cfg);
    const double maxRun = dc.wait_max + dc.duration + dc.limits.stop_timeout + 2.0;   // s, hard cap

    const std::size_t capacity = static_cast<std::size_t>((calTimeout + maxRun + 5) / dc.ts);
    RingLog<DampRec> log(capacity);
    std::string err;
    if (lockMem && !rt::lockMemory(512 * 1024, &err)) std::cerr << "warning: " << err << "\n";

    hw::PiHw hw(chip, pinsFrom(cfg), allowReal, static_cast<int>(cfg.getInt("axis.max_accel")));   // throws if refused
    activeAxis = hw.axis();
    if (!hw.start(rtc, &err)) { std::cerr << "damp: " << err << "\n"; activeAxis = nullptr; return 1; }
    if (rtOn && !rt::makeCurrentThreadRealtime(static_cast<int>(cfg.getInt("rt.loop_prio")),
                                               static_cast<int>(cfg.getInt("rt.loop_cpu")), &err))
        std::cerr << "warning: " << err << "\n";

    const hw::Sample s0 = hw.read();
    if (s0.limit_left || s0.limit_right) {
        std::cerr << "damp: a limit switch is pressed at start; refusing (centre the cart first)\n";
        activeAxis = nullptr;
        return 2;
    }
    dc.calib.pos_center = static_cast<double>(s0.pos_usteps);         // x = 0 where the cart starts
    {
        char b[300];
        std::snprintf(b, sizeof b, "damp on %s (%s): zeta_h %.2f -> gain %.3f (m/s^2)/(rad/s), a_max %.2f m/s^2, "
                      "%.1f s engaged, soft limit +-%.0f mm. Driver OFF until calibrated.", chip.c_str(),
                      hw.chipLabel().c_str(), zeta, dc.gain, dc.a_max, dc.duration, dc.limits.x_max * 1e3);
        std::cout << b << "\n" << std::flush;
    }

    TickClock clk(dc.ts);
    const bool calibrated = calibrateAtStart(hw, cal, clk, calTimeout, [&](const hw::Sample& s, std::int64_t late) {
        DampRec rec{clk.tick(), late, rt::DampRecord{}};
        rec.r.t_ns = s.t_ns;
        rec.r.counts = s.counts;
        rec.r.pos_usteps = s.pos_usteps;
        rec.r.speed_toggles = s.speed_toggles;
        rec.r.n_edges = s.n_edges;
        rec.r.phase = -1;
        rec.r.limit_left = s.limit_left;
        rec.r.limit_right = s.limit_right;
        log.push(rec);
    });

    std::string result = flags.stop ? "stopped during calibration" : "calibration timeout";
    double tEngage = -1, phiEngage = 0, maxX = 0, maxPhiEngaged = 0;
    if (calibrated) {
        dc.calib.c_bottom = cal.status().c_bottom;                     // hanging reference: no upright trim
        rt::DampRuntime drt(dc);
        std::cout << "damp: DRIVER ON now; damping engages at the next zero crossing. Hands off.\n" << std::flush;
        flags.motionSafe = false;
        const double t0 = clk.seconds();
        double nextStatus = t0 + 0.5;
        int lastPhase = -1;
        bool stopSent = false;
        while (drt.phase() != rt::DampPhase::Done) {
            const std::int64_t late = clk.wait();
            const double t = clk.seconds() - t0;
            if (flags.stop && !stopSent) { drt.requestStop(); stopSent = true; }
            if (t > maxRun) {                                          // never expected: the runtime ends itself
                std::cerr << "damp: hard time cap reached, HALT\n";
                hw.halt();
                break;
            }
            const rt::DampRecord r = drt.tick(hw, clk.overrun());
            log.push({clk.tick(), late, r});
            maxX = std::max(maxX, std::fabs(r.x));
            if (r.phase == static_cast<int>(rt::DampPhase::Engaged)) maxPhiEngaged = std::max(maxPhiEngaged, std::fabs(r.phi));
            const bool statusDue = clk.seconds() >= nextStatus;
            if (statusDue) nextStatus += 0.5;
            if (r.phase != lastPhase || statusDue) {
                char b[220];
                std::snprintf(b, sizeof b, "%6.2f s  %-8s phi %+6.2f deg  omega %+5.2f rad/s  x %+6.1f mm  v %+5.3f m/s  a %+5.2f%s%s",
                              t, kPhaseName[r.phase], r.phi * kRad2Deg, r.omega, r.x * 1e3, r.v, r.a_cmd,
                              r.phase != lastPhase && drt.reason()[0] ? "  <- " : "", r.phase != lastPhase ? drt.reason() : "");
                std::cout << b << "\n" << std::flush;
                lastPhase = r.phase;
            }
        }
        tEngage = drt.engagedAt();
        phiEngage = drt.phiEngage();
        result = drt.reason();
    }
    hw.commandAccel(0);
    hw.enableDriver(false);
    flags.motionSafe = true;
    const hw::Sample sEnd = hw.read();
    activeAxis = nullptr;

    const std::string dir = runlog::makeRunDir(logDir, "damp");
    writeCsv(log, dir + "/data.csv",
             "tick,late_ns,t_ns,counts,pos_usteps,speed_toggles,n_edges,phase,phi,omega,x,v,a_cmd,axis_arg,saturated,"
             "overrun,lim_l,lim_r,axis_acc,driver",
             [](std::ostream& o, const DampRec& x) {
                 const rt::DampRecord& r = x.r;
                 char b[360];
                 std::snprintf(b, sizeof b, "%u,%lld,%lld,%lld,%lld,%lld,%d,%d,%.7f,%.6f,%.7f,%.7f,%.6f,%d,%d,%d,%d,%d,%d,%d",
                               x.tick, static_cast<long long>(x.late_ns), static_cast<long long>(r.t_ns),
                               static_cast<long long>(r.counts), static_cast<long long>(r.pos_usteps),
                               static_cast<long long>(r.speed_toggles), r.n_edges, r.phase, r.phi, r.omega, r.x, r.v,
                               r.a_cmd, r.axis_arg, r.saturated ? 1 : 0, r.overrun ? 1 : 0, r.limit_left ? 1 : 0,
                               r.limit_right ? 1 : 0, r.axis_accelerating ? 1 : 0, r.driver_enabled ? 1 : 0);
                 o << b;
             });
    const rt::SwingCalibStatus& st = cal.status();
    char num[64];
    auto f = [&num](double v) { std::snprintf(num, sizeof num, "%.5f", v); return std::string(num); };
    const long long moved = static_cast<long long>(sEnd.pos_usteps) - static_cast<long long>(s0.pos_usteps);
    runlog::writeMeta(dir, "damp", cmdLine, cfg, configPath,
                      {{"chip", chip + " (" + hw.chipLabel() + ")"},
                       {"result", result},
                       {"c_bottom_fit_counts", f(st.c_bottom)},
                       {"c_bottom_se_counts", f(st.fit.se)},
                       {"c_bottom_fit_rms_counts", f(st.fit.rms)},
                       {"calib_amplitude_counts", f(st.amplitude)},
                       {"zeta_h", f(zeta)},
                       {"gain", f(dc.gain)},
                       {"predicted_decay_rate", f(zeta * w0)},
                       {"engaged_at_s", f(tEngage)},
                       {"phi_engage_rad", f(phiEngage)},
                       {"max_abs_phi_engaged_rad", f(maxPhiEngaged)},
                       {"max_abs_x_m", f(maxX)},
                       {"pos_start_usteps", std::to_string(s0.pos_usteps)},
                       {"pos_end_usteps", std::to_string(sEnd.pos_usteps)},
                       {"net_usteps", std::to_string(moved)},
                       {"encoder_same_type", std::to_string(hw.encoder().sameType())},
                       {"encoder_full_batches", std::to_string(hw.encoder().fullBatches())},
                       {"records", std::to_string(log.size())}});
    std::cout << "damp: " << result << "; net " << moved << " usteps, max |x| " << maxX * 1e3 << " mm\n  -> " << dir
              << "\n";
    if (!calibrated) return 3;
    return result == "completed" ? 0 : 3;
}

}
