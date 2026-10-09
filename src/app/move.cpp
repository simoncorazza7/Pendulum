// move (Stage 0.8/0.9): point-to-point bring-up move with the Axis, through hw::PiHw.
//
// Trapezoid profile (core::TrapezoidProfile) tracked with the Axis's own reported position/speed
// (core::trackAccel), streamed as accelerations at the loop rate. Phases: drive -> dwell ->
// (return -> dwell) -> hold -> driver off. Any abort brakes at move.a_brake, then driver off.
// Aborts: unexpected limit switch, tracking error > move.track_tol, 3 consecutive overruns, Axis
// not in ACCELERATE, SIGINT/SIGTERM. With move.stop_at_switch a closing switch ends the move
// (0.8c: switch positions) and is recorded. Refuses the real chip unless move.allow_real_chip.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <iostream>

#include "app/modes.h"
#include "common/runlog.h"
#include "core/profile.h"
#include "core/units.h"
#include "hw/pi_hw.h"
#include "rt/ringlog.h"
#include "rt/rt.h"

namespace app {
namespace {

enum Phase { kDrive = 0, kDwell = 1, kReturn = 2, kDwell2 = 3, kHold = 4, kBrake = 5, kDone = 6 };

struct MoveRecord {
    std::uint32_t tick;
    std::int64_t t_ns, late_ns;
    std::int32_t pos_usteps, speed_toggles, counts, axis_arg;
    std::uint8_t phase, lim_r, lim_l, axis_acc, enc_fault, n_edges;
    double x, v, x_ref, v_ref, a_cmd;
};

hw::PiPins pinsFrom(const Config& c) {
    hw::PiPins p;
    p.step = static_cast<unsigned>(c.getInt("pins.step", p.step));
    p.dir = static_cast<unsigned>(c.getInt("pins.dir", p.dir));
    p.en = static_cast<unsigned>(c.getInt("pins.en", p.en));
    p.limit_right = static_cast<unsigned>(c.getInt("pins.limit_right", p.limit_right));
    p.limit_left = static_cast<unsigned>(c.getInt("pins.limit_left", p.limit_left));
    p.ms1 = static_cast<unsigned>(c.getInt("pins.ms1", p.ms1));
    p.ms2 = static_cast<unsigned>(c.getInt("pins.ms2", p.ms2));
    p.enc_a = static_cast<unsigned>(c.getInt("pins.enc_a", p.enc_a));
    p.enc_b = static_cast<unsigned>(c.getInt("pins.enc_b", p.enc_b));
    return p;
}

}

int runMove(const Config& cfg, const std::string& configPath, const std::string& cmdLine) {
    const std::string chip = cfg.getString("move.chip");
    const bool allowReal = cfg.getBool("move.allow_real_chip");
    const double dist = cfg.getDouble("move.distance");          // m, signed
    const double vmax = cfg.getDouble("move.speed");             // m/s
    const double amax = cfg.getDouble("move.accel");             // m/s^2
    const double dwell = cfg.getDouble("move.dwell");            // s
    const bool doReturn = cfg.getBool("move.return");
    const bool stopAtSwitch = cfg.getBool("move.stop_at_switch");
    const double tol = cfg.getDouble("move.track_tol");          // m
    const double aBrake = cfg.getDouble("move.a_brake");         // m/s^2
    const double kp = cfg.getDouble("move.kp"), kv = cfg.getDouble("move.kv");   // 1/s^2, 1/s
    const double capD = cfg.getDouble("move.max_distance"), capV = cfg.getDouble("move.max_speed");
    const double capA = cfg.getDouble("move.max_accel");
    const double ts = cfg.getDouble("loop.ts");
    core::Calib cal;
    cal.usteps_per_m = cfg.getDouble("axis.usteps_per_m");
    cal.toggles_per_ustep = cfg.getDouble("axis.toggles_per_ustep");
    cal.x_sign = static_cast<int>(cfg.getInt("calib.x_sign", 1));
    const long axisMaxAccel = cfg.getInt("probe.max_accel");     // toggles/s^2: Axis cap far above the move
    const bool lockMem = cfg.getBool("rt.lock_memory"), rtOn = cfg.getBool("rt.enable");
    hw::PiRt rtc;
    rtc.step_prio = rtOn ? static_cast<int>(cfg.getInt("rt.step_prio")) : 0;
    rtc.step_cpu = static_cast<int>(cfg.getInt("rt.step_cpu"));
    rtc.enc_prio = rtOn ? static_cast<int>(cfg.getInt("rt.enc_prio")) : 0;
    rtc.enc_cpu = static_cast<int>(cfg.getInt("rt.enc_cpu"));
    const long capacity = cfg.getInt("log.capacity");
    const std::string logDir = cfg.getString("log.dir");

    if (std::fabs(dist) > capD || vmax <= 0 || vmax > capV || amax <= 0 || amax > capA || aBrake <= 0) {
        std::cerr << "move: refusing |distance| " << std::fabs(dist) << " (cap " << capD << " m), speed " << vmax
                  << " (cap " << capV << " m/s), accel " << amax << " (cap " << capA << " m/s^2), a_brake " << aBrake
                  << "\n";
        return 2;
    }

    RingLog<MoveRecord> log(static_cast<std::size_t>(capacity));
    std::string err;
    if (lockMem && !rt::lockMemory(512 * 1024, &err)) std::cerr << "warning: " << err << "\n";

    hw::PiHw hw(chip, pinsFrom(cfg), allowReal, static_cast<int>(axisMaxAccel));   // throws if refused
    activeAxis = hw.axis();
    if (!hw.start(rtc, &err)) { std::cerr << "move: " << err << "\n"; activeAxis = nullptr; return 1; }
    if (rtOn && !rt::makeCurrentThreadRealtime(static_cast<int>(cfg.getInt("rt.loop_prio")),
                                               static_cast<int>(cfg.getInt("rt.loop_cpu")), &err))
        std::cerr << "warning: " << err << "\n";

    const hw::Sample s0 = hw.read();
    cal.pos_center = static_cast<double>(s0.pos_usteps);          // x = 0 where the cart starts
    const std::int64_t counts0 = s0.counts;
    if (s0.limit_left || s0.limit_right) {
        std::cerr << "move: a limit switch is already pressed at start; refusing\n";
        activeAxis = nullptr;
        return 2;
    }
    core::TrapezoidProfile out(dist, vmax, amax), back(-dist, vmax, amax);
    std::cout << "move: " << dist * 1e3 << " mm (" << dist * cal.usteps_per_m << " usteps) at " << vmax * 1e3
              << " mm/s, " << amax << " m/s^2, " << out.duration() << " s each way" << (doReturn ? ", return" : "")
              << (stopAtSwitch ? ", stop at switch" : "") << " on " << chip << " (" << hw.chipLabel() << ")\n"
              << std::flush;

    flags.motionSafe = false;
    hw.enableDriver(true);
    Phase phase = kDrive;
    double tPhase = 0, maxErr = 0;
    int overruns = 0, stillTicks = 0;
    std::string reason = "completed";
    std::int64_t switchPos = 0;
    int switchSide = 0;                       // +1 left, -1 right
    double segStart = 0;                      // x at the start of the current profile segment
    const std::int64_t periodNs = std::llround(ts * 1e9);
    const std::int64_t start = rt::monoNs() + 10 * periodNs;
    for (std::uint32_t k = 0; phase != kDone; ++k) {
        const std::int64_t deadline = start + std::int64_t(k) * periodNs;
        rt::sleepUntilNs(deadline);
        const std::int64_t now = rt::monoNs();
        overruns = (now - deadline > periodNs) ? overruns + 1 : 0;
        const hw::Sample s = hw.read();
        const double x = core::xFromPos(s.pos_usteps, cal), v = core::vFromAxisSpeed(s.speed_toggles, cal);

        core::ProfilePoint ref{x, 0, 0};
        if (phase == kDrive) ref = out.at(tPhase);
        else if (phase == kReturn) ref = back.at(tPhase);
        else if (phase == kDwell || phase == kHold) ref = {segStart + 0.0, 0, 0};
        else if (phase == kDwell2) ref = {segStart + 0.0, 0, 0};
        if (phase == kReturn) ref.x += dist;   // the return profile starts at +dist

        // Safety checks (not while already braking).
        if (phase != kBrake) {
            std::string abortWhy;
            const bool sw = s.limit_left || s.limit_right;
            if (sw && stopAtSwitch && switchSide == 0) {
                switchSide = s.limit_left ? +1 : -1;
                switchPos = s.pos_usteps;
                reason = std::string("switch reached (") + (s.limit_left ? "left" : "right") + ")";
                phase = kBrake;
            } else if (sw && !stopAtSwitch) {
                abortWhy = std::string("unexpected limit switch (") + (s.limit_left ? "left" : "right") + ")";
            } else if ((phase == kDrive || phase == kReturn) && std::fabs(ref.x - x) > tol) {
                abortWhy = "tracking error " + std::to_string((ref.x - x) * 1e3) + " mm";
            } else if (overruns >= 3) {
                abortWhy = "3 consecutive loop overruns";
            } else if (k > 2 && !s.axis_accelerating) {
                abortWhy = "Axis not in ACCELERATE";
            } else if (flags.stop) {
                abortWhy = "signal";
            }
            if (!abortWhy.empty()) {
                reason = "ABORT: " + abortWhy;
                phase = kBrake;
            }
        }

        double a = 0;
        if (phase == kBrake) {
            a = v != 0 ? -std::copysign(std::min(aBrake, std::fabs(v) / ts), v) : 0.0;
            stillTicks = s.speed_toggles == 0 ? stillTicks + 1 : 0;
            if (stillTicks >= 25) phase = kDone;
        } else {
            a = core::trackAccel(ref, x, v, kp, kv, 1.5 * amax);
            maxErr = std::max(maxErr, std::fabs(ref.x - x));
        }
        const int arg = core::axisAccelArg(a, cal);
        hw.commandAccel(arg);

        log.push({k, now, now - deadline, static_cast<std::int32_t>(s.pos_usteps), static_cast<std::int32_t>(s.speed_toggles),
                  static_cast<std::int32_t>(s.counts), arg, static_cast<std::uint8_t>(phase),
                  static_cast<std::uint8_t>(s.limit_right), static_cast<std::uint8_t>(s.limit_left),
                  static_cast<std::uint8_t>(s.axis_accelerating), static_cast<std::uint8_t>(s.encoder_fault),
                  static_cast<std::uint8_t>(s.n_edges), x, v, ref.x, ref.v, a});

        tPhase += ts;
        if (phase == kDrive && tPhase > out.duration()) { phase = kDwell; tPhase = 0; segStart = dist; }
        else if (phase == kDwell && tPhase > dwell) {
            if (doReturn) { phase = kReturn; tPhase = 0; } else { phase = kHold; tPhase = 0; }
        } else if (phase == kReturn && tPhase > back.duration()) { phase = kDwell2; tPhase = 0; segStart = 0; }
        else if (phase == kDwell2 && tPhase > dwell) { phase = kHold; tPhase = 0; }
        else if (phase == kHold && tPhase > 0.2) phase = kDone;
    }
    hw.commandAccel(0);
    hw.enableDriver(false);
    flags.motionSafe = true;
    const hw::Sample sEnd = hw.read();
    activeAxis = nullptr;

    const std::string dir = runlog::makeRunDir(logDir, "move");
    writeCsv(log, dir + "/data.csv",
             "tick,t_ns,late_ns,pos_usteps,speed_toggles,counts,axis_arg,phase,lim_r,lim_l,axis_acc,enc_fault,n_edges,"
             "x,v,x_ref,v_ref,a_cmd",
             [](std::ostream& o, const MoveRecord& r) {
                 char b[320];
                 std::snprintf(b, sizeof b, "%u,%lld,%lld,%d,%d,%d,%d,%u,%u,%u,%u,%u,%u,%.7f,%.7f,%.7f,%.7f,%.6f",
                               r.tick, static_cast<long long>(r.t_ns), static_cast<long long>(r.late_ns), r.pos_usteps,
                               r.speed_toggles, r.counts, r.axis_arg, r.phase, r.lim_r, r.lim_l, r.axis_acc,
                               r.enc_fault, r.n_edges, r.x, r.v, r.x_ref, r.v_ref, r.a_cmd);
                 o << b;
             });
    const long long moved = static_cast<long long>(sEnd.pos_usteps) - static_cast<long long>(s0.pos_usteps);
    runlog::writeMeta(dir, "move", cmdLine, cfg, configPath,
                      {{"chip", chip + " (" + hw.chipLabel() + ")"},
                       {"result", reason},
                       {"pos_start_usteps", std::to_string(s0.pos_usteps)},
                       {"pos_end_usteps", std::to_string(sEnd.pos_usteps)},
                       {"net_usteps", std::to_string(moved)},
                       {"commanded_usteps", std::to_string(std::llround(dist * cal.usteps_per_m))},
                       {"switch_side", switchSide == 0 ? "none" : (switchSide > 0 ? "left" : "right")},
                       {"switch_pos_usteps", std::to_string(switchPos)},
                       {"max_tracking_error_m", std::to_string(maxErr)},
                       {"encoder_counts_delta", std::to_string(sEnd.counts - counts0)},
                       {"encoder_same_type", std::to_string(hw.encoder().sameType())},
                       {"encoder_full_batches", std::to_string(hw.encoder().fullBatches())},
                       {"records", std::to_string(log.size())}});
    std::cout << "move: " << reason << "; net " << moved << " usteps (start " << s0.pos_usteps << ", end "
              << sEnd.pos_usteps << "), max tracking error " << maxErr * 1e3 << " mm"
              << (switchSide ? ", switch at pos " + std::to_string(switchPos) : std::string()) << "\n  -> " << dir
              << "\n";
    return reason.rfind("ABORT", 0) == 0 ? 3 : 0;
}

}
