// timing-probe (Stage 0.5): a fixed-rate SCHED_FIFO control loop drives the existing Axis step
// thread (also SCHED_FIFO, pinned) on a GPIO chip and records
//   - control-loop wake-up latency and execution time (every tick, plus histogram),
//   - step lateness: actual minus scheduled time of every STEP edge (histogram, per-tick max),
//   - Axis schedule resyncs (loop fell > 500 us behind).
// The driver is never enabled (EN stays 1). By default the chip must be gpio-mockup.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <gpiod.h>
#include <iostream>
#include <limits>
#include <memory>

#include "app/modes.h"
#include "axis.h"
#include "common/runlog.h"
#include "rt/latency.h"
#include "rt/ringlog.h"
#include "rt/rt.h"

namespace app {
namespace {

constexpr double kPi = 3.14159265358979323846;

struct StepStats {
    LatencyHist hist;                              // written by the step thread only
    std::uint64_t edges = 0;
    // Step thread raises it, control loop takes it. 32 bit, because 64-bit atomics are not
    // lock-free on this armhf (ARMv6-baseline) build; ns clamped to INT32_MAX (2.1 s).
    std::atomic<std::int32_t> maxSinceRead{0};
};

void onStep(void* ctx, std::int64_t scheduledNs, std::int64_t actualNs) {
    auto* s = static_cast<StepStats*>(ctx);
    const std::int64_t late = actualNs - scheduledNs;
    s->hist.add(late);
    ++s->edges;
    const auto late32 = static_cast<std::int32_t>(
        std::min<std::int64_t>(late, std::numeric_limits<std::int32_t>::max()));
    std::int32_t prev = s->maxSinceRead.load(std::memory_order_relaxed);
    while (late32 > prev && !s->maxSinceRead.compare_exchange_weak(prev, late32, std::memory_order_relaxed)) {
    }
}

struct ProbeRecord {
    std::uint32_t tick;
    std::int64_t t_ns;            // CLOCK_MONOTONIC at wake-up, ns
    std::int64_t late_ns;         // wake-up minus deadline, ns
    std::int64_t exec_ns;         // time spent in the tick body, ns
    std::int32_t pos_usteps;      // Axis pos, microsteps
    std::int32_t speed_toggles;   // Axis curSpeed (internal sign), STEP toggles/s
    std::int32_t cmd;             // commanded accel (toggles/s^2) or speed (toggles/s)
    std::int64_t step_late_max_ns;// max STEP-edge lateness since the previous tick, ns
    std::uint32_t resyncs;        // cumulative Axis schedule resyncs
};

}

int runTimingProbe(const Config& cfg, const std::string& configPath, const std::string& cmdLine) {
    const std::string chipName = cfg.getString("probe.chip");
    const bool allowReal = cfg.getBool("probe.allow_real_chip");
    const std::string profile = cfg.getString("probe.profile");     // accel-stream | speed | idle
    const double duration = cfg.getDouble("probe.duration");         // s
    const double ts = cfg.getDouble("loop.ts");                      // s
    const long speed = cfg.getInt("probe.speed");                    // toggles/s
    const double speedPeriod = cfg.getDouble("probe.speed_period");  // s, direction change interval
    const double accelAmp = cfg.getDouble("probe.accel_amp");        // toggles/s^2
    const double accelFreq = cfg.getDouble("probe.accel_freq");      // Hz
    const long maxAccel = cfg.getInt("probe.max_accel");             // toggles/s^2, Axis cap
    const long stepLine = cfg.getInt("probe.step_line");             // line offset of STEP
    const bool rtEnable = cfg.getBool("rt.enable");
    const long loopPrio = cfg.getInt("rt.loop_prio"), loopCpu = cfg.getInt("rt.loop_cpu");
    const long stepPrio = cfg.getInt("rt.step_prio"), stepCpu = cfg.getInt("rt.step_cpu");
    const bool lockMem = cfg.getBool("rt.lock_memory");
    const long capacity = cfg.getInt("log.capacity");
    const std::string logDir = cfg.getString("log.dir");
    if (profile != "accel-stream" && profile != "speed" && profile != "idle") {
        std::cerr << "probe.profile must be accel-stream, speed or idle\n";
        return 2;
    }

    gpiod_chip* chip = gpiod_chip_open_by_name(chipName.c_str());
    if (!chip) { std::cerr << "cannot open " << chipName << "\n"; return 1; }
    const std::string label = gpiod_chip_label(chip);
    if (label != "gpio-mockup-A" && !allowReal) {
        std::cerr << "refusing: " << chipName << " is '" << label << "', not gpio-mockup-A. Running the probe on "
                  << "the real chip is a hardware action (set probe.allow_real_chip=true only with a go).\n";
        gpiod_chip_close(chip);
        return 2;
    }

    std::string err;
    if (lockMem && !rt::lockMemory(512 * 1024, &err)) std::cerr << "warning: " << err << "\n";

    RingLog<ProbeRecord> log(static_cast<std::size_t>(capacity));
    LatencyHist loopHist;
    auto steps = std::make_unique<StepStats>();

    std::string stepSched = "SCHED_OTHER (rt.enable=false)", loopSched;
    {
        // Bring-up mode: no homing, EN stays 1. Same line offsets as pendel.cpp except STEP.
        Axis axis(chip, static_cast<unsigned>(stepLine), 22, 24, 2, 9, 20, 21, false);
        activeAxis = &axis;
        flags.motionSafe = false;
        axis.setPositionReference(5000000, 10000000);   // room to move on the mock (no switches)
        axis.setMaxAcceleration(static_cast<int>(maxAccel));
        axis.setStepObserver(onStep, steps.get());
        if (rtEnable) {
            if (!axis.configureRealtime(static_cast<int>(stepPrio), static_cast<int>(stepCpu)))
                std::cerr << "warning: step thread RT setup failed (run with sudo)\n";
            else
                stepSched = "SCHED_FIFO prio " + std::to_string(stepPrio) + " cpu " + std::to_string(stepCpu);
            if (!rt::makeCurrentThreadRealtime(static_cast<int>(loopPrio), static_cast<int>(loopCpu), &err))
                std::cerr << "warning: " << err << "\n";
        }
        loopSched = rt::describeCurrentThread();

        const std::int64_t periodNs = std::llround(ts * 1e9);
        const auto ticks = static_cast<std::uint32_t>(std::llround(duration / ts));
        const std::int64_t start = rt::monoNs() + 100 * periodNs;
        int lastDir = 0;
        for (std::uint32_t k = 0; k < ticks && !flags.stop; ++k) {
            const std::int64_t deadline = start + std::int64_t(k) * periodNs;
            rt::sleepUntilNs(deadline);
            const std::int64_t now = rt::monoNs();
            const double t = (deadline - start) * 1e-9;

            std::int32_t cmd = 0;
            if (profile == "accel-stream") {
                cmd = static_cast<std::int32_t>(std::lround(accelAmp * std::cos(2 * kPi * accelFreq * t)));
                axis.setAcceleration(cmd);
            } else if (profile == "speed") {
                const int dir = (static_cast<long>(t / speedPeriod) % 2 == 0) ? 1 : -1;
                cmd = static_cast<std::int32_t>(dir * speed);
                if (dir != lastDir) { axis.setSpeed(cmd); lastDir = dir; }
            }

            const std::int64_t end = rt::monoNs();
            loopHist.add(now - deadline);
            log.push({k, now, now - deadline, end - now, axis.getPos(), axis.getSpeed(), cmd,
                      steps->maxSinceRead.exchange(0, std::memory_order_relaxed), axis.getResyncCount()});
        }

        // Stop: brake to standstill, driver off (it was never on), stop observing.
        axis.halt();
        for (int i = 0; i < 500 && !axis.isStopped(); ++i) rt::sleepUntilNs(rt::monoNs() + 1000000);
        axis.disableDriver();
        flags.motionSafe = true;
        axis.setStepObserver(nullptr, nullptr);
        activeAxis = nullptr;
    }   // ~Axis joins the step thread and releases the lines
    gpiod_chip_close(chip);

    // After the run: file I/O only.
    const std::string dir = runlog::makeRunDir(logDir, "timing-probe_" + profile);
    writeCsv(log, dir + "/data.csv",
             "tick,t_ns,late_ns,exec_ns,pos_usteps,speed_toggles,cmd,step_late_max_ns,resyncs",
             [](std::ostream& out, const ProbeRecord& r) {
                 char buf[200];
                 std::snprintf(buf, sizeof buf, "%u,%lld,%lld,%lld,%d,%d,%d,%lld,%u", r.tick,
                               static_cast<long long>(r.t_ns), static_cast<long long>(r.late_ns),
                               static_cast<long long>(r.exec_ns), r.pos_usteps, r.speed_toggles, r.cmd,
                               static_cast<long long>(r.step_late_max_ns), r.resyncs);
                 out << buf;
             });
    loopHist.writeCsv(dir + "/hist_loop_late.csv");
    steps->hist.writeCsv(dir + "/hist_step_late.csv");

    auto q = [](const LatencyHist& h, double p) { return std::to_string(h.quantileUs(p)); };
    std::uint32_t resyncs = 0;
    log.forEach([&](const ProbeRecord& r) { resyncs = r.resyncs; });
    runlog::writeMeta(dir, "timing-probe", cmdLine, cfg, configPath,
                      {{"chip", chipName + " (" + label + ")"},
                       {"profile", profile},
                       {"loop_sched", loopSched},
                       {"step_sched", stepSched},
                       {"records", std::to_string(log.size())},
                       {"dropped", std::to_string(log.dropped())},
                       {"stopped_by_signal", flags.stop ? "true" : "false"},
                       {"loop_late_p50_us", q(loopHist, 0.5)},
                       {"loop_late_p99_us", q(loopHist, 0.99)},
                       {"loop_late_p999_us", q(loopHist, 0.999)},
                       {"loop_late_max_us", std::to_string(loopHist.maxNs() / 1000)},
                       {"step_edges", std::to_string(steps->edges)},
                       {"step_late_p50_us", q(steps->hist, 0.5)},
                       {"step_late_p99_us", q(steps->hist, 0.99)},
                       {"step_late_p999_us", q(steps->hist, 0.999)},
                       {"step_late_max_us", std::to_string(steps->hist.maxNs() / 1000)},
                       {"axis_resyncs", std::to_string(resyncs)}});

    std::cout << "timing-probe " << profile << ": " << log.size() << " ticks, loop late p99 " << q(loopHist, 0.99)
              << " us max " << loopHist.maxNs() / 1000 << " us; " << steps->edges << " STEP edges, late p99 "
              << q(steps->hist, 0.99) << " us max " << steps->hist.maxNs() / 1000 << " us; resyncs " << resyncs
              << "\n  loop: " << loopSched << "; step: " << stepSched << "\n  -> " << dir << "\n";
    return 0;
}

}
