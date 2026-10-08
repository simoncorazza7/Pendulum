// log-selftest: fixed-rate loop logging a synthetic signal (Stage 0.4). No GPIO.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <iostream>

#include "app/modes.h"
#include "common/runlog.h"
#include "rt/ringlog.h"
#include "rt/rt.h"

namespace app {
namespace {

constexpr double kPi = 3.14159265358979323846;

struct SelftestRecord {
    std::uint32_t tick;      // loop iteration
    std::int64_t t_ns;       // CLOCK_MONOTONIC at wake-up, ns
    std::int64_t late_ns;    // wake-up time minus scheduled time, ns
    double t;                // s since start of the loop
    double value;            // synthetic signal, sin(2 pi f t)
};

}

int runSelftest(const Config& cfg, const std::string& configPath, const std::string& cmdLine) {
    const double ts = cfg.getDouble("loop.ts");                  // s
    const double duration = cfg.getDouble("selftest.duration");  // s
    const double freq = cfg.getDouble("selftest.freq");          // Hz
    const long capacity = cfg.getInt("log.capacity");            // records
    const bool overwrite = cfg.getBool("log.overwrite");
    const std::string logDir = cfg.getString("log.dir");
    if (ts <= 0 || duration <= 0 || capacity <= 0) {
        std::cerr << "loop.ts, selftest.duration and log.capacity must be > 0\n";
        return 2;
    }

    RingLog<SelftestRecord> log(static_cast<std::size_t>(capacity), overwrite);
    const std::int64_t periodNs = std::llround(ts * 1e9);
    const auto ticks = static_cast<std::uint32_t>(std::llround(duration / ts));

    // Fixed-rate loop: absolute deadlines, so lateness never accumulates.
    const std::int64_t start = rt::monoNs() + periodNs;
    std::int64_t maxLate = 0;
    for (std::uint32_t k = 0; k < ticks && !flags.stop; ++k) {
        const std::int64_t deadline = start + std::int64_t(k) * periodNs;
        rt::sleepUntilNs(deadline);
        const std::int64_t now = rt::monoNs();
        const double t = (deadline - start) * 1e-9;
        log.push({k, now, now - deadline, t, std::sin(2 * kPi * freq * t)});
        if (now - deadline > maxLate) maxLate = now - deadline;
    }

    // After the loop: all file I/O.
    const std::string dir = runlog::makeRunDir(logDir, "log-selftest");
    writeCsv(log, dir + "/data.csv", "tick,t_ns,late_ns,t,value",
             [](std::ostream& out, const SelftestRecord& r) {
                 char buf[160];
                 std::snprintf(buf, sizeof buf, "%u,%lld,%lld,%.6f,%.9g", r.tick,
                               static_cast<long long>(r.t_ns), static_cast<long long>(r.late_ns),
                               r.t, r.value);
                 out << buf;
             });
    runlog::writeMeta(dir, "log-selftest", cmdLine, cfg, configPath,
                      {{"records", std::to_string(log.size())},
                       {"dropped", std::to_string(log.dropped())},
                       {"overwritten", std::to_string(log.overwritten())},
                       {"max_late_ns", std::to_string(maxLate)},
                       {"stopped_by_signal", flags.stop ? "true" : "false"}});

    std::cout << "log-selftest: " << log.size() << " records, " << log.dropped() << " dropped, max late "
              << maxLate / 1000 << " us -> " << dir << "\n";
    return 0;
}

}
