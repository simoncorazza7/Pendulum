// pendctl: controller runtime for the Pendel rig (Stage 0.4 skeleton).
//
// Modes so far:
//   log-selftest   fixed-rate loop logging a synthetic signal; exercises the logging pipeline.
//                  No GPIO code, so it cannot touch the rig.
// Usage:
//   pendctl --mode log-selftest [--config config/pendctl.conf] [--set key=value ...]

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <iostream>
#include <string>
#include <vector>

#include "common/config.h"
#include "common/runlog.h"
#include "rt/ringlog.h"

namespace {

constexpr double kPi = 3.14159265358979323846;

std::int64_t monoNs() {
    timespec ts {};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return std::int64_t(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

timespec toTimespec(std::int64_t ns) {
    return timespec{static_cast<time_t>(ns / 1000000000LL), static_cast<long>(ns % 1000000000LL)};
}

struct SelftestRecord {
    std::uint32_t tick;      // loop iteration
    std::int64_t t_ns;       // CLOCK_MONOTONIC at wake-up, ns
    std::int64_t late_ns;    // wake-up time minus scheduled time, ns
    double t;                // s since start of the loop
    double value;            // synthetic signal, sin(2 pi f t)
};

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
    const std::int64_t start = monoNs() + periodNs;
    std::int64_t maxLate = 0;
    for (std::uint32_t k = 0; k < ticks; ++k) {
        const std::int64_t deadline = start + std::int64_t(k) * periodNs;
        timespec dl = toTimespec(deadline);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &dl, nullptr);
        const std::int64_t now = monoNs();
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
                       {"max_late_ns", std::to_string(maxLate)}});

    std::cout << "log-selftest: " << log.size() << " records, " << log.dropped() << " dropped, max late "
              << maxLate / 1000 << " us -> " << dir << "\n";
    return 0;
}

void usage() {
    std::cout << "Usage: pendctl --mode log-selftest [--config FILE] [--set key=value ...]\n";
}

}

int main(int argc, char** argv) {
    std::string mode;
    std::string configPath = "config/pendctl.conf";
    std::vector<std::string> overrides;
    std::string cmdLine;
    for (int i = 0; i < argc; ++i) cmdLine += (i ? " " : "") + std::string(argv[i]);

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--mode" && i + 1 < argc) mode = argv[++i];
        else if (a == "--config" && i + 1 < argc) configPath = argv[++i];
        else if (a == "--set" && i + 1 < argc) overrides.push_back(argv[++i]);
        else { usage(); return a == "--help" ? 0 : 2; }
    }

    try {
        Config cfg = Config::fromFile(configPath);
        for (const auto& o : overrides) cfg.setFromAssignment(o);
        if (mode == "log-selftest") return runSelftest(cfg, configPath, cmdLine);
        usage();
        return 2;
    } catch (const std::exception& e) {
        std::cerr << "pendctl: " << e.what() << "\n";
        return 1;
    }
}
