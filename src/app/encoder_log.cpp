// encoder-log (Stage 0.6): records every raw edge event of the pendulum encoder, for offline
// comparison of decoders (including a replay of the original Encoder algorithm).
//
// Per event: line (0 = A, 1 = B), kernel edge type, kernel timestamp, time the event was read,
// the levels of A and B read right after the batch, and the batch size of that read.
// The lines are requested as inputs only; nothing is driven. By default the chip must be
// gpio-mockup; the real chip needs enc.allow_real_chip=true (Simon's go).

#include <cstdint>
#include <cstdio>
#include <gpiod.h>
#include <iostream>
#include <poll.h>

#include "app/modes.h"
#include "common/runlog.h"
#include "rt/latency.h"
#include "rt/ringlog.h"
#include "rt/rt.h"

namespace app {
namespace {

struct EdgeRecord {
    std::uint32_t seq;        // order in which events were read
    std::uint8_t line;        // 0 = A, 1 = B
    std::uint8_t type;        // 1 = rising, 2 = falling (as reported by the kernel)
    std::uint8_t lvlA, lvlB;  // line levels read right after this batch
    std::uint8_t batch;       // events returned by this read (16 = kernel FIFO may have overflowed)
    std::int64_t ts_ns;       // kernel event timestamp (CLOCK_MONOTONIC), ns
    std::int64_t read_ns;     // CLOCK_MONOTONIC after the read, ns
};

std::int64_t toNs(const timespec& ts) { return std::int64_t(ts.tv_sec) * 1000000000LL + ts.tv_nsec; }

}

int runEncoderLog(const Config& cfg, const std::string& configPath, const std::string& cmdLine) {
    const std::string chipName = cfg.getString("enc.chip");
    const bool allowReal = cfg.getBool("enc.allow_real_chip");
    const long lineA = cfg.getInt("enc.line_a"), lineB = cfg.getInt("enc.line_b");
    const double duration = cfg.getDouble("enc.duration");            // s
    const bool rtEnable = cfg.getBool("rt.enable");
    const long prio = cfg.getInt("rt.enc_prio"), cpu = cfg.getInt("rt.enc_cpu");
    const bool lockMem = cfg.getBool("rt.lock_memory");
    const long capacity = cfg.getInt("enc.capacity");                 // events
    const std::string logDir = cfg.getString("log.dir");

    gpiod_chip* chip = gpiod_chip_open_by_name(chipName.c_str());
    if (!chip) { std::cerr << "cannot open " << chipName << "\n"; return 1; }
    const std::string label = gpiod_chip_label(chip);
    if (label != "gpio-mockup-A" && !allowReal) {
        std::cerr << "refusing: " << chipName << " is '" << label << "'. Reading the real encoder needs "
                  << "enc.allow_real_chip=true (with Simon's go).\n";
        gpiod_chip_close(chip);
        return 2;
    }
    gpiod_line* lines[2] = {gpiod_chip_get_line(chip, static_cast<unsigned>(lineA)),
                            gpiod_chip_get_line(chip, static_cast<unsigned>(lineB))};
    for (auto* l : lines) {
        if (!l || gpiod_line_request_both_edges_events(l, "encoder-log") < 0) {
            std::cerr << "cannot request encoder lines for events\n";
            gpiod_chip_close(chip);
            return 1;
        }
    }

    std::string err;
    if (lockMem && !rt::lockMemory(512 * 1024, &err)) std::cerr << "warning: " << err << "\n";
    if (rtEnable && !rt::makeCurrentThreadRealtime(static_cast<int>(prio), static_cast<int>(cpu), &err))
        std::cerr << "warning: " << err << "\n";
    const std::string sched = rt::describeCurrentThread();

    RingLog<EdgeRecord> log(static_cast<std::size_t>(capacity));
    LatencyHist readLatency;   // read_ns - ts_ns of every event
    const int startA = gpiod_line_get_value(lines[0]), startB = gpiod_line_get_value(lines[1]);
    pollfd fds[2] = {{gpiod_line_event_get_fd(lines[0]), POLLIN, 0}, {gpiod_line_event_get_fd(lines[1]), POLLIN, 0}};
    gpiod_line_event ev[16];
    std::uint32_t seq = 0;
    unsigned maxBatch = 0;
    std::uint64_t fullBatches = 0;

    std::cout << "encoder-log: recording for " << duration << " s (Ctrl-C stops early), A=" << startA
              << " B=" << startB << " at start\n" << std::flush;
    const std::int64_t end = rt::monoNs() + static_cast<std::int64_t>(duration * 1e9);
    while (!flags.stop && rt::monoNs() < end) {
        if (poll(fds, 2, 100) <= 0) continue;   // 100 ms timeout: check stop/end
        for (int i = 0; i < 2; ++i) {
            if (!(fds[i].revents & POLLIN)) continue;
            const int n = gpiod_line_event_read_multiple(lines[i], ev, 16);
            const std::int64_t readNs = rt::monoNs();
            const auto a = static_cast<std::uint8_t>(gpiod_line_get_value(lines[0]));
            const auto b = static_cast<std::uint8_t>(gpiod_line_get_value(lines[1]));
            if (n <= 0) continue;
            if (static_cast<unsigned>(n) > maxBatch) maxBatch = static_cast<unsigned>(n);
            if (n == 16) ++fullBatches;
            for (int k = 0; k < n; ++k) {
                const std::int64_t ts = toNs(ev[k].ts);
                readLatency.add(readNs - ts);
                log.push({seq++, static_cast<std::uint8_t>(i), static_cast<std::uint8_t>(ev[k].event_type), a, b,
                          static_cast<std::uint8_t>(n), ts, readNs});
            }
        }
    }
    for (auto* l : lines) gpiod_line_release(l);
    gpiod_chip_close(chip);
    flags.motionSafe = true;

    const std::string dir = runlog::makeRunDir(logDir, "encoder-log");
    writeCsv(log, dir + "/data.csv", "seq,line,type,lvl_a,lvl_b,batch,ts_ns,read_ns",
             [](std::ostream& out, const EdgeRecord& r) {
                 char buf[160];
                 std::snprintf(buf, sizeof buf, "%u,%u,%u,%u,%u,%u,%lld,%lld", r.seq, r.line, r.type, r.lvlA,
                               r.lvlB, r.batch, static_cast<long long>(r.ts_ns), static_cast<long long>(r.read_ns));
                 out << buf;
             });
    readLatency.writeCsv(dir + "/hist_read_latency.csv");
    runlog::writeMeta(dir, "encoder-log", cmdLine, cfg, configPath,
                      {{"chip", chipName + " (" + label + ")"},
                       {"sched", sched},
                       {"start_level_a", std::to_string(startA)},
                       {"start_level_b", std::to_string(startB)},
                       {"events", std::to_string(log.size())},
                       {"dropped", std::to_string(log.dropped())},
                       {"max_batch", std::to_string(maxBatch)},
                       {"full_batches", std::to_string(fullBatches)},
                       {"read_latency_p50_us", std::to_string(readLatency.quantileUs(0.5))},
                       {"read_latency_p99_us", std::to_string(readLatency.quantileUs(0.99))},
                       {"read_latency_max_us", std::to_string(readLatency.maxNs() / 1000)},
                       {"stopped_by_signal", flags.stop ? "true" : "false"}});
    std::cout << "encoder-log: " << log.size() << " events (" << log.dropped() << " dropped), max batch " << maxBatch
              << ", read latency p99 " << readLatency.quantileUs(0.99) << " us -> " << dir << "\n";
    return 0;
}

}
