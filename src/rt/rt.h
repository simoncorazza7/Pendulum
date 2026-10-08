#ifndef PENDEL_RT_H
#define PENDEL_RT_H

#include <cstdint>
#include <ctime>
#include <string>

// Real-time helpers (Stage 0.5). SCHED_FIFO and mlockall need root (run pendctl with sudo).
namespace rt {

inline std::int64_t monoNs() {
    timespec ts {};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return std::int64_t(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

// Sleeps until an absolute CLOCK_MONOTONIC time (ns); retries on signal interruption.
void sleepUntilNs(std::int64_t deadlineNs);

// Calling thread: SCHED_FIFO with the given priority, pinned to cpu (cpu < 0: no pinning).
// Returns false and fills *err on failure.
bool makeCurrentThreadRealtime(int priority, int cpu, std::string* err);

// mlockall(current | future) and pre-fault `stackBytes` of the calling thread's stack.
bool lockMemory(std::size_t stackBytes, std::string* err);

// "SCHED_FIFO prio 80 cpu 1" style description of the calling thread.
std::string describeCurrentThread();

}

#endif
