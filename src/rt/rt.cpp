#include "rt/rt.h"

#include <cerrno>
#include <cstring>
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>

namespace rt {

void sleepUntilNs(std::int64_t deadlineNs) {
    timespec ts {static_cast<time_t>(deadlineNs / 1000000000LL), static_cast<long>(deadlineNs % 1000000000LL)};
    while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr) == EINTR) {
    }
}

bool makeCurrentThreadRealtime(int priority, int cpu, std::string* err) {
    sched_param sp {};
    sp.sched_priority = priority;
    int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
    if (rc != 0) {
        if (err) *err = std::string("pthread_setschedparam: ") + std::strerror(rc);
        return false;
    }
    if (cpu >= 0) {
        cpu_set_t set;
        CPU_ZERO(&set);
        CPU_SET(cpu, &set);
        rc = pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
        if (rc != 0) {
            if (err) *err = std::string("pthread_setaffinity_np: ") + std::strerror(rc);
            return false;
        }
    }
    return true;
}

bool lockMemory(std::size_t stackBytes, std::string* err) {
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        if (err) *err = std::string("mlockall: ") + std::strerror(errno);
        return false;
    }
    volatile unsigned char* probe = static_cast<volatile unsigned char*>(__builtin_alloca(stackBytes));
    for (std::size_t i = 0; i < stackBytes; i += 4096) probe[i] = 0;   // fault the stack in now
    return true;
}

std::string describeCurrentThread() {
    int policy = 0;
    sched_param sp {};
    pthread_getschedparam(pthread_self(), &policy, &sp);
    const char* name = policy == SCHED_FIFO ? "SCHED_FIFO" : policy == SCHED_RR ? "SCHED_RR" : "SCHED_OTHER";
    return std::string(name) + " prio " + std::to_string(sp.sched_priority) + " cpu " +
           std::to_string(sched_getcpu());
}

}
