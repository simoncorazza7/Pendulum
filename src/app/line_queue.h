#ifndef PENDEL_APP_LINE_QUEUE_H
#define PENDEL_APP_LINE_QUEUE_H

#include <pthread.h>
#include <sched.h>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <string>
#include <thread>

// Text from a real-time control loop goes through this queue to a printer thread, never straight to stdout: a write
// to stdout (a file on the Pi's SD card, or an ssh pty over Wi-Fi) blocked the loop for 0.23-1.56 s in the 2026-11-23
// actuator mock runs (docs/stage0_9_actuator.md). Single producer (the loop), single consumer (the printer);
// preallocated, so push never blocks. A full queue drops the text and counts it.
namespace app {

class LineQueue {
public:
    static constexpr std::size_t kSlots = 256, kLen = 1024;
    void push(const char* s) {
        const std::size_t h = head_.load(std::memory_order_relaxed);
        if (h - tail_.load(std::memory_order_acquire) >= kSlots) { dropped_.fetch_add(1); return; }
        std::snprintf(buf_[h % kSlots], kLen, "%s", s);
        head_.store(h + 1, std::memory_order_release);
    }
    bool pop(std::string* out) {
        const std::size_t t = tail_.load(std::memory_order_relaxed);
        if (t == head_.load(std::memory_order_acquire)) return false;
        *out = buf_[t % kSlots];
        tail_.store(t + 1, std::memory_order_release);
        return true;
    }
    std::uint32_t dropped() const { return dropped_.load(); }

private:
    char buf_[kSlots][kLen];
    std::atomic<std::size_t> head_{0}, tail_{0};
    std::atomic<std::uint32_t> dropped_{0};
};

// The only writer to stdout while a loop runs. Each queued text is written as given plus "\n" (push without a
// trailing newline). It runs SCHED_OTHER on any core: a thread started by the loop's thread would otherwise inherit
// its SCHED_FIFO priority and its single core. Stops, after draining the queue, once `run` is false.
inline std::thread startPrinter(LineQueue& q, std::atomic<bool>& run) {
    return std::thread([&q, &run] {
        sched_param sp{};
        pthread_setschedparam(pthread_self(), SCHED_OTHER, &sp);
        cpu_set_t cpus;                        // any core: pinned to one, a busy process there delayed it (mock 11-23)
        CPU_ZERO(&cpus);
        const unsigned n = std::thread::hardware_concurrency();
        for (unsigned c = 0; c < (n ? n : 4); ++c) CPU_SET(c, &cpus);
        pthread_setaffinity_np(pthread_self(), sizeof cpus, &cpus);
        std::string l;
        while (true) {
            const bool last = !run.load();
            bool any = false;
            while (q.pop(&l)) { std::cout << l << "\n"; any = true; }
            if (any) std::cout << std::flush;
            if (last) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    });
}

}

#endif
