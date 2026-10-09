#include "hw/edge_reader.h"

#include <algorithm>
#include <cstring>
#include <ctime>
#include <poll.h>
#include <pthread.h>
#include <sched.h>

namespace hw {
namespace {

// Quadrature table of the original code, [prev][cur] with state = 2*A + B.
constexpr int kTable[4][4] = {{0, +1, -1, 0}, {-1, 0, 0, +1}, {+1, 0, 0, -1}, {0, -1, +1, 0}};

std::int64_t toNs(const timespec& ts) { return std::int64_t(ts.tv_sec) * 1000000000LL + ts.tv_nsec; }

struct Ev {
    std::int64_t ts;
    int line;
    int type;
};

}

EdgeReader::EdgeReader(gpiod_chip* chip, unsigned lineA, unsigned lineB)
    : chip_(chip), lineA_(lineA), lineB_(lineB) {}

EdgeReader::~EdgeReader() { stop(); }

bool EdgeReader::start(int prio, int cpu, std::string* err) {
    a_ = gpiod_chip_get_line(chip_, lineA_);
    b_ = gpiod_chip_get_line(chip_, lineB_);
    if (!a_ || !b_ || gpiod_line_request_both_edges_events(a_, "pendctl-enc") < 0 ||
        gpiod_line_request_both_edges_events(b_, "pendctl-enc") < 0) {
        if (err) *err = std::string("cannot request encoder lines: ") + std::strerror(errno);
        return false;
    }
    state_[0] = gpiod_line_get_value(a_) == 1 ? 1 : 0;
    state_[1] = gpiod_line_get_value(b_) == 1 ? 1 : 0;
    timespec now{};
    clock_gettime(CLOCK_MONOTONIC, &now);
    initNs_ = toNs(now);   // edges before this are already in the start levels
    running_ = true;
    thread_ = std::thread(&EdgeReader::loop, this);
    if (prio > 0) {
        sched_param sp{};
        sp.sched_priority = prio;
        if (pthread_setschedparam(thread_.native_handle(), SCHED_FIFO, &sp) != 0 && err)
            *err = "encoder reader: SCHED_FIFO failed (run with sudo)";
        if (cpu >= 0) {
            cpu_set_t set;
            CPU_ZERO(&set);
            CPU_SET(cpu, &set);
            pthread_setaffinity_np(thread_.native_handle(), sizeof(set), &set);
        }
    }
    return true;
}

void EdgeReader::stop() {
    running_ = false;
    if (thread_.joinable()) thread_.join();   // poll() times out within 100 ms
    if (a_) gpiod_line_release(a_);
    if (b_) gpiod_line_release(b_);
    a_ = b_ = nullptr;
}

int EdgeReader::drainEdges(Edge* out, int maxEdges, bool* overflow) {
    int n = 0;
    std::size_t t = tail_.load(std::memory_order_relaxed);
    const std::size_t h = head_.load(std::memory_order_acquire);
    while (t != h && n < maxEdges) {
        out[n++] = ring_[t % kRing];
        ++t;
    }
    tail_.store(t, std::memory_order_release);
    // Overflow = edges lost (the reader overwrote unread ones) or this sample's list is incomplete because
    // `out` is full. An edge that arrives while draining is not lost: it is in the next sample. (Comparing
    // t with the head unconditionally made that race an "encoder fault": shadow run 2026-10-09T14-38-54,
    // two aborts while lifting fast with 1-4 edges per tick.)
    if (overflow)
        *overflow = ringOverflow_.exchange(false) || (n == maxEdges && t != head_.load(std::memory_order_acquire));
    return n;
}

void EdgeReader::loop() {
    pollfd fds[2] = {{gpiod_line_event_get_fd(a_), POLLIN, 0}, {gpiod_line_event_get_fd(b_), POLLIN, 0}};
    gpiod_line_event raw[16];
    Ev evs[32];
    while (running_) {
        if (poll(fds, 2, 100) <= 0) continue;
        int n = 0;
        for (int i = 0; i < 2; ++i) {
            if (!(fds[i].revents & POLLIN)) continue;
            const int k = gpiod_line_event_read_multiple(i == 0 ? a_ : b_, raw, 16);
            if (k == 16) {
                fullBatches_++;
                fault_ = true;
            }
            for (int j = 0; j < k; ++j) evs[n++] = {toNs(raw[j].ts), i, raw[j].event_type};
        }
        std::sort(evs, evs + n, [](const Ev& x, const Ev& y) { return x.ts < y.ts; });
        for (int j = 0; j < n; ++j) {
            const int prev = 2 * state_[0] + state_[1];
            state_[evs[j].line] = evs[j].type == GPIOD_LINE_EVENT_RISING_EDGE ? 1 : 0;
            const int cur = 2 * state_[0] + state_[1];
            if (cur == prev && evs[j].ts > initNs_) {   // earlier: already in the start levels
                sameType_++;
                fault_ = true;
            }
            const std::int32_t c = count_.load(std::memory_order_relaxed) + kTable[prev][cur];
            count_.store(c, std::memory_order_release);
            events_++;
            const std::size_t h = head_.load(std::memory_order_relaxed);
            if (h - tail_.load(std::memory_order_acquire) >= kRing) {
                ringOverflow_ = true;               // consumer too slow: drop the oldest view of edges
            } else {
                ring_[h % kRing] = Edge{evs[j].ts, c};
                head_.store(h + 1, std::memory_order_release);
            }
        }
    }
}

}
